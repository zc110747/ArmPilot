"""Robot Package 契约测试（Phase 1）。

## 这份测试在防什么

`manifest.yaml` 的字段如果**没人读**，它就会腐败成一份"看起来很像真的"的文档。
所以这里的断言分三类：

1. **推导值与期望值对得上** —— `dof` / `actuator_count` 刻意不写进 manifest，
   由 `loader` 从 `robot.yaml` 推导。**期望值就写在这里**，于是
   "改了关节表却忘了改 manifest"这类事在结构上不可能发生（因为 manifest 没写）。
2. **四份声明互相对得上** —— 选择器 / manifest / robot.yaml / 包目录。
   这是 `PackageValidator` 的职责，这里测**它真的在报错**（而不是永远返回 ok）。
3. **哈希是确定的、且排除了产物** —— 陈旧检测的全部价值都建立在这两点上。

## 一个刻意的反面测试

`test_validator_rejects_*` 系列**故意构造坏 manifest**，证明校验器**会**报错。
只测"好 manifest 通过"等于没测 —— 一个永远返回 ok 的校验器也能通过那种测试。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

# core/tests → core/python 需要进 sys.path（见 conftest.py，此处再兜一层以防单文件运行）
_PY_DIR = Path(__file__).resolve().parents[1] / "python"
if str(_PY_DIR) not in sys.path:
    sys.path.insert(0, str(_PY_DIR))

from robopkg import (  # noqa: E402
    PackageError,
    compute_content_hash,
    declared_path,
    declared_paths,
    declared_paths_keys,
    list_package_ids,
    load_manifest,
    load_robot,
    package_dir_of,
    validate_all,
    validate_package,
)
from robopkg.content_hash import hash_paths  # noqa: E402
from robopkg.manifest import MANIFEST_FORMAT, parse_manifest  # noqa: E402
from robopkg.root import PROJECT_ROOT, repo_relative, selftest  # noqa: E402

MEARM = "mearm-v1"
SO101 = "so-arm101"

#: 期望值表 —— **刻意放在测试里**（而不是 manifest 里）。
#: 它是"人核对过的结论"，不是"运行时的真值来源"。
#: 每加一台机器人，这里多一行；改一台机器人的关节表，这里会**失败**并提醒复核。
EXPECTED = {
    MEARM: {
        "model_id": "mearm",
        "model": "MeArm-V1",
        "dof": 4,
        "dof_joints": ("base", "shoulder", "elbow", "gripper"),
        "qpos_joints": ("base", "shoulder", "elbow", "tool", "gripper"),
        "passive_joints": ("tool",),
        "actuators": 4,
        "ik": True,
        "ik_type": "custom",
        "position": True,
        "simulation": True,
        "hardware": True,
        "tcp_site": "tcp",
    },
    SO101: {
        "model_id": "so-arm101",
        "model": "SO-ARM101",
        "dof": 6,
        "dof_joints": (
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ),
        "qpos_joints": (
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ),
        "passive_joints": (),
        "actuators": 6,
        "ik": False,
        "ik_type": "none",
        "position": False,
        "simulation": True,
        "hardware": False,
        "tcp_site": "gripperframe",
    },
}


# ---------------------------------------------------------------------------
# 1. 路径锚点与包发现
# ---------------------------------------------------------------------------


def test_path_anchor_selftest_passes() -> None:
    assert selftest() == []


def test_both_packages_are_discovered() -> None:
    assert list_package_ids() == [MEARM, SO101]


# ---------------------------------------------------------------------------
# 2. 推导值 == 期望值
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("robot_id", [MEARM, SO101])
def test_derived_facts_match_expectations(robot_id: str) -> None:
    exp = EXPECTED[robot_id]
    r = load_robot(robot_id)

    assert r.model_id == exp["model_id"], "**模型 id ≠ 包 id** 是本项目最容易搞混的一处"
    assert r.manifest.name == exp["model"]
    assert r.dof == exp["dof"]
    assert r.dof_joints == exp["dof_joints"]
    assert r.qpos_joints == exp["qpos_joints"]
    assert r.passive_joints == exp["passive_joints"]
    assert r.actuator_count == exp["actuators"]
    assert r.manifest.simulation.tcp_site == exp["tcp_site"]


def test_qpos_and_dof_are_different_concepts() -> None:
    """MeArm 的 `qpos=5 > dof=4` —— 被动腕在 MJCF 里有 qpos 但**没有独立自由度**。

    把两者混为一谈会得到一个五元组的 JR，而固件/串口协议/前端全按四元组解析
    （`JointCfg.has_qpos` 的 docstring 专门写了这条）。
    """
    r = load_robot(MEARM)
    assert r.dof == 4
    assert len(r.qpos_joints) == 5
    assert set(r.qpos_joints) - set(r.dof_joints) == {"tool"}
    assert r.passive_joints == ("tool",)

    s = load_robot(SO101)
    assert s.dof == len(s.qpos_joints) == 6, "SO-101 无被动件/耦合 ⇒ 两个概念在这台上恰好相等"
    assert s.passive_joints == ()


@pytest.mark.parametrize("robot_id", [MEARM, SO101])
def test_capabilities_agree_with_derivation(robot_id: str) -> None:
    exp = EXPECTED[robot_id]
    r = load_robot(robot_id)
    caps = r.manifest.capabilities

    assert caps.gripper == r.has_gripper is True
    assert caps.ik == exp["ik"]
    assert caps.position == exp["position"]
    assert caps.simulation == exp["simulation"]
    assert caps.hardware == exp["hardware"]
    assert r.manifest.kinematics.ik_type == exp["ik_type"]


def test_no_robot_fabricates_ik() -> None:
    """`ik: false` 的包**必须**是 `kinematics.ik.type == none` 且**没有 entry**。

    spec §8/§35：没有可靠依据就不要有逆解。此处把"能不能伪造"变成机器可检的约束 ——
    只要有人把 `type: none` 改成 `custom` 并塞一个 entry，这条就会失败。
    """
    m = load_manifest(SO101)
    assert m.capabilities.ik is False
    assert m.kinematics.ik_type == "none"
    assert m.kinematics.ik_entry is None

    # 反向：MeArm 有 IK，且必须指向一个真实存在的实现
    mm = load_manifest(MEARM)
    assert mm.capabilities.ik is True
    assert mm.kinematics.ik_type == "custom"
    assert mm.kinematics.ik_entry is not None
    assert (PROJECT_ROOT / mm.kinematics.ik_entry).is_file()


# ---------------------------------------------------------------------------
# 3. 四份声明互相一致（PackageValidator 真的在报错吗）
# ---------------------------------------------------------------------------


def test_all_packages_pass_validation() -> None:
    reports = validate_all()
    bad = [r for r in reports if not r.ok]
    assert not bad, "包契约有问题：\n" + "\n".join(r.summary() for r in bad)


def _good_raw(robot_id: str) -> dict:
    import yaml

    path = PROJECT_ROOT / "robot-package" / robot_id / "manifest.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_parser_rejects_unknown_field() -> None:
    """**字段名拼错**必须报错。

    现实后果：`tcp_site` 写成 `tcpSite` 会静默退化成……不，会直接缺字段；
    而若写成 `tcp_Site`，没有未知字段检查时它会被当成"没给"，
    若将来引入缺省就意味着 **SO-101 的 TCP 落到一个官方 MJCF 里不存在的 site 名**。
    """
    raw = _good_raw(MEARM)
    raw["simulation"]["tcpSite"] = "tcp"  # 典型的驼峰误写
    with pytest.raises(PackageError, match="未知字段"):
        parse_manifest(raw)


def test_parser_rejects_missing_required_field() -> None:
    raw = _good_raw(MEARM)
    del raw["capabilities"]["ik"]
    with pytest.raises(PackageError, match="ik"):
        parse_manifest(raw)


def test_parser_rejects_non_bool_capability() -> None:
    raw = _good_raw(MEARM)
    raw["capabilities"]["ik"] = "true"  # YAML 里写成字符串
    with pytest.raises(PackageError, match="必须是布尔"):
        parse_manifest(raw)


def test_parser_rejects_format_mismatch() -> None:
    raw = _good_raw(MEARM)
    raw["package"]["format"] = MANIFEST_FORMAT + 1
    with pytest.raises(PackageError, match="只认"):
        parse_manifest(raw)


def test_validator_rejects_position_without_ik() -> None:
    """能力自洽：位置目标必须经逆解 ⇒ 没有逆解就不能声明位置能力。"""
    raw = _good_raw(SO101)
    raw["capabilities"]["position"] = True
    raw["capabilities"]["ik"] = False
    m = parse_manifest(raw, source_path=PROJECT_ROOT / "robot-package" / SO101 / "manifest.yaml")
    # 直接走能力检查（不落盘也能测）
    from robopkg.validator import ValidationReport, _check_kinematics_decl

    rep = ValidationReport(robot_id=SO101)
    _check_kinematics_decl(rep, m)
    codes = {i.code for i in rep.errors}
    assert "POSITION_WITHOUT_IK" in codes


def test_validator_rejects_engine_fk_with_entry() -> None:
    """`type: engine` 表示"用 Core 的通用 FK"，再给 entry 是**矛盾声明**。

    它会制造两份 FK：渲染走通用那份、引擎走包内那份，两者分家且**不报错**。
    """
    raw = _good_raw(MEARM)
    raw["kinematics"]["fk"]["entry"] = "frontend/src/robot/kinematics/fk.ts"
    m = parse_manifest(raw, source_path=PROJECT_ROOT / "robot-package" / MEARM / "manifest.yaml")
    from robopkg.validator import ValidationReport, _check_kinematics_decl

    rep = ValidationReport(robot_id=MEARM)
    _check_kinematics_decl(rep, m)
    assert "FK_ENGINE_WITH_ENTRY" in {i.code for i in rep.errors}


def test_validator_rejects_unknown_robot_without_fallback() -> None:
    """未知 id **不回退** —— 回退会表现为"写错一个字母就静默换了另一台机器人"。

    ⚠️ 未知 id 在**两层**都会被拒：
      - `robotcfg.resolve_robot_entry` 抛 `ConfigError`（选择器里没这台）
      - `load_manifest` 抛 `PackageError`（没有这个包目录）
    两个都要接受 —— 只认一个的话，将来某层被改掉，这条测试会以"通过了"的形式失效。
    """
    from robopkg.root import load_robotcfg

    config_error = load_robotcfg().ConfigError
    for bad in ("mearm", "no-such-robot"):
        with pytest.raises((PackageError, config_error)):
            load_robot(bad)

    rep = validate_package("no-such-robot")
    assert not rep.ok
    assert "PACKAGE_DIR_MISSING" in {i.code for i in rep.errors}


def test_validator_flags_selector_without_package(monkeypatch: pytest.MonkeyPatch) -> None:
    """反向缺口：选择器声明了、但没有包 ⇒ 必须显式报告。

    它对应现状里那条必然会发生的中间状态（"先加配置再加实现"），
    不能让它表现成"这台机器人不存在"。
    """
    import robopkg.validator as v

    real_list = v.list_package_ids

    def fake_list() -> list[str]:
        return [i for i in real_list() if i != SO101]

    monkeypatch.setattr(v, "list_package_ids", fake_list)
    reports = {r.robot_id: r for r in v.validate_all()}
    assert SO101 in reports
    assert "SELECTOR_WITHOUT_PACKAGE" in {i.code for i in reports[SO101].errors}


# ---------------------------------------------------------------------------
# 4. 内容哈希：确定性 + 排除产物
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("robot_id", [MEARM, SO101])
def test_content_hash_is_deterministic(robot_id: str) -> None:
    a = compute_content_hash(load_manifest(robot_id))
    b = compute_content_hash(load_manifest(robot_id))
    assert a.digest == b.digest
    assert a.file_count > 0


def test_content_hash_is_order_independent(tmp: Path) -> None:
    """输入顺序不同必须得到同一个 hash（陈旧检测靠它跨平台/跨实现稳定）。"""
    (tmp / "a.txt").write_text("A", encoding="utf-8")
    (tmp / "b.txt").write_text("B", encoding="utf-8")
    rel = [repo_relative(tmp / "a.txt"), repo_relative(tmp / "b.txt")]
    d1, _ = hash_paths(rel)
    d2, _ = hash_paths(list(reversed(rel)))
    assert d1 == d2


def test_content_hash_detects_content_change(tmp: Path) -> None:
    """内容变了 hash 必须变 —— 否则陈旧检测永远认为"没变"。"""
    f = tmp / "x.bin"
    f.write_bytes(b"one")
    d1, _ = hash_paths([repo_relative(f)])
    f.write_bytes(b"two")
    d2, _ = hash_paths([repo_relative(f)])
    assert d1 != d2

    # 改名也必须检出（路径也进哈希）
    f.write_bytes(b"two")
    g = tmp / "y.bin"
    g.write_bytes(b"two")
    assert hash_paths([repo_relative(g)])[0] != d2


def test_content_hash_excludes_generated_artifacts() -> None:
    """产物不进哈希：MeArm 的 MJCF（生成物）与 SO-101 的 robot.yaml（生成物）。

    两个被排除的路径都**取自 manifest 的声明**（`simulation.mjcf` / `model.config`
    且带 `generated_by`）—— 不写死字符串，于是这条判据在搬迁后依然成立。
    """
    mearm_manifest = load_manifest(MEARM)
    so101_manifest = load_manifest(SO101)
    assert mearm_manifest.simulation.generated_by, "MeArm 的 MJCF 必须声明为产物"
    assert so101_manifest.model.generated_by, "SO-101 的 robot.yaml 必须声明为产物"

    mearm = compute_content_hash(mearm_manifest)
    so101 = compute_content_hash(so101_manifest)

    gen_mearm = repo_relative(mearm_manifest.mjcf_file)
    gen_so101 = repo_relative(so101_manifest.config_file)
    assert gen_mearm in mearm.excluded_generated
    assert gen_so101 in so101.excluded_generated

    mearm_paths = {e.rel_path for e in mearm.entries}
    so101_paths = {e.rel_path for e in so101.entries}
    assert gen_mearm not in mearm_paths
    assert gen_so101 not in so101_paths


def test_content_hash_includes_official_sources_of_so101() -> None:
    """SO-101 的**上游**必须进哈希：官方官方 MJCF + URDF + 网格。

    它的 robot.yaml 是生成物（被排除），若上游也不进哈希，
    那"换了官方模型却没重新生成"这件事就**完全不可见** —— 包内容哈希不变、
    陈旧检测放行、跑的还是旧模型。这是本条测试存在的唯一理由。
    """
    entries = {e.rel_path for e in compute_content_hash(load_manifest(SO101)).entries}
    assert "assets/models/so-arm101/official/so101_new_calib.xml" in entries
    assert "assets/models/so-arm101/official/so101_new_calib.urdf" in entries
    assert len([p for p in entries if p.endswith(".stl")]) == 13, "13 个官方 STL 都必须在哈希里"


def test_content_hash_covers_mearm_truth_files() -> None:
    """MeArm 的**源**必须在哈希里：robot.yaml / physics.yaml / ik.ts / 测试数据 / 冻结文件。

    路径全部**取自 manifest 的声明**（`declared_path`），不写死字符串 ——
    否则真值随包搬迁后这里会继续断言旧路径，而"断言失败"会被误读成
    "哈希漏了文件"（真正的错因是测试自己过期了）。
    """
    entries = {e.rel_path for e in compute_content_hash(load_manifest(MEARM)).entries}
    cases_dir = declared_path(MEARM, "tests.cases")
    for expected in (
        repo_relative(declared_path(MEARM, "model.config")),
        repo_relative(declared_path(MEARM, "model.physics")),
        repo_relative(declared_path(MEARM, "tests.frozen")),
        # ★ IK 实现与引擎入口都**取自 manifest 声明**。此前这里写死的是
        #   `frontend/src/robot/kinematics/ik.ts` —— Phase 2 步④ 把它搬进包之后，
        #   这条断言会以"哈希漏了文件"的面目失败，而真正的错因是**测试自己过期了**。
        repo_relative(declared_path(MEARM, "kinematics.ik.entry")),
        repo_relative(declared_path(MEARM, "kinematics.engine.entry")),
        repo_relative(cases_dir / "fk_cases.json"),
        repo_relative(cases_dir / "sim2sim.json"),
        repo_relative(package_dir_of(MEARM) / "manifest.yaml"),
    ):
        assert expected in entries, f"{expected} 应当参与 MeArm 的内容哈希"


# ---------------------------------------------------------------------------
# 声明路径（Phase 2 新增）—— 路径只声明在 manifest，解析只发生在 declared_path()
# ---------------------------------------------------------------------------


def test_every_declared_path_key_resolves_for_every_package() -> None:
    """**全部**包 × **全部**登记的声明键都必须解析得到、且真实存在。

    这条测试的价值：它把"路径字段漏登记 / 写错 / 搬迁时漏改"变成**立刻报错**。
    没有它，漏改的表现会是"读到了另一个仍然存在的同名文件"—— 那是最难查的一类。
    """
    keys = declared_paths_keys()
    assert keys, "至少要登记一个声明路径键，否则本测试是空转"
    for rid in list_package_ids():
        load_manifest(rid)  # manifest 本身必须可解析
        for key in keys:
            try:
                p = declared_path(rid, key)
            except PackageError as exc:
                # 允许"manifest 本来就没声明这个可选项"（如 SO-101 没有 tests.frozen）
                if "未声明" in str(exc):
                    continue
                raise AssertionError(f"{rid}.{key} 解析失败：{exc}") from exc
            assert p.exists(), f"{rid}.{key} → {p} 不存在"


def test_unknown_declared_path_key_is_rejected() -> None:
    """未登记的键必须报错 —— 否则新路径字段会静默复用旧解析。"""
    with pytest.raises(PackageError):
        declared_path(MEARM, "tests.nope")
    with pytest.raises(PackageError):
        declared_paths(MEARM, "tests.nope")


def test_every_declared_tool_path_exists() -> None:
    """`tests.tools` 是一份**可执行清单**：声明的每个工具都必须真实存在。

    为什么值得单测：工具搬位置却忘改清单，是最典型的"文档与事实分家" ——
    清单会继续写着旧路径，而没有人会去核对它。
    """
    checked = 0
    for rid in list_package_ids():
        for p in declared_paths(rid, "tests.tools"):
            assert p.is_file(), f"{rid}.tests.tools 声明的 {p} 不是文件"
            checked += 1
    assert checked >= 16, (
        f"两包的工具清单合计应 ≥16 项（MeArm 14 + SO-101 2），实得 {checked} —— "
        "清单被删空的话这条测试会变成空转")


def test_generated_by_points_at_a_real_generator_inside_its_package() -> None:
    """`generated_by` 是**路径**，必须被当作路径校验，且生成器要住在**自己的包**里。

    这是本轮遗漏掉的那条判据：`generated_by` 一直是路径，却没人解析它 ——
    于是"生成器搬走了"只有等人去点那个脚本才会发现。
    """
    checked = 0
    for rid in list_package_ids():
        pkg = package_dir_of(rid)
        for key in ("model.generated_by", "simulation.generated_by"):
            try:
                p = declared_path(rid, key)
            except PackageError as exc:
                if "未声明" in str(exc):
                    continue
                raise
            assert p.is_file(), f"{rid}.{key} = {p} 不是文件"
            assert p.is_relative_to(pkg), (
                f"{rid}.{key} = {p} 不在包 {pkg} 内 —— 生成器应当随包走")
            checked += 1
    assert checked >= 2, (
        "至少应覆盖 MeArm 的 MJCF 生成器 + SO-101 的 robot.yaml 生成器，"
        f"实得 {checked} —— 声明被删空的话这条测试会变成空转")


def test_engine_entry_agrees_with_frontend_discovery() -> None:
    """`kinematics.engine.entry`（**声明**）≡ 按约定模式扫到的引擎文件（**事实**）。

    前端 `RobotRegistry` 用 `import.meta.glob('robot-package/*/kinematics/engine.ts')`
    **自动发现**引擎（Phase 2 步④：不再手写分派表）。于是同一条信息有了两处来源：

      · manifest 的 `engine.entry` → Python 侧 `declared_path()` / 内容哈希读它；
      · glob 扫描结果 → 前端实际加载它。

    两者**各自都能自洽**，合起来才可能矛盾：把 `engine.ts` 挪到子目录之后，
    glob 扫不到（前端表现为"这台机器人没有引擎"），而 manifest 若还写着旧路径，
    Python 侧只会说"文件不存在"——两边的报错都指不到"这两件事分家了"。

    所以这条断言直接**把两份清单对起来**。与 TS 侧的
    `robotManifest.test.ts::manifest.kinematics.engine.entry ≡ 前端 glob 实际发现的引擎文件`
    是**同一条契约的两个视角**（跨端缝必须两端都钉，只钉一端挡不住另一端被改）。

    ⚠️ 拿 glob 结果比，不拿 `robot-package/<id>/kinematics/engine.ts` 拼字符串比 ——
    拼出来的字符串永远等于声明值，那就成了同义反复。
    """
    found = sorted(
        repo_relative(p)
        for p in (PROJECT_ROOT / "robot-package").glob("*/kinematics/engine.ts")
    )
    declared = sorted(
        repo_relative(declared_path(rid, "kinematics.engine.entry"))
        for rid in list_package_ids()
    )
    assert found == declared, (
        "manifest 声明的引擎入口与 glob 扫到的引擎文件不一致：\n"
        f"  声明: {declared}\n  扫到: {found}"
    )
    assert len(found) == len(list_package_ids()), (
        "每个包必须恰好有一个 kinematics/engine.ts；数量对不上说明有包扫不到引擎"
        "（glob 空掉时这条会以'int 比较'的形式失败，而不是静默通过）")


def test_package_local_tests_are_declared_and_non_empty() -> None:
    """`tests.local` 声明的目录里必须**真的有** `test_*.py`。

    Phase 2 步⑤ 的判据是"这条断言换台机器人还成立吗"：不成立 ⇒ 随包走。
    但"随包走"如果只体现为 manifest 里一个字符串，它迟早会退化成一句注释 ——
    所以这里把它变成机器判据：目录在不在、有没有测试、是不是住在自己的包里。

    刻意**不**要求每个包都声明：包内测试是**按需**的（SO-101 目前没有
    需要随包的结构性断言，它的基线在 `tests/sim2sim/` 与前端）。
    所以这里只要求"声明了的必须是真的"，并要求至少有一个包用它（防空转）。
    """
    checked = 0
    for rid in list_package_ids():
        try:
            d = declared_path(rid, "tests.local")
        except PackageError as exc:
            if "未声明" in str(exc):
                continue
            raise
        assert d.is_dir(), f"{rid}.tests.local = {d} 不是目录"
        assert d.is_relative_to(package_dir_of(rid)), (
            f"{rid}.tests.local = {d} 不在包 {package_dir_of(rid)} 内 —— 包内测试必须随包")
        files = sorted(p.name for p in d.glob("test_*.py"))
        assert files, (
            f"{rid}.tests.local = {d} 里没有 test_*.py ⇒ 声明成了空壳；"
            "要么补测试，要么把这条声明去掉")
        checked += 1
    assert checked >= 1, (
        "至少要有一个包声明包内测试，否则这条测试是空转 —— "
        "而它空转的正好是「测试随包走」这条 Phase 2 判据")


def test_pytest_testpaths_covers_package_dirs() -> None:
    """`pytest.ini` 的 `testpaths` 必须覆盖 `robot-package`。

    为什么这条值得单独测：包内测试**存在**和包内测试**被执行**是两件事。
    `testpaths` 漏了 `robot-package` 的话，`pytest -q` 会显示全绿 ——
    而那些测试**从未跑过**。这类"伪造通过"比缺测试更危险，因为它看起来是绿的。
    """
    import configparser

    ini_path = PROJECT_ROOT / "pytest.ini"
    assert ini_path.is_file(), f"缺少 {ini_path}（包内测试的收集范围靠它）"
    cp = configparser.ConfigParser()
    cp.read(ini_path, encoding="utf-8")
    raw = cp["pytest"]["testpaths"]
    paths = [line.strip() for line in raw.splitlines() if line.strip()]
    assert "robot-package" in paths, (
        f"pytest.ini 的 testpaths 没有覆盖 robot-package（实得 {paths}）⇒ 包内测试不会被执行")
    assert "core/tests" in paths and "tests" in paths, (
        f"testpaths 同时要覆盖 Core 的测试（实得 {paths}）")


def test_declared_truth_and_cases_live_inside_their_own_package() -> None:
    """`model.config` / `model.physics` / `tests.cases` 都必须在**包目录内**。

    这就是"包是自包含的"的机器判据（spec §5）。反例（Phase 2 搬迁前的现状）：
    真值住在 `config/robot.yaml`、用例住在 `tests/baseline/<id>/`，
    于是"这个包完整吗"这个问题无法仅凭包本身回答。

    ⚠️ 刻意**不**要求 `simulation.mjcf` 也在包内：MeArm 的 MJCF 是**产物**
    （由 `gen_model.py` 生成到 `simulation/mujoco/`），SO-101 的则是官方原样文件。
    产物与官方上游放哪，由各自的包策略决定 —— 但那两处**必须**由 manifest 显式声明，
    这条由 `test_every_declared_path_key_resolves_for_every_package` 守着。
    """
    for rid in list_package_ids():
        pkg = package_dir_of(rid)
        for key in ("model.config", "model.physics", "tests.cases"):
            try:
                p = declared_path(rid, key)
            except PackageError as exc:
                if "未声明" in str(exc):
                    continue
                raise
            assert p.is_relative_to(pkg), (
                f"{rid}.{key} = {p} 不在包 {pkg} 内（包不自包含）"
            )


def test_manifest_is_queried_by_id_not_by_model_id() -> None:
    """按 `robot.yaml` 路径反查包：MeArm 的真值 → `mearm-v1`（**不是**模型 id `mearm`）。

    ⚠️ 路径取自 manifest 的声明，**不写死** —— 写死的断言在真值随包搬迁后会变成
    "查不到包"的假故障。
    """
    from robopkg.manifest import find_manifest_for_config

    mearm_yaml = declared_path(MEARM, "model.config")
    assert find_manifest_for_config(mearm_yaml).id == MEARM
    # 两者的 id **确实不同**（同名的话这条测试就测不出东西了）
    assert load_robot(MEARM).model_id == "mearm"
    with pytest.raises(PackageError):
        find_manifest_for_config("config/does-not-exist.yaml")


# ---------------------------------------------------------------------------
# fixture
# ---------------------------------------------------------------------------


@pytest.fixture()
def tmp(tmp_path: Path) -> Path:
    """仓库内的临时目录（哈希按仓库相对路径算，故不能用系统 temp）。"""
    d = PROJECT_ROOT / ".workbuddy" / "captures" / "robopkg-test" / tmp_path.name
    d.mkdir(parents=True, exist_ok=True)
    yield d
    for f in d.glob("*"):
        if f.is_file():
            f.unlink()
    d.rmdir()
