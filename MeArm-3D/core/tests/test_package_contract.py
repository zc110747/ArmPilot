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
    list_package_ids,
    load_manifest,
    load_robot,
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
    """产物不进哈希：MeArm 的 MJCF（生成物）与 SO-101 的 robot.yaml（生成物）。"""
    mearm = compute_content_hash(load_manifest(MEARM))
    so101 = compute_content_hash(load_manifest(SO101))

    assert "simulation/mujoco/mearm.xml" in mearm.excluded_generated
    assert "config/robots/so-arm101/robot.yaml" in so101.excluded_generated

    mearm_paths = {e.rel_path for e in mearm.entries}
    so101_paths = {e.rel_path for e in so101.entries}
    assert "simulation/mujoco/mearm.xml" not in mearm_paths
    assert "config/robots/so-arm101/robot.yaml" not in so101_paths


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
    """MeArm 的**源**必须在哈希里：robot.yaml / physics.yaml / ik.ts / 测试数据 / 冻结文件。"""
    entries = {e.rel_path for e in compute_content_hash(load_manifest(MEARM)).entries}
    for expected in (
        "config/robot.yaml",
        "config/physics.yaml",
        "config/baseline-kinematics-physics.json",
        "frontend/src/robot/kinematics/ik.ts",
        "tests/baseline/mearm-v1/fk_cases.json",
        "tests/baseline/mearm-v1/sim2sim.json",
        "robot-package/mearm-v1/manifest.yaml",
    ):
        assert expected in entries, f"{expected} 应当参与 MeArm 的内容哈希"


def test_manifest_is_queried_by_id_not_by_model_id() -> None:
    """按 `robot.yaml` 路径反查包：`config/robot.yaml` → `mearm-v1`（不是 `mearm`）。"""
    from robopkg.manifest import find_manifest_for_config

    assert find_manifest_for_config("config/robot.yaml").id == MEARM
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
