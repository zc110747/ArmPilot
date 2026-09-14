#!/usr/bin/env python3
"""冻结 / 校验「运动学 + 物理」真值基线。

背景
----
用户要求：「保存目前的运动学物理数据不变」。字面做法是给两份 yaml 存一份快照 +
一个哈希守卫；但**整文件哈希是个错误的判据**，因为 `robot-package/mearm-v1/model/robot.yaml` 同时装了两类
完全不同的东西：

  · **运动学真值**：`links[].length` / `joints[]` / `actuators[]` / `robot.tcp` /
    `robot.homePose` —— FK / IK / 标定 / 安全校验全部由它派生，**这才是要冻结的**。
  · **外观几何**：`links[].geometry` / `links[].details`（程序化拼装的板件 / 舵机 /
    轴销 + 十六进制颜色）—— 项目里明确写着「改 geometry 只影响外观，绝不影响
    FK / IK / 标定」。

后续「图像采集 → 更真实的视觉建模」必然要动外观那一层。整文件哈希会把**合法**的
外观变更判成违规，于是守卫会被绕过或直接删掉 —— 一个会被绕过的守卫等于没有守卫。

因此本模块分两级：

  Level 1  整文件 SHA256         记录用。变了就提示：「去看看是不是只动了外观」。
  Level 2  **语义核心 SHA256**   真正的判据，只覆盖运动学 / 物理字段。

    改外观（geometry / details / color） ⇒ L1 变、L2 不变 ⇒ **放行**
    改长度 / 限位 / 耦合 / 轴 / 标定 / 质量 / 摩擦 ⇒ L2 也变 ⇒ **报错**
      （必须显式 `--update` 重新冻结，从而在 diff 里留下"我确实动了物理"的痕迹）

用法
----
    python core/tools/freeze_baseline.py            # 校验（CI / 测试用），不符合退出码 1
    python core/tools/freeze_baseline.py --update   # 重新冻结（有意识地改过参数后）
    python core/tools/freeze_baseline.py --show     # 打印当前核心摘要，不写文件
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import yaml

#: 缺省目标机器人 —— **Golden Baseline**，显式写 id 而不是读选择器的 `default`
#: （依赖缺省的话，改了 `default` 就会静默换掉被守卫的对象，而守卫照样绿）。
DEFAULT_ROBOT_ID = "mearm-v1"


def _find_repo_root() -> Path:
    """仓库根 —— **向上找标记**（同时含 `core/` 与 `robot-package/` 的那一层）。

    ⚠️ 这里此前写的是 `Path(__file__).resolve().parent.parent`。在 `tools/` 时代它恰好
    等于仓库根；搬到 `core/tools/` 之后它变成 `core/` ⇒ 于是"基线文件不存在"、
    "真值读不到"，而报错全都指向别处（看着像路径写错了），完全指不到真正的原因。
    ⇒ 判据换成"哪一层同时有 core/ 和 robot-package/"，它**不依赖文件所在层数**。
    """
    for parent in Path(__file__).resolve().parents:
        if (parent / "core").is_dir() and (parent / "robot-package").is_dir():
            return parent
    raise RuntimeError(f"从 {__file__} 向上找不到仓库根（需同时含 core/ 与 robot-package/）")


PROJECT_ROOT = _find_repo_root()
_CORE_PY = PROJECT_ROOT / "core" / "python"
if str(_CORE_PY) not in sys.path:
    sys.path.insert(0, str(_CORE_PY))

from robopkg import declared_path  # noqa: E402
from robopkg.root import repo_relative  # noqa: E402


def paths(robot_id: str | None = None) -> tuple[Path, Path, Path]:
    """`(robot.yaml, physics.yaml, 冻结基线 json)` —— **全部来自 manifest 的声明**。

    这是本模块**唯一**算路径的地方。此前这里是三条硬编码
    （`PROJECT_ROOT / "config" / "robot.yaml"` 之类）。它的危险之处不在于会坏，
    而在于**坏得不明显**：快照里的**键**当年已经被改成包内路径、读的却还是
    `config/` —— 两处各说各话，而"键"看上去完全正确。
    ⇒ 现在键与读取用**同一个来源**，结构上不可能分家。

    `robot_id` 省略时用 `DEFAULT_ROBOT_ID`。
    """
    rid = robot_id or DEFAULT_ROBOT_ID
    return (
        declared_path(rid, "model.config"),
        declared_path(rid, "model.physics"),
        # ⚠️ 冻结基线是**按包声明**的可选项（`tests.frozen`）。
        declared_path(rid, "tests.frozen"),
    )


def _rel_names(robot_id: str | None = None) -> tuple[str, str]:
    """`(robot.yaml, physics.yaml)` 的**仓库相对路径** = 基线 `files` 的键。"""
    robot_yaml, physics_yaml, _baseline = paths(robot_id)
    return repo_relative(robot_yaml), repo_relative(physics_yaml)


def _truth_rel_keys(robot_id: str | None = None) -> tuple[tuple[str, str], ...]:
    """`((真值文件的仓库相对路径, 该文件要盯的语义哈希键), ...)`。

    刻意返回**扁平的对**而不是"路径 → 键列表"：两个文件各盯一个键，
    嵌套形状只会让调用方多写一层循环。
    """
    robot_rel, physics_rel = _rel_names(robot_id)
    return (
        (robot_rel, "kinematic_core_sha256"),
        (physics_rel, "physics_core_sha256"),
    )

# ---------------------------------------------------------------------------
# 语义核心的字段白名单
#   白名单（而不是黑名单）是刻意的：将来 yaml 里**新增**一个运动学字段时，
#   若忘了把它加进来，本守卫会**漏检**而不是误报。漏检是这里可以接受的方向 ——
#   误报会让人把守卫关掉，那才是真正的失败。
# ---------------------------------------------------------------------------
KINEMATIC_ROBOT_KEYS = ("id", "name", "units", "tcp", "homePose")
KINEMATIC_LINK_KEYS = ("id", "parent", "length")
KINEMATIC_JOINT_KEYS = (
    "id", "role", "type", "parentLink", "childLink", "axis", "origin", "limit", "coupling",
)
KINEMATIC_ACTUATOR_KEYS = ("id", "jointId", "channel", "offset", "scale", "reverse", "limits")

# physics.yaml 全是物理量，但 `recording`（I/O 配置）与 `deterministic.seed`
# （运行参数）不是"物理" —— 它们改动不该被当成物理参数变更而报错。
PHYSICS_SECTIONS = (
    "timestep", "solver", "gravity", "servo", "inertia", "contact", "limits", "calibration",
)

# 明确"被排除在冻结之外"的字段，写进基线文件里当文档
EXCLUDED_FROM_KINEMATIC = (
    "links[].geometry", "links[].details",
    "robot.yaml 全部注释（注释里确实有信息，但它属于文档而非真值）",
)


def _project(obj, keys):
    """按白名单取字段；缺失的键**不补 None**（补了会让"删掉一个字段"看起来像没变）。"""
    assert isinstance(obj, dict), f"期望 dict，得到 {type(obj).__name__}"
    return {k: obj[k] for k in keys if k in obj}


def robot_kinematic_core(doc: dict) -> dict:
    """从 robot.yaml 提取**运动学语义核心**（剔除外观几何）。"""
    robot = doc.get("robot") or {}
    links = doc.get("links") or []
    joints = doc.get("joints") or []
    actuators = doc.get("actuators") or []
    return {
        "robot": _project(robot, KINEMATIC_ROBOT_KEYS),
        "links": [_project(l, KINEMATIC_LINK_KEYS) for l in links],
        "joints": [_project(j, KINEMATIC_JOINT_KEYS) for j in joints],
        "actuators": [_project(a, KINEMATIC_ACTUATOR_KEYS) for a in actuators],
    }


def physics_core(doc: dict) -> dict:
    """从 physics.yaml 提取物理语义核心（剔除 recording / deterministic）。"""
    return {k: doc[k] for k in PHYSICS_SECTIONS if k in doc}


def digest(obj) -> str:
    """规范化 JSON 的 SHA256 —— 与键顺序、缩进无关，只与内容有关。"""
    canonical = json.dumps(obj, sort_keys=True, ensure_ascii=False, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def file_digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_yaml(path: Path) -> dict:
    # ⚠️ Windows 下 open() 默认是 GBK ⇒ 必须显式 utf-8，否则中文注释/键直接炸
    with path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def snapshot(robot_id: str | None = None) -> dict:
    """当前状态的完整快照（L1 文件哈希 + L2 语义核心 + 人类可读摘要）。"""
    rid = robot_id or DEFAULT_ROBOT_ID
    robot_yaml, physics_yaml, _baseline = paths(rid)
    robot_doc = _load_yaml(robot_yaml)
    physics_doc = _load_yaml(physics_yaml)
    k_core = robot_kinematic_core(robot_doc)
    p_core = physics_core(physics_doc)
    return {
        # ⚠️ 键**取自声明**（`repo_relative(declared_path(...))`），不是手写的字符串。
        #    手写时它与上面读的那个文件是两个独立的真相 —— 实测就是这么分家的
        #    （键已改成包内路径、读的还是 config/），而"键"看上去完全正确。
        "files": {
            repo_relative(robot_yaml): {
                "sha256": file_digest(robot_yaml),
                "kinematic_core_sha256": digest(k_core),
                "frozen_fields": {
                    "robot": list(KINEMATIC_ROBOT_KEYS),
                    "links": list(KINEMATIC_LINK_KEYS),
                    "joints": list(KINEMATIC_JOINT_KEYS),
                    "actuators": list(KINEMATIC_ACTUATOR_KEYS),
                },
                "excluded_from_freeze": list(EXCLUDED_FROM_KINEMATIC),
            },
            repo_relative(physics_yaml): {
                "sha256": file_digest(physics_yaml),
                "physics_core_sha256": digest(p_core),
                "frozen_sections": list(PHYSICS_SECTIONS),
                "excluded_from_freeze": ["recording", "deterministic"],
            },
        },
        "robot_id": rid,
        "kinematic_summary": k_core,
        "physics_summary": p_core,
    }


def _read_baseline(robot_id: str | None = None) -> dict | None:
    _robot, _physics, baseline = paths(robot_id)
    if not baseline.exists():
        return None
    with baseline.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def check(robot_id: str | None = None) -> list[str]:
    """返回违规清单（空 = 通过）。"""
    rid = robot_id or DEFAULT_ROBOT_ID
    _robot, _physics, baseline = paths(rid)
    base = _read_baseline(rid)
    if base is None:
        return [
            f"基线文件不存在：{baseline.relative_to(PROJECT_ROOT)}",
            f"第一次使用请先执行：python core/tools/freeze_baseline.py --robot {rid} --update",
        ]
    if base.get("robot_id") != rid:
        # 一份基线只装一台机器人的真值。问错了机器人时**必须显式说清**，
        # 否则会拿 A 的哈希去判 B，报出来的是"真值被改了"（假故障）。
        return [
            f"基线 {baseline.relative_to(PROJECT_ROOT)} 记的是 robot_id="
            f"{base.get('robot_id')!r}，问的却是 {rid!r} ⇒ 两者不是同一台机器人。"
        ]

    cur = snapshot(rid)
    problems: list[str] = []

    # ---- Level 2：语义核心（硬判据） ----
    # ⚠️ 路径**从声明取**（`_truth_rel_keys()`），不写字符串 —— 写死的那一版在真值
    #    随包搬迁之后仍指向旧路径，于是 `base["files"][旧路径]` 直接 KeyError，
    #    报错看着像"基线文件坏了"，真正的原因是这里自己过期了。
    for rel, key in _truth_rel_keys(rid):
        want = base["files"][rel][key]
        got = cur["files"][rel][key]
        if want != got:
            problems.append(
                f"{rel} 的{key} 已变："
                f"{want[:12]}… -> {got[:12]}…（**运动学/物理真值被改动**）"
            )

    if problems:
        problems += _diff_summary(base, cur)
        problems.append(
            "若这是**有意的**参数变更：请先跑 `python robot-package/mearm-v1/tools/gen_model.py` "
            "重新生成 MJCF（否则 mearm.xml 会与配置脱同步），再执行 "
            "`python core/tools/freeze_baseline.py --update` 重新冻结基线。"
        )
    return problems


def stale_level1_files(robot_id: str | None = None) -> list[str]:
    """整文件哈希变了、但**语义核心没变**的文件 —— 放行，只是记录该刷新了。

    典型情形：换外观几何 / 改注释 / 调素材挂载点。这些正是「视觉建模」要走的路，
    不该被判成违规；但也别让基线里的 L1 记录一直烂着。
    """
    base = _read_baseline(robot_id)
    if base is None:
        return []
    cur = snapshot(robot_id)
    rels = _truth_rel_keys(robot_id)
    for rel, key in rels:
        if base["files"][rel][key] != cur["files"][rel][key]:
            return []   # 语义核心已违规，不再重复提示
    return [
        rel for rel, _key in rels
        if base["files"][rel]["sha256"] != cur["files"][rel]["sha256"]
    ]


def _diff_summary(base: dict, cur: dict) -> list[str]:
    """把语义核心的差异逐条列出来 —— 只报"某哈希变了"帮不上任何忙。"""
    out: list[str] = []
    for section in ("kinematic_summary", "physics_summary"):
        want, got = base.get(section, {}), cur.get(section, {})
        for top in sorted(set(want) | set(got)):
            a, b = want.get(top), got.get(top)
            if a == b:
                continue
            if isinstance(a, list) and isinstance(b, list):
                # 标量列表（gravity = [0, 0, -9.81] / max_velocity = [..]）**不是** dict 列表，
                # 不能拿元素去 .get()。判据看首元素，别看 len()（空列表两可）。
                if not (a and isinstance(a[0], dict)):
                    out.append(f"  · {section}/{top}: {a!r} -> {b!r}")
                else:
                    for item_a, item_b in zip(a, b):
                        if item_a != item_b:
                            changed = {
                                k: (item_a.get(k), item_b.get(k))
                                for k in set(item_a) | set(item_b)
                                if item_a.get(k) != item_b.get(k)
                            }
                            ident = item_a.get("id") or item_a.get("name") or "?"
                            out.append(f"  · {section}/{top}/{ident}: {changed}")
                if len(a) != len(b):
                    out.append(f"  · {section}/{top}: 条目数 {len(a)} -> {len(b)}")
            else:
                out.append(f"  · {section}/{top}: {a!r} -> {b!r}")
    return out


def write_baseline(robot_id: str | None = None) -> Path:
    rid = robot_id or DEFAULT_ROBOT_ID
    _robot, _physics, baseline = paths(rid)
    snap = {
        "note": (
            "本文件是「运动学 + 物理」真值的冻结基线。判据是 *_core_sha256（语义核心），"
            "整文件 sha256 仅作记录。改外观几何（links[].geometry / details）不会触发失败；"
            "改 length / 限位 / 耦合 / 标定 / 质量 / 摩擦会。"
            "重新冻结：python core/tools/freeze_baseline.py --update"
        ),
        "frozen_at": _now(),
        **snapshot(rid),
    }
    baseline.parent.mkdir(parents=True, exist_ok=True)
    with baseline.open("w", encoding="utf-8") as fh:
        json.dump(snap, fh, ensure_ascii=False, indent=2, sort_keys=False)
        fh.write("\n")
    return baseline


def _now() -> str:
    from datetime import datetime
    return datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %z")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="冻结 / 校验运动学与物理真值基线")
    ap.add_argument("--robot", default=DEFAULT_ROBOT_ID,
                    help=f"要守卫的机器人 id（缺省 {DEFAULT_ROBOT_ID}，Golden Baseline）")
    ap.add_argument("--update", action="store_true", help="重新冻结基线（有意识改过参数后）")
    ap.add_argument("--show", action="store_true", help="打印当前语义核心摘要，不写文件")
    args = ap.parse_args(argv)
    rid = args.robot

    if args.show:
        snap = snapshot(rid)
        print(json.dumps(
            {"kinematic_summary": snap["kinematic_summary"],
             "physics_summary": snap["physics_summary"]},
            ensure_ascii=False, indent=2))
        return 0

    if args.update:
        path = write_baseline(rid)
        print(f"已冻结基线：{path.relative_to(PROJECT_ROOT)}  (robot_id={rid})")
        snap = snapshot(rid)
        # ⚠️ 打印时同样**从声明取键**，不写字符串 —— 写死的键在搬迁后会 KeyError，
        #    而那是"报告失败"，不是"守卫失败"，很容易被误读成真值被改了。
        robot_rel, physics_rel = _rel_names(rid)
        print(f"  {robot_rel}   语义核心 "
              f"{snap['files'][robot_rel]['kinematic_core_sha256'][:16]}…")
        print(f"  {physics_rel} 语义核心 "
              f"{snap['files'][physics_rel]['physics_core_sha256'][:16]}…")
        return 0

    problems = check(rid)
    if problems:
        print("✗ 运动学/物理真值偏离冻结基线：", file=sys.stderr)
        for line in problems:
            print(line, file=sys.stderr)
        return 1
    print("✓ 运动学与物理真值与冻结基线一致（未改动）")
    stale = stale_level1_files(rid)
    if stale:
        print(f"ℹ️ 整文件哈希已变、但语义核心未变（只动了外观几何/注释）：{', '.join(stale)}")
        print("   属于放行情形（视觉建模正需要）；可 --update 刷新记录。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
