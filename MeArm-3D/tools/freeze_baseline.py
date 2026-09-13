#!/usr/bin/env python3
"""冻结 / 校验「运动学 + 物理」真值基线。

背景
----
用户要求：「保存目前的运动学物理数据不变」。字面做法是给两份 yaml 存一份快照 +
一个哈希守卫；但**整文件哈希是个错误的判据**，因为 `config/robot.yaml` 同时装了两类
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
    python tools/freeze_baseline.py            # 校验（CI / 测试用），不符合退出码 1
    python tools/freeze_baseline.py --update   # 重新冻结（有意识地改过参数后）
    python tools/freeze_baseline.py --show     # 打印当前核心摘要，不写文件
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
ROBOT_YAML = PROJECT_ROOT / "config" / "robot.yaml"
PHYSICS_YAML = PROJECT_ROOT / "config" / "physics.yaml"
BASELINE = PROJECT_ROOT / "config" / "baseline-kinematics-physics.json"

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


def snapshot() -> dict:
    """当前状态的完整快照（L1 文件哈希 + L2 语义核心 + 人类可读摘要）。"""
    robot_doc = _load_yaml(ROBOT_YAML)
    physics_doc = _load_yaml(PHYSICS_YAML)
    k_core = robot_kinematic_core(robot_doc)
    p_core = physics_core(physics_doc)
    return {
        "files": {
            "config/robot.yaml": {
                "sha256": file_digest(ROBOT_YAML),
                "kinematic_core_sha256": digest(k_core),
                "frozen_fields": {
                    "robot": list(KINEMATIC_ROBOT_KEYS),
                    "links": list(KINEMATIC_LINK_KEYS),
                    "joints": list(KINEMATIC_JOINT_KEYS),
                    "actuators": list(KINEMATIC_ACTUATOR_KEYS),
                },
                "excluded_from_freeze": list(EXCLUDED_FROM_KINEMATIC),
            },
            "config/physics.yaml": {
                "sha256": file_digest(PHYSICS_YAML),
                "physics_core_sha256": digest(p_core),
                "frozen_sections": list(PHYSICS_SECTIONS),
                "excluded_from_freeze": ["recording", "deterministic"],
            },
        },
        "kinematic_summary": k_core,
        "physics_summary": p_core,
    }


def _read_baseline() -> dict | None:
    if not BASELINE.exists():
        return None
    with BASELINE.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def check() -> list[str]:
    """返回违规清单（空 = 通过）。"""
    base = _read_baseline()
    if base is None:
        return [
            f"基线文件不存在：{BASELINE.relative_to(PROJECT_ROOT)}",
            "第一次使用请先执行：python tools/freeze_baseline.py --update",
        ]

    cur = snapshot()
    problems: list[str] = []

    # ---- Level 2：语义核心（硬判据） ----
    for rel, keys in (
        ("config/robot.yaml", ("kinematic_core_sha256",)),
        ("config/physics.yaml", ("physics_core_sha256",)),
    ):
        for key in keys:
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
            "若这是**有意的**参数变更：请先跑 `python simulation/mujoco/gen_model.py` "
            "重新生成 MJCF（否则 mearm.xml 会与配置脱同步），再执行 "
            "`python tools/freeze_baseline.py --update` 重新冻结基线。"
        )
    return problems


def stale_level1_files() -> list[str]:
    """整文件哈希变了、但**语义核心没变**的文件 —— 放行，只是记录该刷新了。

    典型情形：换外观几何 / 改注释 / 调素材挂载点。这些正是「视觉建模」要走的路，
    不该被判成违规；但也别让基线里的 L1 记录一直烂着。
    """
    base = _read_baseline()
    if base is None:
        return []
    cur = snapshot()
    for rel, key in (
        ("config/robot.yaml", "kinematic_core_sha256"),
        ("config/physics.yaml", "physics_core_sha256"),
    ):
        if base["files"][rel][key] != cur["files"][rel][key]:
            return []   # 语义核心已违规，不再重复提示
    return [
        rel for rel in ("config/robot.yaml", "config/physics.yaml")
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


def write_baseline() -> Path:
    snap = snapshot()
    snap = {
        "note": (
            "本文件是「运动学 + 物理」真值的冻结基线。判据是 *_core_sha256（语义核心），"
            "整文件 sha256 仅作记录。改外观几何（links[].geometry / details）不会触发失败；"
            "改 length / 限位 / 耦合 / 标定 / 质量 / 摩擦会。"
            "重新冻结：python tools/freeze_baseline.py --update"
        ),
        "frozen_at": _now(),
        **snap,
    }
    with BASELINE.open("w", encoding="utf-8") as fh:
        json.dump(snap, fh, ensure_ascii=False, indent=2, sort_keys=False)
        fh.write("\n")
    return BASELINE


def _now() -> str:
    from datetime import datetime
    return datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %z")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="冻结 / 校验运动学与物理真值基线")
    ap.add_argument("--update", action="store_true", help="重新冻结基线（有意识改过参数后）")
    ap.add_argument("--show", action="store_true", help="打印当前语义核心摘要，不写文件")
    args = ap.parse_args(argv)

    if args.show:
        snap = snapshot()
        print(json.dumps(
            {"kinematic_summary": snap["kinematic_summary"],
             "physics_summary": snap["physics_summary"]},
            ensure_ascii=False, indent=2))
        return 0

    if args.update:
        path = write_baseline()
        print(f"已冻结基线：{path.relative_to(PROJECT_ROOT)}")
        snap = snapshot()
        print(f"  robot.yaml   语义核心 {snap['files']['config/robot.yaml']['kinematic_core_sha256'][:16]}…")
        print(f"  physics.yaml 语义核心 {snap['files']['config/physics.yaml']['physics_core_sha256'][:16]}…")
        return 0

    problems = check()
    if problems:
        print("✗ 运动学/物理真值偏离冻结基线：", file=sys.stderr)
        for line in problems:
            print(line, file=sys.stderr)
        return 1
    print("✓ 运动学与物理真值与冻结基线一致（未改动）")
    stale = stale_level1_files()
    if stale:
        print(f"ℹ️ 整文件哈希已变、但语义核心未变（只动了外观几何/注释）：{', '.join(stale)}")
        print("   属于放行情形（视觉建模正需要）；可 --update 刷新记录。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
