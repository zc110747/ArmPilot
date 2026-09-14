#!/usr/bin/env python3
"""SO-ARM101 物理参数快照校验器（Audit Snapshot Checker）。

## 它解决什么问题

`config/robots/so-arm101/physics.yaml` 的 `observed` 段是「官方 MJCF 生效物理量的
指纹」。指纹有一个必然的失效方式：

    官方模型换了版本 / 有人手改了它 → 指纹变成**过期的断言**，但没人会知道。

所以指纹必须是**可复核的**，而不是手抄一遍就算数。本脚本就是那把尺子：

    python tools/inspect_so101_physics.py --check    # 逐值复核，有差异则 exit 1
    python tools/inspect_so101_physics.py --emit     # 重新生成 observed 段

## 为什么"读出来的"比"写下来的"可信

本脚本从**已加载的 MjModel**里读值，而不是解析 XML 文本。两者的区别不是洁癖：

  · XML 里 `<default class="sts3215">` 写的 forcerange 是 ±2.94，
    而 6 个 `<position>` **逐个覆盖**成 ±3.35 ⇒ **文本会骗你，MjModel 不会**。
  · 官方 MJCF 没有 `<option>` 段 ⇒ timestep 等值根本不在文本里，
    只有 MjModel 才知道"生效值 = MuJoCo 缺省 0.002"。

⇒ 所以校验对象是"引擎实际会用的那个模型"，与 Phase 6 的仿真完全同源。

## 使用

    PY=~/.workbuddy/binaries/python/envs/default/Scripts/python.exe
    PYTHONIOENCODING=utf-8 "$PY" tools/inspect_so101_physics.py --check
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
MODEL_XML = REPO / "assets" / "models" / "so-arm101" / "official" / "so101_new_calib.xml"
PHYSICS_YAML = REPO / "config" / "robots" / "so-arm101" / "physics.yaml"

# MjModel 的枚举值 → 人类可读名（只映射我们要断言的那几个）
INTEGRATORS = {0: "Euler", 1: "RK4", 2: "implicit", 3: "implicitfast"}
SOLVERS = {0: "PGS", 1: "CG", 2: "Newton"}
CONES = {0: "pyramidal", 1: "elliptic"}
JACOBIANS = {0: "dense", 1: "sparse", 2: "auto"}

# 浮点比较容差。物理量本身是官方满精度值，容差只用来吸收 YAML 往返的表示误差。
TOL = 1e-9


def build_observed() -> dict:
    """加载官方 MJCF，读出**生效**物理量，构造成与 physics.yaml `observed` 同构的 dict。"""
    import mujoco

    m = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    o = m.opt

    # ---- 关节侧：官方 6 个 hinge 全部使用同一个 class ⇒ 断言"全部一致"比逐个列表更严 ----
    dampings, frictions, armatures = set(), set(), set()
    for j in range(m.njnt):
        dof = m.jnt_dofadr[j]
        dampings.add(round(float(m.dof_damping[dof]), 9))
        frictions.add(round(float(m.dof_frictionloss[dof]), 9))
        armatures.add(round(float(m.dof_armature[dof]), 9))

    # ---- 执行器侧：同理，断言"6 个执行器完全同参"，并单独记录被逐个覆盖的 forcerange ----
    kps, kvs, forceranges, gears = set(), set(), set(), set()
    for a in range(m.nu):
        # position 执行器：gainprm[0] = kp，biasprm[1] = -kp，biasprm[2] = -kv
        kps.add(round(float(m.actuator_gainprm[a][0]), 9))
        kvs.add(round(float(-m.actuator_biasprm[a][2]), 9))
        forceranges.add(
            (round(float(m.actuator_forcerange[a][0]), 9), round(float(m.actuator_forcerange[a][1]), 9))
        )
        gears.add(round(float(m.actuator_gear[a][0]), 9))

    # ---- 几何分组：按 (contype, conaffinity, group) 计数 ----
    groups: dict[tuple[int, int, int], int] = {}
    for g in range(m.ngeom):
        key = (int(m.geom_contype[g]), int(m.geom_conaffinity[g]), int(m.geom_group[g]))
        groups[key] = groups.get(key, 0) + 1

    def group_entry(key: tuple[int, int, int], role: str) -> dict:
        return {"contype": key[0], "conaffinity": key[1], "group": key[2], "count": groups[key], "role": role}

    visual_key = (0, 0, 2)
    collision_key = (1, 1, 3)
    # 万一官方换了分组，直接报错而不是产出一个"看起来正常"的空条目
    missing = [k for k in (visual_key, collision_key) if k not in groups]
    if missing:
        raise SystemExit(
            f"[inspect_so101_physics] 官方 MJCF 的 geom 分组与预期不符，缺少 {missing}；"
            f"实际分组: {sorted(groups)}。请人工确认后再更新本脚本的期望值。"
        )

    # ---- 质量：逐 body（跳过 world）----
    masses: dict[str, float] = {}
    for b in range(m.nbody):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
        if name == "world":
            continue
        masses[name] = round(float(m.body_mass[b]), 9)

    # 关节侧/执行器侧的"唯一性"是断言，不是描述 ⇒ 不唯一就说明官方把 class 拆开了，
    # 那时 physics.yaml 的单一数值写法不再成立，必须当场失败。
    if len(dampings) != 1 or len(frictions) != 1 or len(armatures) != 1:
        raise SystemExit(
            f"[inspect_so101_physics] 关节参数在 6 个关节间不再一致 "
            f"(damping={sorted(dampings)} frictionloss={sorted(frictions)} armature={sorted(armatures)})；"
            "physics.yaml 的单值写法需改成逐关节表。"
        )
    if len(kps) != 1 or len(kvs) != 1 or len(forceranges) != 1 or len(gears) != 1:
        raise SystemExit(
            f"[inspect_so101_physics] 执行器参数不再一致 "
            f"(kp={sorted(kps)} kv={sorted(kvs)} forcerange={sorted(forceranges)} gear={sorted(gears)})。"
        )

    return {
        "option": {
            "timestep": round(float(o.timestep), 9),
            "integrator": INTEGRATORS[int(o.integrator)],
            "solver": SOLVERS[int(o.solver)],
            "iterations": int(o.iterations),
            "tolerance": float(o.tolerance),
            "gravity": [round(float(v), 9) for v in o.gravity],
            "cone": CONES[int(o.cone)],
            "jacobian": JACOBIANS[int(o.jacobian)],
        },
        "model_size": {
            "nq": int(m.nq),
            "nv": int(m.nv),
            "nu": int(m.nu),
            "nbody": int(m.nbody),
            "ngeom": int(m.ngeom),
            "nsite": int(m.nsite),
            "nmesh": int(m.nmesh),
        },
        "joints": {
            "damping": sorted(dampings)[0],
            "frictionloss": sorted(frictions)[0],
            "armature": sorted(armatures)[0],
            "type": "hinge",
        },
        "actuators": {
            "kind": "position",
            "kp": sorted(kps)[0],
            "kv": sorted(kvs)[0],
            "forcerange": list(sorted(forceranges)[0]),
            "gear": sorted(gears)[0],
        },
        "geom_groups": [group_entry(visual_key, "visual"), group_entry(collision_key, "collision")],
        "geom_friction": [round(float(v), 9) for v in m.geom_friction[0]],
        "inertia": {
            "mode": "official_full_inertia",
            "bodies": masses,
            "total_mass": round(sum(masses.values()), 9),
        },
    }


def numeric_view(observed: dict) -> dict:
    """把 observed 段规约成"可数值比较"的键值对（跳过描述性字段）。"""
    flat: dict[str, object] = {}

    def walk(prefix: str, node) -> None:
        if isinstance(node, dict):
            for key, value in node.items():
                # role / kind / ctrlrange_source 这类是**说明**，不是可复核的数值
                if key in {"role", "kind", "ctrlrange_source"}:
                    continue
                walk(f"{prefix}.{key}" if prefix else str(key), value)
        elif isinstance(node, list):
            for index, value in enumerate(node):
                walk(f"{prefix}[{index}]", value)
        else:
            flat[prefix] = node

    walk("", observed)
    return flat


def compare(expected: dict, actual: dict) -> list[str]:
    """逐键比较，返回人类可读的差异列表（空 = 完全一致）。"""
    exp, act = numeric_view(expected), numeric_view(actual)
    diffs: list[str] = []

    for key in sorted(set(exp) | set(act)):
        if key not in exp:
            diffs.append(f"  + {key}: 快照里缺失，实际 = {act[key]!r}")
            continue
        if key not in act:
            diffs.append(f"  - {key}: 快照里有 {exp[key]!r}，但模型里读不到")
            continue
        a, b = exp[key], act[key]
        if isinstance(a, float) and isinstance(b, float):
            if not math.isclose(a, b, rel_tol=0.0, abs_tol=TOL):
                diffs.append(f"  ~ {key}: 快照 {a!r} != 实际 {b!r}")
        elif a != b:
            diffs.append(f"  ~ {key}: 快照 {a!r} != 实际 {b!r}")

    return diffs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--check", action="store_true", help="逐值复核 physics.yaml 的 observed 段")
    group.add_argument("--emit", action="store_true", help="打印重新生成的 observed 段（YAML）")
    group.add_argument("--json", action="store_true", help="打印实测值（JSON，便于人眼核对）")
    args = parser.parse_args()

    actual = build_observed()

    if args.json:
        print(json.dumps(actual, ensure_ascii=False, indent=2))
        return 0

    if args.emit:
        print(yaml.safe_dump({"observed": actual}, allow_unicode=True, sort_keys=False, default_flow_style=False))
        return 0

    # --check
    if not PHYSICS_YAML.exists():
        print(f"[FAIL] 找不到 {PHYSICS_YAML.relative_to(REPO)}", file=sys.stderr)
        return 1

    doc = yaml.safe_load(PHYSICS_YAML.read_text(encoding="utf-8"))
    expected = (doc or {}).get("observed")
    if not isinstance(expected, dict):
        print(f"[FAIL] {PHYSICS_YAML.name} 缺少 observed 段", file=sys.stderr)
        return 1

    diffs = compare(expected, actual)
    if diffs:
        print(f"[FAIL] SO-ARM101 物理快照与官方 MJCF 不一致（{len(diffs)} 处）：")
        print("\n".join(diffs))
        print(
            "\n处理方式（选一个，别只改一半）：\n"
            "  ① 若官方模型**不该变** ⇒ 说明资产被误改，回滚 assets/models/so-arm101/official/\n"
            "  ② 若官方确实升级了 ⇒ 重跑 `--emit` 更新 physics.yaml，并在 SOURCE.md 记录新 commit/sha256\n"
            "  ⚠️ 禁止为了让本检查通过而改官方 MJCF —— 那正是本检查要防的事。"
        )
        return 1

    n = len(numeric_view(expected))
    print(f"[OK] SO-ARM101 物理快照与官方 MJCF 一致（{n} 个数值逐项吻合）。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
