#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 **MeArm-V1 黄金测试数据**（Golden Test Dataset）。

## 这是什么

`tests/baseline/mearm-v1/*.json` 是"MeArm-V1 冻结时的行为快照"。
本脚本负责采集，测试负责比对 —— 数据**必须**由本脚本产出，禁止手写。

## 为什么叫"黄金数据"而不是"期望值写死"

本项目的既有测试大多是「**当场算两遍、互相比对**」（例如 FK 参考实现 vs MuJoCo）。
那种测试能证明"两套实现对得上"，但**证明不了"今天的行为和上周一样"** ——
重构之后两套实现可能一起变，判据照样全绿。

黄金数据补的就是这一环：把当前行为**落盘**，之后每次跑都拿今天的实现去对昨天的记录。
于是「抽象前后行为一致」（spec §19）从一句口号变成一条可执行的断言。

## 采集原则（重要）

1. **期望值一律来自实跑，绝不手算。** FK 期望值来自**前端真实的 `fk.ts`**
   （经 `frontend/tests/tools/kinematics-bridge.mjs`，Vite SSR 加载同一份源码），
   不是 Python 重写的一份 —— 重写的那份只能证明"我写了两遍而且自洽"。
2. **固定 seed，全流程可复现。** 同一条命令任何时候跑都得到逐位相同的结果
   （唯一会变的是 `generated_at` 时间戳）。
3. **随机只作为补充。** 主数据集是**显式枚举**的定点用例（零位 / HOME / 各限位 / 中点 /
   角点），随机的 100 组是加料，不是唯一判据（spec §6）。

## 用法

```bash
cd MeArm-3D
<python> tools/gen_mearm_v1_baseline.py             # 写入 4 个 JSON
<python> tools/gen_mearm_v1_baseline.py --check     # 只校验"能否逐位复现"，不写文件
```

`--check` 是给 CI / 重构后自证用的：它重新采集一遍并与**已提交的文件**逐位比对。
逐位相同 ⇒ 当前实现与冻结时完全一致；有任何差异都会打印出来。

> ⚠️ 需要同时具备：node（跑前端运动学桥）+ mujoco（跑物理侧 FK）。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "simulation" / "mujoco"))
sys.path.insert(0, str(PROJECT_ROOT / "tests" / "sim"))

from fkref import fk_joint_origins_mm, fk_tcp_mm          # noqa: E402
from ikbridge import KinematicsBridge                      # noqa: E402
from model import MeArmSim                                 # noqa: E402
from robotcfg import (                                     # noqa: E402
    load_physics,
    load_robot_by_id,
    resolve_robot_entry,
)
from units import ensure_utf8_stdout                       # noqa: E402

ensure_utf8_stdout()

OUT_DIR = PROJECT_ROOT / "tests" / "baseline" / "mearm-v1"
GENERATOR = "tools/gen_mearm_v1_baseline.py"

#: 本工具**只**负责 MeArm-V1。刻意显式写出 id，不读选择器的 `default` ——
#: 否则将来改 `default` 会让这套黄金数据**静默**换成另一台机器人的行为快照
#: （文件名还叫 mearm-v1，内容却已经不是了），而 `--check` 照样能绿。
#: 与前端 22 处 `loadRobotModel('mearm-v1')`、`tests/sim/conftest.py` 是同一条纪律。
MEARM_V1 = "mearm-v1"

#: 固定 seed —— 换它等于换了一整套数据，必须同时重新生成 4 个文件
SEED = 20260914
#: 落盘的随机用例数（spec §18 的 500 / 1000 组由测试按同一 seed 现场生成做一致性扫描，
#: 不入库 —— 那些是"实现内部自洽"检查，不需要冻结具体数值）
N_RANDOM_JOINTS = 100
N_RANDOM_IK = 100

#: 落盘数值的小数位。12 位远严于所有容差（最小容差 1e-9 mm），
#: 却能让 JSON 稳定、diff 友好 —— 浮点尾数的最后几位本来就是平台相关的噪声。
DECIMALS = 12


# ---------------------------------------------------------------------------
# 数值规整
# ---------------------------------------------------------------------------

def r(x: float) -> float:
    """定点小数化。`-0.0 → 0.0`（否则 JSON 里会出现刺眼的负零，且影响逐位比对）。"""
    v = round(float(x), DECIMALS)
    return 0.0 if v == 0 else v


def rv(v) -> list[float]:
    return [r(x) for x in v]


def roundtrip(obj):
    """递归把全部浮点规整到 DECIMALS —— 保证"写出去的"等于"读回来的"。"""
    if isinstance(obj, float):
        return r(obj)
    if isinstance(obj, dict):
        return {k: roundtrip(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [roundtrip(v) for v in obj]
    return obj


# ---------------------------------------------------------------------------
# 1. 用例集构造（全部确定性）
# ---------------------------------------------------------------------------

def joint_cases(robot) -> list[dict]:
    """显式枚举的关节位形用例集。

    覆盖 spec §4.2 要求的五类：**零位 / 各关节限位 / 中间姿态 / 合法随机 / 边界姿态**。
    """
    movable = robot.movable_joints()
    home = {j.id: float(robot.home_pose[j.id]) for j in movable}
    cases: list[dict] = []

    def add(cid: str, label: str, joints: dict[str, float]) -> None:
        cases.append({
            "id": cid,
            "label": label,
            "joints": {k: r(v) for k, v in joints.items()},
        })

    # ① 零位（⚠️ elbow 的 0° 在真机限位外，但 FK/纯运动学求值仍然成立 ——
    #    它是一条**独立于任何 FK 实现**的几何常量核对：竖直段 = 60+80+80，水平段 = 40）
    add("zero", "关节空间原点（全 0°；elbow 超限但 FK 可求值，用于几何常量核对）",
        {j.id: 0.0 for j in movable})

    # ② HOME（固件 RESET 位 = 四舵机 90°）
    add("home", "HOME 位（固件 RESET 位）", home)

    # ③ 单关节扫到两端点 / 中点（其余关节固定在 HOME）
    for j in movable:
        for tag, val in (("min", j.limit_min), ("mid", (j.limit_min + j.limit_max) / 2),
                         ("max", j.limit_max)):
            add(f"{j.id}_{tag}", f"{j.id} = {tag}（其余关节取 HOME）",
                {**home, j.id: float(val)})

    # ④ 边界角点（全最小 / 全最大）
    add("all_min", "全部关节取限位下限", {j.id: float(j.limit_min) for j in movable})
    add("all_max", "全部关节取限位上限", {j.id: float(j.limit_max) for j in movable})

    # ⑤ 合法随机（固定 seed）
    rng = np.random.default_rng(SEED)
    for i in range(N_RANDOM_JOINTS):
        add(f"rand_{i:03d}", f"合法随机位形 #{i}（seed={SEED}）",
            {j.id: float(rng.uniform(j.limit_min, j.limit_max)) for j in movable})

    return cases


def _target_cases(robot, cases: list[dict]) -> list[dict]:
    """IK / 工作空间用例的目标点集。

    目标点分三类，全部**由实现自己产生**（不手挑坐标，避免把"我猜它能到"写进基线）：
      ① 可达：取合法关节位形的 FK 结果 ⇒ 由构造保证可达
      ② 显式越界：远离工作空间的点
      ③ 边界邻域：沿可达目标的径向外推，用来钉住"越界判据在哪一格翻转"
    """
    returns: list[dict] = []
    home = {j.id: float(robot.home_pose[j.id]) for j in robot.movable_joints()}

    for c in cases:
        returns.append({"id": f"target_{c['id']}", "label": f"可达目标 ← {c['label']}",
                        "kind": "reachable", "source": c["id"], "joints": c["joints"]})

    # 显式越界
    for cid, pt, why in (
        ("far_x", [500.0, 0.0, 100.0], "远超连杆总长"),
        ("far_diagonal", [300.0, 300.0, 300.0], "三轴同时远离"),
        ("below_floor", [100.0, 0.0, -200.0], "远低于底座"),
    ):
        returns.append({"id": cid, "label": f"越界目标：{why}", "kind": "out_of_workspace",
                        "target": rv(pt), "joints": home})

    # 底座方位角越界（几何可达、但 base 限位 ±60° 不允许）
    returns.append({"id": "azimuth_90", "label": "方位角 90°（超出 base ±60°）",
                    "kind": "joint_limit", "target": [0.0, 150.0, 100.0], "joints": home})
    returns.append({"id": "azimuth_180", "label": "方位角 180°（正后方）",
                    "kind": "joint_limit", "target": [-150.0, 0.0, 100.0], "joints": home})

    return returns


# ---------------------------------------------------------------------------
# 2. 采集
# ---------------------------------------------------------------------------

def _fk_side(robot, joints: dict[str, float], sim: MeArmSim) -> dict:
    """同一组关节角的**三个独立侧面**：参考实现 / MuJoCo / （前端由桥另给）。

    ⚠️ 走 `reset()` 而**不是** `settle()`：`reset()` 内部只做 `mj_forward`，
    是**纯运动学求值**，不推进物理、不引入重力/接触/求解器迭代的噪声。
    需要"几何定义层面"的判据时（FK/IK 一致性就是），必须走这条路 ——
    否则断言会变成"在某个动力学平衡态附近成立"，边界处随机飘。
    """
    sim.reset(dict(joints))
    return {
        "tcpRef": rv(fk_tcp_mm(robot, joints)),
        "tcpMujoco": rv(sim.end_effector_mm()),
        "framesRef": {jid: rv(p) for jid, p in fk_joint_origins_mm(robot, joints).items()},
    }


def collect(robot, physics, bridge: KinematicsBridge) -> dict[str, dict]:
    sim = MeArmSim(robot=robot, physics=physics)

    jc = joint_cases(robot)

    # ---- FK：前端（桥）------------------------------------------------------
    fk_res = bridge.forward([{"id": i, "joints": c["joints"]} for i, c in enumerate(jc)])
    fk_by_idx = {int(x["id"]): x for x in fk_res}

    fk_cases_out = []
    for i, c in enumerate(jc):
        got = fk_by_idx.get(i)
        if not got or not got.get("success"):
            raise RuntimeError(f"前端 FK 对用例 {c['id']} 求值失败：{got}")
        side = _fk_side(robot, c["joints"], sim)
        fk_cases_out.append({
            "id": c["id"],
            "label": c["label"],
            "joints": c["joints"],
            "tcpFrontend": rv(got["tcp"]),
            "tcpFrontendRotation": rv(got["tcpRotation"]),
            "framesFrontend": {jid: rv(tf["position"]) for jid, tf in got["frames"].items()},
            **side,
        })

    # ---- IK ----------------------------------------------------------------
    jc_index = {c["id"]: i for i, c in enumerate(jc)}
    tcs = _target_cases(robot, jc)
    ik_req = []
    for i, t in enumerate(tcs):
        target = t.get("target")
        if target is None:
            # 可达目标：先由前端 FK 把关节角换成目标点（与"目标由实现产生"一致）
            target = fk_by_idx[jc_index[t["source"]]]["tcp"]
        ik_req.append({"id": i, "target": [float(v) for v in target]})
    ik_res = bridge.solve(ik_req)
    ik_by_idx = {int(x["id"]): x for x in ik_res}

    ik_cases_out = []
    for i, t in enumerate(tcs):
        got = ik_by_idx.get(i)
        if got is None:
            raise RuntimeError(f"前端 IK 对用例 {t['id']} 无结果")
        target = ik_req[i]["target"]
        entry = {
            "id": t["id"],
            "label": t["label"],
            "kind": t["kind"],
            "target": rv(target),
            "expect": {"success": bool(got.get("success"))},
        }
        if got.get("success"):
            entry["expect"].update({
                "branch": got.get("branch"),
                "joints": {k: r(v) for k, v in got["joints"].items()},
                "residual": r(got.get("residual", 0.0)),
                "azimuth": r(got.get("azimuth", 0.0)),
                "relativeAngle": r(got.get("relativeAngle", 0.0)),
                "tcpFrontend": rv(got.get("tcpFrontend", [])),
            })
            # 同一解在 MuJoCo 里落到哪 —— 这是 §17.5 的期望值
            sim.reset({k: float(v) for k, v in got["joints"].items()})
            entry["expect"]["tcpMujoco"] = rv(sim.end_effector_mm())
        else:
            entry["expect"].update({
                "reason": got.get("reason"),
                "joint": got.get("joint"),
                "message": got.get("message"),
                "candidateCount": len(got.get("candidates") or []),
            })
        ik_cases_out.append(entry)

    # ---- 工作空间 ----------------------------------------------------------
    info = bridge.model_info()
    g = info["geometry"]
    from harness import sagittal_distance_mm  # noqa: E402  （复用既有判据换算，不另写一份）

    ws_cases_out = []
    for i, t in enumerate(tcs):
        got = ik_by_idx[i]
        target = ik_req[i]["target"]
        ws_cases_out.append({
            "id": t["id"],
            "label": t["label"],
            "kind": t["kind"],
            "target": rv(target),
            "sagittalDistance": r(sagittal_distance_mm(info, target)),
            "reach": rv(g["reach"]),
            "geometry": {
                "pivotZ": r(g["pivotZ"]), "pivotR": r(g["pivotR"]),
                "l1": r(g["l1"]), "l2": r(g["l2"]),
                "toolOffset": rv(g["toolOffset"]),
            },
            "expect": {
                "success": bool(got.get("success")),
                "reason": got.get("reason") if not got.get("success") else None,
            },
        })

    return {"fk": fk_cases_out, "ik": ik_cases_out, "ws": ws_cases_out}


# ---------------------------------------------------------------------------
# 3. 组装 / 落盘
# ---------------------------------------------------------------------------

def _meta(robot, payload_key: str, extra: dict | None = None) -> dict:
    return {
        "model": robot.model,
        "modelVersion": robot.model_version,
        "robotId": robot.id,
        "robotName": robot.name,
        "seed": SEED,
        "generator": GENERATOR,
        "note": (
            "本文件是 MeArm-V1 冻结时的**行为快照**（characterization baseline）。"
            "全部数值由 " + GENERATOR + " 从当时的实现**实跑采集**，不是手算、不是推测。"
            "它只作回归判据：抽象/重构前后必须逐位一致；"
            "有意改变行为时必须重新生成并说明原因。"
            f"本文件负责的载荷：{payload_key}"
        ),
        **(extra or {}),
    }


def build_documents(robot, collected: dict) -> dict[str, dict]:
    fk = collected["fk"]
    ik = collected["ik"]
    ws = collected["ws"]

    joint_doc = _meta(robot, "关节位形用例集（FK/Three.js/MuJoCo 三条判据的公共输入）", {
        "count": len(fk),
        "categories": {
            "zero/home": 2,
            "per-joint min/mid/max": 3 * len(robot.movable_joints()),
            "boundary corners": 2,
            "seeded random": N_RANDOM_JOINTS,
        },
        "cases": [{"id": c["id"], "label": c["label"], "joints": c["joints"]} for c in fk],
    })

    fk_doc = _meta(robot, "Joint → FK（前端 fk.ts / 参考实现 / MuJoCo 三侧）", {
        "tolerances": {
            "frontend_vs_recorded_mm": 1e-9,
            "ref_vs_mujoco_mm": 1e-6,
            "threejs_vs_frontend_matrix": 1e-6,
        },
        "cases": fk,
    })

    ik_doc = _meta(robot, "XYZ → IK →（FK / MuJoCo）", {
        "tolerances": {"closed_loop_mm": 1e-9},
        "cases": ik,
    })

    ws_doc = _meta(robot, "工作空间判定（可达性 / 错误码 / 2R 几何）", {
        "cases": ws,
    })

    return {
        "joint_cases.json": roundtrip(joint_doc),
        "fk_cases.json": roundtrip(fk_doc),
        "ik_cases.json": roundtrip(ik_doc),
        "workspace_cases.json": roundtrip(ws_doc),
    }


def write_documents(docs: dict[str, dict]) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name, doc in docs.items():
        path = OUT_DIR / name
        path.write_text(
            json.dumps(doc, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        print(f"  写入 {path.relative_to(PROJECT_ROOT)}  ({path.stat().st_size} B)")


def check_documents(docs: dict[str, dict]) -> int:
    """与已提交文件逐位比对（忽略 generated_at）。"""
    bad = 0
    for name, doc in docs.items():
        path = OUT_DIR / name
        if not path.is_file():
            print(f"  ✗ 缺少 {name}，请先不带 --check 跑一次")
            bad += 1
            continue
        old = json.loads(path.read_text(encoding="utf-8"))
        for key in ("generated_at",):
            old.pop(key, None)
            doc.pop(key, None)
        if old == doc:
            print(f"  ✓ {name} 逐位一致（当前实现 == 冻结基线）")
        else:
            print(f"  ✗ {name} 与冻结基线**不一致** —— 实现行为已变")
            bad += 1
            _report_diff(old, doc)
    return bad


def _report_diff(old: dict, new: dict, prefix: str = "") -> None:
    """把差异逐条列出来 —— 只说"某处不同"帮不上任何忙。"""
    shown = 0
    if isinstance(old, dict) and isinstance(new, dict):
        for k in sorted(set(old) | set(new)):
            if shown > 20:
                print("    …（更多差异已省略）")
                return
            a, b = old.get(k, "<缺失>"), new.get(k, "<缺失>")
            if a == b:
                continue
            if isinstance(a, (dict, list)) and isinstance(b, (dict, list)):
                if isinstance(a, list) and isinstance(b, list) and len(a) != len(b):
                    print(f"    · {prefix}{k}: 条目数 {len(a)} -> {len(b)}")
                    shown += 1
                    continue
                _report_diff(a, b, f"{prefix}{k}.")
            else:
                print(f"    · {prefix}{k}: {a!r} -> {b!r}")
                shown += 1
    elif isinstance(old, list) and isinstance(new, list):
        for i, (a, b) in enumerate(zip(old, new)):
            if a != b:
                _report_diff(a, b, f"{prefix}[{i}].")
                shown += 1
                if shown > 20:
                    return


# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="生成 / 校验 MeArm-V1 黄金测试数据")
    ap.add_argument("--check", action="store_true",
                    help="只校验当前实现能否逐位复现已提交的基线（不写文件）")
    args = ap.parse_args(argv)

    robot = load_robot_by_id(MEARM_V1)
    physics = load_physics(resolve_robot_entry(MEARM_V1).physics_file)
    print(f"[gen-baseline] 模型 = {robot.model} v{robot.model_version} "
          f"(id={robot.id} · selector id={MEARM_V1}) · 可动关节 = {robot.joint_order()}")

    with KinematicsBridge(robot_id=MEARM_V1) as bridge:
        # 桥自检：加载到的模型必须就是 robot.yaml 里的那一个
        info = bridge.model_info()
        if info.get("model") != robot.model:
            print(f"  ✗ 前端加载到的模型 '{info.get('model')}' != robot.yaml 的 '{robot.model}'")
            return 1
        collected = collect(robot, physics, bridge)

    docs = build_documents(robot, collected)

    if args.check:
        print("[gen-baseline] --check：与已提交基线逐位比对")
        bad = check_documents(docs)
        print("✓ 当前实现与 MeArm-V1 冻结基线逐位一致" if not bad
              else f"✗ 发现 {bad} 个文件与基线不一致")
        return 1 if bad else 0

    from datetime import datetime
    stamp = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %z")
    for doc in docs.values():
        doc["generated_at"] = stamp

    print(f"[gen-baseline] 采集完成：FK {len(collected['fk'])} 例 · "
          f"IK {len(collected['ik'])} 例 · 工作空间 {len(collected['ws'])} 例")
    write_documents(docs)
    print(f"[gen-baseline] 完成（seed={SEED}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
