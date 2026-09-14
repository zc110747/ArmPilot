# -*- coding: utf-8 -*-
"""Sim2Sim 回归 · **MuJoCo 侧** —— MeArm-V1 黄金基线（spec §16–§19 / §17.4 / §17.5）。

## 这个文件回答的问题

> 「抽象之后，MeArm-V1 的行为必须与抽象之前一致。」（spec §19）

判据不是"新架构自己测试通过"，而是 **旧基线 VS 新实现**：
期望值来自 `tests/baseline/mearm-v1/*.json`（由 `tools/gen_mearm_v1_baseline.py`
在冻结时**实跑采集**），这里拿今天的实现去对。

## 与前端那个文件的分工

|                     | `frontend/tests/sim2sim/mearm-v1-baseline.test.ts` | 本文件 |
|---|---|---|
| 参考面              | `fk.ts` / `ik.ts` / Three.js `matrixWorld`         | MuJoCo 编译后的 body 树 |
| 基线文件            | 同一批 `tests/baseline/mearm-v1/*.json`            | 同一批 |
| IK 由谁解           | 前端 `ik.ts`（被测即参考）                          | 前端 `ik.ts`（经桥，Python 侧不解释语义） |

两边**必须读同一批基线文件**：否则「一致」会被拆成两套互不相干的标准，
而两边各自都能自己绿。

## 覆盖（spec §17）

* **§17.4 FK ↔ MuJoCo** —— 同一个关节状态：`q ├─ fkref ─┐`，比较 **TCP** 与
  **每个关节坐标系原点**（含被动腕 `tool` 与叶关节 `gripper`）。
* **§17.5 IK ↔ MuJoCo** —— `target XYZ → IK → q → MuJoCo → TCP'`，验证最终位置误差。
* **§18 固定 seed 扫描** —— 500 组 `Joint → MuJoCo`、1000 组 `XYZ → IK → MuJoCo`。

## 关于容差

**全部从基线文件里读**（`doc["tolerances"]`），不在测试文件里另立一套 ——
基线自带的 `frontend_vs_recorded_mm = 1e-9` / `ref_vs_mujoco_mm = 1e-6` /
`closed_loop_mm = 1e-9` 就是契约。落盘数值保留 12 位小数 ⇒ 复算与记录之间
最多差 5e-13，1e-9 有 3 个数量级余量。**只允许比基线更严，不允许更松。**
"""
from __future__ import annotations

import numpy as np
import pytest

from fkref import fk_joint_origins_mm, fk_tcp_mm
from harness import (
    MM_TOL,
    case_joints,
    mujoco_joint_origin_mm,
    mujoco_tcp_mm,
    random_pose,
    sagittal_distance_mm,
)
from mearmV1Baseline import SEED as BASELINE_SEED
from mearmV1Baseline import load_cases, load_doc, tolerance

#: 基线记录值 ↔ 今天复算值的容差（基线自带 1e-9）
TOL_RECORDED_MM = tolerance(load_doc("fk"), "frontend_vs_recorded_mm", 1e-9)
#: 参考实现 `fkref.py` ↔ MuJoCo 的容差（基线自带 1e-6，与既有 Phase 9 契约同值）
TOL_REF_MJ_MM = tolerance(load_doc("fk"), "ref_vs_mujoco_mm", MM_TOL)
#: IK 闭环容差（基线自带 1e-9）
TOL_CLOSED_LOOP_MM = tolerance(load_doc("ik"), "closed_loop_mm", 1e-9)

N_SWEEP_FK = 500
N_SWEEP_IK = 1000


def _max_abs(a, b) -> float:
    return float(np.max(np.abs(np.asarray(a, dtype=float) - np.asarray(b, dtype=float))))


def _norm(a, b) -> float:
    return float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))


# ---------------------------------------------------------------------------
# 0. 基线属于哪个模型
# ---------------------------------------------------------------------------


def test_baseline_is_for_this_model(robot):
    """**脚手架自检** —— 不过这一步，后面所有数字都没有意义。

    基线若来自另一个模型/另一版配置，我们就会拿 A 的期望值去对 B 的实现，
    而表面上一切绿灯。
    """
    for name in ("joint", "fk", "ik", "workspace"):
        doc = load_doc(name)
        assert doc["model"] == "MeArm-V1", f"{name}: model"
        assert doc["modelVersion"] == "1.0.0", f"{name}: modelVersion"
        assert doc["robotId"] == robot.id, f"{name}: robotId"
        assert doc["seed"] == BASELINE_SEED, f"{name}: seed"
        assert doc["generator"] == "tools/gen_mearm_v1_baseline.py", f"{name}: generator"

    assert robot.model == "MeArm-V1", "config/robot.yaml 缺少 model 标识"
    assert robot.model_version == "1.0.0", "config/robot.yaml 的 version 不是 1.0.0"


def test_baseline_joint_set_matches_this_model(robot):
    """基线记录的可动关节集合必须与当前模型一致（含数量）。"""
    fk = load_cases("fk")
    joint = load_doc("joint")
    assert len(fk) == joint["count"], "joint_cases 与 fk_cases 的用例数不一致"
    assert sorted(fk[0]["joints"]) == sorted(j.id for j in robot.movable_joints())
    for c in fk:
        assert sorted(c["joints"]) == sorted(j.id for j in robot.movable_joints()), c["id"]


# ---------------------------------------------------------------------------
# 1. §17.4 Joint → MuJoCo（FK）
# ---------------------------------------------------------------------------


def test_fk_mujoco_matches_baseline(sim, robot):
    """116 例：同一关节状态，MuJoCo 的 TCP 必须复现基线的 `tcpMujoco`。

    ⚠️ 走 `reset()`（纯 `mj_forward`）而不是 `settle()` —— 后者会引入重力、
    接触与求解器迭代的噪声，把"几何定义层面"的判据变成一个会飘的判据。
    """
    cases = load_cases("fk")
    worst = (0.0, "")
    for c in cases:
        err = _max_abs(mujoco_tcp_mm(sim, c["joints"]), c["tcpMujoco"])
        if err > worst[0]:
            worst = (err, c["id"])

    print(f"\n[sim2sim·MuJoCo·FK] N={len(cases)}  TCP vs 基线 max|Δ| = {worst[0]:.3e} mm"
          f"（最差 {worst[1]}；容差 {TOL_RECORDED_MM:g}）")
    assert worst[0] < TOL_RECORDED_MM, f"最差用例 {worst[1]}"


def test_fk_reference_matches_baseline(robot):
    """116 例：`fkref.py`（独立于 MuJoCo 的参考实现）复现基线的 `tcpRef` / `framesRef`。

    这条把"基线的 FK 期望值"钉在参考实现上；下一条再把参考实现钉在 MuJoCo 上。
    三者串起来才能说"三侧同源"，而不是"两两之间看起来像"。
    """
    cases = load_cases("fk")
    worst_tcp = (0.0, "")
    worst_frame = (0.0, "")
    for c in cases:
        err = _max_abs(fk_tcp_mm(robot, c["joints"]), c["tcpRef"])
        if err > worst_tcp[0]:
            worst_tcp = (err, c["id"])

        origins = fk_joint_origins_mm(robot, c["joints"])
        assert set(origins) == set(c["framesRef"]), (
            f"{c['id']}：参考实现的关节集合与基线不一致")
        for jid, want in c["framesRef"].items():
            e = _max_abs(origins[jid], want)
            if e > worst_frame[0]:
                worst_frame = (e, f"{c['id']}/{jid}")

    print(f"[sim2sim·MuJoCo·FK] 参考实现 TCP N={len(cases)}  max|Δ| = {worst_tcp[0]:.3e} mm"
          f"（最差 {worst_tcp[1]}）")
    print(f"[sim2sim·MuJoCo·FK] 参考实现 关节坐标系 N={len(cases)}×5  max|Δ| = "
          f"{worst_frame[0]:.3e} mm（最差 {worst_frame[1]}）")
    assert worst_tcp[0] < TOL_RECORDED_MM
    assert worst_frame[0] < TOL_RECORDED_MM


def test_joint_origins_mujoco_matches_baseline(sim, robot):
    """116 例：**每个关节坐标系原点**（不止末端）在 MuJoCo 里也要对得上。

    比只对 TCP 更有诊断价值：某段几何错了，TCP 偏差只是"结果"，
    锚点逐级对比能直接指出错在哪一节。

    覆盖范围刻意包含 `robot.yaml` 的**全部关节**：被动腕 `tool` 在 MJCF 里没有独立
    `<joint>`（走"子连杆 body 的 `xpos`"这条等价路径），叶关节 `gripper` 不参与末端
    定位但仍是坐标系列的一部分 —— 这两类最容易在"只有 4 个关节"的心智模型里被漏掉。
    """
    cases = load_cases("fk")
    ids = sorted(cases[0]["framesRef"])
    assert set(ids) == {j.id for j in robot.joints}, "基线覆盖的关节集合与 robot.yaml 不一致"
    passive = [j.id for j in robot.joints if j.is_passive]
    assert passive, "模型里已没有被动机 —— 若确实去掉，这条断言应随模型一起调整"

    worst = (0.0, "")
    for c in cases:
        sim.reset(dict(c["joints"]))
        for jid, want in c["framesRef"].items():
            err = _max_abs(mujoco_joint_origin_mm(sim, robot, jid), want)
            if err > worst[0]:
                worst = (err, f"{c['id']}/{jid}")

    print(f"[sim2sim·MuJoCo·FK] 关节锚点 N={len(cases)}×{len(ids)}（含被动 {passive}）"
          f"  max|Δ| = {worst[0]:.3e} mm（最差 {worst[1]}；容差 {TOL_REF_MJ_MM:g}）")
    assert worst[0] < TOL_REF_MJ_MM


# ---------------------------------------------------------------------------
# 2. §17.5 XYZ → IK → MuJoCo
# ---------------------------------------------------------------------------


def test_ik_to_mujoco_matches_baseline(sim, robot, kinematics):
    """121 例：成功 / 失败 / 分支 / 闭环 / MuJoCo 落点全部与基线一致。

    ⚠️ IK 由**前端真实的 `ik.ts`** 解（经 `frontend/tests/tools/kinematics-bridge.mjs`
    用 Vite SSR 加载同一份源码）。Python 侧**只递 JSON、不解释任何运动学语义** ——
    一旦这里开始自己算几何，"独立判据"就没了（理由见 `tests/sim/ikbridge.py` 文件头）。

    三条误差各有分工，**不可混为一谈**：
      ``闭环``     MuJoCo(IK 解) vs 目标点      —— IK 的准确性（本测试的被测对象）
      ``对基线``   MuJoCo(IK 解) vs `tcpMujoco` —— 行为是否与冻结时一致（本文件的主题）
    """
    cases = load_cases("ik")
    results = kinematics.solve([{"id": c["id"], "target": list(c["target"])} for c in cases])
    assert len(results) == len(cases)

    successes = failures = 0
    worst_closed = (0.0, "")
    worst_base = (0.0, "")
    max_joint_delta = 0.0

    for c, res in zip(cases, results):
        expect = c["expect"]
        assert bool(res["success"]) is bool(expect["success"]), (
            f"{c['id']}：基线 success={expect['success']}，当前 {res['success']}"
            f"（{res.get('message')}）")

        if not expect["success"]:
            failures += 1
            assert res.get("reason") == expect["reason"], f"{c['id']} reason"
            assert res.get("joint") == expect.get("joint"), f"{c['id']} joint"
            continue

        successes += 1
        assert res["branch"] == expect["branch"], f"{c['id']} branch"

        joints = case_joints(res)
        for jid, want in expect["joints"].items():
            max_joint_delta = max(max_joint_delta, abs(joints[jid] - want))

        mj = mujoco_tcp_mm(sim, joints)
        err_closed = _norm(mj, c["target"])
        err_base = _max_abs(mj, expect["tcpMujoco"])
        if err_closed > worst_closed[0]:
            worst_closed = (err_closed, c["id"])
        if err_base > worst_base[0]:
            worst_base = (err_base, c["id"])

    print(f"[sim2sim·MuJoCo·IK] N={len(cases)}  成功 {successes} / 失败 {failures}"
          f" · 闭环 max={worst_closed[0]:.3e} mm（最差 {worst_closed[1]}）"
          f" · vs 基线 tcpMujoco max={worst_base[0]:.3e} mm"
          f" · 关节角 vs 基线 max={max_joint_delta:.3e}°")

    assert successes + failures == len(cases)
    assert successes > 0, "基线里应当有成功用例"
    assert failures > 0, "错误码也是被冻结的行为，必须有覆盖"
    assert worst_closed[0] < TOL_CLOSED_LOOP_MM
    assert worst_base[0] < TOL_RECORDED_MM
    assert max_joint_delta < TOL_RECORDED_MM


def test_workspace_verdicts_match_baseline(sim, kinematics):
    """121 例的可达性 / 错误码与基线一致，且 2R 几何量未变。

    第二半是**必要**的：IK 的全部数字都从这 5 个几何量推出（`pivotZ/pivotR/l1/l2/
    toolOffset`）。它们一旦漂移，判据仍在"可达/不可达"这个粗粒度上保持一致，
    误差却是毫米级的 —— 所以必须单独钉住。
    """
    cases = load_cases("workspace")
    results = kinematics.solve([{"id": c["id"], "target": list(c["target"])} for c in cases])
    assert len(results) == len(cases)

    reachable = outside = limited = 0
    for c, res in zip(cases, results):
        assert bool(res["success"]) is bool(c["expect"]["success"]), (
            f"{c['id']}：基线 {c['expect']['success']}，当前 {res['success']}")
        if res["success"]:
            reachable += 1
        else:
            assert res.get("reason") == c["expect"]["reason"], f"{c['id']} reason"
            outside += res["reason"] == "OUT_OF_WORKSPACE"
            limited += res["reason"] == "JOINT_LIMIT"

    print(f"[sim2sim·MuJoCo·工作空间] N={len(cases)}  可达 {reachable} / "
          f"超出空间 {outside} / 限位拒绝 {limited}")
    assert reachable + outside + limited == len(cases)
    assert outside > 0 and limited > 0, "两类拒绝都必须有覆盖"

    # ---- 2R 几何量：基线记录值 == 今天的实现 ----
    info = kinematics.model_info()
    g = info["geometry"]
    ref = cases[0]["geometry"]
    for key in ("pivotZ", "pivotR", "l1", "l2"):
        assert g[key] == pytest.approx(ref[key], abs=1e-9), f"IK 几何量 {key} 已变"
    assert g["toolOffset"] == pytest.approx(ref["toolOffset"], abs=1e-9)
    assert g["reach"] == pytest.approx(cases[0]["reach"], abs=1e-9)

    # ---- 越界判据用的矢状面距离：判据换算本身也要复现 ----
    worst = (0.0, "")
    for c in cases:
        d = abs(sagittal_distance_mm(info, c["target"]) - c["sagittalDistance"])
        if d > worst[0]:
            worst = (d, c["id"])
    print(f"[sim2sim·MuJoCo·工作空间] 矢状面距离 max|Δ| = {worst[0]:.3e} mm（最差 {worst[1]}）")
    assert worst[0] < 1e-9


# ---------------------------------------------------------------------------
# 3. §18 固定 seed 随机扫描
# ---------------------------------------------------------------------------


def test_sweep_joint_to_mujoco(sim, robot):
    """500 组随机关节位形：`fkref` ↔ MuJoCo 不得发散。

    与基线同一 seed（`20260914`），但这里的 RNG 是 numpy 的 `default_rng`，
    与前端那条链不同 —— 所以它**不是**在复现基线，而是在更宽的入参空间上
    检验"两条独立实现仍然对得上"。
    """
    rng = np.random.default_rng(BASELINE_SEED)
    worst = (0.0, "")
    for i in range(N_SWEEP_FK):
        js = random_pose(robot, rng)
        err = _max_abs(fk_tcp_mm(robot, js), mujoco_tcp_mm(sim, js))
        if err > worst[0]:
            worst = (err, f"#{i}")

    print(f"[sim2sim·MuJoCo·sweep{N_SWEEP_FK}] Joint→MuJoCo  max|Δ| = {worst[0]:.3e} mm"
          f"（最差 {worst[1]}；容差 {TOL_REF_MJ_MM:g}）")
    assert worst[0] < TOL_REF_MJ_MM


def test_sweep_ik_to_mujoco(sim, robot, kinematics):
    """1000 组随机可达目标：`XYZ → IK → MuJoCo` 闭环不退化。

    目标点全部由**参考 FK 从真机限位内的随机位形**生成 ⇒ 每一个都保证可达，
    因此"解不出来"必然是 **IK 的缺陷**，而不是"目标太刁钻"。断言刻意收紧到 100%。

    ⚠️ IK 走**一次批量调用**：桥是"一次性 CLI"，逐点调用会起 1000 个 node 进程。
    """
    rng = np.random.default_rng(BASELINE_SEED + 1)
    targets = [fk_tcp_mm(robot, random_pose(robot, rng)) for _ in range(N_SWEEP_IK)]
    results = kinematics.solve(
        [{"id": i, "target": [float(v) for v in t]} for i, t in enumerate(targets)]
    )
    assert len(results) == len(targets)

    failures = 0
    worst = (0.0, "")
    branches: dict[str, int] = {}
    for i, (res, target) in enumerate(zip(results, targets)):
        if not res["success"]:
            failures += 1
            continue
        branches[res["branch"]] = branches.get(res["branch"], 0) + 1
        err = _norm(mujoco_tcp_mm(sim, case_joints(res)), target)
        if err > worst[0]:
            worst = (err, f"#{i}")

    print(f"[sim2sim·MuJoCo·sweep{N_SWEEP_IK}] XYZ→IK→MuJoCo  失败 {failures}"
          f" · 最大闭环残差 {worst[0]:.3e} mm（最差 {worst[1]}）"
          f" · 分支 {branches}")
    assert failures == 0, f"{failures} 个**保证可达**的目标被 IK 拒绝"
    assert worst[0] < TOL_CLOSED_LOOP_MM
