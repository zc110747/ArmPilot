# -*- coding: utf-8 -*-
"""Phase 9 验收（二）：IK 一致性（spec §21 / §36 Acceptance 8）。

## 判据结构：让「被测对象」只剩一个

    ┌─ 生成 ─┐   在真机限位内随机采样关节角 ──► MuJoCo FK ──► 目标点 T
    │        └─ 两侧都用 MuJoCo，于是"目标点"不带任何前端语义
    │
    ├─ 被测 ─┐   前端真实 `ik.ts`：T ──► 关节角
    │        └─ 这是唯一被检验的对象，且加载的是**同一份源码**（不是 Python 重写版）
    │
    └─ 评判 ─┐   关节角 ──► MuJoCo FK ──► TCP'
             └─ 误差 = |TCP' − T|；评判器与生成器同源，因此不偏袒任何一方

误差门槛：spec §21 要求统计 success rate / mean / max。实测残差在 **1e-12 mm**
量级（IK 是解析解，误差只来自浮点），因此断言门槛取 1e-6 mm 仍有 6 个数量级余量。

## ⚠️ 为什么必须加载真实的 `ik.ts` 而不是在 Python 里重写

重写一份 IK 只能证明"我又写了一遍、而且它自洽"，证明不了 `ik.ts` 是对的。
而 `ik.ts` 恰恰是**真正驱动用户机械臂**的那一份 —— 它的错不会被前端自己的
FK 测试发现（两者共享同一套旋转约定，会一起错）。

## 本文件顺带钉住的一条机构学结论

`elbow-down` 支在本机**恒不可行**，不是"偶尔解不出来"：
    θe = θs + α，而 `elbow.limit_min (108.44°) > shoulder.limit_max (49.45°)`
    ⇒ 合法域要求 θe − θs > 0 ⇒ 相对肘角 α 恒为正 ⇒ 只有 elbow-up 一支存在。
见 `test_elbow_down_branch_cannot_exist_within_real_limits`。
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from harness import (
    MM_TOL,
    case_joints,
    grid_poses,
    mujoco_tcp_mm,
    random_pose,
    reach_bounds_mm,
    sagittal_distance_mm,
)

#: spec §21 要求 ≥100 个随机可达点
N_REACHABLE = 120

#: spec §33：固定 seed，任何一次运行都能复现同一批位形
SEED = 20260913


# ---------------------------------------------------------------------------
# 0. 先验：桥本身加载的配置必须就是 config/robot.yaml
# ---------------------------------------------------------------------------


def test_bridge_model_matches_robot_yaml(robot, kinematics):
    """**脚手架自检** —— 不过这一步，后面所有 IK 数字都没有意义。

    桥是用 Vite 加载前端的 `loadRobotModel.ts` 拿到模型的。如果它因为别名解析、
    构建缓存等原因加载了**另一份** robot.yaml，我们就会拿 A 配置的 IK 去对
    B 配置的 MuJoCo，结论完全无效，而表面上一切"绿灯"。
    """
    info = kinematics.model_info()

    assert info["units"] == "mm"
    assert info["tcp"]["joint"] == robot.tcp_joint
    assert info["tcp"]["offset"] == pytest.approx(list(robot.tcp_offset), abs=1e-12)
    assert info["homePose"] == pytest.approx(dict(robot.home_pose), abs=1e-12)

    by_id = {j["id"]: j for j in info["joints"]}
    assert set(by_id) == {j.id for j in robot.movable_joints()}, (
        "前端可动关节集合与 robot.yaml 不一致")

    for j in robot.movable_joints():
        f = by_id[j.id]
        assert f["type"] == j.type, f"{j.id}: type"
        assert f["axis"] == pytest.approx(list(j.axis), abs=1e-12), f"{j.id}: axis"
        assert f["limit"]["min"] == pytest.approx(j.limit_min, abs=1e-12), f"{j.id}: limit.min"
        assert f["limit"]["max"] == pytest.approx(j.limit_max, abs=1e-12), f"{j.id}: limit.max"
        assert f["parentLink"] == j.parent_link, f"{j.id}: parentLink"
        assert f["childLink"] == j.child_link, f"{j.id}: childLink"
        if j.coupling:
            assert f["coupling"] is not None, f"{j.id}: 前端丢了 coupling"
            assert f["coupling"]["jointId"] == j.coupling[0], f"{j.id}: coupling.joint"
            assert f["coupling"]["gain"] == pytest.approx(j.coupling[1], abs=1e-12)
        else:
            assert f["coupling"] is None, f"{j.id}: 前端多出了 coupling"

    act = {a["id"]: a for a in info["actuators"]}
    assert set(act) == {a.id for a in robot.actuators}, "前端执行器集合与 robot.yaml 不一致"
    for a in robot.actuators:
        f = act[a.id]
        assert f["jointId"] == a.joint_id
        assert f["channel"] == a.channel
        assert f["offset"] == pytest.approx(a.offset, abs=1e-12)
        assert f["scale"] == pytest.approx(a.scale, abs=1e-12)
        assert f["reverse"] is a.reverse
        assert f["limits"]["min"] == pytest.approx(a.servo_min, abs=1e-12)
        assert f["limits"]["max"] == pytest.approx(a.servo_max, abs=1e-12)


def test_ik_geometry_agrees_with_mujoco_link_lengths(robot, kinematics):
    """IK 内部用的 2R 几何量（l1 / l2 / 枢轴 / 常量偏移）必须与关节锚点的实际间距一致。

    这一条把「IK 的解析前提」和「MuJoCo 里的真实几何」钉在一起：
    `ik.ts` 是从"全零位形跑一次 FK"推出这些量的，如果模型树与 MJCF 不一致，
    解析式会**静默解错**（不报错，只是答案偏），必须用独立来源核对。

    ⚠️ `l2` 量的是「肘枢轴 → **腕枢轴**」，**不**含那 40mm 的爪：
    爪被被动腕锁成水平后，「腕枢轴 → TCP」是一段**常量**矢状面偏移
    （`geometry.toolOffset`，本机 = [40, 0]），IK 会先减掉它再做 2R。
    把它算进 `l2`（旧写法 `forearm.length + |tcp_offset|`）会让可达球壳变成
    [40, 200]，而且每个目标点都凭空多出 40mm 的径向分量。
    """
    g = kinematics.model_info()["geometry"]

    # 全零位形下机构沿 +Z 竖直伸展 ⇒ 相邻枢轴的间距就是矢状面内的等效杆长
    from fkref import fk_tcp_mm

    l1 = float(robot.link("upper_arm_link").length)
    l2 = float(robot.link("forearm_link").length)
    assert g["l1"] == pytest.approx(l1, abs=1e-9), f"IK l1={g['l1']} vs 模型 {l1}"
    assert g["l2"] == pytest.approx(l2, abs=1e-9), f"IK l2={g['l2']} vs 模型 {l2}"
    assert g["pivotZ"] == pytest.approx(float(robot.link("column_link").length), abs=1e-9)
    assert g["pivotR"] == pytest.approx(0.0, abs=1e-9), "肩枢轴应落在偏航轴上"
    assert g["reach"] == pytest.approx([abs(l1 - l2), l1 + l2], abs=1e-9)

    # 顺带确认全零位形确实是竖直的（上面那条"枢轴间距 = 杆长"的前提）
    zero_tcp = fk_tcp_mm(robot, {j.id: 0.0 for j in robot.movable_joints()})
    assert abs(zero_tcp[1]) < MM_TOL

    # ★ 常量工具偏移：用**零位的 MuJoCo FK** 独立核对 IK 自己推出的那一段
    #   （零位时腕枢轴落在偏航轴上、水平半径 0，所以两个分量可以直接相减得到）
    wrist_at_zero = np.array([0.0, 0.0, g["pivotZ"] + l1 + l2])
    assert g["wristId"] == robot.tcp_joint, (
        f"腕关节应为 {robot.tcp_joint!r}，实得 {g['wristId']!r}")
    assert g["toolOffset"] == pytest.approx(
        [zero_tcp[0] - wrist_at_zero[0], zero_tcp[2] - wrist_at_zero[2]], abs=1e-9), (
        "IK 的常量偏移与零位实测不符 ⇒ 它和 MuJoCo 的几何不是同一回事")
    assert g["toolOffset"][1] == pytest.approx(0.0, abs=1e-9), (
        "本机爪恒水平 ⇒ 腕→TCP 的偏移不该有竖直分量；若有，说明锁的不是水平")
    assert zero_tcp[2] == pytest.approx(g["pivotZ"] + l1 + l2 + g["toolOffset"][1], abs=1e-9)


# ---------------------------------------------------------------------------
# 1. ★ 主判据：随机可达点 → IK → MuJoCo 评判（spec §21）
# ---------------------------------------------------------------------------


def _run_reachable_set(sim, robot, kinematics, poses):
    """把一批位形变成"目标点 → IK → MuJoCo 复算"，返回逐点明细。

    三条误差各有分工，**不可混为一谈**：
      ``err_ik``       MuJoCo(IK解) vs 目标       —— IK 的准确性（本测试的被测对象）
      ``err_fk``       前端FK(IK解) vs MuJoCo(IK解) —— 前端 FK 与 MuJoCo 的一致性
      ``err_frontend`` 前端FK(IK解) vs 目标        —— 前端自己的残差（**自证，仅作参考**）
    """
    targets = [mujoco_tcp_mm(sim, js) for js in poses]
    cases = [{"id": i, "target": [float(v) for v in t]} for i, t in enumerate(targets)]
    results = kinematics.solve(cases)
    assert len(results) == len(cases)

    rows = []
    for i, (res, target, pose) in enumerate(zip(results, targets, poses)):
        if not res["success"]:
            rows.append({"i": i, "target": target, "pose": pose, "ok": False, "res": res})
            continue
        joints = case_joints(res)
        mj = mujoco_tcp_mm(sim, joints)
        fe = np.asarray(res["tcpFrontend"], dtype=float)
        rows.append({
            "i": i,
            "ok": True,
            "res": res,
            "joints": joints,
            "target": target,
            "pose": pose,
            "tcp_mujoco": mj,
            "tcp_frontend": fe,
            "err_ik": float(np.max(np.abs(mj - target))),
            "err_fk": float(np.max(np.abs(fe - mj))),
            "err_frontend": float(np.max(np.abs(fe - target))),
        })
    return rows


def test_reachable_targets_ik_then_mujoco(sim, robot, kinematics):
    """★ 120 个随机可达点：IK 必须全部成功，且 MuJoCo 复算的误差 < 1e-6 mm。

    目标点由 **MuJoCo FK** 从真机限位内的随机位形生成 ⇒ 每一个都保证可达。
    因此"解不出来"必然是 **IK 的缺陷**（例如可行性判据写错、选支逻辑漏解），
    而不是"目标太刁钻"。这条断言刻意收紧到 100%：没有灰色地带可躲。
    """
    rng = np.random.default_rng(SEED)
    poses = [random_pose(robot, rng) for _ in range(N_REACHABLE)]
    rows = _run_reachable_set(sim, robot, kinematics, poses)

    bad = [r for r in rows if not r["ok"]]
    assert not bad, (
        f"{len(bad)}/{len(rows)} 个**保证可达**的目标被 IK 拒绝：\n"
        + "\n".join(f"  目标 {np.round(r['target'], 4)} → {r['res'].get('reason')} "
                    f"{r['res'].get('message', '')}" for r in bad[:5]))

    assert all(r["res"]["branch"] == "elbow-up" for r in rows), (
        "在本机限位下只可能存在 elbow-up 一支，见 "
        "test_elbow_down_branch_cannot_exist_within_real_limits")

    e_ik = np.array([r["err_ik"] for r in rows])
    e_fk = np.array([r["err_fk"] for r in rows])
    worst = rows[int(e_ik.argmax())]
    print(f"\n[IK] N={len(rows)}  success={len(rows)}/{len(rows)} (100%)")
    print(f"[IK] MuJoCo 复算   max={e_ik.max():.3e} mm  mean={e_ik.mean():.3e} mm")
    print(f"[IK] 前端FK↔MuJoCo max={e_fk.max():.3e} mm  mean={e_fk.mean():.3e} mm")

    assert e_ik.max() < MM_TOL, (
        f"最差位形：目标 {np.round(worst['target'], 4)} → MuJoCo "
        f"{np.round(worst['tcp_mujoco'], 4)}（差 {worst['err_ik']:.3e} mm）\n"
        f"  IK 解：{ {k: round(v, 6) for k, v in worst['joints'].items()} }")


def test_frontend_fk_agrees_with_mujoco(sim, robot, kinematics):
    """★ 前端 `fk.ts` 与 MuJoCo 必须逐点一致（第三个独立实现的对撞）。

    这比 §20 的"参考实现 vs MuJoCo"更进一步：`fkref.py` 是我为验收写的，
    而 `fk.ts` 是**真正画 3D 视图、真正做鼠标拖动**的那一份。它若与 MuJoCo
    有系统性偏差，用户在界面上看到的位姿就是错的 —— 那才是最贵的错误。
    """
    rng = np.random.default_rng(SEED + 1)
    poses = [random_pose(robot, rng) for _ in range(N_REACHABLE)]
    rows = _run_reachable_set(sim, robot, kinematics, poses)
    assert all(r["ok"] for r in rows)

    e_fk = np.array([r["err_fk"] for r in rows])
    assert e_fk.max() < MM_TOL, (
        f"前端 FK 与 MuJoCo 最大偏差 {e_fk.max():.3e} mm（阈值 {MM_TOL:g}）")


def test_limit_corner_targets_ik_then_mujoco(sim, robot, kinematics):
    """限位端点上的 4^4 = 256 个角点：随机采样几乎打不到边界，这里专门打。

    边界位形的意义：`cosAlpha` 接近 ±1 的退化区、以及限位钳位逻辑的边缘 ——
    这两处正是解析式最容易出 ±ε 符号错误的地方。
    """
    poses = grid_poses(robot, 4)
    rows = _run_reachable_set(sim, robot, kinematics, poses)
    bad = [r for r in rows if not r["ok"]]
    assert not bad, (
        f"{len(bad)}/{len(rows)} 个角点目标被拒：\n"
        + "\n".join(f"  {np.round(r['target'], 4)} → {r['res'].get('reason')}" for r in bad[:5]))

    e_ik = np.array([r["err_ik"] for r in rows])
    print(f"[IK] 角点 N={len(rows)}  max={e_ik.max():.3e} mm")
    assert e_ik.max() < MM_TOL


def test_ik_solution_is_consistent_with_requested_seed(sim, robot, kinematics):
    """IK 解出的定位关节角，必须与生成目标点的那个位形一致（不是"另一个也合法的解"）。

    本机限位只允许 elbow-up 一支且该支对给定目标是唯一的，所以"解"与"原构型"
    必须逐项相等 —— 若不等，说明解算公式里有系统性偏移（例如把绝对角当局部角）。
    """
    rng = np.random.default_rng(SEED + 2)
    poses = [random_pose(robot, rng) for _ in range(40)]
    targets = [mujoco_tcp_mm(sim, js) for js in poses]
    results = kinematics.solve([{"id": i, "target": [float(v) for v in t]}
                                for i, t in enumerate(targets)])
    for res, pose in zip(results, poses):
        assert res["success"], res.get("message")
        got = case_joints(res)
        for jid in ("base", "shoulder", "elbow"):
            assert got[jid] == pytest.approx(pose[jid], abs=1e-6), (
                f"{jid}: IK 解 {got[jid]:.9f} vs 原构型 {pose[jid]:.9f}")


# ---------------------------------------------------------------------------
# 2. 机构学结论：本机只有一支解
# ---------------------------------------------------------------------------


def test_elbow_down_branch_cannot_exist_within_real_limits(robot):
    """★ 算术证明：`elbow-down` 支在本机**恒不可行**。

    `θe = θs + α`（α 为相对肘角）。合法域要求 `θe ≥ 108.4415` 且 `θs ≤ 49.4549`，
    两式相减得 α ≥ 58.9866 > 0 ⇒ α 恒为正 ⇒ 只有 elbow-up 一支。
    这不是"偶尔解不出来"，而是**机构自由度上的真实约束**：
    它同时解释了为什么真机 HOME 位与 `docs/coordinate-system.md` 都只描述这一支。
    """
    e = robot.joint("elbow")
    s = robot.joint("shoulder")
    assert e.limit_min > s.limit_max, (
        f"前提变了：elbow.limit_min={e.limit_min} 不再大于 shoulder.limit_max={s.limit_max}；"
        "请重新评估 IK 是否出现第二支可行解")
    alpha_min = e.limit_min - s.limit_max
    assert alpha_min > 0.0
    print(f"[IK] 相对肘角下界 = {alpha_min:.4f}° ⇒ elbow-down（α<0）恒不可行")


def test_requesting_elbow_down_falls_back_to_the_only_feasible_branch(sim, robot, kinematics):
    """显式请求 `elbow-down` 时，IK 必须**退回可行支**而不是返回不可行解。

    这是 `ik.ts` 的 `selectCandidate` 契约：指定支不可行时退到可用的 elbow-up
    （真机 HOME 所在支），**绝不静默返回一个越界的解** ——
    越界的解送到真机上会被上层拦掉，用户看到的是"点了没反应"，极难排查。
    """
    rng = np.random.default_rng(SEED + 3)
    pose = random_pose(robot, rng)
    target = mujoco_tcp_mm(sim, pose)
    res = kinematics.solve_one(target, prefer="elbow-down")
    assert res["success"], res.get("message")
    assert res["branch"] == "elbow-up", (
        f"请求 elbow-down，却真的返回了 {res['branch']} —— 该支在本机是不可行的")
    # 而它返回的解仍然要经得起 MuJoCo 的检验
    mj = mujoco_tcp_mm(sim, case_joints(res))
    assert float(np.max(np.abs(mj - target))) < MM_TOL


# ---------------------------------------------------------------------------
# 3. 负向：不可达的目标必须被明确拒绝，且拒绝理由正确
# ---------------------------------------------------------------------------


def test_out_of_workspace_targets_are_rejected(sim, robot, kinematics):
    """球壳之外的目标：必须报 `OUT_OF_WORKSPACE`，且**不能**悄悄给一个近似解。

    三个目标分别打三种越界：
      * 正上方过远（`d > l1 + l2`，dz > 0）
      * 正下方过远（同样超球壳，但 dz < 0 —— 覆盖符号相反的那条路径）
      * 侧向过远（同时验证非零方位角路径）

    ⚠️ 旧版本第二个目标是「正上方过近（`d < |l1 − l2|`）」，**本机已不成立**：
    `l1 === l2 === 80` ⇒ 内半径 `|l1 − l2| = 0`，球壳没有内锥空洞，
    再近的目标也是「几何可达」的，会在关节限位那一关被拦下 —— 那会得到
    `JOINT_LIMIT`，理由分类是**对的**（见 ik.test.ts 的同名用例）。
    所以这里换掉它，用「正下方过远」把三种越界重新凑齐。
    """
    info = kinematics.model_info()
    reach_min, reach_max = reach_bounds_mm(info)
    pivot_z = float(info["geometry"]["pivotZ"])

    targets = {
        "上方过远": [0.0, 0.0, pivot_z + reach_max + 50.0],
        "下方过远": [0.0, 0.0, pivot_z - reach_max - 50.0],
        "侧向过远": [reach_max + 100.0, 0.0, pivot_z],
    }
    results = kinematics.solve([{"id": i, "target": t}
                                for i, t in enumerate(targets.values())])
    for (name, t), res in zip(targets.items(), results):
        d = sagittal_distance_mm(info, t)
        assert res["success"] is False, f"{name} {t} 竟被解出可行解：{res}"
        assert res["reason"] == "OUT_OF_WORKSPACE", (
            f"{name}: 期望 OUT_OF_WORKSPACE，实得 {res['reason']}（{res['message']}）")
        assert d > reach_max or d < reach_min, (
            f"{name}: 桥报越界但按几何算 d={d:.4f} 落在 [{reach_min}, {reach_max}] 内 ⇒ "
            "两边的可达判据不一致")

    # ★ 独立佐证：用 MuJoCo 扫一大批**限位内**位形，最近的一次也够不着这些点。
    #   这一步不依赖 IK 的公式，是纯粹"物理上到不了"的证据。
    rng = np.random.default_rng(SEED + 4)
    far = np.asarray(targets["上方过远"], dtype=float)
    best = float("inf")
    for _ in range(20_000):
        tcp = mujoco_tcp_mm(sim, random_pose(robot, rng))
        best = min(best, float(np.linalg.norm(tcp - far)))
    assert best > 20.0, (
        f"2 万个限位内位形里有 TCP 距目标仅 {best:.3f} mm ⇒ 该目标其实可达，"
        "是判据把它误判成越界")
    print(f"[IK] 越界目标独立佐证：{best:.3f} mm 为 2 万个样品中最近的一次")


def test_geometrically_reachable_but_limits_block(sim, robot, kinematics):
    """★★ 最有价值的一条：**几何可达，但真机限位不允许**。

    构造：`elbow` 取到绝对限位下限再往下 55°（≈53.4°），`shoulder` 取到下限。
    此时机构在几何上完全能把 TCP 送到那个点（2R 的 `d` 落在可达球壳内），
    真机的 S8 舵机结构上却转不到那个绝对角。

    ⇒ 这条同时钉住三件事：
      1. IK 必须报 `JOINT_LIMIT`（而不是 `OUT_OF_WORKSPACE`）—— 理由分类正确
      2. MuJoCo 的 hinge range **不是**限位真值（否则上面那步就过不了）
      3. 限位一致性的唯一把关人是上层（IK / Go controller / `limits.py`）

    ⚠️ 第 2 条的成立方式在被动腕改造后**变窄了**，务必理解清楚再改本用例：
    `sim.reset()` 是"设定初始条件"（只写 qpos + `mj_forward`），**从不校验 hinge
    range**，所以它对这个位形照常接受 —— 这条断言因此仍然成立。
    但"命令-响应"那条路（`set_target_joints` + `settle`）**会**受 range 约束：
    被动腕 `tool` 的外接区间由 elbow 限位派生，与等式约束联立后把 elbow 的绝对角
    钳在 ≥ 106.4415°。于是「命令得动、真机做不到」的窗口只剩约 2°（见
    test_joint_limits.py::test_mujoco_accepts_real_machine_impossible_pose）。
    """
    elbow = robot.joint("elbow")
    shoulder = robot.joint("shoulder")
    illegal = {
        "base": 0.0,
        "shoulder": shoulder.limit_min,
        "elbow": elbow.limit_min - 55.0,
        "gripper": 0.0,
    }
    info = kinematics.model_info()

    # --- 前提 1：几何上可达（在矢状面可达球壳内）------------------------------
    from fkref import fk_tcp_mm

    target = fk_tcp_mm(robot, illegal)
    reach_min, reach_max = reach_bounds_mm(info)
    d = sagittal_distance_mm(info, target)
    assert reach_min < d < reach_max, (
        f"反例构造失败：目标枢轴距离 {d:.4f} 不在 [{reach_min}, {reach_max}] 内，"
        "那就不是「几何可达但限位不允许」了")
    assert sagittal_distance_mm(info, mujoco_tcp_mm(sim, illegal)) == pytest.approx(d, abs=1e-6), (
        "真值源与 MuJoCo 对这个位形的几何解释不一致")

    # --- 前提 2：MuJoCo 照常接受该位形（range 只是数值保护，不是限位真值）------
    sim.reset(illegal)
    mj_elbow = float(np.degrees(sim.data.qpos[sim.angle_map.qpos_index("elbow")]))
    expect_local = illegal["elbow"] - illegal["shoulder"]      # 局部角 = θe − θs
    assert mj_elbow == pytest.approx(expect_local, abs=1e-9), (
        f"MuJoCo 的 elbow 局部角（{mj_elbow:.6f}°）与「绝对角 − 肩角」"
        f"（{expect_local:.6f}°）不符 ⇒ units.py 的耦合映射出了问题")

    # --- 结论：IK 必须拒绝，且理由必须是 JOINT_LIMIT ---------------------------
    res = kinematics.solve_one(target)
    assert res["success"] is False, f"IK 竟然接受了真机做不到的位形：{res}"
    assert res["reason"] == "JOINT_LIMIT", (
        f"期望 JOINT_LIMIT，实得 {res['reason']}（{res['message']}）—— "
        "把「越限」报成「不可达」会让用户去挪机械臂，而真正该做的是修改目标")
    assert res["joint"] == "elbow", f"越界关节应报 elbow，实得 {res['joint']}"
    cands = res["candidates"]
    assert cands and all(c["feasible"] is False for c in cands)
    print(f"[IK] 越限反例：elbow 目标 {illegal['elbow']:.4f}° "
          f"(限 {elbow.limit_min:.4f}..{elbow.limit_max:.4f})，"
          f"最接近的一支仍差 {min(c['violation'] for c in cands):.3f}°")


def test_yaw_axis_is_outside_the_reachable_workspace(sim, robot, kinematics):
    """★ 真机**够不到自己的中轴线** —— 可达工作空间的内锥是空的。

    直觉上会以为"离底盘越近越容易够到"，实际相反。**TCP** 的水平半径
        `dr(θs, θe) = l1·sin θs + l2·sin θe + off_r`
    其中 `off_r` 是「腕枢轴 → TCP」的**常量**径向偏移（本机 40mm —— 爪锁水平，
    那一段完全落在 +X）。在合法域内：
      * 对 θs 单调**递增**（θs ∈ [−6.09, 49.45]，cos θs > 0）
      * 对 θe 单调**递减**（θe ∈ [108.44, 141.86] ⊂ (90°, 180°)）
      * `off_r` 与两者无关，只整体平移曲线，不改变极值位置
    ⇒ 最小值在角点 `(θs_min, θe_max)`，实测 **80.92 mm > 0**
    （旧模型的 65.62mm 漏掉了那 40mm 偏移，凭空少算一截）。

    也就是说：`shoulder` 上限只有 49.45°、`elbow` 绝对角下限却有 108.44°，
    手臂**折不过去**，TCP 永远离中轴线至少 8.1cm。

    ## 这条结论对上层的一个直接含义

    `ik.ts` 里那段「目标落在偏航轴上 ⇒ 方位角不定 ⇒ 沿用 `near`」的分支，
    在本机上**永远不会被走到** —— 因为 `r ≈ 0` 的目标本身不可达，
    会先在"关节越限"那一关被拦下。分支本身没错（模型换一个更宽的限位就会走到），
    但**不能**把它当成已被覆盖的路径。

    ## 判据的独立性

    闭式推导（`robot.yaml` 的限位 + 桥导出的 l1/l2）与 MuJoCo 网格扫描
    必须给出同一个最小值。两者若不同，说明"矢状面 2R"这个解析前提在本模型上不成立。
    """
    info = kinematics.model_info()
    l1 = float(info["geometry"]["l1"])
    l2 = float(info["geometry"]["l2"])
    off_r = float(info["geometry"]["toolOffset"][0])      # 腕枢轴 → TCP 的常量径向偏移
    s = robot.joint("shoulder")
    e = robot.joint("elbow")

    def dr(ts: float, te: float) -> float:
        return (off_r + l1 * math.sin(math.radians(ts))
                + l2 * math.sin(math.radians(te)))

    # --- ① 闭式：先确认公式本身对（用 HOME 位复现实测 TCP 的水平半径）----------
    home = dict(robot.home_pose)
    home_r_measured = float(np.hypot(*mujoco_tcp_mm(sim, home)[:2]))
    assert dr(home["shoulder"], home["elbow"]) == pytest.approx(home_r_measured, abs=1e-6), (
        "闭式 dr(θs, θe) 与 MuJoCo 实测的水平半径不符 ⇒ 解析前提不成立，"
        "下面的推导全部作废")

    # --- ② 最小水平半径在角点，且严格为正 -------------------------------------
    dr_min = dr(s.limit_min, e.limit_max)
    assert dr_min > 0.0, (
        f"存在 dr ≤ 0 的合法位形（{dr_min:.4f} mm）⇒ 中轴线可达，本结论不再成立；"
        "请重新评估 ik.ts 里「方位角不定」那条分支的覆盖情况")

    # --- ③ MuJoCo 网格扫描（含两端点，因此必然取到闭式的角点最小值）------------
    scan = [
        float(np.hypot(*mujoco_tcp_mm(
            sim, {"base": 0.0, "shoulder": float(ts), "elbow": float(te), "gripper": 0.0})[:2]))
        for ts in np.linspace(s.limit_min, s.limit_max, 40)
        for te in np.linspace(e.limit_min, e.limit_max, 40)
    ]
    scan_min = min(scan)
    assert scan_min == pytest.approx(dr_min, abs=1e-6), (
        f"闭式给 {dr_min:.6f} mm，MuJoCo 扫描给 {scan_min:.6f} mm")
    print(f"[IK] 可达工作空间内锥：最小水平半径 {dr_min:.4f} mm "
          f"（角点 θs={s.limit_min:.4f}°, θe={e.limit_max:.4f}°）")

    # --- ④ 于是"中轴线上的目标"必须被拒绝，并有独立证据 ------------------------
    reach_min, reach_max = reach_bounds_mm(info)
    target = [0.0, 0.0, float(info["geometry"]["pivotZ"]) + 0.5 * (reach_min + reach_max)]
    res = kinematics.solve_one(target)
    assert res["success"] is False, (
        f"中轴线上的目标 {target} 竟被解出可行解：{res}")
    assert res["reason"] in ("JOINT_LIMIT", "OUT_OF_WORKSPACE"), res["reason"]

    closest = min(
        float(np.linalg.norm(mujoco_tcp_mm(
            sim, {"base": 0.0, "shoulder": float(ts), "elbow": float(te), "gripper": 0.0})
            - np.asarray(target)))
        for ts in np.linspace(s.limit_min, s.limit_max, 40)
        for te in np.linspace(e.limit_min, e.limit_max, 40)
    )
    assert closest > 60.0, f"某个合法位形距中轴目标仅 {closest:.3f} mm ⇒ 它其实可达"
    print(f"[IK] 中轴目标被拒（{res['reason']}），合法域内最近距离 {closest:.3f} mm")


def test_ik_never_returns_out_of_limit_joints(sim, robot, kinematics):
    """**任何**成功解都必须在真机限位内 —— 否则"成功"是个危险的谎。

    覆盖随机位形 + 限位角点两批（376 个点），逐个用 `robot.yaml` 校验。
    """
    from limits import validate_joints

    rng = np.random.default_rng(SEED + 5)
    poses = [random_pose(robot, rng) for _ in range(N_REACHABLE)] + grid_poses(robot, 4)
    targets = [mujoco_tcp_mm(sim, js) for js in poses]
    results = kinematics.solve([{"id": i, "target": [float(v) for v in t]}
                                for i, t in enumerate(targets)])
    ok = [r for r in results if r["success"]]
    assert len(ok) == len(results), (
        f"{len(results) - len(ok)} 个保证可达的目标被拒："
        + "; ".join(f"{r.get('reason')} {r.get('message', '')}" for r in results if not r["success"])[:600])

    for res in ok:
        joints = case_joints(res)
        v = validate_joints(robot, joints)
        assert v is None, (
            f"IK 返回了越限解：{v.message() if v else ''}；解 = "
            f"{ {k: round(x, 6) for k, x in joints.items()} }")
    print(f"[IK] 限位自查 N={len(ok)}  全部在 config/robot.yaml 限位内")
