# -*- coding: utf-8 -*-
"""Phase 9 验收（一）：FK 一致性（spec §20 / §36 Acceptance 8）。

**判据的两个侧面都是独立的**：
  * A 面 = MuJoCo 编译后的模型（`sim.model` / `sim.data.xanchor`）
  * B 面 = `fkref.py` —— 直接读 `config/robot.yaml` 原始几何的参考实现

两者共享的唯一东西是 `config/robot.yaml`（唯一真值源）。中间那层生成器
（`gen_model.py`）**不在判据里** —— 用它自己的中间结果去对，只能证明它自洽。

误差门槛：spec §20 要求「<1mm，初期 <5mm」。实测是 0.0mm（逐位相同），
这里仍按 1e-6mm 设断言 —— 留出浮点余量，但任何真实的几何/旋转约定错误
（例如 intrinsic XYZ 写成 extrinsic、耦合项漏掉）都会立刻冲到毫米甚至厘米级。
"""
from __future__ import annotations

import numpy as np
import pytest

from fkref import fk_joint_origins_mm, fk_tcp_mm
from harness import MM_TOL as TOL_MM
from harness import grid_poses, mujoco_tcp_mm, random_pose

N_RANDOM = 120         # spec §21 要求 ≥100


# ---------------------------------------------------------------------------
# 辅助
# ---------------------------------------------------------------------------


def mj_id(sim, obj, name: str) -> int:
    import mujoco

    i = mujoco.mj_name2id(sim.model, obj, name)
    assert i >= 0, f"模型里找不到 {obj} {name}"
    return i


def joint_name_to_id(sim, name: str) -> int:
    import mujoco

    return mj_id(sim, mujoco.mjtObj.mjOBJ_JOINT, name)


def joint_origin_mm(sim, robot, jid: str) -> np.ndarray:
    """关节坐标系原点的世界坐标（mm）—— 从 **MuJoCo 侧**取。

    ⚠️ 固定关节（`robot.yaml` 里 `type: fixed`）在 MJCF 里**没有 `<joint>` 元素**
    （见 `gen_model.py`：`if joint is not None and not joint.is_fixed` 才写 joint），
    所以 `mj_name2id(..., mjOBJ_JOINT, "tool")` 会返回 −1。

    但两条路径取到的是**同一个量**：生成器把 body 放在
    `parent.length + origin.position` 处，可动关节的 anchor 就是这个 body 原点，
    固定关节的坐标系原点也是这个 body 原点。可动关节读 `xanchor`，
    固定关节读**子连杆 body 的 `xpos`**。
    """
    joint = robot.joint(jid)
    if joint.is_fixed:
        i = mj_id(sim, mujoco_obj("body"), joint.child_link)
        return np.asarray(sim.data.xpos[i], dtype=float) * 1000.0
    i = joint_name_to_id(sim, jid)
    return np.asarray(sim.data.xanchor[i], dtype=float) * 1000.0


def mujoco_obj(name: str):
    import mujoco

    return getattr(mujoco.mjtObj, f"mjOBJ_{name.upper()}")


# ---------------------------------------------------------------------------
# 1. 定点核对（独立几何常量）
# ---------------------------------------------------------------------------


def test_zero_pose_tcp_is_sum_of_lengths(sim, robot):
    """零位 TCP z = column + upper_arm + forearm + tool 四个 length 之和。"""
    zero = {j.id: 0.0 for j in robot.movable_joints()}
    want = sum(robot.link(i).length for i in
               ("column_link", "upper_arm_link", "forearm_link", "tool_link"))
    got = mujoco_tcp_mm(sim, zero)
    assert got[2] == pytest.approx(want, abs=TOL_MM)
    assert abs(got[0]) < TOL_MM and abs(got[1]) < TOL_MM


def test_home_pose_fk_matches(sim, robot):
    """HOME 位：参考 FK 与 MuJoCo 必须逐位相同。"""
    home = dict(robot.home_pose)
    a = fk_tcp_mm(robot, home)
    b = mujoco_tcp_mm(sim, home)
    assert np.allclose(a, b, atol=TOL_MM), f"参考 {a} vs MuJoCo {b}"


# ---------------------------------------------------------------------------
# 2. ★ 随机位形批量对比（spec §20 的主判据）
# ---------------------------------------------------------------------------


def test_random_poses_fk_agreement(sim, robot):
    """≥100 个随机构型：报告 max / mean 误差，并要求全部 < 1e-6 mm。

    ⚠️ 用 `reset()`（纯运动学），**不是** `settle()` —— 后者会带入重力、
    接触与求解器迭代的动力学噪声，把"定义层面"的断言变成一个会飘的断言。
    """
    rng = np.random.default_rng(20260913)     # spec §33：固定 seed，可复现
    errs = []
    worst = None
    for _ in range(N_RANDOM):
        js = random_pose(robot, rng)
        a = fk_tcp_mm(robot, js)
        b = mujoco_tcp_mm(sim, js)
        d = float(np.max(np.abs(a - b)))
        errs.append(d)
        if worst is None or d > worst[0]:
            worst = (d, js, a, b)
    errs_arr = np.array(errs)
    print(f"\n[FK] N={N_RANDOM}  max={errs_arr.max():.3e} mm  "
          f"mean={errs_arr.mean():.3e} mm  (阈值 {TOL_MM:g})")
    assert errs_arr.max() < TOL_MM, (
        f"最大偏差 {errs_arr.max():.6f} mm 超阈值；最差位形 {worst[1]}\n"
        f"  参考 {worst[2]}\n  MuJoCo {worst[3]}")


def test_limit_corner_poses_fk_agreement(sim, robot):
    """限位端点网格（4^4 = 256 个角点）：随机采样难以命中边界，这里专门打。"""
    poses = grid_poses(robot, 4)
    errs = [float(np.max(np.abs(fk_tcp_mm(robot, js) - mujoco_tcp_mm(sim, js))))
            for js in poses]
    print(f"[FK] 角点 N={len(poses)}  max={max(errs):.3e} mm")
    assert max(errs) < TOL_MM


def test_coupling_heavy_poses_fk_agreement(sim, robot):
    """专打**耦合最重**的区域：shoulder 与 elbow 同时扫到两端。

    elbow 存的是**绝对倾角**、且与 shoulder 通过平行四连杆耦合（gain=-1）。
    局部角 = θ_elbow − θ_shoulder，所以这一段最容易出现"绝对/局部搞混"的错误 ——
    一旦搞混，TCP 会偏出几十毫米，而不是几微米。
    """
    sh = robot.joint("shoulder")
    el = robot.joint("elbow")
    base = robot.joint("base")
    errs = []
    for s in np.linspace(sh.limit_min, sh.limit_max, 7):
        for e in np.linspace(el.limit_min, el.limit_max, 7):
            js = {"base": 0.0, "shoulder": float(s), "elbow": float(e), "gripper": 0.0}
            errs.append(float(np.max(np.abs(fk_tcp_mm(robot, js) - mujoco_tcp_mm(sim, js)))))
            for b in (base.limit_min, 0.0, base.limit_max):
                js2 = dict(js, base=float(b))
                errs.append(float(np.max(np.abs(fk_tcp_mm(robot, js2)
                                                - mujoco_tcp_mm(sim, js2)))))
    print(f"[FK] 耦合区 N={len(errs)}  max={max(errs):.3e} mm")
    assert max(errs) < TOL_MM


def test_gripper_does_not_move_tcp(sim, robot):
    """夹爪开合不得影响 TCP（robot.yaml 刻意把 tcp.joint 取为 tool）。"""
    base = {"base": 0.0, "shoulder": 20.0, "elbow": 125.0, "gripper": 0.0}
    a = fk_tcp_mm(robot, dict(base, gripper=0.0))
    b = fk_tcp_mm(robot, dict(base, gripper=90.0))
    assert np.array_equal(a, b), f"参考 FK 里夹爪影响了 TCP：{a} vs {b}"
    pa = mujoco_tcp_mm(sim, dict(base, gripper=0.0))
    pb = mujoco_tcp_mm(sim, dict(base, gripper=90.0))
    assert np.array_equal(pa, pb), f"MuJoCo 里夹爪影响了 TCP：{pa} vs {pb}"


# ---------------------------------------------------------------------------
# 3. 逐关节定位（偏差一旦出现，这条能指出是哪一段）
# ---------------------------------------------------------------------------


def test_joint_anchors_match_mujoco(sim, robot):
    """每个**关节锚点**的世界坐标都要对得上。

    比只对 TCP 更有诊断价值：如果某段几何错了，TCP 偏差只是"结果"，
    锚点逐级对比能直接指出错在哪一节。

    覆盖范围是 `robot.yaml` 的**全部关节**，含固定关节 `tool` ——
    它没有独立 `<joint>`，锚点走"子连杆 body 的 `xpos`"这条等价路径
    （见 `joint_origin_mm()` 的说明）。
    """
    js = {"base": 20.0, "shoulder": 30.0, "elbow": 120.0, "gripper": 40.0}
    sim.reset(js)
    ref = fk_joint_origins_mm(robot, js)
    assert set(ref) == {j.id for j in robot.joints}, "参考 FK 的关节集合与 robot.yaml 不一致"
    assert any(robot.joint(j).is_fixed for j in ref), (
        "本测试的价值有一半在于覆盖固定关节；robot.yaml 里若没有固定关节，"
        "这条断言应当随模型一起调整，而不是悄悄退化成一条冗余的重复检查")
    for jid, want in ref.items():
        got = joint_origin_mm(sim, robot, jid)
        assert np.allclose(got, want, atol=TOL_MM), (
            f"关节 {jid} 锚点：MuJoCo {np.round(got, 4)} vs 参考 {np.round(want, 4)}")
