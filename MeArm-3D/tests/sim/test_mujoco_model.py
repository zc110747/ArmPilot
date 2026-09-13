"""Phase 2 验收：MJCF 模型本身正确（spec §36 Acceptance 1、§7、§11）。

判据全部取自**独立于生成器**的来源：
  - MuJoCo 编译后的运行时模型（`sim.model`）
  - `config/robot.yaml`（真值）
  - 手算的几何常量
不拿生成器的中间结果当证据（那只能证明生成器自洽）。
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from conftest import zero_pose


def test_model_compiles(sim):
    """spec §36 Acceptance 1：XML 可以正常加载。"""
    m = sim.model
    # ⚠️ nq = 5 而 nu = 4（**刻意不等**）：被动腕 `tool` 是真实存在的转动副，
    # 必须建模成 hinge，否则爪不会随小臂保持水平；但它**没有自由度** ——
    # 被 `<tendon>` + `<equality>` 锁死在「绝对倾角 = 90°」，也没有执行器。
    # `nq` 数的是**坐标**，`nu` 数的是**驱动**。见 docs/decisions.md D70。
    assert m.nq == 5, f"应有 5 个 qpos 坐标（4 自由度 + 1 被动腕），实测 {m.nq}"
    assert m.nv == 5
    assert m.njnt == 5
    assert m.nu == 4, f"应有 4 个执行器（被动腕没有输入），实测 {m.nu}"
    # 锁定机构必须是"固定腱 + 等式约束"：腱是 qpos 的线性组合，才能表达
    # 「绝对角 = 常量」这种**跨多个关节**的关系（只写 `joint1/joint2` 会漏项，见 D70）
    assert m.neq == 1, f"应有 1 条等式约束（被动腕锁定），实测 {m.neq}"
    assert m.ntendon == 1, f"应有 1 条固定腱，实测 {m.ntendon}"
    # world + 6 个机械臂体 + 场景静态体（工作台）
    assert m.nbody == len(ARM_BODIES) + 2, f"刚体数应为 {len(ARM_BODIES) + 2}，实测 {m.nbody}"


def test_no_nan_or_invalid_inertia(sim):
    """spec §36 Acceptance 1：不能存在 NaN / invalid inertia。"""
    m = sim.model
    assert np.isfinite(m.body_mass).all(), "body_mass 含 NaN/Inf"
    assert np.isfinite(m.body_inertia).all(), "body_inertia 含 NaN/Inf"
    ids = arm_body_ids(m)
    assert (m.body_mass[ids] > 0).all(), "机械臂存在零质量 body"
    assert (m.body_inertia[ids] > 0).all(), "机械臂存在零惯量 body（会导致关节不受力矩）"
    assert np.isfinite(m.jnt_range).all()


def test_total_mass_matches_config(sim, physics):
    """总质量必须等于 physics.yaml 各 body 质量之和（单位一致性的硬校验）。

    ⚠️ 这一条抓的是"单位双重换算"这类**完全静默**的错误：
    早期版本把 physics.yaml 的米又乘了一次 1e-3，惯量掉到 1e-11，
    格式化后显示成 0，而模型照样能编译通过。
    """
    expect = sum(
        float(physics.body_inertia_of(l.id)["mass"]) for l in sim.robot.links
    )
    # 只统计**机械臂** body：场景静态体（工作台）质量合法地为 0，不参与这条核对
    got = float(sim.model.body_mass[arm_body_ids(sim.model)].sum())
    assert got == pytest.approx(expect, rel=1e-9)


def test_body_tree_matches_robot_chain(sim):
    """刚体树必须与 robot.yaml 的连杆链逐级一致（父子关系不能错）。"""
    m = sim.model
    id_of = {body_name(m, i): i for i in range(m.nbody)}

    expected_edges = []
    for link in sim.robot.links:
        if link.parent:
            expected_edges.append((link.parent, link.id))

    actual_edges = []
    for name, bid in id_of.items():
        p = int(m.body_parentid[bid])
        if p > 0:
            actual_edges.append((body_name(m, p), name))

    assert sorted(actual_edges) == sorted(expected_edges)


def test_joint_axes_and_order(sim):
    """关节的顺序与轴必须与 robot.yaml 一致（位次错位是最危险的错误）。

    ⚠️ 对照的是 `qpos_joints()`（**含被动腕**），不是 `movable_joints()`：
    MJCF 里 5 个 hinge 一个都不能少，顺序也必须是 yaml 的声明顺序 ——
    `model.py` 启动时会拿 `angle_map.joint_ids` 再做一次同样的自检。
    """
    m = sim.model
    got_order = [
        (joint_name(m, i), tuple(np.round(m.jnt_axis[i], 9))) for i in range(m.njnt)
    ]
    expected_order = [
        (j.id, tuple(round(float(a), 9) for a in j.axis)) for j in sim.robot.qpos_joints()
    ]
    assert got_order == expected_order

    # 顺带把"qpos 坐标"与"状态帧自由度"这两个**刻意不相等**的集合钉住
    assert [j.id for j in sim.robot.qpos_joints()] == sim.angle_map.joint_ids
    assert sim.angle_map.dof_ids == sim.robot.joint_order()
    assert sim.angle_map.dof_ids == ["base", "shoulder", "elbow", "gripper"]
    assert len(sim.angle_map.joint_ids) == len(sim.angle_map.dof_ids) + 1


def test_tcp_site_at_geometric_sum(sim, robot):
    """零位 TCP 的几何常量核对：**高度** = column + upper_arm + forearm，**前伸** = tool 长度。

    ⚠️ 不能把 `tool_link.length` 加进高度 —— 那是**旧模型**（爪刚性固连小臂）的算法。
    `tool` 现在是被动腕关节（爪锁水平），其坐标系原点就是**腕枢轴**，而
    `tool_link.length = 40` 量的是「腕枢轴 → 爪铰点」的**水平**前伸量：
    爪锁水平 ⇒ 该段完全落在 +X 上。`tcp.offset [0,0,40]` 沿 tool 关节坐标系 +Z
    伸出，零位下该 +Z 恰好也指向 +X，两者同向叠加。

    ⇒ 零位 TCP = (40, 0, 60+80+80) = (40, 0, 220)mm。
    """
    sim.reset(zero_pose(robot))
    p = sim.end_effector_mm()
    expected_z = (robot.link("column_link").length + robot.link("upper_arm_link").length
                  + robot.link("forearm_link").length)
    assert p[2] == pytest.approx(expected_z, abs=1e-9), "零位 TCP 高度"
    assert p[0] == pytest.approx(robot.link("tool_link").length, abs=1e-9), "零位 TCP 前伸"
    assert abs(p[1]) < 1e-9


def test_tcp_unaffected_by_gripper(sim, robot):
    """TCP 必须**不被夹爪开合角污染**（robot.yaml 刻意把 tcp.joint 取为 tool）。

    这条测试守着一个很容易被"顺手优化"掉的架构决定：
    若把 TCP 挂到 jaw 上，夹爪一动末端位置就跟着动，整套 IK 会静默失效。

    ⚠️ 用 **`reset()` 后的纯运动学**结果比较（`reset` 内部调了 `mj_forward`），
    而不是"step 一段时间后的动力学结果"—— 后者会因重力/接触/求解器迭代
    带入 ~10µm 的**动力学噪声**，把一个"定义层面"的断言变成一个会飘的断言。
    """
    a = dict(robot.home_pose)
    b = dict(robot.home_pose)
    b["gripper"] = b["gripper"] + 30.0
    sim.reset(a)
    pa = sim.end_effector_mm()
    sim.reset(b)
    pb = sim.end_effector_mm()
    # 纯运动学量，应**逐位相同**（不是"接近"）
    assert np.array_equal(pa, pb), f"夹爪影响了 TCP：{pa} vs {pb}"


def test_every_movable_joint_has_actuator(sim):
    """每个可动关节都必须有执行器（否则命令发不出去）。"""
    for j in sim.robot.movable_joints():
        assert j.id in sim._actuator_of_joint, f"关节 {j.id} 没有执行器"


def test_local_range_is_outer_bound(sim, robot, physics):
    """MJCF 的 hinge range 必须等于 robot.yaml 限位换算出的**外接区间**（+ padding）。

    这条把 ARCHITECTURE_ANALYSIS §6.3 的推导变成可执行断言：
    `elbow` 的外接区间是 [108.4415 − 49.4549, 141.8582 + 6.0937]（含 padding ±2）。
    """
    pad = float(physics.limits["range_padding_deg"])
    m = sim.model
    adr = {joint_name(m, i): i for i in range(m.njnt)}
    for j in robot.movable_joints():
        i = adr[j.id]
        lo, hi = (math.degrees(float(x)) for x in m.jnt_range[i])
        exp_lo = j.limit_min - pad
        exp_hi = j.limit_max + pad
        if j.coupling:
            other, gain = j.coupling
            oj = robot.joint(other)
            exp_lo = (j.limit_min - pad) + (gain * oj.limit_max if gain < 0 else gain * oj.limit_min)
            exp_hi = (j.limit_max + pad) + (gain * oj.limit_min if gain < 0 else gain * oj.limit_max)
        assert lo == pytest.approx(exp_lo, abs=1e-6), f"{j.id} 下限"
        assert hi == pytest.approx(exp_hi, abs=1e-6), f"{j.id} 上限"


def test_deterministic_reset(sim, robot):
    """spec §32/§33：相同 reset 入参 ⇒ 逐位相同的初始状态。"""
    p = dict(robot.home_pose)
    sim.reset(p)
    a = np.array(sim.data.qpos, copy=True)
    sim.step_seconds(0.5)
    sim.reset(p)
    b = np.array(sim.data.qpos, copy=True)
    assert np.array_equal(a, b)
    assert np.array_equal(np.array(sim.data.qvel), np.zeros(sim.model.nv))


# ---------------------------------------------------------------------------
# 辅助
# ---------------------------------------------------------------------------

# 机械臂自身的刚体（root → tip）。场景静态体（工作台等）刻意**不在**这一列。
ARM_BODIES = ("base_link", "column_link", "upper_arm_link", "forearm_link",
              "tool_link", "jaw_link")


def arm_body_ids(model) -> list[int]:
    """只取**机械臂**的 body 索引。

    ⚠️ 别对 `body_mass[1:]` 做全数组断言。场景里随时可能加入静态体
    （工作台 / 障碍物 / 相机支架），它们的质量与惯量**合法地为 0**，
    会让"存在零质量运动 body"这类断言误报 —— 本文件就实际发生过：
    加入工作台后 `nbody` 由 7 变 8、`body_mass[1] == 0`，两条断言立刻变红。
    """
    import mujoco

    ids = []
    for n in ARM_BODIES:
        i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
        assert i >= 0, f"模型里找不到机械臂 body {n}"
        ids.append(i)
    return ids


def body_name(model, i: int) -> str:
    import mujoco

    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""


def joint_name(model, i: int) -> str:
    import mujoco

    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or ""
