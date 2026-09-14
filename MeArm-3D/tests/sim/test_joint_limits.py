"""Phase 4 / spec §14 验收：关节限位与"四方一致"。

本文件的核心不是"限位能不能拦住"，而是**"MuJoCo 的 hinge range 不等于限位真值"**
这个架构结论 —— 它由 ARCHITECTURE_ANALYSIS §6.3 推导得出，这里用**实测**把它固定下来。

结论（三条，各有对应测试）：
  1. MuJoCo 的 hinge range 是**保守外接**，比真机限位宽（含 padding）→ `test_*padding*`
  2. 因此 MuJoCo 会**放行**真机不可能达到的位形 → `test_mujoco_accepts_...`
  3. 限位一致性必须由上层（Go controller / Python `limits.py`）保证 → `test_upper_*`
"""
from __future__ import annotations

import numpy as np
import pytest

from limits import validate_joints, violators


# ---------------------------------------------------------------------------
# 1. 物理层的保护确实存在（但它只是"保护"）
# ---------------------------------------------------------------------------


def test_upper_limit_blocks_motion(sim):
    """命令远超物理上限时，机构被拦在 range 内。"""
    sim.reset()
    sim.set_target_joints({"shoulder": 200.0})
    sim.settle(8.0)
    got = sim.state().joint_angles["shoulder"]
    phys_max = float(np.degrees(sim.model.jnt_range[sim.joint_ids.index("shoulder")][1]))
    assert got <= phys_max + 0.2, f"越过物理上限：{got:.4f} > {phys_max:.4f}"
    assert got > 40.0, "命令完全没生效"


def test_lower_limit_blocks_motion(sim):
    """命令远低于物理下限时，机构被拦在 range 内。"""
    sim.reset()
    sim.set_target_joints({"shoulder": -80.0})
    sim.settle(8.0)
    got = sim.state().joint_angles["shoulder"]
    phys_min = float(np.degrees(sim.model.jnt_range[sim.joint_ids.index("shoulder")][0]))
    assert got >= phys_min - 0.2, f"越过物理下限：{got:.4f} < {phys_min:.4f}"


def test_physical_range_includes_padding(sim, physics):
    """★ MuJoCo 的 range 比真机限位**宽** padding（这是刻意的，不是配错了）。

    padding 的作用：让"上层拒绝一条越界命令"这件事**可被观测** ——
    如果物理层先把命令按真值钳住了，我们就分不清"是上层拒了"还是"物理层钳了"。
    """
    pad = float(physics.limits["range_padding_deg"])
    assert pad > 0.0
    sim.reset()
    sim.set_target_joints({"shoulder": 200.0})
    sim.settle(8.0)
    got = sim.state().joint_angles["shoulder"]
    truth = sim.robot.joint("shoulder").limit_max
    assert got > truth + pad * 0.5, (
        f"物理上限（{got:.3f}°）没有超过真机限位（{truth:.3f}°）+ padding ⇒ "
        f"padding 没生效，上层拒绝将无法被观测"
    )


# ---------------------------------------------------------------------------
# 2. ★ 核心：MuJoCo 会放行真机不可能的位形
# ---------------------------------------------------------------------------
#
# ⚠️ Phase 2 步⑤ 迁走（**留指针，不留尸体**）：
#
#   test_arithmetic_legal_region_is_oblique
#   test_mujoco_accepts_real_machine_impossible_pose
#   test_upper_layer_rejects_that_same_pose
#       → robot-package/mearm-v1/tests/test_mearm_v1_structure.py
#
# 判据：这三条论证的是「合法域在局部角空间是**斜的**」，而"斜"是
# `elbow.min > shoulder.max`（MeArm 的平行四连杆耦合）造成的构造事实；
# 反例窗口只有 2° 更是**被动腕**这个 MeArm 特有件的副产品。换台机器人不成立 ⇒ 属于包。
# 原地迁走时顺手修掉了一处重复真值：期望文案里的 `108.44..141.86`
# 现在从 `robot.joint("elbow")` 派生。
#
# 本节留下的都是**机制**：物理层保护存在、padding 生效、上层校验与 Go 同语义。


# ---------------------------------------------------------------------------
# 3. 上层校验与 Go / 前端同语义
# ---------------------------------------------------------------------------


def test_validate_matches_go_error_format(robot):
    """文案格式必须与固件 / Go `Violation.Error()` 一致（UI 与 e2e 都依赖它）。"""
    v = validate_joints(robot, {"elbow": 90.0})
    assert v is not None
    assert v.message().startswith("ERR JOINT elbow")
    assert "(limit " in v.message()


def test_validate_accepts_boundary_values(robot):
    """限位**边界值本身**是合法的（闭区间），不能因浮点比较把它拒掉。"""
    assert validate_joints(robot, {"base": 60.0}) is None
    assert validate_joints(robot, {"base": -60.0}) is None
    assert validate_joints(robot, {j: robot.joint(j).limit_max for j in robot.joint_order()}) is None
    assert validate_joints(robot, {j: robot.joint(j).limit_min for j in robot.joint_order()}) is None


def test_validate_reports_first_violation_in_joint_order(robot):
    """多个越界时报告**关节顺序最靠前**的那个（顺序敏感，避免非确定性）。"""
    bad = {"elbow": 0.0, "base": 999.0}
    v = validate_joints(robot, bad)
    assert v is not None and v.joint_id == "base", "应按 joint_order 而非 dict 顺序报告（base 在前）"
    allv = violators(robot, bad)
    assert {x.joint_id for x in allv} == {"base", "elbow"}


def test_home_pose_is_within_limits(robot):
    """HOME 位必须在限位内，且执行器映射回舵机 90°（标定自洽性）。"""
    assert validate_joints(robot, robot.home_pose) is None
    for a in robot.actuators:
        theta = robot.home_pose[a.joint_id]
        assert a.joint_to_servo(theta) == pytest.approx(90.0, abs=1e-6), (
            f"{a.id}: HOME 位应映射到舵机 90°，实测 {a.joint_to_servo(theta):.6f}"
        )
