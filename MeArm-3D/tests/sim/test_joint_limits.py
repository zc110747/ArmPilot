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


def test_arithmetic_legal_region_is_oblique(robot):
    """★ 算术证明：合法域在**局部角空间是斜的**，单一 range 无法表达。

    这是纯配置算术（不是仿真观测），用于给下面的实测提供理论依据。

    `elbow` 的绝对限位 [108.4415, 141.8582]、`shoulder` 的 [-6.0937, 49.4549]，
    局部角 L = θ_elbow − θ_shoulder 的外接区间是
        [108.4415 − 49.4549, 141.8582 + 6.0937] = [58.9866, 147.9519]
    但取该区间的**下界**配合肩角**下界**，得到的绝对是越界的：
        θ_elbow = 58.9866 + (−6.0937) = 52.89°  <  108.4415°   ✗
    ⇒ 外接盒严格大于合法域。反之"内切"是空集（下界 114.5352 > 上界 92.4033）。
    """
    e = robot.joint("elbow")
    s = robot.joint("shoulder")
    L_lo = e.limit_min - s.limit_max
    theta_e_illegal = L_lo + s.limit_min
    assert theta_e_illegal < e.limit_min - 50.0, "反例构造失败"

    inner_lo = e.limit_min - s.limit_min
    inner_hi = e.limit_max - s.limit_max
    assert inner_lo > inner_hi, (
        f"内切区间非空（{inner_lo:.4f} … {inner_hi:.4f}）⇒ 前提变了，"
        f"请重新检查 ARCHITECTURE_ANALYSIS §6.3"
    )


def test_mujoco_accepts_real_machine_impossible_pose(sim):
    """★★ 决定性实测：MuJoCo 会执行一个真机**物理上不可能**的位形。

    命令 `(shoulder = −6.0°, elbow = 53.0°)`：
      - 真机：elbow 绝对角 53° 远低于下限 108.4415° ⇒ 舵机 S8 结构上转不到
      - MuJoCo：只看到局部角 53 − (−6) = 59°，落在外接区间 [56.99, 149.95] 内 ⇒ 照常执行

    这不是 MuJoCo 的 bug，而是"单一 hinge range 表达不了斜的合法域"的必然结果。
    本测试把它固定成**已知事实**，防止后来者误以为 MuJoCo 的 range 就是限位真值。
    """
    sim.reset()
    sim.set_target_joints({"shoulder": -6.0, "elbow": 53.0})
    sim.settle(8.0)
    st = sim.state()
    truth_min = sim.robot.joint("elbow").limit_min

    assert st.joint_angles["elbow"] < truth_min - 40.0, (
        f"（前提变了）MuJoCo 竟然拦住了 elbow："
        f"实测 {st.joint_angles['elbow']:.3f}°，真机下限 {truth_min:.3f}°"
    )
    # 记录实际位形，便于回归时发现物理行为漂移
    assert st.joint_angles["shoulder"] == pytest.approx(-6.0, abs=3.0)


def test_upper_layer_rejects_that_same_pose(sim):
    """★ 配对测试：同一条命令，上层校验**必须**拒绝（那是限位一致性的唯一保证）。"""
    cmd = {"shoulder": -6.0, "elbow": 53.0}
    v = validate_joints(sim.robot, cmd)
    assert v is not None, "上层竟未拒绝真机不可达的位形"
    assert v.joint_id == "elbow"
    assert v.message() == "ERR JOINT elbow 53.00 (limit 108.44..141.86)"


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
