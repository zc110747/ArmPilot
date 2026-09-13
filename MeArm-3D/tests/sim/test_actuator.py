"""Phase 4 验收：执行器 / 位置控制 / 舵机模型（spec §12、§13、§22、§36 Acceptance 3/4）。

核心纪律
--------
**运动只能经由 actuator 产生**（spec §12 禁止用 `qpos` 直写来"驱动"）。
`test_actuation_disabled_means_no_motion` 是这条纪律的可执行判据：
断开致动器后，即使照常下发命令，机械臂也**不应该**动。
若有人偷偷用 `qpos` 写目标，这条会立刻失败。
"""
from __future__ import annotations

import numpy as np
import pytest


# ---------------------------------------------------------------------------
# 单关节（spec §36 Acceptance 3 / §22 Test B）
# ---------------------------------------------------------------------------


def test_single_joint_base(sim):
    """只动 base：其余关节保持，且转角正确。"""
    sim.reset()
    sim.set_target_joints({"base": 30.0})
    sim.settle(6.0)
    st = sim.state()
    assert st.joint_angles["base"] == pytest.approx(30.0, abs=0.05)
    # base 无重力矩，其余关节应保持在 HOME 附近
    assert abs(st.joint_angles["shoulder"] - 0.8499) < 1.0
    assert abs(st.joint_angles["elbow"] - 112.6186) < 1.0


def test_single_joint_shoulder(sim):
    """只动 shoulder：竖直→前倾。"""
    sim.reset()
    sim.set_target_joints({"shoulder": 40.0})
    sim.settle(6.0)
    st = sim.state()
    # 有重力 ⇒ 稳态误差为正（往前多倒一点）
    assert st.joint_angles["shoulder"] == pytest.approx(40.0, abs=1.5)


def test_base_rotation_is_pure_yaw(sim):
    """base 是绕 Z 的纯偏航：末端高度与半径不该变。"""
    sim.reset()
    sim.settle(6.0)
    p0 = sim.end_effector_mm()
    r0 = float(np.hypot(p0[0], p0[1]))

    sim.reset()
    sim.set_target_joints({"base": 45.0})
    sim.settle(6.0)
    p1 = sim.end_effector_mm()
    r1 = float(np.hypot(p1[0], p1[1]))

    assert p1[2] == pytest.approx(p0[2], abs=0.5)
    assert r1 == pytest.approx(r0, abs=0.5)


# ---------------------------------------------------------------------------
# 关节角语义（★ 本项目最容易搞错的地方）
# ---------------------------------------------------------------------------


def test_elbow_command_is_absolute_angle(sim):
    """命令 `elbow` 必须是**绝对倾角**，MuJoCo 的 qpos 是**局部角**。

    `qpos_local[elbow] = θ_elbow − θ_shoulder`（coupling gain = −1）。
    这条测试同时锁死：① 绝对角被正确执行；② 局部角换算正确。
    """
    sim.reset()
    sim.set_target_joints({"elbow": 125.0})
    sim.settle(6.0)
    st = sim.state()
    idx = sim.joint_ids.index("elbow")

    assert st.joint_angles["elbow"] == pytest.approx(125.0, abs=1.5)
    local_deg = float(np.degrees(st.qpos_local[idx]))
    expect_local = st.joint_angles["elbow"] - st.joint_angles["shoulder"]
    assert local_deg == pytest.approx(expect_local, abs=1e-6)


def test_multi_joint_coupling_local_angle(sim):
    """★ 肩+肘同动时，局部角仍严格等于「绝对肘 − 肩」。

    这是耦合换算在**两个关节都动**时的验证 —— 单关节测试无法覆盖：
    若把绝对角直接当局部角写入，单关节时误差恰好被 shoulder 的小值掩盖，
    同动时才会暴露成"小臂跟不跟肩"的机构性错误。
    """
    sim.reset()
    sim.set_target_joints({"shoulder": 40.0, "elbow": 125.0})
    sim.settle(8.0)
    st = sim.state()
    idx = sim.joint_ids.index("elbow")
    local_deg = float(np.degrees(st.qpos_local[idx]))
    expect = st.joint_angles["elbow"] - st.joint_angles["shoulder"]
    assert local_deg == pytest.approx(expect, abs=1e-6)
    assert local_deg == pytest.approx(85.0, abs=3.0), (
        f"期望局部角≈85°（125−40），实测 {local_deg:.3f}°"
    )


def test_all_joints_together(sim):
    """spec §36 Acceptance 4：全部关节一起控制。"""
    target = {"base": -45.0, "shoulder": 30.0, "elbow": 120.0, "gripper": 20.0}
    sim.reset()
    sim.set_target_joints(target)
    sim.settle(8.0)
    st = sim.state()
    for jid, want in target.items():
        assert st.joint_angles[jid] == pytest.approx(want, abs=2.0), (
            f"{jid} 目标 {want}，实测 {st.joint_angles[jid]:.3f}"
        )


# ---------------------------------------------------------------------------
# 舵机模型（spec §13）
# ---------------------------------------------------------------------------


def test_servo_velocity_limit(sim):
    """★ 舵机有速度上限：阶跃命令后**不能**瞬移到位。

    判据：阶跃 60° 后经过 0.1s，转过的角度 ≤ max_velocity × 0.1 + 余量。
    若有人绕过速率限制（或把 position actuator 调成刚性），这条立刻失败。
    """
    sim.reset()
    vmax_deg = float(np.degrees(sim.max_velocity_rad_s))
    sim.set_target_joints({"base": 60.0})
    sim.step_seconds(0.1)
    moved = sim.state().joint_angles["base"]
    budget = vmax_deg * 0.1 + 1.0          # 余量覆盖控制周期离散化
    assert moved < budget, (
        f"0.1s 内转过 {moved:.2f}°，超过速度上限 {vmax_deg:.0f}°/s ⇒ 舵机模型没生效"
    )
    assert moved > 0.0, "命令完全没生效（控制周期累加器或 ctrl 映射可能坏了）"


def test_servo_reaches_target_eventually(sim):
    """速率限制不影响最终到位（只是变慢）。"""
    sim.reset()
    sim.set_target_joints({"base": 60.0})
    sim.settle(8.0)
    assert sim.state().joint_angles["base"] == pytest.approx(60.0, abs=0.05)


def test_actuation_disabled_means_no_motion(sim):
    """★ spec §12 的可执行判据：断开致动器 ⇒ 命令不产生运动。

    位置环被拿掉后，`ctrl` 仍然被写入（代码路径照跑），但力矩为 0。
    若实现里存在"直接写 qpos"的捷径，它**不会**被 disableflags 拦住，
    这条测试就会失败 —— 这正是它存在的意义。

    ⚠️ 必须**同时关掉重力**：否则无驱动的臂会因为重力塌落，
    测出来的位移是重力造成的，而不是"命令生效了"。两个因素必须分离。
    """
    sim.set_gravity(False)
    sim.set_actuation_enabled(False)
    sim.reset()
    before = sim.state().joint_angles
    sim.set_target_joints({"base": 50.0, "shoulder": 40.0})
    sim.step_seconds(1.0)
    after = sim.state().joint_angles
    sim.set_actuation_enabled(True)
    sim.set_gravity(True)

    for jid in before:
        assert abs(after[jid] - before[jid]) < 1e-9, (
            f"致动器已禁用，{jid} 仍从 {before[jid]:.6f} 动到 {after[jid]:.6f} "
            f"⇒ 存在绕过 actuator 的驱动路径"
        )


def test_target_persists_across_control_cycles(sim):
    """目标必须被保持（不是"发一次就走一次"）。"""
    sim.reset()
    sim.set_target_joints({"base": 25.0})
    sim.step_seconds(0.5)
    t1 = sim.state().joint_angles["base"]
    sim.step_seconds(1.0)          # 继续跑，不再下发命令
    t2 = sim.state().joint_angles["base"]
    assert t2 > t1
    assert t2 == pytest.approx(25.0, abs=0.1)


# ---------------------------------------------------------------------------
# 数值稳定性（spec §22 Test D）
# ---------------------------------------------------------------------------


def test_extreme_step_does_not_explode(sim):
    """大角度阶跃（含限位边界）不得发散 / 穿模 / 无限振荡。"""
    sim.reset()
    sim.set_target_joints({"base": 60.0, "shoulder": 49.0, "elbow": 141.0, "gripper": 90.0})
    sim.step_seconds(5.0)
    st = sim.state()
    assert not np.isnan(st.qpos_local).any(), "qpos 出现 NaN"
    assert np.isfinite(st.end_effector).all(), "末端位置非有限"
    assert np.max(np.abs(st.joint_velocities)) < 50.0, (
        f"关节速度失控：{np.round(st.joint_velocities, 3)} rad/s"
    )


def test_oscillation_decays(sim):
    """到达目标后不该持续振荡（阻尼/摩擦必须真的存在）。"""
    sim.reset()
    sim.set_target_joints({"shoulder": 30.0})
    sim.settle(8.0)
    v1 = np.max(np.abs(sim.state().joint_velocities))
    sim.step_seconds(1.0)
    v2 = np.max(np.abs(sim.state().joint_velocities))
    assert v2 <= 1e-3, f"稳态仍有残余振荡：{v1:.5f} → {v2:.5f} rad/s"
