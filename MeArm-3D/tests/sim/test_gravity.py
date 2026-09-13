"""Phase 3 验收：重力与惯量真的在起作用（spec §18 Gravity Test、§36 Acceptance 6）。

判据设计原则
------------
"机械臂动了"本身**不是**重力生效的证据 —— 位置环也能让它动。
必须做**对照**：同一初始条件、同一驱动状态，**只改重力**，
观察到的差异才唯一地归因于重力。`test_no_gravity_no_motion` 就是这个对照。
"""
from __future__ import annotations

import numpy as np
import pytest


def _delta_deg(a: dict[str, float], b: dict[str, float]) -> dict[str, float]:
    return {k: b[k] - a[k] for k in a}


def test_gravity_pulls_arm_down(sim):
    """spec §18 Test 1：禁用致动器后，机械臂**不会**保持绝对静止。

    HOME 位 shoulder≈0.85°（大臂近竖直）、elbow≈112.6°（小臂前倾 22.6°），
    小臂侧的重力矩显著 ⇒ 应当明显塌落。
    """
    sim.reset()
    sim.set_actuation_enabled(False)
    before = sim.joint_angles_deg()
    sim.step_seconds(3.0)
    after = sim.joint_angles_deg()

    d = _delta_deg(before, after)
    worst = max(abs(v) for v in d.values())
    assert worst > 5.0, f"无驱动时应因重力明显位移，实测各关节位移 {d}"

    # 末端的落点也必须变（不能只是关节空转）
    assert abs(after["elbow"] - before["elbow"]) > 5.0, f"小臂未塌落：{d}"

    sim.set_actuation_enabled(True)


def test_no_gravity_no_motion(sim):
    """对照：关掉重力后，无驱动 + 无初速 ⇒ **完全不动**。

    这一条把"位移确实来自重力"与"位移来自数值噪声 / 接触力 / 积分漂移"区分开。
    没有它，上面那条测试通过也可能只是因为求解器在发散。
    """
    sim.set_gravity(False)
    sim.reset()
    sim.set_actuation_enabled(False)
    before = sim.joint_angles_deg()
    sim.step_seconds(3.0)
    after = sim.joint_angles_deg()

    for jid, d in _delta_deg(before, after).items():
        assert abs(d) < 1e-6, f"无重力时 {jid} 不该动，实测 Δ={d:.3e}°"

    sim.set_actuation_enabled(True)
    sim.set_gravity(True)


def test_hold_home_resists_gravity(sim):
    """spec §18 Test 3：逐渐增加执行器控制 ⇒ 能抵抗重力。

    有驱动时姿态保持在 HOME 附近（不是精确等于 —— 见下一条测试）。
    """
    sim.reset()
    sim.settle(8.0)
    st = sim.state()
    for jid, target in sim.robot.home_pose.items():
        err = abs(st.joint_angles[jid] - target)
        assert err < 2.0, f"{jid} 偏离 HOME {err:.3f}°（位置环未能抵抗重力）"


def _dof_row(sim) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """取出**自由度顺序**的 (qpos_local, ctrl, τ_bias) 三个向量。

    ⚠️ 三个顺序互不相同，必须分别映射，不能图省事切片：
      * `data.qpos` 是 **qpos 顺序**（含被动腕 `tool`，夹在 elbow 与 gripper 中间）
      * `data.ctrl` 是 **执行器声明顺序**
      * 状态帧是 `robot.joint_order()`（= 4 个自由度）
    被动腕的坐标一旦被错位当成 gripper，量纲相同、不报错，只会让这条断言悄悄失真。
    """
    order = sim.robot.joint_order()
    qpos = np.array([float(sim.data.qpos[sim.angle_map.qpos_index(j)]) for j in order])
    return qpos, sim.ctrl_vector(), sim.gravity_torque() + sim.constraint_torque()


def test_steady_state_error_is_physical(sim):
    """★ 稳态误差必须与「(重力矩 + 约束力矩) ÷ kp」自洽 —— 这是"真物理"与"装出来"的分界。

    位置环是 P 控制：稳态时 `kp·(ctrl − qpos) = τ_bias`。
    因此 `qpos = ctrl − τ_bias/kp`，误差**必然非零**。

    若误差恒为 0（例如有人把 kp 调到无穷、或直接用 qpos 写目标），
    这条会立刻失败 —— 那是"凭空造出一个无限刚性舵机"。

    ⚠️ `τ_bias` 必须**同时**含重力项与**约束项**：被动腕的「绝对角锁定」是一条跨
    `shoulder/elbow/tool` 三个坐标的等式约束，物理上等价于一根刚性连杆 ——
    它把力矩**传回被驱动的肩 / 肘**（实测该位形下 λ 约 0.007 N·m ⇒ λ/kp ≈ 1.4e-3 rad，
    与容差同量级）。漏掉它，这条断言会在容差边缘反复横跳，而表现成"偶发不稳"。

    容差取 3e-3 rad（≈0.17°），为下列两项留余量：
      * 关节库仑摩擦 frictionloss/kp = 0.005/5 = 1e-3
      * `settle()` 退出时残留的微小速度（kv·qvel 项）
    """
    sim.reset()
    sim.settle(8.0)
    kp = float(sim.physics.servo["kp"])
    qpos, ctrl, tau = _dof_row(sim)
    predicted = ctrl - tau / kp
    assert np.allclose(qpos, predicted, atol=3e-3), (
        f"稳态位置与 kp/τ 预测不符\n  qpos     ={np.round(qpos, 5)}\n"
        f"  predicted={np.round(predicted, 5)}\n  τ={np.round(tau, 5)}"
    )


def test_steady_state_error_vanishes_without_gravity(sim):
    """★ 去掉重力后，稳态误差必须**大幅缩小**（位置环不再需要对抗任何东西）。

    与上一条配对：一条证明"误差是重力造成的"，一条证明"误差确实随重力消失"。

    ⚠️ 遍历用的是 `robot.joint_order()`（4 个自由度），**不是** `sim.joint_ids`
    （5 个 qpos 坐标）：后者含被动腕 `tool`，而它既不在 `joint_angles` 里、
    也不在 `home_pose` 里 —— 拿它去索引会直接 KeyError。
    """
    order = sim.robot.joint_order()

    def err_deg() -> float:
        st = sim.state()
        return max(abs(st.joint_angles[j] - sim.robot.home_pose[j]) for j in order)

    sim.reset()
    sim.settle(8.0)
    err_g = err_deg()

    sim.set_gravity(False)
    sim.reset()
    sim.settle(8.0)
    err_ng = err_deg()

    sim.set_gravity(True)
    assert err_g > 0.2, f"有重力时应有可观测误差，实测 {err_g:.4f}°"
    assert err_ng < 0.05, f"无重力时误差应趋于 0，实测 {err_ng:.4f}°"


def test_gravity_torque_grows_with_reach(sim):
    """重力矩必须随"臂伸得更远"而增大（量纲与几何方向的双重核对）。"""
    idx = sim.robot.joint_order().index("shoulder")
    sim.reset()
    sim.settle(8.0)
    near = abs(sim.gravity_torque()[idx])

    sim.reset({"base": 0.0, "shoulder": 40.0, "elbow": 120.0, "gripper": 50.0})
    sim.settle(8.0)
    far = abs(sim.gravity_torque()[idx])

    assert far > near, f"肩重力矩应随前伸增大：近 {near:.5f} → 远 {far:.5f} N·m"
