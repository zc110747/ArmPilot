# -*- coding: utf-8 -*-
"""**MeArm-V1 的结构性断言** —— 从 `tests/sim/` 迁入（Phase 2 步⑤）。

## 为什么这几条属于**包**而不是 Core

判据只有一条：**把这条断言放到另一台机器人上，它还是"对的说法"吗？**

| 断言 | 换一台机器人还成立吗 |
|---|---|
| `test_zero_pose_tcp_is_offset_by_the_wrist` | ✗ 它断言"竖直段 = column+upper_arm+forearm、水平段 = tool_link 那 40mm" —— 这是 MeArm 的连杆命名与被动腕 90° 锁定的**合力结果** |
| `test_gripper_does_not_move_tcp` | ✗ 依据是 `robot.yaml` 刻意把 `tcp.joint` 取为 `tool`（被动腕）而不是夹爪 |
| `test_coupling_heavy_poses_fk_agreement` | ✗ 专打 `elbow` 存**绝对角** + 与 `shoulder` 经平行四连杆耦合（gain = −1）的重灾区 |
| `test_arithmetic_legal_region_is_oblique` | ✗ 证明合法域在局部角空间是**斜的**（`elbow.min > shoulder.max` 造成的），是 MeArm 的耦合结构决定的 |
| `test_mujoco_accepts_real_machine_impossible_pose` | ✗ 反例窗口只有 2°，是**被动腕**这个 MeArm 特有构造的副产品 |
| `test_upper_layer_rejects_that_same_pose` | ✗ 与上一条配对，打的是同一个窗口 |

留 Core 的那一半是**机制**：随机位形 FK 对比、角点扫描、锚点逐级核对、
限位校验的格式与顺序语义 —— 那些换台机器人只是**数据**不同。

## ⚠️ 迁移纪律（本次执行时逐条核对的）

1. **一条都不许删、不许放宽**：判据、容差、反例构造全部逐字照搬。
2. **不许批量复制 Core 的辅助函数进来**：`joint_origin_mm` 之类的机制留在 Core，
   这里只 import 用得到的那几个（`mujoco_tcp_mm` / `fk_tcp_mm` / `validate_joints`）。
3. **迁走的函数在 Core 里留"指针注释"，不是留尸体**：留一份注释掉的副本迟早
   会有人取消注释，然后两处一起漂移。
4. **期望值从真值派生**（`robot.joint(...)`），不写第二份字面量 ——
   这一条在迁移时把原 `"ERR JOINT elbow 53.00 (limit 108.44..141.86)"` 修掉了。
"""
from __future__ import annotations

import re

import numpy as np
import pytest

from conftest import PACKAGE_ID, SHARED_MEARM_ID
from fkref import fk_tcp_mm
from harness import MM_TOL as TOL_MM
from harness import mujoco_tcp_mm
from limits import validate_joints


# ---------------------------------------------------------------------------
# 0. 身份自检 —— 复用共享夹具时必须防的那件事
# ---------------------------------------------------------------------------


def test_package_identity() -> None:
    """本目录**必须**是 `mearm-v1` 的包 —— 否则下面的断言会张冠李戴。

    `tests/sim/conftest.py` 的 `robot` 夹具写死了 `mearm-v1`（刻意显式，不靠选择器
    的 `default`）。包内测试复用它 ⇒ 一旦本文件被拷到 `so-arm101/tests/` 而没改
    夹具，这里的六条断言会**以 SO-101 的名义**全绿，测的却是 MeArm。

    这条断言把那种"拷过去忘了改"从"静默假绿"变成"启动即红"。
    """
    assert PACKAGE_ID == SHARED_MEARM_ID == "mearm-v1", (
        f"包目录名 {PACKAGE_ID!r} 与共享夹具写死的 {SHARED_MEARM_ID!r} 不一致。"
        "包内测试若要用共享夹具，两者必须相同；不同的话请在本目录的 conftest.py 里"
        "按自己的 id 建夹具，而不是复用这一份。")


# ---------------------------------------------------------------------------
# 1. 零位几何（独立于 FK 实现的三段常量核对）
# ---------------------------------------------------------------------------


def test_zero_pose_tcp_is_offset_by_the_wrist(sim, robot):
    """零位：竖直段只有 column + upper_arm + forearm；`tool_link` 那 40mm 是**水平**的。

    ⚠️ 这条断言的形状由被动腕决定：爪被锁成水平（绝对倾角 90°），
    所以零位 TCP 不在正上方，而是落在 `(40, 0, 220)`。
    老模型（爪固连在小臂上、沿小臂延伸）才会得到 `(0, 0, 260)`。

    它是一个**独立于 FK 实现**的几何常量核对：
      竖直段 = column + upper_arm + forearm 三个 length 之和
      水平段 = tool_link.length（锁 90° ⇒ 这 40mm 完全落在 +径向）
    """
    zero = {j.id: 0.0 for j in robot.movable_joints()}
    got = mujoco_tcp_mm(sim, zero)
    want_z = sum(robot.link(i).length for i in
                 ("column_link", "upper_arm_link", "forearm_link"))
    want_x = robot.link("tool_link").length
    assert got[2] == pytest.approx(want_z, abs=TOL_MM), f"竖直段 {got[2]} vs {want_z}"
    assert got[0] == pytest.approx(want_x, abs=TOL_MM), f"水平段 {got[0]} vs {want_x}"
    assert abs(got[1]) < TOL_MM


def test_gripper_does_not_move_tcp(sim, robot):
    """夹爪开合不得影响 TCP（`robot.yaml` 刻意把 `tcp.joint` 取为 `tool`，不是夹爪）。"""
    base = {"base": 0.0, "shoulder": 20.0, "elbow": 125.0, "gripper": 0.0}
    a = fk_tcp_mm(robot, dict(base, gripper=0.0))
    b = fk_tcp_mm(robot, dict(base, gripper=90.0))
    assert np.array_equal(a, b), f"参考 FK 里夹爪影响了 TCP：{a} vs {b}"
    pa = mujoco_tcp_mm(sim, dict(base, gripper=0.0))
    pb = mujoco_tcp_mm(sim, dict(base, gripper=90.0))
    assert np.array_equal(pa, pb), f"MuJoCo 里夹爪影响了 TCP：{pa} vs {pb}"


# ---------------------------------------------------------------------------
# 2. 耦合重灾区的批量对比（随机采样命中率太低，这里专打边界）
# ---------------------------------------------------------------------------


def test_coupling_heavy_poses_fk_agreement(sim, robot):
    """专打**耦合最重**的区域：shoulder 与 elbow 同时扫到两端。

    `elbow` 存的是**绝对倾角**、且与 shoulder 通过平行四连杆耦合（gain = −1）。
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
    print(f"[FK·MeArm-3D 耦合区] N={len(errs)}  max={max(errs):.3e} mm")
    assert max(errs) < TOL_MM


# ---------------------------------------------------------------------------
# 3. 合法域是"斜的" ⇒ 单一 hinge range 表达不了（含反例实测）
# ---------------------------------------------------------------------------


def test_arithmetic_legal_region_is_oblique(robot):
    """★ 算术证明：合法域在**局部角空间是斜的**，单一 range 无法表达。

    这是纯配置算术（不是仿真观测），用于给下面的实测提供理论依据。
    论证只用 `robot.joint(...)` 的限位，所以**换台机器人也照样算** ——
    但"它是斜的"这个结论依赖 `elbow.min > shoulder.max` 的耦合结构，
    因此这条断言住在 MeArm 的包里。
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

    真机：`elbow` 绝对角低于下限 ⇒ 舵机 S8 结构上转不到；
    MuJoCo：它只看到肘的**局部角**（= θe − θs），落在外接区间内 ⇒ 照常执行。
    这不是 MuJoCo 的 bug，而是"单一 hinge range 表达不了斜的合法域"的必然结果。
    本测试把它固定成**已知事实**，防止后来者误以为 hinge range 就是限位真值。

    ⚠️ 反例窗口只有约 **2°**，这是被动腕带来的**副产品**（务必看完再改）：
    被动腕 `tool` 也是一条 hinge，它的外接区间由 elbow 的限位派生；
    与等式约束联立后**恰好**把 elbow 的绝对角钳在 ≥ `truth_min − pad`。
    于是"命令得动、但真机做不到"的区间被压缩成

        [tool_hi 派生下限, truth_min)      ← 下界来自 tool 的 hinge 区间

    也就是说 MuJoCo 现在**部分**挡住了越限位形（拦得住夸张的，拦不住窗口内的）。
    ⇒ 结论不变、而且更强：限位一致性的唯一把关人**仍然只能是上层**
      （IK / Go controller / `limits.py`），不能指望物理引擎替我们挡。
    """
    truth_min = sim.robot.joint("elbow").limit_min
    lock = sim.robot.joint("tool").limit_min          # 被动腕的锁定绝对角（= 90°）
    tool_hi = float(np.degrees(
        sim.model.jnt_range[sim.joint_ids.index("tool")][1]))    # tool hinge 区间上界
    window_lo = lock - tool_hi                        # q_t 顶到上界时的 elbow 下界

    elbow_cmd = 0.5 * (window_lo + truth_min)         # 取窗口正中
    assert window_lo < elbow_cmd < truth_min, (
        f"反例构造失败：{elbow_cmd:.4f}° 不在真机做不到的窗口 "
        f"[{window_lo:.4f}, {truth_min:.4f}) 内")

    sim.reset()
    sim.set_target_joints({"shoulder": -6.0, "elbow": elbow_cmd})
    sim.settle(8.0)
    st = sim.state()

    assert st.joint_angles["elbow"] < truth_min, (
        f"（前提变了）MuJoCo 竟然拦住了 elbow：实测 {st.joint_angles['elbow']:.4f}°，"
        f"真机下限 {truth_min:.4f}° —— 若它真能拦住，本测试与配套的 "
        f"`test_ik.py::test_geometrically_reachable_but_limits_block` 都要重新论证")
    # 记录实际位形，便于回归时发现物理行为漂移
    assert st.joint_angles["elbow"] == pytest.approx(elbow_cmd, abs=1.0), (
        f"MuJoCo 应基本如实执行（只受 hinge range 约束），实测 {st.joint_angles['elbow']:.4f}°")
    assert st.joint_angles["shoulder"] == pytest.approx(-6.0, abs=3.0)


def test_upper_layer_rejects_that_same_pose(sim):
    """★ 配对测试：同一条命令，上层校验**必须**拒绝（那是限位一致性的唯一保证）。

    ⚠️ 期望的文案**从 robot.yaml 派生**，不写第二份字面量。原版把
    `"ERR JOINT elbow 53.00 (limit 108.44..141.86)"` 抄在测试里 —— 那是把
    限位真值复制了一份，重新标定后会以"文案不对"的面目失败，而错因是测试过期。
    这里用 `fullmatch`，于是"两位小数"「`..`」这些**格式契约**仍被钉住。
    """
    el = sim.robot.joint("elbow")
    cmd = {"shoulder": -6.0, "elbow": 53.0}
    v = validate_joints(sim.robot, cmd)
    assert v is not None, "上层竟未拒绝真机不可达的位形"
    assert v.joint_id == "elbow"
    assert re.fullmatch(
        rf"ERR JOINT elbow 53\.00 \(limit {el.limit_min:.2f}\.\.{el.limit_max:.2f}\)",
        v.message(),
    ), v.message()
