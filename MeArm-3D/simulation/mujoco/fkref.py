# -*- coding: utf-8 -*-
"""参考正运动学（spec §20）—— **独立实现，只用于交叉验证，不参与运行时**。

为什么要单独写一份，而不是"复用生成器的推导"：
    如果验证用的是生成器自己的中间结果，那只能证明"生成器自洽"（自证）。
    要对 MuJoCo 模型下判断，判据必须**独立于 MJCF**。这里直接读
    `config/robot.yaml` 的原始几何（length / origin / axis / coupling）
    按链式公式算 TCP，与 MuJoCo 编译后的模型对撞。

链式公式（三处实现必须逐项一致）：
    `fk.ts`          前端 3D / IK 的 FK
    `gen_model.py`   把同一套几何翻译成 MJCF 的 `<body pos>` 与关节
    `fkref.py`       本文件，参考实现

    T ← T · Tz(length_of_parent_link) · T(origin.position)
          · R_eulerXYZ(origin.rotation) · R_axis(joint.axis, θ_effective)

其中 `θ_effective = θ_关节 + gain × θ_被耦合关节`（elbow 与 shoulder 的平行四连杆耦合）。
TCP 是 `tcp.joint` 坐标系下的 `tcp.offset`，所以链算到该关节为止。

单位：**输入输出全部是 mm 与 deg**（robot.yaml 的单位），与 SI 无关 ——
这样参考实现可以和配置文件逐字对照，不必在脑子里做换算。
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

import numpy as np

from robotcfg import JointCfg, RobotCfg

# 三维旋转用行向量约定：p_world = p_local @ R（与 numpy 的行主序配合方便）。
# 这里统一用**列向量左乘**约定（T 是 4×4 齐次矩阵，v_world = T @ v_homo），
# 与 fk.ts 的矩阵语义一致。


def _eye() -> np.ndarray:
    return np.eye(4, dtype=float)


def translate(v: Sequence[float]) -> np.ndarray:
    t = _eye()
    t[0, 3], t[1, 3], t[2, 3] = (float(v[0]), float(v[1]), float(v[2]))
    return t


def translate_z(d: float) -> np.ndarray:
    t = _eye()
    t[2, 3] = float(d)
    return t


def rot_x(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    t = _eye()
    t[1, 1], t[1, 2], t[2, 1], t[2, 2] = c, -s, s, c
    return t


def rot_y(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    t = _eye()
    t[0, 0], t[0, 2], t[2, 0], t[2, 2] = c, s, -s, c
    return t


def rot_z(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    t = _eye()
    t[0, 0], t[0, 1], t[1, 0], t[1, 1] = c, -s, s, c
    return t


def euler_xyz(rot_deg: Sequence[float]) -> np.ndarray:
    """intrinsic XYZ：R = Rx · Ry · Rz（与 three.js `Euler('XYZ')` 同序）。

    ⚠️ 顺序写错是本项目踩过的坑：它与 "extrinsic XYZ"（即 Rz·Ry·Rx）**不等价**，
    而且在小角度下差别极小 —— 只有在大角度下才会暴露成几十毫米的 TCP 偏差。
    """
    rx, ry, rz = (float(x) for x in rot_deg)
    return rot_x(rx) @ rot_y(ry) @ rot_z(rz)


def axis_angle(axis: Sequence[float], deg: float) -> np.ndarray:
    """绕任意轴的旋转（Rodrigues 公式）。"""
    a = np.array([float(x) for x in axis], dtype=float)
    n = float(np.linalg.norm(a))
    if n == 0.0:
        raise ValueError("关节 axis 不能是零向量")
    a = a / n
    th = math.radians(float(deg))
    c, s = math.cos(th), math.sin(th)
    kx, ky, kz = a
    k = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]], dtype=float)
    r = np.eye(3) + s * k + (1.0 - c) * (k @ k)
    t = _eye()
    t[:3, :3] = r
    return t


def joint_value_deg(joint: JointCfg, joints: Mapping[str, float]) -> float:
    """关节角（**绝对语义**, deg）。

    ⚠️ 被动关节（本机的腕 `tool`）**不在 `joints` 里** —— 它根本不是状态变量：
    值由定义决定，恒取锁定角 `limit_min`（= 绝对倾角 90°，爪水平）。
    与前端 `jointAngleOf()` 的缺省回退是**同一条规则**
    （`state[id] ?? limits.min`），三处实现必须逐项一致。
    """
    if joint.id in joints:
        return float(joints[joint.id])
    if joint.is_fixed:
        return 0.0
    return float(joint.limit_min)


def effective_angle_deg(joint: JointCfg, joints: Mapping[str, float]) -> float:
    """关节角 + 耦合项（平行四连杆）。固定关节恒为 0。"""
    if joint.is_fixed:
        return 0.0
    value = joint_value_deg(joint, joints)
    if joint.coupling:
        other, gain = joint.coupling
        value += float(gain) * float(joints.get(other, 0.0))
    return value


def fk_chain(robot: RobotCfg, joints: Mapping[str, float],
             stop_joint: str | None = None) -> np.ndarray:
    """算到某个关节为止的齐次变换（默认算到 `tcp.joint`）。"""
    stop = stop_joint or robot.tcp_joint
    t = _eye()
    for _link, joint in robot.chain_from_root():
        if joint is None:
            continue                                   # 根连杆没有进入关节
        parent = robot.link(joint.parent_link)
        t = t @ translate_z(parent.length)
        t = t @ translate(joint.origin_position)
        t = t @ euler_xyz(joint.origin_rotation)
        t = t @ axis_angle(joint.axis, effective_angle_deg(joint, joints))
        if joint.id == stop:
            break
    return t


def fk_tcp_mm(robot: RobotCfg, joints: Mapping[str, float]) -> np.ndarray:
    """TCP 的世界坐标（mm）。"""
    return (fk_chain(robot, joints) @ translate(robot.tcp_offset))[:3, 3].copy()


def fk_joint_origins_mm(robot: RobotCfg,
                        joints: Mapping[str, float]) -> dict[str, np.ndarray]:
    """每个关节坐标系原点的世界坐标（mm）—— 用于定位"哪一段的偏差最大"。"""
    out: dict[str, np.ndarray] = {}
    t = _eye()
    for _link, joint in robot.chain_from_root():
        if joint is None:
            continue
        parent = robot.link(joint.parent_link)
        t = t @ translate_z(parent.length)
        t = t @ translate(joint.origin_position)
        t = t @ euler_xyz(joint.origin_rotation)
        out[joint.id] = t[:3, 3].copy()
        t = t @ axis_angle(joint.axis, effective_angle_deg(joint, joints))
    return out
