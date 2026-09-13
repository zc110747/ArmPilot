"""Phase 9 验收共用的小工具。

FK 与 IK 两条判据都要"在真机限位内采样位形""用 MuJoCo 做纯运动学求值"，
所以抽到这里 —— 两份实现一旦漂移，两个文件就会各自验出不同的结论，
而差异来源（采样范围不同）会被误读成"模型不一致"。

只放**与结论无关**的机械动作。任何带判断的断言都留在各自的测试文件里。
"""
from __future__ import annotations

from typing import Any, Mapping, Sequence

import numpy as np

#: 位置精度的公用容差（mm）。实测 FK 逐位一致（~1e-13 mm），
#: 这里留到 1e-6 mm —— 仍是"微米级的千分之一"，任何真实的几何/旋转约定
#: 错误（intrinsic XYZ 写成 extrinsic、耦合项漏掉）都会冲到毫米以上。
MM_TOL = 1e-6


def random_pose(robot, rng: np.random.Generator) -> dict[str, float]:
    """在**真机限位内**均匀采样一个可达位形。

    `elbow` 存的是绝对倾角，所以这里直接对 `[limit_min, limit_max]` 均匀采样即可 ——
    不需要（也不能）在局部角空间采样，否则会采到真机做不到的位形。
    """
    return {j.id: float(rng.uniform(j.limit_min, j.limit_max))
            for j in robot.movable_joints()}


def grid_poses(robot, n: int = 4) -> list[dict[str, float]]:
    """限位端点上的张量网格 —— 专门打边界，随机采样很难命中这些角点。"""
    axes = []
    for j in robot.movable_joints():
        axes.append(np.linspace(j.limit_min, j.limit_max, n).tolist())
    out = []
    for a in axes[0]:
        for b in axes[1]:
            for c in axes[2]:
                for d in axes[3]:
                    out.append({j.id: v for j, v in
                                zip(robot.movable_joints(), (a, b, c, d))})
    return out


def mujoco_tcp_mm(sim, joints: Mapping[str, float]) -> np.ndarray:
    """把关节角写进 MuJoCo 求 TCP（mm）。

    用 `reset()` 而**不是** `settle()`：`reset()` 内部只做 `mj_forward`，
    是纯运动学求值，不推进物理、不引入重力/接触/求解器迭代的噪声。
    需要"纯几何定义层面"的判据时（FK/IK 一致性就是），必须走这条路 ——
    否则断言会变成"在某个动力学平衡态附近成立"，边界处随机飘。
    """
    sim.reset(dict(joints))
    return sim.end_effector_mm()


def case_joints(res: Mapping[str, Any]) -> dict[str, float]:
    """从桥返回的成功结果里取出关节角（已经是绝对角语义，可直接喂 `reset()`）。"""
    out = {k: float(v) for k, v in res["joints"].items()}
    return out


def reach_bounds_mm(info: Mapping[str, Any]) -> tuple[float, float]:
    """从桥导出的模型元信息里取矢状面 2R 的可达球壳 [min, max]（mm）。"""
    r = info["geometry"]["reach"]
    return float(r[0]), float(r[1])


def sagittal_distance_mm(info: Mapping[str, Any], target: Sequence[float]) -> float:
    """枢轴到目标的距离（mm）—— 判断"几何可达性"用的就是它。

    注意是**矢状面内**的距离：`hypot(hypot(x,y) − pivotR, z − pivotZ)`，
    而不是三维距离（`pivotR` 为 0 时两者才相等）。
    """
    g = info["geometry"]
    r = float(np.hypot(target[0], target[1]))
    return float(np.hypot(r - float(g["pivotR"]), target[2] - float(g["pivotZ"])))
