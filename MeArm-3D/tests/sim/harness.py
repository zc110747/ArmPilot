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
    """**腕枢轴**到肩枢轴的距离（mm）—— 判断"几何可达性"用的就是它。

    ⚠️ 本机的爪被被动腕锁成水平，TCP 比腕枢轴多出一个**常量**矢状面偏移
    （`geometry.toolOffset` = `[40, 0]`），而 2R 的可达球壳约束的是**腕枢轴**。
    所以必须先减掉这个偏移：否则每个目标点都会凭空多出 40mm 的径向分量，
    越界判据与 `ik.ts` 的实现就会各说各话（而且两边看起来都很合理）。

    注意是**矢状面内**的距离：`hypot(hypot(x,y) − pivotR − off_r, z − pivotZ − off_z)`，
    而不是三维距离（`pivotR` 为 0 时两者才相等）。
    """
    g = info["geometry"]
    off_r, off_z = (float(v) for v in g["toolOffset"])
    r = float(np.hypot(target[0], target[1]))
    return float(
        np.hypot(r - float(g["pivotR"]) - off_r, float(target[2]) - float(g["pivotZ"]) - off_z)
    )


def mujoco_joint_origin_mm(sim, robot, jid: str) -> np.ndarray:
    """某个**关节坐标系原点**的世界坐标（mm）—— 从 **MuJoCo 侧**取。

    调用前必须已经 `sim.reset(joints)`：本函数只读 `data`，不推进任何东西。

    ⚠️ 固定关节（`robot.yaml` 里 `type: fixed`）在 MJCF 里**没有 `<joint>` 元素**
    （见 `gen_model.py`：`if joint is not None and not joint.is_fixed` 才写 joint），
    所以按关节名查会拿到 −1。但两条路径取到的是**同一个量**：生成器把 body 放在
    `parent.length + origin.position` 处，可动关节的 anchor 就是这个 body 原点，
    固定关节的坐标系原点也是这个 body 原点。所以可动关节读 `xanchor`，
    固定关节读**子连杆 body 的 `xpos`**（与 `test_fk.py::joint_origin_mm` 同一条规则）。
    """
    import mujoco

    joint = robot.joint(jid)
    if joint.is_fixed:
        i = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, joint.child_link)
        if i < 0:
            raise KeyError(f"MJCF 里找不到 body {joint.child_link!r}（关节 {jid!r}）")
        return np.asarray(sim.data.xpos[i], dtype=float) * 1000.0
    i = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, jid)
    if i < 0:
        raise KeyError(f"MJCF 里找不到 joint {jid!r}")
    return np.asarray(sim.data.xanchor[i], dtype=float) * 1000.0
