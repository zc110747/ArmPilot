"""关节限位校验 —— 与 Go `robot.Model.Validate` / 前端 `isJointStateWithinLimits` **同语义**。

为什么需要它（而不是依赖 MuJoCo 的 hinge range）
------------------------------------------------
`config/robot.yaml` 里 `elbow` 存的是**绝对倾角**，且与 `shoulder` 通过平行四连杆耦合。
它在 MuJoCo 的"局部角"空间里的合法域是**斜的**，任何单一 `hinge range` 都表达不了 ——
已用反例与空集证明，见 docs/ARCHITECTURE_ANALYSIS.md §6.3。

实测佐证（tests/sim/test_joint_limits.py 锁死）：
    命令 `elbow = 90°`（绝对角，低于真机下限 108.4415° ⇒ 真机**不可达**）
    MuJoCo 只看到局部角 88.8°，落在外接区间 [56.99, 149.95] 内 ⇒ **照常执行**。

⇒ 结论：MuJoCo 的 range 只是**数值保护**，"四方限位一致"必须由这一层保证。
   Go 的 controller 已经在 `Apply()` 里做了同样的事（返回 `ERR JOINT ...`），
   本模块是 Python 侧的对应物，保证物理后端与真机不对同一命令给出不同结论。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from robotcfg import RobotCfg


@dataclass(frozen=True)
class Violation:
    """一次限位越界。文案格式与固件 `ERR JOINT` 一致（前端 UI 与 e2e 断言都依赖它）。"""

    joint_id: str
    value: float
    limit_min: float
    limit_max: float

    def message(self) -> str:
        return (
            f"ERR JOINT {self.joint_id} {self.value:.2f} "
            f"(limit {self.limit_min:.2f}..{self.limit_max:.2f})"
        )


def validate_joints(
    robot: RobotCfg,
    joints: Mapping[str, float],
    *,
    eps: float = 1e-9,
) -> Violation | None:
    """按关节顺序逐个校验限位，返回**第一个**越界项；全部合法返回 None。

    ⚠️ 顺序敏感：必须按 `joint_order()` 而非 dict 迭代顺序，
    否则同一组越界命令在不同次运行里会报出不同关节（Go 侧注释点明了同一问题）。
    """
    for jid in robot.joint_order():
        if jid not in joints:
            continue
        v = float(joints[jid])
        j = robot.joint(jid)
        if v < j.limit_min - eps or v > j.limit_max + eps:
            return Violation(jid, v, j.limit_min, j.limit_max)
    return None


def violators(robot: RobotCfg, joints: Mapping[str, float]) -> list[Violation]:
    """返回**全部**越界项（诊断用）。"""
    out: list[Violation] = []
    for jid in robot.joint_order():
        if jid not in joints:
            continue
        v = float(joints[jid])
        j = robot.joint(jid)
        if v < j.limit_min - 1e-9 or v > j.limit_max + 1e-9:
            out.append(Violation(jid, v, j.limit_min, j.limit_max))
    return out
