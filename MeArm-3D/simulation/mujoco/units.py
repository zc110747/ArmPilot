"""单位与关节角语义的**单点**转换层。

本项目有两类极易静默出错的转换，二者都必须**只在这个文件里**发生：

  1. **单位**：配置与 UI 用 mm / degree，MuJoCo 内部用 m / radian。
     散落在各处做 `* 1e-3` 迟早会出现"某处忘了除 1000"，而它**不会报错**，
     只会让机械臂尺寸差三个数量级地静默错误。

  2. **角度语义**（本项目特有，比单位更危险）：
     `config/robot.yaml` 的关节角是**绝对语义**（`elbow` 存的是"离天顶的绝对倾角"），
     而 MuJoCo 的 hinge `qpos` 是**相对父 body 的局部角**。
     两者差一个 coupling 项。直接搬会导致"小臂不跟随肩转动"的错误机构
     —— 本项目已经踩过一次同类错误（按固件命名推定舵机角色 ⇒ 反向机构）。

详见 docs/ARCHITECTURE_ANALYSIS.md §5 / §6。
"""
from __future__ import annotations

import math
import sys
from typing import Mapping, Sequence

# ---------------------------------------------------------------------------
# 1. 单位常量与标量转换
# ---------------------------------------------------------------------------

MM2M = 1e-3
M2MM = 1e3
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
G2KG = 1e-3
NMM2NM = 1e-3


def mm2m(v: float) -> float:
    return v * MM2M


def m2mm(v: float) -> float:
    return v * M2MM


def deg2rad(v: float) -> float:
    return v * DEG2RAD


def rad2deg(v: float) -> float:
    return v * RAD2DEG


def vec_mm2m(v: Sequence[float]) -> list[float]:
    return [x * MM2M for x in v]


# ---------------------------------------------------------------------------
# 2. 关节角语义转换
# ---------------------------------------------------------------------------


class CouplingError(ValueError):
    """coupling 定义不合法（引用不存在的关节，或引用链后方未解出的关节）。"""


def _field(j: object, name: str, default: object = None) -> object:
    """同时支持 Mapping（原始 yaml dict）与 `robotcfg.JointCfg` 对象。

    这样 units.py 不必 import robotcfg（避免循环依赖），也能独立单测。
    """
    if isinstance(j, Mapping):
        return j.get(name, default)
    return getattr(j, name, default)


def _coupling_pair(c: object) -> tuple[str, float] | None:
    if c is None:
        return None
    if isinstance(c, Mapping):
        return str(c["joint"]), float(c["gain"])
    seq = tuple(c)  # type: ignore[arg-type]
    return str(seq[0]), float(seq[1])


class JointAngleMap:
    """关节角（绝对语义, degree）↔ MuJoCo `qpos`（局部语义, radian）的转换。

    只做**运动学语义**的转换，不做限位判断（限位由上层统一校验，见 ADR）。

    构造入参 `joints` 是 config/robot.yaml 的 `joints` 列表（原序），
    元素可以是原始 dict，也可以是 `robotcfg.JointCfg`。
    其中 `type == "fixed"` 的关节**不产生 qpos**，自动跳过。

    ⚠️ 因此 `joint_ids`（= qpos 坐标顺序）**不等于** `dof_ids`（= 状态帧自由度）：
    本机的被动腕 `tool` **有 qpos 但没有自由度**。两者混用会得到五元组的 JR，
    而固件 / 串口协议 / 前端全部按四元组解析 —— 见 `robotcfg.JointCfg.has_qpos`。
    """

    def __init__(self, joints: Sequence[object]) -> None:
        self._ids: list[str] = []
        self._by_id: dict[str, object] = {}
        for j in joints:
            jid = str(_field(j, "id"))
            self._by_id[jid] = j
            if str(_field(j, "type", "revolute")) != "fixed":
                self._ids.append(jid)

        # 预先校验 coupling 可顺序求解：被耦合的关节必须排在当前关节**之前**
        for jid in self._ids:
            c = self._coupling_of(jid)
            if c is None:
                continue
            other = c[0]
            if other not in self._by_id:
                raise CouplingError(f"关节 {jid} 耦合到不存在的关节 {other}")
            if str(_field(self._by_id[other], "type", "revolute")) == "fixed":
                raise CouplingError(f"关节 {jid} 耦合到固定关节 {other}（固定关节无状态）")
            if self._ids.index(other) >= self._ids.index(jid):
                raise CouplingError(
                    f"关节 {jid} 耦合到链后方的 {other} —— 无法顺序反解。"
                    f"请把被耦合关节排在前面。"
                )

    # -- 基本访问 ---------------------------------------------------------

    @property
    def joint_ids(self) -> list[str]:
        """产生 qpos 的关节顺序（= MJCF 里 joint 的声明顺序）。**含被动腕**。"""
        return list(self._ids)

    @property
    def dof_ids(self) -> list[str]:
        """有独立自由度的关节（= 状态帧 / JR 四元组）。被动腕**不在**其中。"""
        return [jid for jid in self._ids if self._is_dof(jid)]

    @property
    def nq(self) -> int:
        return len(self._ids)

    def _is_dof(self, joint_id: str) -> bool:
        return str(_field(self._by_id[joint_id], "type", "revolute")) == "revolute"

    def _coupling_of(self, joint_id: str) -> tuple[str, float] | None:
        return _coupling_pair(_field(self._by_id[joint_id], "coupling"))

    def _value_of(self, joint_id: str, angles: Mapping[str, float]) -> float:
        """取关节角（绝对语义, deg）。

        ⚠️ 被动关节**不会出现在 `angles` 里** —— 它不是状态变量，值由定义决定
        （恒取锁定角 `limit_min`，与前端 `jointAngleOf()` 的缺省回退同一条规则）。
        其余关节缺值仍是调用方的错，直接 KeyError（不静默兜底成 0）。
        """
        if joint_id in angles:
            return float(angles[joint_id])
        if str(_field(self._by_id[joint_id], "type", "revolute")) == "passive":
            return float(_field(self._by_id[joint_id], "limit_min", 0.0))
        raise KeyError(f"关节 {joint_id} 的角度缺失（angles={sorted(angles)}）")

    # -- 正向：关节角 → 局部角 --------------------------------------------

    def effective_deg(self, joint_id: str, angles: Mapping[str, float]) -> float:
        """关节角（绝对）→ 该关节在串联网里的**局部旋转角**（degree）。

        `实际旋转 = 关节角 + gain × 被耦合关节的关节角`
        （与前端 `effectiveJointAngle()` 逐字同式，见 frontend/src/robot/kinematics/fk.ts）
        """
        value = self._value_of(joint_id, angles)
        c = self._coupling_of(joint_id)
        if c is None:
            return value
        other, gain = c
        return value + gain * self._value_of(other, angles)

    def to_qpos(self, angles: Mapping[str, float]) -> list[float]:
        """关节角 dict（deg）→ qpos 数组（rad），顺序同 `joint_ids`。

        入参可以只带**自由度**关节（4 个）—— 被动腕会按锁定角自动补全，
        于是 `len(结果) == nq == MJCF 的 qpos 维度`。
        """
        return [deg2rad(self.effective_deg(jid, angles)) for jid in self._ids]

    # -- 反向：局部角 → 关节角 --------------------------------------------

    def from_qpos(self, qpos: Sequence[float]) -> dict[str, float]:
        """qpos 数组（rad）→ 关节角 dict（deg，**绝对语义**）。

        按链序顺序反解：`关节角 = 局部角 − gain × 被耦合关节的关节角`。
        因为构造时已保证被耦合关节在前，这里一次线性遍历即可。

        ⚠️ 返回值只含**自由度**关节（4 个）：它是"状态帧"，不是"qpos 向量"。
        前端 `JointState` / Go `JointOrder()` / 串口 JR 都是这 4 个位次。
        （内部仍会把被动腕一路解出来，因为它可能是别的关节的耦合源。）
        """
        if len(qpos) != self.nq:
            raise ValueError(f"qpos 长度 {len(qpos)} != 关节数 {self.nq}")
        out: dict[str, float] = {}
        for jid, q in zip(self._ids, qpos):
            local_deg = rad2deg(float(q))
            c = self._coupling_of(jid)
            if c is None:
                out[jid] = local_deg
            else:
                other, gain = c
                out[jid] = local_deg - gain * out[other]
        return {jid: out[jid] for jid in self.dof_ids}

    def qpos_index(self, joint_id: str) -> int:
        """该关节在 qpos 数组里的位次。"""
        return self._ids.index(joint_id)


# ---------------------------------------------------------------------------
# 3. 便于脚本使用的日志（Windows 控制台默认 GBK，中文会乱码）
# ---------------------------------------------------------------------------


def ensure_utf8_stdout() -> None:
    """把 stdout/stderr 切到 UTF-8。

    Windows 控制台默认代码页是 GBK，直接 print 中文会抛 UnicodeEncodeError
    或输出乱码（本项目在 tools/*.py 里已多次踩到）。
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")  # type: ignore[union-attr]
        except (AttributeError, ValueError):
            pass
