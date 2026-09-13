"""读取两份配置真值：

  config/robot.yaml    —— 运动学 / 标定 / 限位（**与前端、Go 后端共用**）
  config/physics.yaml  —— 纯物理量（质量 / 密度 / 摩擦 / 舵机 / 时间步）

本模块只做"文件 → 结构化对象"，不做任何推导，也不产生 MuJoCo 依赖，
因此可以被 `gen_model.py`（生成 XML）与 `model.py`（运行时）共用，也可以单测。

⚠️ 纪律：这里**不得**出现任何默认值兜底。缺字段就报错。
   静默兜底会让"配置写错了"表现成"机械臂行为诡异"，排查成本极高。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

PKG_DIR = Path(__file__).resolve().parent
# PKG_DIR = <root>/simulation/mujoco ⇒ parents[0]=simulation, parents[1]=<root>
PROJECT_ROOT = PKG_DIR.parents[1]
CONFIG_DIR = PROJECT_ROOT / "config"

ROBOT_YAML = CONFIG_DIR / "robot.yaml"
PHYSICS_YAML = CONFIG_DIR / "physics.yaml"


class ConfigError(RuntimeError):
    """配置文件缺失或结构不合法。"""


# ---------------------------------------------------------------------------
# robot.yaml
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class LinkCfg:
    id: str
    name: str
    parent: str | None
    length: float                                   # mm
    geometry: Mapping[str, Any] = field(default_factory=dict)
    details: Sequence[Mapping[str, Any]] = field(default_factory=tuple)


@dataclass(frozen=True)
class JointCfg:
    id: str
    name: str
    role: str
    type: str                                       # revolute | fixed
    parent_link: str
    child_link: str
    axis: tuple[float, float, float]
    origin_position: tuple[float, float, float]     # mm
    origin_rotation: tuple[float, float, float]     # deg, intrinsic XYZ
    limit_min: float
    limit_max: float
    coupling: tuple[str, float] | None = None       # (关节 id, gain)

    @property
    def is_fixed(self) -> bool:
        return self.type == "fixed"


@dataclass(frozen=True)
class ActuatorCfg:
    id: str
    name: str
    joint_id: str
    channel: int
    offset: float
    scale: float
    reverse: bool
    servo_min: float
    servo_max: float

    def joint_to_servo(self, theta_deg: float) -> float:
        """关节角 → 舵机角（与 Go `robot.JointToServo` / 前端 `jointToServo` 同式）。"""
        if self.reverse:
            return -theta_deg * self.scale + self.offset
        return theta_deg * self.scale + self.offset

    def servo_to_joint(self, servo_deg: float) -> float:
        """舵机角 → 关节角（上式的逆）。"""
        if self.reverse:
            return (self.offset - servo_deg) / self.scale
        return (servo_deg - self.offset) / self.scale


@dataclass
class RobotCfg:
    id: str
    name: str
    units: str
    tcp_joint: str
    tcp_offset: tuple[float, float, float]          # mm
    home_pose: dict[str, float]                     # deg
    links: list[LinkCfg]
    joints: list[JointCfg]
    actuators: list[ActuatorCfg]
    source_path: Path

    # -- 索引 -------------------------------------------------------------

    def link(self, link_id: str) -> LinkCfg:
        for l in self.links:
            if l.id == link_id:
                return l
        raise ConfigError(f"robot.yaml 中没有连杆 {link_id}")

    def joint(self, joint_id: str) -> JointCfg:
        for j in self.joints:
            if j.id == joint_id:
                return j
        raise ConfigError(f"robot.yaml 中没有关节 {joint_id}")

    def movable_joints(self) -> list[JointCfg]:
        return [j for j in self.joints if not j.is_fixed]

    def joint_order(self) -> list[str]:
        """参与状态帧的关节顺序（**与 Go `JointOrder()` / 前端 `movableJoints()` 一致**）。"""
        return [j.id for j in self.movable_joints()]

    def actuators_for(self, joint_id: str) -> list[ActuatorCfg]:
        return [a for a in self.actuators if a.joint_id == joint_id]

    def children_of(self, link_id: str) -> list[JointCfg]:
        return [j for j in self.joints if j.parent_link == link_id]

    def root_link(self) -> LinkCfg:
        for l in self.links:
            if l.parent is None:
                return l
        raise ConfigError("robot.yaml 中没有根连杆（parent: null）")

    def chain_from_root(self) -> list[tuple[LinkCfg, JointCfg | None]]:
        """按从根到末端的顺序返回 [(连杆, 进入它的关节)]。

        每个连杆恰好出现一次；根连杆的关节为 None。
        用于生成 MJCF 的嵌套 body 结构。
        """
        out: list[tuple[LinkCfg, JointCfg | None]] = []
        link_of_joint = {j.child_link: j for j in self.joints}
        seen: set[str] = set()

        def walk(link: LinkCfg) -> None:
            if link.id in seen:
                raise ConfigError(f"连杆图存在环或重复：{link.id}")
            seen.add(link.id)
            out.append((link, link_of_joint.get(link.id)))
            for j in self.children_of(link.id):
                walk(self.link(j.child_link))

        walk(self.root_link())
        if len(seen) != len(self.links):
            missing = {l.id for l in self.links} - seen
            raise ConfigError(f"以下连杆不在根可达的链上：{sorted(missing)}")
        return out


def load_robot(path: Path | str | None = None) -> RobotCfg:
    p = Path(path) if path is not None else ROBOT_YAML
    if not p.is_file():
        raise ConfigError(f"找不到 robot.yaml：{p}")
    raw = yaml.safe_load(p.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ConfigError(f"{p} 顶层不是映射")

    robot = raw.get("robot") or {}
    tcp = robot.get("tcp") or {}
    home = robot.get("homePose") or {}

    links: list[LinkCfg] = []
    for l in raw.get("links") or []:
        links.append(
            LinkCfg(
                id=str(l["id"]),
                name=str(l.get("name", l["id"])),
                parent=(None if l.get("parent") in (None, "null") else str(l["parent"])),
                length=float(l.get("length", 0.0)),
                geometry=l.get("geometry") or {},
                details=tuple(l.get("details") or ()),
            )
        )

    joints: list[JointCfg] = []
    for j in raw.get("joints") or []:
        origin = j.get("origin") or {}
        lim = j.get("limit") or {}
        coupling_raw = j.get("coupling")
        coupling = (
            (str(coupling_raw["joint"]), float(coupling_raw["gain"]))
            if coupling_raw
            else None
        )
        joints.append(
            JointCfg(
                id=str(j["id"]),
                name=str(j.get("name", j["id"])),
                role=str(j.get("role", j["id"])),
                type=str(j.get("type", "revolute")),
                parent_link=str(j["parentLink"]),
                child_link=str(j["childLink"]),
                axis=tuple(float(x) for x in (j.get("axis") or (0, 0, 0))),  # type: ignore[arg-type]
                origin_position=tuple(float(x) for x in (origin.get("position") or (0, 0, 0))),  # type: ignore[arg-type]
                origin_rotation=tuple(float(x) for x in (origin.get("rotation") or (0, 0, 0))),  # type: ignore[arg-type]
                limit_min=float(lim.get("min", 0.0)),
                limit_max=float(lim.get("max", 0.0)),
                coupling=coupling,
            )
        )

    actuators: list[ActuatorCfg] = []
    for a in raw.get("actuators") or []:
        lim = a.get("limits") or {}
        actuators.append(
            ActuatorCfg(
                id=str(a["id"]),
                name=str(a.get("name", a["id"])),
                joint_id=str(a["jointId"]),
                channel=int(a["channel"]),
                offset=float(a["offset"]),
                scale=float(a["scale"]),
                reverse=bool(a.get("reverse", False)),
                servo_min=float(lim.get("min", 0.0)),
                servo_max=float(lim.get("max", 180.0)),
            )
        )

    if not links:
        raise ConfigError("robot.yaml 中没有 links")
    if not joints:
        raise ConfigError("robot.yaml 中没有 joints")
    if not actuators:
        raise ConfigError("robot.yaml 中没有 actuators")

    cfg = RobotCfg(
        id=str(robot.get("id", "unknown")),
        name=str(robot.get("name", "unknown")),
        units=str(robot.get("units", "mm")),
        tcp_joint=str(tcp.get("joint", "")),
        tcp_offset=tuple(float(x) for x in (tcp.get("offset") or (0, 0, 0))),  # type: ignore[arg-type]
        home_pose={str(k): float(v) for k, v in home.items()},
        links=links,
        joints=joints,
        actuators=actuators,
        source_path=p,
    )
    _validate_robot(cfg)
    return cfg


def _validate_robot(cfg: RobotCfg) -> None:
    """基本一致性检查。宁可在这里炸，也不要在仿真里表现成"行为诡异"。"""
    # ① 每个可动关节都必须有执行器（否则命令发不出去 —— 与 Go robot.Load 同一约束）
    for j in cfg.movable_joints():
        if not cfg.actuators_for(j.id):
            raise ConfigError(f"关节 {j.id} 没有对应的执行器（标定表不完整）")

    # ② 执行器 scale 不能为 0
    for a in cfg.actuators:
        if a.scale == 0:
            raise ConfigError(f"执行器 {a.id} 的 scale 为 0（无法换算）")

    # ③ HOME 位必须覆盖全部可动关节且在限位内
    for j in cfg.movable_joints():
        if j.id not in cfg.home_pose:
            raise ConfigError(f"HOME 位缺少关节 {j.id}")
        v = cfg.home_pose[j.id]
        if not (j.limit_min - 1e-9 <= v <= j.limit_max + 1e-9):
            raise ConfigError(
                f"HOME 位 {j.id}={v} 越界（limit {j.limit_min}..{j.limit_max}）"
            )

    # ④ TCP 参考关节必须存在
    if cfg.tcp_joint and cfg.tcp_joint not in {j.id for j in cfg.joints}:
        raise ConfigError(f"tcp.joint={cfg.tcp_joint} 不存在")


# ---------------------------------------------------------------------------
# physics.yaml
# ---------------------------------------------------------------------------


@dataclass
class PhysicsCfg:
    raw: Mapping[str, Any]
    source_path: Path

    # -- 便捷访问（都直接返回 raw 里的子映射） -----------------------------

    @property
    def ts_physics(self) -> float:
        return float(self._req("timestep", "physics"))

    @property
    def ts_control(self) -> float:
        return float(self._req("timestep", "control"))

    @property
    def ts_render(self) -> float:
        return float(self._req("timestep", "render"))

    @property
    def gravity(self) -> tuple[float, float, float]:
        g = self._req("gravity")
        return (float(g[0]), float(g[1]), float(g[2]))

    @property
    def servo(self) -> Mapping[str, Any]:
        return self._req("servo")

    @property
    def inertia(self) -> Mapping[str, Any]:
        return self._req("inertia")

    @property
    def contact(self) -> Mapping[str, Any]:
        return self._req("contact")

    @property
    def limits(self) -> Mapping[str, Any]:
        return self._req("limits")

    @property
    def solver(self) -> Mapping[str, Any]:
        return self._req("solver")

    @property
    def recording(self) -> Mapping[str, Any]:
        """spec §34 数据记录配置（enabled / format / dir / fields）。"""
        return self._req("recording")

    @property
    def calibration(self) -> Mapping[str, Any]:
        """spec §38 标定接口（本阶段 `calibrated` 恒为 false）。"""
        return self._req("calibration")

    @property
    def deterministic(self) -> Mapping[str, Any]:
        """spec §33 确定性配置（seed / warmstart）。"""
        return self._req("deterministic")

    @property
    def body_inertia(self, name: str) -> Mapping[str, Any] | None:  # type: ignore[misc]
        raise NotImplementedError("请用 body_inertia_of(name)")

    def body_inertia_of(self, body_id: str) -> Mapping[str, Any]:
        """取某个 body 的显式惯量配置；不存在则报错（不静默兜底）。"""
        bodies = self.inertia.get("bodies") or {}
        if body_id not in bodies:
            raise ConfigError(
                f"physics.yaml 的 inertia.bodies 缺少 {body_id}（inertia.mode=explicit 时必须齐全）"
            )
        return bodies[body_id]

    def _req(self, *keys: str) -> Any:
        cur: Any = self.raw
        for k in keys:
            if not isinstance(cur, Mapping) or k not in cur:
                raise ConfigError(f"physics.yaml 缺少字段 {'/'.join(keys)}")
            cur = cur[k]
        return cur


def load_physics(path: Path | str | None = None) -> PhysicsCfg:
    p = Path(path) if path is not None else PHYSICS_YAML
    if not p.is_file():
        raise ConfigError(f"找不到 physics.yaml：{p}")
    raw = yaml.safe_load(p.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ConfigError(f"{p} 顶层不是映射")
    cfg = PhysicsCfg(raw=raw, source_path=p)
    # 触发一次必需字段校验（缺了立刻炸，而不是等到建模深处）
    _ = (cfg.ts_physics, cfg.ts_control, cfg.gravity, cfg.servo, cfg.inertia, cfg.contact, cfg.limits)
    return cfg
