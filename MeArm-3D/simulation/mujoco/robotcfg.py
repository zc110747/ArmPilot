"""读取模型配置真值（**与前端、Go 后端共用同一批文件**）：

    config/robots.yaml           —— 模型**选择器**（只有 id / name / config + 文件指针）
    config/robot.yaml            —— MeArm-V1 的运动学 / 标定 / 限位
    config/physics.yaml          —— MeArm-V1 的纯物理量（质量 / 密度 / 摩擦 / 舵机 / 时间步）
    config/robots/<id>/robot.yaml    —— 其他机器人的运动学（如 SO-ARM101，生成产物）
    config/robots/<id>/physics.yaml  —— 其他机器人的物理量（真值来源可能是官方 MJCF）

本模块只做"文件 → 结构化对象"，不做任何推导，也不产生 MuJoCo 依赖，
因此可以被 `gen_model.py`（生成 XML）与 `model.py`（运行时）共用，也可以单测。

⚠️ 纪律：这里**不得**出现任何默认值兜底。缺字段就报错。
   静默兜底会让"配置写错了"表现成"机械臂行为诡异"，排查成本极高。

## 三种 physics.yaml 形态（同一套读取器）

| 形态 | 谁在用 | 特征 | 真值来源 |
|---|---|---|---|
| `legacy`（顶层） | MeArm-V1 | `timestep` / `servo` / `gravity` / `contact` / `inertia` … 全在顶层 | 我们自己估算 ⇒ 该文件即真值 |
| `driver`（`driver:` 段） | SO-ARM101 | 顶层是 `source` / `observed` / `not_declared_by_official`；`driver:` 段只放 ArmPilot 自己的驱动参数 | **官方 MJCF** ⇒ 该文件不复制任何物理量 |

判据是"顶层有没有 `driver:` 段"，不新增配置项 —— 判据写进配置就成了第二处要维护的东西。
`load_sim_driver()` 是**两者共同的规范化视图**：仿真层只认它，不认具体形态。
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

SELECTOR_YAML = CONFIG_DIR / "robots.yaml"
ROBOTS_DIR = CONFIG_DIR / "robots"

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
    origin_rotation: tuple[float, float, float]     # deg
    limit_min: float
    limit_max: float
    coupling: tuple[str, float] | None = None       # (关节 id, gain)
    #: 旋转约定。`"xyz"`（缺省）= intrinsic XYZ（`R = Rx·Ry·Rz`，three.js `Euler('XYZ')` 同序）；
    #: `"rpy"` = URDF `<origin rpy>` 的 fixed-axis XYZ（`R = Rz·Ry·Rx`）。
    #:
    #: ⚠️ 两者**不是同一种参数化**（extrinsic XYZ ≡ intrinsic ZYX），仅在 90° 的整数倍附近
    #:    偶然接近。让配置显式声明约定，官方 URDF 的数值就能**原样落盘**（可逐个复核），
    #:    而不是在 yaml 里塞一批「官方文件里查不到的数」。
    #: （带缺省 ⇒ 必须排在无缺省字段之后，别往前挪。）
    origin_rotation_convention: str = "xyz"

    @property
    def is_fixed(self) -> bool:
        return self.type == "fixed"

    @property
    def is_passive(self) -> bool:
        """被动关节：**会转，但没有独立输入** —— 角度完全由 `coupling` 派生，
        值恒取 `limit_min`（故校验层强制 `min == max`）。

        本机用它表达**腕**：爪被平行四连杆锁成水平（绝对倾角恒 90°）。
        """
        return self.type == "passive"

    @property
    def is_dof(self) -> bool:
        """有独立自由度的关节 —— 即参与 JointState / JR 四元组 / 执行器表的那些。

        ⚠️ 判据是 `revolute`，**不是**"非 fixed"。这与前端 `isMovableJoint()`、
        Go `JointOrder()` 是**同一条规则**，三处必须一致，否则 JR 位次会静默错位。
        """
        return self.type == "revolute"

    @property
    def has_qpos(self) -> bool:
        """在 MJCF 里是否产生 `qpos` 坐标。

        ⚠️ 它与 `is_dof` **不是一回事** —— 这是本模型最容易搞混的一处：

        | 关节 | 有 qpos？ | 有自由度？ |
        |------|-----------|-----------|
        | base/shoulder/elbow/gripper | ✔ | ✔ |
        | tool（被动腕）| ✔（要建模成 hinge，爪才会被连杆带着转）| ✘（被 equality 约束锁死）|

        把两者混为一谈，就会得到一个**五元组的 JR** —— 而固件、串口协议、
        前端全部按四元组解析，每一条指令的位次都会错。
        """
        return not self.is_fixed


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
    #: 执行器空间。`"deg"`（缺省）= 0..180 舵机行程；
    #: `"joint"` = 关节空间（ctrlrange ≡ 关节范围、gear=1）⇒ `servo_min/max` 就是关节区间，
    #: 且 `offset/scale/reverse` 必然是 0/1/false（没有标定可标）。
    unit: str = "deg"

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

    #: 模型标识（`robot.yaml` 的 `robot.model`，如 `"MeArm-V1"`）。
    #: **只读元数据，不参与任何运动学/物理推导** —— 它唯一的用途是让
    #: 「抽象前后行为一致」这句验收话可被机器检查（见 tests/baseline/mearm-v1/）。
    model: str | None = None
    #: 模型语义版本（`robot.yaml` 的 `robot.version`）。同上，只读元数据。
    model_version: str | None = None

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
        """**有独立自由度**的关节（= 状态帧 / JR 四元组 / 执行器表）。

        ⚠️ 只收 `revolute` —— 被动腕 `tool` **不在**其中（见 `JointCfg.is_dof`）。
        """
        return [j for j in self.joints if j.is_dof]

    def qpos_joints(self) -> list[JointCfg]:
        """在 MJCF 里产生 `qpos` 的关节（**含被动腕**），顺序 = MJCF 里 joint 的声明顺序。

        必须与 `units.JointAngleMap.joint_ids` 一致，否则 `model.py` 启动时的
        顺序自检会直接抛 `SimError`。
        """
        return [j for j in self.joints if j.has_qpos]

    def passive_joints(self) -> list[JointCfg]:
        """被动关节：有 qpos、无自由度，角度由 `coupling` 派生。"""
        return [j for j in self.joints if j.is_passive]

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
                origin_rotation_convention=str(origin.get("rotationConvention", "xyz")),
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
                unit=str(a.get("unit", "deg")),
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
        # 只读元数据：缺省为 None（缺省不影响任何行为）
        model=(str(robot["model"]) if robot.get("model") is not None else None),
        model_version=(str(robot["version"]) if robot.get("version") is not None else None),
    )
    _validate_robot(cfg)
    return cfg


def _validate_robot(cfg: RobotCfg) -> None:
    """基本一致性检查。宁可在这里炸，也不要在仿真里表现成"行为诡异"。"""
    # ① 每个**自由度**关节都必须有执行器（否则命令发不出去 —— 与 Go robot.Load 同一约束）
    for j in cfg.movable_joints():
        if not cfg.actuators_for(j.id):
            raise ConfigError(f"关节 {j.id} 没有对应的执行器（标定表不完整）")

    # ①b 被动关节：没有独立输入，角度由 `coupling` 派生 —— 三条约束缺一不可。
    #     这三条与前端 `validateRobotModel()` 的 JOINT_PASSIVE_LIMIT /
    #     JOINT_PASSIVE_NO_COUPLING / ACTUATOR_FIXED_JOINT 是同一套判据。
    for j in cfg.passive_joints():
        if abs(j.limit_max - j.limit_min) > 1e-9:
            raise ConfigError(
                f"被动关节 {j.id} 的限位必须 min == max（值恒取锁定角 = 锁定角度），"
                f"当前 {j.limit_min}..{j.limit_max} —— 否则「到底取哪个值」就成了隐式约定"
            )
        if not j.coupling:
            raise ConfigError(f"被动关节 {j.id} 必须声明 coupling（它没有独立输入）")
        if cfg.actuators_for(j.id):
            raise ConfigError(
                f"被动关节 {j.id} 不得挂执行器：它没有输入，"
                f"挂上等于给一个被连杆锁死的关节发命令"
            )

    # ② 执行器 scale 不能为 0
    for a in cfg.actuators:
        if a.scale == 0:
            raise ConfigError(f"执行器 {a.id} 的 scale 为 0（无法换算）")

    # ③ HOME 位必须覆盖全部**自由度**关节且在限位内（被动关节不进 homePose）
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
# robots.yaml —— 模型**选择器**
# ---------------------------------------------------------------------------
#
# ★ 铁律：选择器里只允许出现「指针」（文件路径 / MJCF site 名），
#   **不允许出现任何数值**（尺寸、限位、标定、物理量）。
#   把参数抄进选择器就等于制造第二份真值 —— 与把限位抄进 backend/config.yaml
#   是同一类错误，而它的表现是"改了一处、另一处还是旧值，且没有任何报错"。


@dataclass(frozen=True)
class RobotEntry:
    """选择器里的一条记录（= 一台机器人涉及的全部文件）。"""

    id: str
    name: str
    #: `robot.yaml` 的**仓库相对路径**
    config_path: str
    #: `physics.yaml` 的仓库相对路径。缺省按 `config/robots/<id>/physics.yaml` 猜。
    physics_path: str
    #: MJCF 的仓库相对路径。`None` = 由 `gen_model.py` 从 robot.yaml 生成（MeArm 路线）。
    mjcf_path: str | None
    #: MJCF 里代表 TCP 的 site 名。缺省 `"tcp"`。
    tcp_site: str
    #: 该机器人的 physics.yaml 是哪种形态（判据 = 顶层有没有 `driver:` 段）
    physics_kind: str

    @property
    def config_file(self) -> Path:
        return PROJECT_ROOT / self.config_path

    @property
    def physics_file(self) -> Path:
        return PROJECT_ROOT / self.physics_path

    @property
    def mjcf_file(self) -> Path | None:
        return None if self.mjcf_path is None else PROJECT_ROOT / self.mjcf_path


@dataclass(frozen=True)
class RobotSelector:
    default: str
    robots: Mapping[str, RobotEntry]
    source_path: Path


def _physics_kind_of(raw: Mapping[str, Any]) -> str:
    """`driver`（顶层带 `driver:` 段）还是 `legacy`（MeArm 风格，参数全在顶层）。"""
    return "driver" if isinstance(raw.get("driver"), Mapping) else "legacy"


def load_robot_selector(path: Path | str | None = None) -> RobotSelector:
    """读 `config/robots.yaml`。

    未知 id、缺 `default`、`default` 不在表里、`config` 指向不存在的文件
    —— **一律报错，绝不回退**。「id 写错一个字」若被静默兜底成"加载了另一台机器人"，
    是本项目最贵的一类错误：模型不对，所有 FK/限位判据全部失真却都能跑。
    """
    p = Path(path) if path is not None else SELECTOR_YAML
    if not p.is_file():
        raise ConfigError(
            f"找不到模型选择器 robots.yaml：{p}\n"
            f"  （它是「当前该加载哪台机器人」的唯一声明处；三端都读它）"
        )
    raw = yaml.safe_load(p.read_text(encoding="utf-8"))
    if not isinstance(raw, Mapping):
        raise ConfigError(f"{p} 顶层不是映射")

    robots_raw = raw.get("robots")
    if not isinstance(robots_raw, Mapping) or not robots_raw:
        raise ConfigError(f"{p}: robots 必须是非空映射")

    robots: dict[str, RobotEntry] = {}
    for rid, item in robots_raw.items():
        if not isinstance(item, Mapping):
            raise ConfigError(f"{p}: robots.{rid} 必须是映射")
        name = item.get("name")
        config_path = item.get("config")
        if not isinstance(name, str) or not name.strip():
            raise ConfigError(f"{p}: robots.{rid}.name 必须是非空字符串")
        if not isinstance(config_path, str) or not config_path.strip():
            raise ConfigError(f"{p}: robots.{rid}.config 必须是非空字符串")
        if not (PROJECT_ROOT / config_path).is_file():
            raise ConfigError(f"{p}: robots.{rid}.config = {config_path!r} 不存在")

        physics_path = item.get("physics")
        if physics_path is None:
            # 约定：`config/robots/<id>/robot.yaml` 的兄弟文件
            physics_path = str(Path(config_path).parent / "physics.yaml")
        if not isinstance(physics_path, str) or not (PROJECT_ROOT / physics_path).is_file():
            raise ConfigError(
                f"{p}: robots.{rid}.physics = {physics_path!r} 不存在"
                f"（缺省按 config 的兄弟目录猜，可用 `physics:` 显式指定）"
            )

        sim = item.get("simulation") or {}
        if not isinstance(sim, Mapping):
            raise ConfigError(f"{p}: robots.{rid}.simulation 必须是映射")
        mjcf = sim.get("mjcf")
        if mjcf is not None:
            if not isinstance(mjcf, str) or not (PROJECT_ROOT / mjcf).is_file():
                raise ConfigError(f"{p}: robots.{rid}.simulation.mjcf = {mjcf!r} 不存在")
        tcp_site = sim.get("tcpSite", "tcp")
        if not isinstance(tcp_site, str) or not tcp_site.strip():
            raise ConfigError(f"{p}: robots.{rid}.simulation.tcpSite 必须是非空字符串")

        physics_raw = yaml.safe_load((PROJECT_ROOT / physics_path).read_text(encoding="utf-8"))
        if not isinstance(physics_raw, Mapping):
            raise ConfigError(f"{PROJECT_ROOT / physics_path} 顶层不是映射")

        robots[str(rid)] = RobotEntry(
            id=str(rid),
            name=name,
            config_path=config_path,
            physics_path=physics_path,
            mjcf_path=(None if mjcf is None else str(mjcf)),
            tcp_site=tcp_site,
            physics_kind=_physics_kind_of(physics_raw),
        )

    default = raw.get("default")
    if not isinstance(default, str) or default not in robots:
        raise ConfigError(
            f"{p}: default = {default!r} 必须是 robots 里的一个 key，"
            f"可选: {' / '.join(robots)}"
        )
    return RobotSelector(default=default, robots=robots, source_path=p)


def resolve_robot_entry(robot_id: str | None = None,
                        selector: Path | str | None = None) -> RobotEntry:
    """id → 选择器记录。`robot_id=None` 取选择器的 `default`。"""
    sel = load_robot_selector(selector)
    rid = sel.default if robot_id is None else str(robot_id)
    entry = sel.robots.get(rid)
    if entry is None:
        raise ConfigError(
            f"未知机器人 id {rid!r}；可选: {' / '.join(sel.robots)}"
        )
    return entry


def load_robot_by_id(robot_id: str | None = None,
                     selector: Path | str | None = None) -> RobotCfg:
    """按 id 读运动学真值（`robot.yaml`）。"""
    return load_robot(resolve_robot_entry(robot_id, selector).config_file)


def resolve_robot_entry_by_config(config_path: Path | str,
                                  selector: Path | str | None = None) -> RobotEntry:
    """按**配置文件路径**反查选择器记录。

    ⚠️ 为什么不用 `RobotCfg.id` 反查：选择器的 key 是**注册表 id**（`mearm-v1`），
    而 `robot.yaml → robot.id` 是模型自己的名字（`mearm`）—— 两者**本来就不同名**，
    也不应该强制同名（前者是"在 ArmPilot 里叫哪一号"，后者是"这台机器是什么"）。
    拿 `robot.id` 去查选择器会得到一个"未知机器人 id"的假故障。

    路径比较前先 `resolve()`，因此既能接受绝对路径也能接受相对路径。
    """
    target = Path(config_path).resolve()
    sel = load_robot_selector(selector)
    for entry in sel.robots.values():
        if entry.config_file.resolve() == target:
            return entry
    keys = " / ".join(f"{e.id}({e.config_path})" for e in sel.robots.values())
    raise ConfigError(
        f"配置文件 {target} 不在模型选择器里（{sel.source_path}）。\n"
        f"  选择器声明了: {keys}\n"
        f"  未登记的自定义 robot.yaml 无法解析出物理/MJCF 资产 —— "
        f"请先把它加进 config/robots.yaml。"
    )


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


# ---------------------------------------------------------------------------
# 仿真驱动参数（**机器人无关的规范化视图**）
# ---------------------------------------------------------------------------
#
# 为什么要有这一层：仿真层需要的东西只有四样 —— 物理步长、控制周期、
# 舵机速率上限、重力。而它们在不同机器人上的**真值来源不同**：
#
#   MeArm   gravity / max_velocity 都是我们估算的 ⇒ 写在 physics.yaml 顶层（legacy）
#   SO-101  gravity / 时间步都在官方 MJCF 里  ⇒ physics.yaml 只声明 ArmPilot 的驱动参数
#
# 仿真层若直接读 physics.yaml 的某个具体键，SO-101 就必须伪造一份 MeArm 形状的文件
# —— 那正是本项目禁止的"第二份真值"。故在此收敛成唯一的规范化视图。


@dataclass(frozen=True)
class SimDriverCfg:
    """跑仿真所需的全部「我们怎么跑」参数（不含任何「机器人是什么」）。"""

    ts_physics: float
    ts_control: float
    ts_render: float
    max_velocity_deg_s: float
    #: `None` = 本机器人不声明重力 ⇒ 取**已加载的 MjModel** 里的值（官方 MJCF 就是真值）。
    #: 这比在配置里再抄一份 "0 0 -9.81" 诚实：官方改了，仿真跟着改。
    gravity: tuple[float, float, float] | None
    deterministic: Mapping[str, Any]
    recording: Mapping[str, Any]
    #: `legacy`（MeArm）/ `driver`（SO-101）—— 仅用于诊断与文档
    kind: str
    source_path: Path

    def max_velocity_rad_s(self) -> float:
        return float(self.max_velocity_deg_s) * DEG2RAD_LOCAL


#: 本模块不做单位转换（那是 `units.py` 的职责），但 `max_velocity` 需要 deg→rad。
#: 用字面常量而不是 import units：robotcfg 刻意不依赖 units（units 也不依赖它）。
DEG2RAD_LOCAL = 0.017453292519943295  # math.pi / 180


def _driver_section(raw: Mapping[str, Any]) -> Mapping[str, Any]:
    """取驱动参数所在的段：有 `driver:` 用它，否则整个文件就是驱动段（MeArm legacy）。"""
    sec = raw.get("driver")
    return sec if isinstance(sec, Mapping) else raw


def load_sim_driver(entry: RobotEntry | None = None, *,
                    robot_id: str | None = None,
                    physics_path: Path | str | None = None) -> SimDriverCfg:
    """读某一台机器人的**规范化驱动参数**。

    三选一：给 `entry`（选择器记录）/ 给 `robot_id`（回查选择器）/ 给 `physics_path`。
    都不给 ⇒ 选择器的 `default` 那台。
    """
    if entry is None:
        if physics_path is not None:
            p = Path(physics_path)
            raw = yaml.safe_load(p.read_text(encoding="utf-8"))
            if not isinstance(raw, Mapping):
                raise ConfigError(f"{p} 顶层不是映射")
            kind = _physics_kind_of(raw)
        else:
            entry = resolve_robot_entry(robot_id)
    if entry is not None:
        p = entry.physics_file
        raw = yaml.safe_load(p.read_text(encoding="utf-8"))
        if not isinstance(raw, Mapping):
            raise ConfigError(f"{p} 顶层不是映射")
        kind = entry.physics_kind

    sec = _driver_section(raw)

    ts = sec.get("timestep")
    if not isinstance(ts, Mapping):
        raise ConfigError(f"{p}: 缺 timestep 段（physics / control / render）")

    def _ts(key: str) -> float:
        if key not in ts:
            raise ConfigError(f"{p}: 缺 timestep.{key}")
        return float(ts[key])

    servo = sec.get("servo") or {}
    if not isinstance(servo, Mapping):
        raise ConfigError(f"{p}: servo 必须是映射")
    mv = servo.get("max_velocity") or {}
    if not isinstance(mv, Mapping):
        raise ConfigError(f"{p}: servo.max_velocity 必须是映射")
    # ⚠️ 顺序**必须**是 rad_per_s 优先：MeArm 的 config/physics.yaml 两个都写了
    #    （`deg_per_s: 573` / `rad_per_s: 10`），而 573°/s 换算回来是 10.000737 rad/s。
    #    谁优先会改变舵机限速的步长 ⇒ 会改变既有仿真轨迹。取 rad_per_s 是**改前行为**。
    if "rad_per_s" in mv:
        max_vel = float(mv["rad_per_s"]) / DEG2RAD_LOCAL
    elif "deg_per_s" in mv:
        max_vel = float(mv["deg_per_s"])
    else:
        # ⚠️ 刻意**不给缺省值**：没有速率上限，MuJoCo 的位置执行器会让舵机"瞬移"，
        #    整条"响应慢 / 滞后 / 跟不上"的真实特征会消失 —— 那是一种**静默**的失真。
        raise ConfigError(
            f"{p}: 缺 servo.max_velocity.{{deg_per_s|rad_per_s}} —— "
            f"控制层的舵机速率限制必须有真值，不能默认"
        )

    g_raw = sec.get("gravity")
    gravity = None if g_raw is None else tuple(float(x) for x in g_raw)  # type: ignore[assignment]

    det = sec.get("deterministic")
    if not isinstance(det, Mapping):
        raise ConfigError(f"{p}: 缺 deterministic 段")
    rec = sec.get("recording")
    if not isinstance(rec, Mapping):
        raise ConfigError(f"{p}: 缺 recording 段")

    return SimDriverCfg(
        ts_physics=_ts("physics"),
        ts_control=_ts("control"),
        ts_render=_ts("render"),
        max_velocity_deg_s=max_vel,
        gravity=gravity,  # type: ignore[arg-type]
        deterministic=det,
        recording=rec,
        kind=kind,
        source_path=p,
    )
