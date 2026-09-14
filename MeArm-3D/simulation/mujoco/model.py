"""MuJoCo 物理后端封装（**机器人无关**）。

对外暴露 spec §31 要求的五个能力：

    load        —— 构造即加载（`RobotSim(...)`）
    reset       —— `reset()` 复位到 HOME（或指定位形），**可重复**
    step        —— `step(n)` 推进物理
    command     —— `set_target_joints()` 下发关节角命令
    state       —— `state()` 读取统一状态（关节角/速度/力矩/末端位姿/接触）

## 同一份代码服务两台完全不同的机器人

| | MeArm-V1 | SO-ARM101 |
|---|---|---|
| MJCF 从哪来 | `gen_model.py` 从 `robot.yaml` 生成 | **官方 MJCF 原样**（官方仓库布局，零修改） |
| TCP 从哪读 | `mearm.xml` 的 `tcp` site | 官方 MJCF 的 `gripperframe` site |
| 物理量真值 | `config/physics.yaml`（我们估算） | 官方 MJCF（`observed` 段只是审计快照） |
| 关节语义 | 绝对角 + coupling（被动腕） | 原生关节角（无 coupling、无被动件） |

这三处差异**全部由配置声明**（`config/robots.yaml` 的指针 + 各自的 robot.yaml），
本文件里**没有一处** `if robot == …` 的特判。新增一台机器人 = 加配置 + 加引擎实现。

设计纪律
--------
1. **绝不用 `qpos` 直写来"驱动"机械臂**（spec §12 明令禁止）。
   运动只能经由 `ctrl` → actuator → 物理动力学产生。`reset()` 是唯一允许写 `qpos`
   的地方 —— 那是"设定初始条件"，不是"控制"。
2. **舵机速率限制在控制层实现**（spec §13 的 `max_velocity`）：
   MuJoCo 的 position actuator 本身没有速度上限，只有 `kp/kv/forcerange`。
   若不做这一步，阶跃命令会让舵机"瞬移"，整条"响应慢/滞后/跟不上"的真实特征消失。
   ⚠️ 官方 SO-101 的 MJCF **也没有**声明速度上限 ⇒ 这一层对两台机器人都是**必需**的。
3. 关节角语义换算（绝对 ↔ 局部）**只经 `units.JointAngleMap`**，见 ARCHITECTURE_ANALYSIS §6。
   MeArm 的 coupling 语义由它承担；SO-101 没有 coupling，它退化成恒等映射
   —— 也就是说**同一条代码路径**服务两种语义。
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import mujoco  # noqa: E402

from robotcfg import (  # noqa: E402
    PKG_DIR,
    PhysicsCfg,
    RobotCfg,
    RobotEntry,
    SimDriverCfg,
    load_physics,
    load_robot_by_id,
    load_sim_driver,
    resolve_robot_entry_by_config,
)
from units import JointAngleMap, deg2rad, rad2deg  # noqa: E402

#: 历史缺省（只有一条 MJCF 的年代）。现在真正的缺省来自选择器记录
#: （`config/robots.yaml` → `simulation.mjcf`），文件不存在而选择器又没声明时才用它兜底。
DEFAULT_XML = PKG_DIR / "mearm.xml"
#: 历史缺省 TCP site 名（MeArm 的 mearm.xml 里就是它）
DEFAULT_TCP_SITE = "tcp"


class SimError(RuntimeError):
    """仿真层错误（配置不一致 / 状态非法）。"""


@dataclass
class SimState:
    """spec §19 的统一状态快照。"""

    time: float
    joint_angles: dict[str, float]          # deg，**绝对语义**（与 UI / 协议一致）
    qpos_local: np.ndarray                  # rad，**局部语义**（MuJoCo 原生，长度 = nq）
    joint_velocities: np.ndarray            # rad/s，**按 `robot.joint_order()`**（状态帧四元组）
    joint_torques: np.ndarray               # N·m，同上（actuator 施加的广义力）
    target_angles: dict[str, float]         # deg，控制器当前的速率受限目标
    end_effector: np.ndarray                # m，TCP 世界坐标
    contact_count: int

    def as_dict(self, robot: RobotCfg) -> dict[str, Any]:
        return {
            "time": self.time,
            "joint_angles": dict(self.joint_angles),
            "joint_velocities": {
                jid: float(v) for jid, v in zip(robot.joint_order(), self.joint_velocities)
            },
            "joint_torques": {
                jid: float(t) for jid, t in zip(robot.joint_order(), self.joint_torques)
            },
            "target_joints": dict(self.target_angles),
            "end_effector": {
                "x": float(self.end_effector[0]),
                "y": float(self.end_effector[1]),
                "z": float(self.end_effector[2]),
            },
            "contact_count": self.contact_count,
        }


class RobotSim:
    """一台机器人的 MuJoCo 物理实例（MeArm-V1 / SO-ARM101 共用）。"""

    def __init__(
        self,
        xml_path: Path | str | None = None,
        robot: RobotCfg | None = None,
        physics: PhysicsCfg | None = None,
        *,
        robot_id: str | None = None,
        driver: SimDriverCfg | None = None,
        tcp_site: str | None = None,
        gravity: bool | None = None,
        xml_text: str | None = None,
    ) -> None:
        self.robot = robot if robot is not None else load_robot_by_id(robot_id)
        # 选择器记录：MJCF 路径 / TCP site 名 / physics 形态的唯一来源。
        # 先按显式 id 查，再按"配置文件路径"反查（不依赖 robot.id 与注册表 key 同名）。
        self.entry: RobotEntry = resolve_robot_entry_by_config(self.robot.source_path)
        self.driver: SimDriverCfg = driver if driver is not None else load_sim_driver(self.entry)

        # MeArm 的 physics.yaml 还带着 contact / inertia / limits / solver / calibration
        # 这些**它自己才有**的东西，那些由 MeArm 专属的测试与工具消费；
        # SO-101 的真值在官方 MJCF 里 ⇒ 不加载 `PhysicsCfg`（它没有也不该有那些段）。
        if physics is None and self.entry.physics_kind == "legacy":
            physics = load_physics(self.entry.physics_file)
        self.physics = physics

        # TCP site：显式参数 > 选择器声明 > 历史缺省
        self.tcp_site = tcp_site or self.entry.tcp_site or DEFAULT_TCP_SITE

        # ⚠️ 变体场景（抬高地面 / 改碰撞体尺寸）**必须在加载期改 XML**。
        #
        # 不能走"运行时改 `model.geom_pos` / `model.geom_size`"这条路 ——
        # 实测：这样写进 MjModel 后，`mj_forward` **不会**重算静态 geom 的
        # `data.geom_xpos` 与 broadphase AABB，碰撞检测完全无视改动
        # （把 table 从 z=15mm 抬到 z=100mm，臂的稳态位置与接触对一字不变）。
        # 所以这里留一个"直接喂 XML 文本"的入口，供测试与参数扫描使用。
        if xml_text is not None:
            self.xml_path: Path | None = None
            self.model = mujoco.MjModel.from_xml_string(xml_text)
        else:
            path = Path(xml_path) if xml_path is not None else self.entry.mjcf_file
            if path is None:
                path = DEFAULT_XML          # 老路径：选择器没声明 mjcf（MeArm 走生成）
            if not path.is_file():
                raise SimError(
                    f"找不到 MJCF：{path}\n"
                    f"请先运行：python simulation/mujoco/gen_model.py"
                )
            self.xml_path = path
            self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)

        # ---- 物理步长一致性自检 --------------------------------------------
        # 物理时间只由 `model.opt.timestep` 决定，而"实时对齐"的换算却可能用配置里的
        # 数字。两者不一致时仿真会以错误的速率前进（SO-101 官方是 0.002、MeArm 是 0.001），
        # 而**不会报任何错** —— 只是"看起来有点快/有点慢"。这里把它变成一条明确的自检。
        ts_model = float(self.model.opt.timestep)
        if abs(ts_model - self.driver.ts_physics) > 1e-12:
            raise SimError(
                f"MJCF 的 timestep={ts_model!r} 与驱动配置的 timestep.physics="
                f"{self.driver.ts_physics!r} 不一致（{self.driver.source_path}）。\n"
                f"  MJCF: {self.xml_path or '(xml_text)'}\n"
                f"  物理时间只由 MJCF 决定；不一致会让仿真以错误的速率前进且不报错。"
            )

        # ---- 关节地址映射（不依赖数组顺序，全部按名字解析）----------------
        self.angle_map = JointAngleMap(self.robot.joints)
        self._qpos_adr: dict[str, int] = {}
        self._dof_adr: dict[str, int] = {}
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name is None:
                continue
            if self.model.jnt_type[i] != mujoco.mjtJoint.mjJNT_HINGE:
                continue
            self._qpos_adr[name] = int(self.model.jnt_qposadr[i])
            self._dof_adr[name] = int(self.model.jnt_dofadr[i])

        mj_order = sorted(self._qpos_adr, key=lambda n: self._qpos_adr[n])
        if mj_order != self.angle_map.joint_ids:
            raise SimError(
                "MJCF 的关节顺序与 robot.yaml 不一致：\n"
                f"  MJCF      : {mj_order}\n"
                f"  robot.yaml: {self.angle_map.joint_ids}\n"
                f"请重跑 gen_model.py（两边都应由同一份 yaml 派生）"
            )
        self.joint_ids = mj_order

        # ---- 执行器 → 关节 ------------------------------------------------
        self._actuator_of_joint: dict[str, int] = {}
        for i in range(self.model.nu):
            jid = int(self.model.actuator_trnid[i, 0])
            jname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if jname is None:
                continue
            # 多执行器驱动同一关节时取第一个；上层若需区分可扩展
            self._actuator_of_joint.setdefault(jname, i)

        # ---- TCP site ------------------------------------------------------
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.tcp_site)
        if sid < 0:
            raise SimError(
                f"MJCF 里没有名为 {self.tcp_site!r} 的 site（无法读末端位姿）。\n"
                f"  该名字来自 {self.entry.config_path} 所属的选择器记录"
                f"（robots.{self.entry.id}.simulation.tcpSite）\n"
                f"  MJCF: {self.xml_path or '(xml_text)'}"
            )
        self._tcp_site = sid

        # ---- 重力基准 -------------------------------------------------------
        # 驱动配置声明了就用它（MeArm：我们估算的 0 0 -9.81）；
        # 没声明就取**已加载 MjModel 自己的值**（SO-101：官方 MJCF 就是真值）。
        # 后者比在配置里再抄一份 "0 0 -9.81" 诚实 —— 官方改了，仿真跟着改。
        self._base_gravity = np.array(
            self.driver.gravity if self.driver.gravity is not None else self.model.opt.gravity,
            dtype=float,
        )

        # ---- 舵机速率限制状态 ----------------------------------------------
        self._ctrl_qpos = np.zeros(self.model.nq)
        self._target_qpos = np.zeros(self.model.nq)
        self._ctrl_accum = 0.0          # 控制周期累加器（必须是实例状态，见 step()）

        if gravity is not None:
            self.set_gravity(gravity)

        self.reset()

    # ------------------------------------------------------------------
    # 配置
    # ------------------------------------------------------------------

    @property
    def max_velocity_rad_s(self) -> float:
        return self.driver.max_velocity_rad_s()

    def set_gravity(self, enabled: bool) -> None:
        """开关重力。

        Gravity Test（spec §18）需要"同一初始条件、仅重力不同"的对照，
        用来区分"位移是重力造成的"还是"控制器造成的"。
        """
        self.model.opt.gravity[:] = self._base_gravity if enabled else (0.0, 0.0, 0.0)

    @property
    def gravity_enabled(self) -> bool:
        return bool(np.any(self.model.opt.gravity))

    def set_actuation_enabled(self, enabled: bool) -> None:
        """启用/禁用**全部致动器**。

        Gravity Test（spec §18）需要"无驱动"对照：禁用后机械臂只剩重力与关节摩擦，
        因此它的运动**只可能**由重力解释 —— 这是把"动力学真的在起作用"与
        "位置环把一切抹平了"区分开的唯一办法。

        ⚠️ `mjtDisableBit` 是 enum，不能直接 `~`（会 TypeError）；必须先取 int。
        """
        flag = int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
        if enabled:
            self.model.opt.disableflags &= ~flag
        else:
            self.model.opt.disableflags |= flag

    # ------------------------------------------------------------------
    # reset（spec §32）
    # ------------------------------------------------------------------

    def reset(self, joints: Mapping[str, float] | None = None) -> None:
        """复位到 HOME（或指定关节角），速度清零，时间归零。

        保证**可重复**：相同入参 ⇒ 逐位相同的初始状态（spec §32）。
        """
        pose = dict(self.robot.home_pose if joints is None else joints)
        mujoco.mj_resetData(self.model, self.data)
        qpos = self.angle_map.to_qpos(pose)
        self.data.qpos[:] = qpos
        self.data.qvel[:] = 0.0
        # 控制器内部状态也归零，否则上一轮的残余目标会立刻把臂拉走
        self._ctrl_qpos = np.array(qpos, dtype=float)
        self._target_qpos = np.array(qpos, dtype=float)
        self._ctrl_accum = 0.0
        self._apply_ctrl()
        mujoco.mj_forward(self.model, self.data)

    # ------------------------------------------------------------------
    # command（spec §13：目标 → 速率限制 → ctrl）
    # ------------------------------------------------------------------

    def set_target_joints(self, joints: Mapping[str, float]) -> None:
        """下发关节角命令（deg，**绝对语义**）。

        命令会在 100Hz 控制环里被**速率限制**后写入 `ctrl`；
        本方法只更新目标，不直接改状态。
        """
        full = dict(self.target_angles_deg())
        for k, v in joints.items():
            if k not in self._qpos_adr:
                raise SimError(f"未知关节 {k!r}（可动关节：{self.joint_ids}）")
            full[k] = float(v)
        self._target_qpos = np.array(self.angle_map.to_qpos(full), dtype=float)

    def set_target_qpos_local(self, qpos_local: Sequence[float]) -> None:
        """直接下发**局部角**目标（rad）。给不关心语义换算的底层测试用。"""
        if len(qpos_local) != self.model.nq:
            raise SimError(f"qpos 长度 {len(qpos_local)} != {self.model.nq}")
        self._target_qpos = np.array(qpos_local, dtype=float)

    def target_angles_deg(self) -> dict[str, float]:
        """上层**请求**的目标（deg，绝对语义）—— 未经速率限制。"""
        return self.angle_map.from_qpos(self._target_qpos)

    def ctrl_angles_deg(self) -> dict[str, float]:
        """100Hz 控制环**限速后真正写进 `ctrl`** 的目标（deg，绝对语义）。

        与 `target_angles_deg()` 的分工（两者不是一回事，别混用）：
          `target_angles_deg()`  上层请求的目标 —— 可以一步跨 60°
          `ctrl_angles_deg()`    舵机速率限制**允许**当前达到的目标
        两者之差 = "舵机正在欠多少债"（速度限幅造成的滞后）。
        `joint_angle` 与 `ctrl_angles` 之差才是位置环自身的跟踪误差。
        """
        return self.angle_map.from_qpos(self._ctrl_qpos)

    def _apply_ctrl(self) -> None:
        """把（已限速的）内部目标写到 `data.ctrl`。

        ⚠️ ctrl 的顺序是**执行器声明顺序**，与 qpos 顺序无关 —— 必须按名字映射，
        否则会出现"命令 base 动了 elbow"这种极难排查的错位。
        """
        for jname, aid in self._actuator_of_joint.items():
            self.data.ctrl[aid] = self._ctrl_qpos[self._qpos_adr[jname]]

    def _advance_ctrl(self, dt: float) -> None:
        """舵机速率限制：把内部目标以 ≤ max_velocity 的速率推向命令目标。"""
        max_step = self.max_velocity_rad_s * dt
        delta = self._target_qpos - self._ctrl_qpos
        self._ctrl_qpos = self._ctrl_qpos + np.clip(delta, -max_step, max_step)

    # ------------------------------------------------------------------
    # step（spec §23/§24：物理步与控制步解耦）
    # ------------------------------------------------------------------

    def step(self, n: int = 1, *, control: bool = True) -> None:
        """推进 `n` 个**物理步**。

        每 `control` 周期（`physics.timestep.control`）重算一次 ctrl，
        模拟"控制器 100 Hz、物理 1 kHz"的真实分层（spec §23）。

        ⚠️ 控制周期累加器必须是**实例状态**，不能是局部变量。
        写成局部变量时，`step(1)` 被连续调用（`settle()` 就是这样）会让累加器
        每次都从 0 开始、永远达不到 10ms 阈值 ⇒ **ctrl 永不更新 ⇒ 命令完全不生效**，
        而 `step(200)` 这种一次调用的写法却完全正常 —— 是个只在特定调用方式下
        才暴露、且表现为"命令没生效"的静默错误（实测踩到）。
        """
        dt = float(self.model.opt.timestep)
        ts_ctrl = float(self.physics.ts_control)
        for _ in range(int(n)):
            if control:
                self._ctrl_accum += dt
                if self._ctrl_accum >= ts_ctrl - 1e-12:
                    self._ctrl_accum -= ts_ctrl      # 保留余数，避免长期相位漂移
                    self._advance_ctrl(ts_ctrl)
                    self._apply_ctrl()
            mujoco.mj_step(self.model, self.data)

    def step_seconds(self, seconds: float, *, control: bool = True) -> None:
        self.step(int(round(seconds / float(self.model.opt.timestep))), control=control)

    def settle(self, seconds: float = 4.0, *, tolerance_rad: float = 1e-3,
               hold_s: float = 0.2) -> float:
        """推进直到关节**持续**静止，返回实际耗时（秒）。

        ⚠️ 判据必须是"持续 `hold_s` 秒内速度都低于阈值"，**不能是"某一步速度低就停"**：
        `reset()` 之后 qvel 恒为 0，若只看单步，第一次 step 就会判定"已静止"并返回 ——
        于是所有"下达命令然后 settle"的测试都会读到**从未移动过的**初始位形，
        表现为"命令完全没生效"（本条是实测踩到的，不是假想）。
        """
        dt = float(self.model.opt.timestep)
        max_steps = max(1, int(round(seconds / dt)))
        need = max(1, int(round(hold_s / dt)))
        quiet = 0
        for k in range(max_steps):
            self.step(1)
            if float(np.max(np.abs(self.data.qvel))) < tolerance_rad:
                quiet += 1
                if quiet >= need:
                    return (k + 1) * dt
            else:
                quiet = 0
        return seconds

    # ------------------------------------------------------------------
    # state（spec §19）
    # ------------------------------------------------------------------

    def state(self) -> SimState:
        qpos = np.array(self.data.qpos, dtype=float)
        return SimState(
            time=float(self.data.time),
            joint_angles=self.angle_map.from_qpos(qpos),
            qpos_local=qpos,
            # ⚠️ 速度 / 力矩只取**自由度**坐标（4 个），与 `robot.joint_order()` 逐位对齐
            # —— 状态帧（`SimState.as_dict` / JR 四元组）就是这个位次。
            # 被动腕 `tool` 有 qpos 但没有自由度，它的坐标**不在**这里；若照搬 nq 长度
            # 的数组，`as_dict` 的 zip 会把 gripper 读成 tool 的值（静默错位）。
            joint_velocities=self.dof_vector(self.data.qvel),
            joint_torques=self.dof_vector(self.data.qfrc_actuator),
            target_angles=self.target_angles_deg(),
            end_effector=np.array(self.data.site_xpos[self._tcp_site], dtype=float),
            contact_count=int(self.data.ncon),
        )

    def dof_vector(self, src: Sequence[float]) -> np.ndarray:
        """把 `nv` 长度的广义向量（`qvel` / `qfrc_*` / `qacc`）抽成**自由度顺序**（4 个）。

        ⚠️ 不能写成 `src[:4]`：被动腕的坐标夹在 `elbow` 与 `gripper` **中间**
        （见 `self.joint_ids`），必须按名字查 `dofadr`，否则会把 tool 的坐标
        当成 gripper 的 —— 而两者量纲相同，错了也不会报错。
        """
        return np.array([float(src[self._dof_adr[j]]) for j in self.robot.joint_order()],
                        dtype=float)

    def ctrl_vector(self) -> np.ndarray:
        """`data.ctrl` 按**自由度顺序**重排。

        三个顺序互不相同，必须按名字映射：执行器声明顺序（`ctrl` 的位次）、
        qpos 顺序（`joint_ids`）、状态帧顺序（`robot.joint_order()`）。
        """
        return np.array([float(self.data.ctrl[self._actuator_of_joint[j]])
                         for j in self.robot.joint_order()], dtype=float)

    def joint_angles_deg(self) -> dict[str, float]:
        return self.angle_map.from_qpos(np.array(self.data.qpos, dtype=float))

    def end_effector_m(self) -> np.ndarray:
        return np.array(self.data.site_xpos[self._tcp_site], dtype=float)

    def end_effector_mm(self) -> np.ndarray:
        return self.end_effector_m() * 1000.0

    # ------------------------------------------------------------------
    # 便利方法
    # ------------------------------------------------------------------

    def gravity_torque(self) -> np.ndarray:
        """当前位形下，仅重力 / 科氏力产生的关节广义力（N·m），**按自由度顺序**。

        用 `mj_rne`（递归牛顿-欧拉）在 qacc=0 下求逆动力学得到的正是"维持静止所需的力"。
        ⚠️ 这是**测试/分析**用，不是控制路径。
        """
        qfrc = np.zeros(self.model.nv)
        mujoco.mj_rne(self.model, self.data, 0, qfrc)
        return self.dof_vector(qfrc)

    def constraint_torque(self) -> np.ndarray:
        """约束（等式 / 限位 / 接触）作用在自由度上的广义力（N·m），**按自由度顺序**。

        被动腕的「绝对角被锁死」是一条**跨 shoulder / elbow / tool 三个坐标**的等式
        约束，所以它会把力矩**传回被驱动的肩 / 肘**。位置环的稳态平衡必须把它算进去：
        否则「kp·(ctrl − qpos) = τ_bias」这条力平衡式会差 λ/kp
        （见 tests/sim/test_gravity.py::test_steady_state_error_is_physical）。
        """
        return self.dof_vector(self.data.qfrc_constraint)

    def body_names(self) -> list[str]:
        out = []
        for i in range(self.model.nbody):
            n = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if n:
                out.append(n)
        return out

    def info(self) -> dict[str, Any]:
        return {
            "robot_id": self.entry.id,
            "xml": None if self.xml_path is None else str(self.xml_path),
            "tcp_site": self.tcp_site,
            "nq": int(self.model.nq),
            "nv": int(self.model.nv),
            "nbody": int(self.model.nbody),
            "ngeom": int(self.model.ngeom),
            "nu": int(self.model.nu),
            "total_mass_kg": float(self.model.body_mass.sum()),
            "timestep": float(self.model.opt.timestep),
            "gravity": list(self.model.opt.gravity),
            "joints": list(self.joint_ids),
        }


#: 旧名字（MeArm 单机器人时代）。保留它是因为 `tools/*.py` 与 `tests/sim/*` 里
#: 有几十处 `from model import MeArmSim` —— 逐个改名只会制造 diff 噪声，
#: 而**没有**任何语义收益（现在是同一份代码服务两台机器人）。
#: 新代码请用 `RobotSim`。
MeArmSim = RobotSim
