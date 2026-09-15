# ArmPilot / MeArm-3D 架构勘察报告

> **目的**：为「把 MeArm-3D 从几何/运动学仿真升级为 MuJoCo 物理仿真」提供**基于实际代码**的事实基线。
> **纪律**：本文所有数字、路径、调用链均**从仓库实际文件读出**，不凭记忆。凡属推断的地方显式标注 `[推断]`。
> **生成时间**：2026-09-13 · 对应提交 `b2ae0c1`（Phase 11–13 完成后）

---

## 0. 勘察范围（读了什么）

| 层 | 文件 | 用途 |
|---|---|---|
| 参数真值 | `config/robot.yaml` | **唯一数据源**：连杆 / 关节 / 限位 / 标定 / TCP / HOME |
| 运行配置 | `backend/config.yaml`、`backend/config.serial.yaml` | 端口 / 设备模式 / sim 调参 |
| 后端 | `backend/main.go`、`internal/{robot,controller,protocol,device,wsserver,cfg}` | 分层与接入点 |
| 前端运动学 | `frontend/src/robot/kinematics/fk.ts` | FK 链公式（与 MuJoCo 建模一一对应的依据） |
| 前端协议 | `frontend/src/robot/transport/wsProtocol.ts` | JSON 信封 / 错误码 / 模型一致性校验 |
| 前端接线 | `frontend/src/store/transportBridge.ts` | 安全门 / 设备类型判定 |
| 文档 | `docs/coordinate-system.md`、`docs/model-structure.md`、`docs/serial-v1.md`、`docs/decisions.md` | 既有约定与 ADR |
| 工具 | `tools/*.py`、`tools/*.mjs` | 实测与验收脚本 |

**环境事实**（实测）：Python `3.13.14` @ `~/.workbuddy/binaries/python/envs/default`，已装 `mujoco 3.13.0`（cp313 win_amd64）+ `numpy 2.5.3`。编译 / 步进 / 重力已跑通（最小模型 100 步无 NaN）。

---

## 1. 系统分层（实测调用链）

```
Browser (Vite + React + three.js)
   │  ① WebSocket JSON, version=1, :8090/ws/joint
   │     下行: joint_command / ping / status_request
   │     上行: hello / joint_state / error / pong / device_status
   ▼
backend/  (Go, module armpilot/backend)
   ├─ internal/wsserver    只做传输 + hello 元数据，**不碰设备**
   ├─ internal/controller  限位校验 / 标定换算 / ACK 门控 / latest-wins / 安全态
   ├─ internal/protocol    JSON 与 JR 文本的编解码，**不认识机械结构**
   ├─ internal/robot       读 config/robot.yaml，暴露 JointToServo / Validate / JointOrder
   └─ internal/device      ★ 链路末端抽象：Device 接口
                             ├─ sim.go     内置假固件（240°/s + 15ms 延迟 + 限位拒绝）
                             └─ serial.go  真串口（DTR 复位窗口 / 暖机 / 非重叠 I/O）
   ▼
AVR (MeArm-Device)  ← JR 文本协议
```

**关键结构事实**：`device.Device` 是一个 7 方法的**极小接口**：

```go
type Device interface {
    Kind() string
    Connected() bool
    UnavailableReason() string
    WriteLine(line string) error
    Lines() <-chan Line
    OnStatus(fn StatusHandler) func()
    Close() error
}
```

**契约**（`device.go` 头注释）：每条指令**恰好回一行** OK/ERR；异步上报以 `STATE` 等主动帧出现；`WriteLine` 可并发调用；不可用时返回错误而非静默丢弃。

> **这是本次升级最重要的发现**：新增一个物理后端**不需要**碰 WebSocket、controller、protocol 或前端 —— 只要再写一个 `Device` 实现。见 §10。

---

## 2. spec §3.1 回答：当前机械臂有几个实际自由度？

### 2.1 关节清单（`config/robot.yaml` → `joints`）

| 位次 | id | role | type | 轴（关节系） | 限位 min..max（度） | 舵机通道 |
|---|---|---|---|---|---|---|
| 1 | `base` | base | revolute | `[0,0,1]` | **−60.0 .. +60.0** | S9 |
| 2 | `shoulder` | shoulder | revolute | `[0,1,0]` | **−6.0936827341 .. +49.454929245** | S7 |
| 3 | `elbow` | elbow | revolute | `[0,1,0]` | **108.4414852068 .. 141.8582211436** | S8 |
| — | `tool` | tool | **passive** | `[0,1,0]` | **90.0 .. 90.0**（锁定值；无独立输入 ⇒ **无自由度**） | — |
| 4 | `gripper` | gripper | revolute | `[1,0,0]` | **10 .. 100** | S6 |

**结论**：

- **可动关节（DOF）= 4**：`base` / `shoulder` / `elbow` / `gripper`
- **参与末端定位的 DOF = 3**（IK 只解 `base` / `shoulder` / `elbow`；`gripper` 是叶关节，不进定位链）
- **舵机数 = 4**，与 DOF 一一对应（无多舵机关节，虽然配置层已支持）
- `tool` 是**被动关节**（2026-09-13 由 `fixed` 改，见 ADR **D70**）：**会转，但没有独立输入** ——
  爪被连杆锁成**水平**（绝对倾角恒 90°，故 `limits.min == max`），角度完全由 `coupling{gain:-1}→elbow` 派生。
  它只为标定 TCP 参考点而存在，**不计入 DOF**，也不进 `JointState` / UI 滑杆 / JR 协议 / `homePose` / 执行器。
  ⚠️ 但在 MuJoCo 串联网里它**必须是 hinge**（要有 `qpos`）⇒ **`nq` 会把它算进去（=5），而自由度仍是 4**。
  这个口径差是本项目最容易静默出错的地方 —— 前端判据是 `type === 'revolute' && max > min`、
  Go 是 `type == "revolute"`、Python 是 `is_dof = (type == "revolute")` 且 `has_qpos = (type != "fixed")`，
  三处必须一致；混用会协商出**五元组的 JR**，而固件与串口协议只认四元组。见 ADR **D70** 与
  `simulation/README.md` §4.1 事实 6 / 6b。

### 2.2 ⚠️ 五种角度**不可互换**（spec 明确要求区分）

| 概念 | 定义 | 与其它角的关系 | 本项目实测值 |
|---|---|---|---|
| **Servo angle**（舵机角） | 舵机自身的 0..180° 指令值 | `servo = reverse ? −θ·scale + offset : θ·scale + offset` | S9 30..150 / S7 80..160 / S8 20..100 / S6 40..130 |
| **Joint angle**（关节角 θ） | `robot.yaml` 关节空间量，**UI/IK/协议层都用它** | 线性映射自舵机角 | 见上表 |
| **Physical joint angle**（物理机构角） | 真机连杆的真实几何角 | 无传感器 ⇒ **只能由相机反解** | 与 θ 差 ±5° 量级不确定度 |
| **UI angle** | 用户看到/拖动的角 | **≡ Joint angle**（全精度浮点） | HOME 位 elbow = `112.6185771989` |
| **IK angle** | IK 解出的角 | 受 `limit` 钳制后的 Joint angle | 见 `docs/coordinate-system.md` §3.1 |

**已经踩过的坑（必须继承的教训）**：

1. **舵机 ≠ 关节**：S7 的 `scale = 1.44018` 关节度/舵机度，**不是 1:1**。0.5° 的舵机取整在关节侧是 **0.347°**（`wsProtocol.quantizeViaServo` 头注释实测）。
2. **`left/right` 是安装位，不是 kinematic 角色**：固件命名推定角色会得到**反向的机构**（已实测纠正：S7=肩 / S8=肘）。
3. **`elbow` 存的是绝对倾角，不是相对角**（平行四连杆）。见 §6 与 §7。

### 2.3 标定表自洽性核对（本轮实测复算）

用 `robot.yaml` 的 `offset/scale/reverse` 把关节限位代回，应恰好落在舵机硬限位上：

| 关节 | 代入 | 算出舵机角 | 配置的舵机限位 | 判定 |
|---|---|---|---|---|
| base | θ=−60 / +60 | 30.00 / 150.00 | 30 .. 150 | ✅ |
| shoulder | θ=−6.0936827341 | **80.000** | 80 .. 160 | ✅ |
| shoulder | θ=+49.454929245 | **160.000** | 80 .. 160 | ✅ |
| elbow | θ=108.4414852068 | **100.000** | 20 .. 100 | ✅ |
| elbow | θ=141.8582211436 | **20.000** | 20 .. 100 | ✅ |
| gripper | θ=0 / 90 | 40.00 / 130.00 | 40 .. 130 | ✅ |

**四个舵机代入 HOME（全 90°）恰好得到 `homePose`** → 标定表自洽，可作为 MuJoCo 参数生成的**交叉校验**（见 §12 验收项）。

---

## 3. 尺寸链与几何真值（spec §4）

### 3.1 连杆长度（`links[].length`，单位 mm，**运动学唯一真值**）

| 连杆 | length | 显示几何（仅外观，不参与运动学） |
|---|---|---|
| `base_link` | **0** | plate 94×82×7 |
| `column_link` | **60** | plate 66×60×7 + 两片立柱侧板 + S7 舵机 |
| `upper_arm_link` | **80** | 两片板 22×5×74 + S8 舵机 |
| `forearm_link` | **80** | 两片板 18×5×68 |
| `tool_link` | **40** | 腕座 + S6 舵机 + 爪铰轴 |
| `jaw_link` | **0** | 仅提供爪片尺寸 [9, 4.5, 34] |

### 3.2 IK 派生几何量（`docs/coordinate-system.md` §3.1）

```
pivotZ = column_link.length            = 60 mm
pivotR = 0 mm                          （肩枢轴落在偏航轴上）
L1     = upper_arm_link.length         = 80 mm
L2     = forearm_link.length           = 80 mm   ← ⚠️ 肘枢轴 → **腕枢轴**，不含腕→TCP
toolOffset = [40, 0] mm                腕枢轴 → TCP 的**常量**偏移（径向 40 / 竖直 0）
reach  = [|L1−L2|, L1+L2]              = [0, 160] mm  ← L1 == L2 ⇒ 内半径退化为 0
```

> ⚠️ 这四项在 **2026-09-13 被动腕改造**后全部重导（旧值 `L2 = 120`、`reach = [40, 200]`）。
> 爪被连杆锁成**水平**之后，`elbow → TCP` 不再是定长直线段（随 θe 在 109~120mm 间变），
> 于是 2R 的作用对象改为「肘枢轴 → 腕枢轴」，而「腕枢轴 → TCP」退化成一个**常量矢量**，
> 解算前从目标点里减掉即可。`ikGeometry()` 会在 **≥3 个合法姿态**上数值验证这个"常量"前提
> （容差 1e-6 mm），不成立就抛 `IkModelError` —— 而不是让所有解静默偏 40mm。见 ADR **D70**。

> **铁律**：`geometry` / `details` **只影响外观**。有测试锁死：把整份 geometry/details 换成 `none`，`endEffectorPosition()` **逐位不变**（`tests/unit/linkGeometry.test.ts`）。
> **对 MuJoCo 的含义**：Visual / Collision / Physics 三种几何可以各自简化，但**必须从同一份 `robot.yaml` 的 length 派生**。

---

## 4. 坐标系与单位（spec §9 / §10）

### 4.1 统一坐标系（`docs/coordinate-system.md` §1，实测）

**右手系，Z-up**：

| 轴 | 含义 | 正方向 |
|---|---|---|
| X | 前后 | **+X = 正前方**（机械臂伸出方向） |
| Y | 左右 | **+Y = 左侧** |
| Z | 上下 | **+Z = 上方** |

### 4.2 各坐标系的映射关系

| 坐标系 | 与 Robot 系的关系 | 备注 |
|---|---|---|
| World | **≡ Robot**（根连杆近端坐标系即世界原点） | `fk.ts` 用 `mat4Identity()` 起步 |
| Robot | 定义如上 | `robot.yaml` 的 `tcp.joint` / `links` 均在此系 |
| Web 3D (three.js) | **1 unit = 1 mm**，无缩放无旋转 | `docs/coordinate-system.md` §5 |
| **MuJoCo** | **同样是右手 Z-up** ⇒ **无需任何旋转对齐** | MuJoCo 世界系约定即 +Z up、右手 |
| Camera | 正交侧视（当前实测方式） | 只用于外部真值，不进仿真 |

> ✅ **重大利好**：本项目坐标系与 MuJoCo 默认约定**天然一致**，不存在"为了对齐而旋转模型"的需求（spec §9 明令禁止用视觉旋转掩盖坐标错误）。
> **唯一必须做的转换是单位**（见下）。

### 4.3 单位转换层（spec §10）

| 量 | UI / 配置 / 协议 | MuJoCo 内部 | 转换 |
|---|---|---|---|
| 长度 | **mm** | **m** | `× 1e-3` |
| 角度 | **degree** | **radian** | `× π/180` |
| 角速度 | deg/s | rad/s | `× π/180` |
| 质量 | g（配置里建议） | **kg** | `× 1e-3` |
| 力矩 | N·mm（配置里建议） | **N·m** | `× 1e-3` |
| 重力 | — | `0 0 -9.81` | 固定 |

**纪律**：转换必须在**一处分层**完成（建议 `simulation/mujoco/units.py`），禁止散落在各处 —— 否则必然出现"某处忘了除 1000"的静默错误。

---

## 5. 关节角语义（MuJoCo 建模的成败关键）

`fk.ts` 的 FK 链公式（**逐字来自源码**）：

```
T_J = T_parent · Tz(parentLink.length) · T(origin.position)
              · R_eulerXYZ(origin.rotation) · R_axis(θ_effective)
```

其中 `θ_effective = effectiveJointAngle(model, state, joint)`：

```
θ_effective = θ_joint + coupling.gain × θ_couplingJoint      // 无 coupling 时即 θ_joint
```

### 5.1 各关节的 θ=0 语义与转向（`docs/coordinate-system.md` §3）

| 关节 | 轴 | θ=0 的物理含义 | +θ 方向 |
|---|---|---|---|
| `base` | `[0,0,1]` | 指向 +X（正前方） | 绕 +Z 逆时针 → 转向 +Y（左） |
| `shoulder` | `[0,1,0]` | 大臂**竖直向上** | 绕 +Y → 向 +X 前方倾倒 |
| `elbow` | `[0,1,0]` | 小臂**竖直向上**（绝对角，指天顶） | 向前倾倒（**绝对值**，见 §6） |
| `gripper` | `[1,0,0]` | 完全**闭合** | 张开（两爪沿 ±Y 分开） |

### 5.2 HOME 位（`robot.yaml` → `homePose`，相机实测反算）

| 关节 | θ（度） | 对应舵机角 |
|---|---|---|
| base | 0 | S9 = 90 |
| shoulder | 0.8498937633 | S7 = 90 |
| elbow | 112.6185771989 | S8 = 90 |
| gripper | 50 | S6 = 90 |

---

## 6. ⚠️ 最高风险项：`elbow` 的耦合与限位（本轮最重要的技术发现）

### 6.1 事实（`robot.yaml`，相机实测）

真机小臂由**独立舵机 S8 经平行四连杆**驱动，存的是**离开天顶的绝对倾角**：

> 扫 S7 使**肩关节**转过 **52.91°** 时，小臂**绝对倾角只漂 10.08°**（相对角却变了 42.83°）
> ⇒ 模型用 `coupling: {joint: shoulder, gain: -1}` 表达。

即：**局部旋转角 = θ_elbow − θ_shoulder**。

### 6.2 MuJoCo 建模必须做的换算

MuJoCo 的 `hinge` joint 的 `qpos` 是**相对于父 body 的局部角**。因此：

```
mujoco.qpos["elbow"] = radians(θ_elbow − θ_shoulder)      ← 不是 θ_elbow！
mujoco.qpos["shoulder"] = radians(θ_shoulder)
mujoco.qpos["base"]     = radians(θ_base)
```

**把 `θ_elbow` 直接写进 MuJoCo 会产生一条"小臂不跟随肩"的错误机构** —— 这正是本项目已经踩过一次的同类错误（把舵机角色按命名推定导致反向机构）。

### 6.3 ⚠️⚠️ 单一 `hinge range` **无法**表达 `elbow` 的限位

`robot.yaml` 里两者**各自**受限：

```
θ_shoulder ∈ [−6.0937, +49.4549]
θ_elbow    ∈ [108.4415, +141.8582]     （绝对角）
```

MuJoCo 只能对 `elbow` 的**局部角** L = θ_elbow − θ_shoulder 设一个 `range`。代入得：

```
L 的外接区间 = [108.4415 − 49.4549, 141.8582 − (−6.0937)] = [58.9866, 147.9519]
```

**但这个外接盒会放行真机不允许的位形**。反例：`θ_shoulder = −6.0937, L = 58.9866`
⇒ `θ_elbow = 52.89°`，**远低于** elbow 的下限 108.44°。

反向试"内切"（要求 ∀θ_shoulder 都合法）：

```
需要 L ≥ 108.4415 − θ_shoulder  ⇒  L ≥ 108.4415 − (−6.0937) = 114.5352
需要 L ≤ 141.8582 − θ_shoulder  ⇒  L ≤ 141.8582 − 49.4549  =  92.4033
⇒ [114.5352, 92.4033] = 空集
```

**⇒ 结论：`elbow` 的合法域在局部角空间是"斜的"，任何单一 `range` 都表达不了。**

### 6.4 因此的架构决定（写进 ADR）

| 层 | 职责 |
|---|---|
| `config/robot.yaml` | **唯一**限位真值（绝对角语义） |
| Go `controller.Apply` | 已在做：命令到达前 `model.Validate` ⇒ 越界**同步拒绝**（`ERR JOINT ...`） |
| MuJoCo `hinge range` | 只设**保守外接**（防数值爆炸），**不承担限位一致性职责** |
| MuJoCo 控制层 | 必须**用同一份 `robot.yaml`** 做 `JointLimit` 校验，越界钳制/拒绝 |
| 前端 UI / IK | 已受 `limit` 约束 |

**验收判据**（spec §14 的"四方一致"）：对同一组随机命令，`UI 放行集合 ≡ IK 解集合 ≡ Go controller 放行集合 ≡ MuJoCo 放行集合`。
故 §12 必须有一条**专门测试**：把 `θ_elbow`/`θ_shoulder` 推到外接盒角点上，断言 MuJoCo 侧与 Go 侧**拒绝对称**。

---

## 7. FK ↔ MuJoCo 刚体树的一一对应推导

由 §5 的 FK 公式与 MuJoCo 的 `body` 变换语义（`body frame = parent · T(body.pos) · R(body.quat) · R_axis(θ)`）可得**逐项对应**：

| MuJoCo body | 相对父的 `pos`（m） | joint | `axis` | 对应 FK 项 |
|---|---|---|---|---|
| `column` | `0 0 0` | `base` | `0 0 1` | `Tz(base_link.length=0)` |
| `upper_arm` | `0 0 0.060` | `shoulder` | `0 1 0` | `Tz(column_link.length=60)` |
| `forearm` | `0 0 0.080` | `elbow` | `0 1 0` | `Tz(upper_arm_link.length=80)` |
| `tool` | `0 0 0.080` | *（fixed，无 joint）* | — | `Tz(forearm_link.length=80)` |
| `jaw` | `0 0 0.040` | `gripper` | `1 0 0` | `Tz(tool_link.length=40)` |

**规则**：`body.pos = [0,0, parentLink.length] + joint.origin.position`（本项目 `origin.position` 全为 `[0,0,0]`，`origin.rotation` 全为 0 ⇒ 无额外旋转）。
**通用化**：若将来 `origin.rotation` 非零，须写成 `body.quat = quat(eulerXYZ(origin.rotation))`。**生成器必须实现这条**，不能写死。

### 7.1 TCP 的落点（易错）

```
TCP = T_tool · T(tcp.offset = [0,0,40])
```

`tcp.joint = "tool"` 是**刻意**选择（不是 `gripper`）——使 TCP **不被夹爪开合角污染**。

⇒ MuJoCo 里 TCP 必须放在 **`tool` body** 上（`site pos="0 0 0.040"`），
**不能**放在 `jaw` body 的原点上（那会被 `gripper` joint 的旋转带走）。

> ⚠️ 2026-09-13 起 `tool` 是**被动 hinge**（不再固定），但这条结论**不变**：
> TCP 仍挂在 `tool` body 上。区别只是 `T_tool` 现在含一个由 `coupling` 派生的局部角
> —— 那个角正好表达"爪被连杆锁平"，**不需要**在 TCP 定义里另加任何补偿。
> 一旦想把爪的锁定当成"TCP 的常量偏移"写死在这里，就会和 `ik.ts` 的求导前提打架。

### 7.2 `gripper` 的建模取舍（显式声明）

真实 meArm 的夹爪是**对称开合**（渲染层特例化为两片爪各转 `±θ/2`）。
MuJoCo 一个 `hinge` 只能驱动一个 body ⇒ 第一版采用：

> `jaw` body 挂单个 `hinge`（`axis 1 0 0`），几何用**简化表达**（一片随动爪 + 一片固定爪）。
> **明确声明**：未建模平行开合连杆，夹取几何为简化近似。

`gripper` 不参与末端定位，故该简化**不影响** spec §20/§21 的 FK/IK 验收。

---

## 8. API / 协议现状（spec §25：不要重新设计协议）

### 8.1 消息类型（`protocol.go` + `wsProtocol.ts`，`Version = 1`）

| 方向 | type | 载荷 |
|---|---|---|
| C→S | `joint_command` | `{version, type, timestamp, seq?, joints:{id:deg}}` |
| C→S | `ping` / `status_request` | — |
| S→C | `hello` | `model: {id,name,source,jointOrder,limits,calibration,homePose}` |
| S→C | `joint_state` | `joints: {id:deg}` |
| S→C | `error` | `code, message` |
| S→C | `pong` / `device_status` | — |

错误码：`BAD_MESSAGE` / `VERSION_MISMATCH` / `JOINT_LIMIT` / `DEVICE_UNAVAILABLE` / `ACK_TIMEOUT` / `INTERNAL`。

### 8.2 设备文本协议（Go ↔ AVR，`docs/serial-v1.md` §4）

```
JR <j1> <j2> <j3> <grip>      下行（关节角，保留 1 位小数）
OK JR S9=.. S7=.. S8=.. S6=.. 回执（舵机角 —— ⚠️ 是目标角，不是实际位置）
STATE <j1> <j2> <j3> <grip>   主动上报（关节角，2 位小数）
ERR JOINT <id> <v> (limit <min>..<max>)
```

### 8.3 现有关键语义（**MuJoCo 后端必须复刻**）

1. **`OK JR` 只用于标定核对，绝不发布状态**（controller.go 注释：拿它当 Actual ⇒ 误差恒 0）。
2. **`STATE` 才是状态源**，由 `ServoAnglesToJoints` 从**舵机角反算关节角**（检验标定表可逆性）。
3. **ACK 门控 + latest-wins**：同一时刻只允许 1 条 JR 在途，新命令只覆盖待发槽，**不排队**。
4. **链路精度 = 跟踪误差可分辨下限**：`WIRE_JOINT_STEP_DEG = 0.1`。
5. **`calib_tolerance_deg` 必须按末端量化步长设**：sim = 0.1 / serial = 1.0。

> **对 MuJoCo 的含义**：MuJoCo 后端回执里的舵机角，应取**物理状态反算**值（而非目标值），否则会退化成"乐观 ACK"，把"物理滞后/下垂"这条最重要的观测抹掉。

### 8.4 前端对 `device` 的处理（实测）

| 位置 | 逻辑 | 对 `mujoco` 的影响 |
|---|---|---|
| `transportBridge.ts:162` | 安全门：`device === 'serial'` 才视为真机 | `mujoco` ⇒ 当仿真放行 ✅（正确） |
| `WebSocketTransport.ts:528` | `serial` 用 `ARRIVED_EPS_SERIAL_DEG`，否则 `ARRIVED_EPS_DEG` | MuJoCo 用严格的 0.1 阈值 ✅ |
| `ConnectionControl.tsx:287` | 直接显示 `wsStats.device` | 显示 "mujoco" ✅ |

**⇒ 前端零改动即可接入。**

---

## 9. 状态模型（spec §19）

### 9.1 现状

`joint_state` 只有 `joints`（关节角）。前端 `TransportStats` 另含 `device` / `lagDeg` 等链路统计。

### 9.2 MuJoCo 可提供的**增量**（协议扩展候选）

| 量 | MuJoCo 来源 | 协议现状 |
|---|---|---|
| `joint_angles` | `qpos` → 反算关节角 | ✅ 已有（`joint_state`） |
| `joint_velocities` | `qvel` | ❌ 缺 |
| `joint_torques` | `qfrc_actuator` / `actuator_force` | ❌ 缺 |
| `end_effector` | `site_xpos` | ⚠️ 前端由 FK 自算（可信，与后端一致） |
| `contact_count` | `ncon` | ❌ 缺 |

⇒ **Phase 7/8 需要新增一条上行消息 `physics_state`**（或 `joint_state` 加可选字段，保持向后兼容）。
**纪律**：`Version` 保持 1，新增字段必须**可选**（老前端忽略之仍工作），避免 `VERSION_MISMATCH` 打死现有链路。

---

## 10. ★ MuJoCo 接入点分析（本报告的核心结论）

### 10.1 三个候选方案对比

| 方案 | 改动面 | 优点 | 缺点 |
|---|---|---|---|
| **A. 第三个 `device.Device` 实现**（推荐） | 后端 +1 文件；`main.go` +1 case；前端 0 | 完全复用 ACK 门控 / 限位 / 标定 / latest-wins / 重连；前端零改动；启动一条命令 | Python 进程通信需定义（见 10.2） |
| B. 独立 Python WS 服务，前端加第三个 transport | 前端 transport 层 + 协议 + 安全门全要改 | MuJoCo 侧自治 | 绕过 controller ⇒ **限位/标定真值出现第二份**，违反铁律；前端改动大 |
| C. Go 内嵌（cgo / 重写动力学） | 大量 Go 代码 | 单进程 | 不现实；MuJoCo 官方绑定是 C/Python |

**推荐 A**，理由：
1. spec §2「不要让 MuJoCo 代码直接侵入现有 Web UI」✅
2. spec §25「不要重新设计协议」✅
3. spec §40「MuJoCo 是 Physics Backend，Real Robot 将来也是另一个 Backend」✅ —— `device.Device` **就是**那个 Backend 抽象，已经现成
4. 项目铁律「模型/标定/限位真值只有一份」✅ —— 方案 A 让 controller 继续做唯一裁决者

### 10.2 Python 侧通信方式（方案 A 的子选择）

| 方式 | 评价 |
|---|---|
| **stdio JSONL 子进程**（推荐） | Go `exec.Command` 拉起 `python -u`；生命周期绑定；无端口冲突；`-u` 禁缓冲即可 |
| localhost TCP | 需端口管理 + 启动顺序协调 |
| 共享内存 | Windows 上复杂，收益低 |

**推荐 stdio JSONL**：Go 侧 `WriteLine` → 写一行 JSON 到 stdin；Python 侧按物理步进，用 stdout 回 `OK JR ...` / `STATE ...`（**直接复用现有文本协议**，`protocol.ParseReply` 零改动）。

> ⚠️ **潜在风险（需在验证时确认）**：MuJoCo Viewer 需要 GUI 主线程 + `launch_passive` 的 `sync()` 循环，
> 与 stdio 协议循环**冲突**。故必须分成两个入口：
> - `run.py` —— 带 Viewer 的独立演示（spec §27）
> - `server.py` —— 无头 stdio 服务（供 Go 驱动）
> 两者**共享同一份模型与参数**（从 `robot.yaml` 生成）。

### 10.3 建议的目录结构（已过 gitignore 探针）

spec 推荐 `simulation/mujoco/`。**探针结论**：
`git check-ignore simulation/` 会报命中（误导），但**决定性判据 `git add -A -n` 显示会正常收录**（实测输出 `add 'MeArm-3D/simulation/mujoco/probe.xml'`）。
⇒ **`simulation/` 目录名可用**，提交前用 `git add -A -n | grep simulation` 复验。

```
MeArm-3D/
├── config/
│   ├── robot.yaml            # ★ 运动学/标定/限位唯一真值（已存在）
│   └── physics.yaml          # ★ 新增：质量/密度/摩擦/舵机/时间步（物理真值）
├── simulation/
│   └── mujoco/
│       ├── mearm.xml         # 生成产物（source of truth 的编译结果，可重生成）
│       ├── gen_model.py      # ★ robot.yaml + physics.yaml → mearm.xml（禁止手工改 xml）
│       ├── units.py          # deg↔rad / mm↔m 单点转换层
│       ├── model.py          # 加载 / reset / step / state（spec §31 接口）
│       ├── server.py         # 无头 stdio 服务（Go 后端驱动）
│       ├── run.py            # 带 Viewer 的独立演示（spec §27）
│       ├── record.py         # JSONL / CSV 记录（spec §34）
│       └── calibrate.py      # 标定接口占位（spec §38）
├── tests/sim/                # spec §35 的 pytest 套件
└── backend/internal/device/mujoco.go   # ★ 第三个 Device 实现
```

> **`mearm.xml` 是生成物还是手写源？**
> spec §11 要求「`mearm.xml` 必须保留作为 source of truth」。
> 本项目的更高层铁律是「参数真值只有一份 = `robot.yaml`」。
> **调和方案**：`mearm.xml` 作为**唯一的 MJCF 模型源**入库（可读、可审），由 `gen_model.py` 从两份 yaml 生成；**禁止手工编辑 xml**（生成器重跑即覆盖）。这样既满足 spec，又不产生第二份参数真值。

---

## 11. 参数配置化（spec §28）

| 参数类 | 归属文件 | 理由 |
|---|---|---|
| 连杆长度 / 关节轴 / 限位 / 标定 / TCP / HOME | **`config/robot.yaml`**（已存在） | 与前端、Go 后端**共用**；已有在线互检机制 |
| 质量 / 密度 / 惯量 / 摩擦 / 接触刚度 / 阻尼 | **`config/physics.yaml`**（新增） | 纯物理量，前端与真机固件都不需要 |
| 舵机模型（max_torque / kp / kd / max_velocity） | `config/physics.yaml` | spec §13 / §38 |
| 仿真时间步 / 积分器 / 求解器 | `config/physics.yaml` | spec §23 |
| 运行参数（端口 / 设备模式 / 串口） | `backend/config.yaml`（已存在） | 已有约定：**只放运行参数** |
| 重力 | `config/physics.yaml` | 便于做"关重力"对照实验 |

**纪律**：`backend/config.yaml` 的既有约定（"只放运行参数，禁写限位与标定"）**继续有效**，`physics.yaml` 同样**不得**包含限位/标定。

---

## 12. 已知漂移与风险清单（交付时必须处理）

| # | 问题 | 证据 | 处置 |
|---|---|---|---|
| R1 | `docs/model-structure.md` §4 仍写 **S8=肩 / S7=肘** | 与 `robot.yaml` 实测结论（S7=肩 / S8=肘）**相反** | 修正文档（显示层注释漂移） |
| R2 | `elbow` 限位无法用单一 hinge range 表达 | §6.3 反例 + 空集证明 | 架构决定见 §6.4；MuJoCo 侧必须做 JointLimit 校验 |
| R3 | 肩标定增益偏差 **−13.1%**，测量跨批不可复现 | `verify_calib_repro.py`（极差 肩 10.8% / 肘 52.9% > 5%） | **不在本阶段修**；MuJoCo 用当前 `robot.yaml` 值，误差在 README 声明 |
| R4 | 真机无位置回读（MG90S） | `main.go` 启动日志明确警告 | MuJoCo 是**唯一**能提供真值状态的环节，正好补此短板 |
| R5 | `OK JR` 的舵机角是**目标值** | `controller.go` 注释 | MuJoCo 后端回执必须给**物理反算值** |
| R6 | 物理参数（质量/惯量/摩擦/舵机扭矩）**均无实测** | 无任何称重/堵转测试记录 | **必须在 README 显著声明**：本阶段为 **Level 3→4 参数化物理仿真**，非 Level 5 标定模型（spec §37） |
| R7 | `simulation/` 的 gitignore 假阳性 | `check-ignore` 与 `git add -n` 结论冲突 | 提交前用 `git add -A -n` 复验 |

---

## 13. 阶段映射（spec §39 → 本项目）

| spec Phase | 本项目任务 | 依赖 |
|---|---|---|
| 1 分析 | ✅ 本文 | — |
| 2 MJCF 刚体树 | `simulation/mujoco/mearm.xml` + `gen_model.py` | robot.yaml |
| 3 质量/惯量/重力 | `config/physics.yaml` + Gravity Test | Phase 2 |
| 4 actuator / 限位 | 舵机模型 + JointLimit 双端一致 | Phase 3 |
| 5 碰撞/摩擦 | contype/conaffinity 分组 + exclude | Phase 4 |
| 6 Python Backend | `model.py` / `server.py` / `run.py` | Phase 5 |
| 7 WebSocket 接入 | `backend/internal/device/mujoco.go` | Phase 6 |
| 8 Web UI 回传 | `physics_state` 可选字段 | Phase 7 |
| 9 FK/IK/物理一致性 | 100 点 IK + FK<1mm 对比 | Phase 7 |
| 10 测试/文档/配置 | `tests/sim/*` + README | 全部 |

---

## 14. 本阶段**明确不做**（spec §30）

`AI` / `LLM` / `Agent` / `RL` / 自训练 / 视觉学习 / 数据集训练 / 策略训练 / 自动数据采集 —— **一律不引入**。

本轮唯一目标：**建立可信的 MeArm 物理仿真地基**，并预留 spec §31 的
`Observation / Action / Reset / Step / State` 五个扩展接口。
