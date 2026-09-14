# MeArm-V1 基线冻结 · 现状分析

> **本文档是"动手前的审查记录"，不是设计文档。**
> 目的只有一个：在给 MeArm-3D 加任何抽象之前，把"现在的 MeArm 到底是什么样"
> 逐项钉死。之后所有抽象都以此为界 —— 凡是要改到这里描述的**任何一条数学模型**，
> 一律停止（spec §1.2 / §26）。
>
> - 对应 spec 章节：§2「第一阶段：冻结当前 MeArm 模型」
> - 结论性文档：[`mearm-v1-baseline.md`](./mearm-v1-baseline.md)
> - 无关问题登记：[`mearm-v1-followups.md`](./mearm-v1-followups.md)
> - 生成日期：2026-09-14

---

## 0. 一句话结论

**MeArm-3D 的现有实现已经是"一个 Robot Model + 一套运动学"的干净结构**，
缺的不是架构，而是三样东西：

```text
① 一个明确的版本标识（现在只有 robot.id = mearm，没有 "V1" 这个概念）
② 一份可复现的黄金测试数据（现在全是"随机 seed + 当场比"，没有落盘的期望值）
③ 一层薄接口（现在上层组件直接 import fk.ts / ik.ts 的函数）
```

因此本阶段的抽象是 **"包一层"**，不是 **"重做一层"**。这一点决定了后面所有取舍。

---

## 1. spec §2.1 十问 · 逐项作答

### 1.1 当前 Robot Model 从哪里读取

| 消费方 | 入口 | 读法 |
|---|---|---|
| 前端 | `frontend/src/robot/model/loadRobotModel.ts` → `loadRobotModel()` | 编译期 `import robotYamlText from '@config/robot.yaml?raw'`（**内联进产物**，运行时不读盘） |
| Python / MuJoCo | `simulation/mujoco/robotcfg.py` → `load_robot()` | `yaml.safe_load` 读 `config/robot.yaml` |
| Go 后端 | `backend/internal/robot/robot.go` → `Load()` | `yaml.Unmarshal`（**非严格模式**，未知字段忽略） |

**唯一真值源**：`config/robot.yaml`（`version: 1`）。三端**都读同一份**，
且 Go 后端在 WebSocket `hello` 里回传限位/标定 ⇒ 两端不一致会**在线互检报出**（ADR D27–D33）。

> ⚠️ **`config/physics.yaml` 是第二份真值，但它只放物理量（全 SI）**。
> 运动学量**绝不允许**在 physics.yaml 里出现 —— 由 `test_config_truth_is_not_duplicated` 盯着。

### 1.2 FK 从哪里读取机器人参数

`frontend/src/robot/kinematics/fk.ts`，**全部参数来自传入的 `RobotModel`**，零硬编码：

| FK 用到的模型字段 | 用途 |
|---|---|
| `links[].length` | 父关节系 → 子关节系沿 +Z 的伸展 |
| `joints[].origin.position` / `.rotation` | 关节坐标系固定变换 |
| `joints[].axis` | 绕轴旋转 |
| `joints[].coupling` | 平行四连杆：`effectiveJointAngle() = value + gain × otherValue` |
| `joints[].limits.min` | **缺值回退**（`jointAngleOf`：`state[id] ?? limits.min`）—— 被动关节靠它取值 |
| `tcp.joint` / `tcp.offset` | 末端 = 该关节系下的固定偏移点 |

链式公式（三处实现必须逐项一致，见 `fkref.py` 头注释）：

```text
T_J = T_parent · Tz(parentLink.length) · T(origin.position)
                · R_eulerXYZ(origin.rotation) · R_axis(axis, θ_effective)
```

`jointMatrices()` 沿连杆图**递归遍历**（`walk(link, parentMatrix)`），
因此天然支持分支与叶关节（`gripper` 挂在 `tool_link` 上，与 `tool` 是兄弟）。

### 1.3 IK 从哪里读取机器人参数

`frontend/src/robot/kinematics/ik.ts`，**几何量在运行时从模型求导，不写死常数**：

`ikGeometry(model)` 在 **3 个合法姿态**上跑 FK，交叉验证后导出：

| 量 | 来源 | 本机值 |
|---|---|---|
| `pivotZ` | 肩枢轴世界 z（底座平面之上） | 60 |
| `pivotR` | 肩枢轴水平半径 | 0 |
| `l1` | 肩枢轴 → 肘枢轴 | 80 |
| `l2` | 肘枢轴 → **腕枢轴**（⚠️ **不是**肘→TCP） | 80 |
| `toolOffset` | 腕枢轴 → TCP 的**常量**矢状面偏移 | `[40, 0]` |
| `reach` | `[|l1−l2|, l1+l2]` | `[0, 160]` |

**求导结果是 `WeakMap` 缓存**（`geometryCache`），按模型对象缓存。

三条**主动拦截**（把"改配置后的隐形错误"变成明确报错，而不是静默解出偏解）：

```text
assertPlanar2R()              base.axis=[0,0,1] · shoulder/elbow.axis=[0,1,0] · origin.rotation 全 0
assertPlanarWrist()           tcp.joint 不得是 fixed · axis=[0,1,0] · 深度必须排在 elbow 之后
requireConstantToolOffset()   「腕→TCP」必须在 ≥3 姿态下逐位一致且无矢状面外分量
```

### 1.4 Three.js 使用哪些机器人参数

`frontend/src/components/RobotScene/buildRobotObject3D.ts`（纯 three，无 React）：

| 用途 | 字段 |
|---|---|
| 关节树结构 | `joints[]`（一个关节 = 一个 `THREE.Group`）+ `links[].parent` |
| 连杆伸展 | `links[].length` |
| 关节固定朝向 | `joints[].origin` |
| **关节运动** | `effectiveJointAngle()`（**与 fk.ts 同一函数**，见该文件 import） |
| 显示几何 | `links[].geometry` / `links[].details`（**纯外观，不参与运动学**） |
| 纹理 | `geometry.texture` → `textureRegistry.ts`（`import.meta.glob` 静态登记） |
| TCP 标记 | `tcp.joint` / `tcp.offset` |
| 材质环境反射 | `model.appearance`（`appearance.texturedPlate.*`） |

> ★ **关键事实**：`buildRobotObject3D.ts` 是 `import { effectiveJointAngle } from '@robot/kinematics/fk'`
> —— 渲染层的关节旋转**与 FK 共用同一个耦合表达式**。这是 Phase 3「FK == Three.js」
> 能到 `8.673e-14 mm` 的根本原因，也是**抽象层绝不能复制一份这个函数**的原因。

### 1.5 MuJoCo 使用哪些机器人参数

`simulation/mujoco/gen_model.py` 从 **两份 yaml** 生成 MJCF：

```text
config/robot.yaml    → body.pos / body.quat / joint.axis / joint.range / tendon / actuator / tcp site
config/physics.yaml  → timestep / gravity / inertial / friction / solver / servo 增益
```

**body.pos 的推导与 FK 逐项对应**（`gen_model.py` 头注释）：

```text
body.pos  = origin.position + [0, 0, parentLink.length]
body.quat = eulerXYZ(origin.rotation)
```

**角度语义在 `units.JointAngleMap` 一处转换**：robot.yaml 的 `elbow` 是**绝对角**，
MuJoCo hinge `qpos` 是**局部角**，差一个 coupling 项。

> ⚠️ **`nq = 5` 而自由度 = 4**：被动腕 `tool` 必须有 `qpos`（否则爪不会被连杆带着转），
> 但被 `<equality><tendon>` 锁死 ⇒ **没有自由度**。两种口径
> （`joint_ids` vs `dof_ids`）**混用会得到五元组的 JR** —— 见 `robotcfg.JointCfg.has_qpos`。

### 1.6 MJCF 是如何生成的

```text
config/robot.yaml + config/physics.yaml
        │
        ▼  simulation/mujoco/gen_model.py  (build_xml)
simulation/mujoco/mearm.xml   ← **产物，禁止手改**（重跑即覆盖）
        │
        ▼  mujoco.MjModel.from_xml_path
MeArmSim (model.py)
```

- 生成器关键函数：`local_range_deg`（限位 → 局部空间）/ `absolute_lock_terms`（被动腕锁定项）
  / `inertial_xml` / `collision_geom_xml` / `visual_geoms_xml` / `build_xml`。
- **同步守卫**：`test_generated_mjcf_is_in_sync_with_config` 会盯着"yaml 改了但 xml 没重生成"。
- **限位分工**（刻意，非妥协）：MJCF hinge range 是**保守外接**（`range_padding_deg: 2.0`）
  ⇒ **不是限位真值**；真正的把关人是 Go controller + `limits.py`（ADR D49）。

### 1.7 Sim2Sim 当前测试入口在哪里

**目前没有名为 "sim2sim" 的目录** —— 跨端一致性判据散落在两处，**且已经存在**：

| 判据 | 位置 | 形态 |
|---|---|---|
| **FK(MuJoCo) ↔ FK(参考实现)** | `tests/sim/test_fk.py` | pytest，120 随机 + 256 角点 + 耦合区，`reset()` 纯运动学 |
| **IK(前端) → MuJoCo** | `tests/sim/test_ik.py` | pytest，**加载真实 `ik.ts`**（经 `ikbridge.py`） |
| **FK(前端) ↔ Three.js** | `frontend/tests/acceptance/fk-three-alignment.test.ts` | vitest，200 组随机关节状态 |
| **FK(IK(XYZ)) 闭环（前端）** | `frontend/tests/acceptance/ik-fk-roundtrip.test.ts` | vitest，2000 + 300 + 500 组 |
| 系统级（时间步 / 确定性） | `tests/sim/test_simulation.py` | pytest |

**独立裁判机制**（本项目最重要的一条测试纪律，spec §21 的同义要求）：

```text
tests/sim/ikbridge.py  →  frontend/tests/tools/kinematics-bridge.mjs
                            │  Vite SSR 加载器加载**同一份** ik.ts / fk.ts
                            ▼
                       只用文件递 JSON，Python 侧**不解释任何运动学语义**
```

⇒ 若验收程序用 Python 重写一份 IK，证明的只是"我又写了一遍、而且自洽"（自证）。
**所以基线阶段必须沿用这条桥，不得另起一份 Python 实现。**

### 1.8 当前测试覆盖哪些功能

| 轨 | 命令 | 规模 |
|---|---|---|
| TypeScript | `tsc -b --force` | 0 error |
| 前端单测 | `vitest run` | 318 项 / 24 文件（unit + acceptance） |
| 前端 e2e | `node tests/e2e/ui-smoke.mjs` | 88 项（隔离端口下 84 项，见基线文档说明） |
| 生产构建 | `vite build` | 通过；产物 `__armPilot` **0 命中** |
| Go 后端 | `go test ./...` | 65 项 / 5 包 |
| MuJoCo / 跨端 | `pytest tests/sim -q` | 147 项 / 12 文件 |

覆盖到的能力：模型校验 · 标定 · FK · IK · 拖动平面 · Mock 闭环 · WebSocket 闭环 ·
真机准入拒绝 · 链路误差语义 · 幽灵臂 · 示教录制回放 · 纹理管线 · MJCF 同步 · 真值冻结。

### 1.9 哪些代码是 MeArm 专属

| 位置 | 专属点 | 能否抽象掉 |
|---|---|---|
| `ik.ts` `assertPlanar2R()` | 断言「1 偏航 + 1 组共面 2R」，不满足直接抛 `IkModelError` | **不能** —— 这是"算法适用范围"的诚实声明，不是缺陷 |
| `ik.ts` `solveIkCandidates()` | 矢状面解析 2R + `toolOffset` 预扣 | **不能**（spec 禁止重写 IK） |
| `robot.yaml` 的数值 | 所有 length / limit / offset / scale / reverse | 不适用（那本来就是实例数据） |
| `Link.ts` `jaw` 几何类型 | 夹爪参数化轮廓（ADR D66） | 不适用（**外观层**，已在 D55 冻结之外） |
| `joints.tool` = `passive` + `coupling{gain:-1}` | 本机特有的"爪被连杆锁平" | 不适用（但**暴露了通用机制**：`passive` + `coupling` 本身是通用的） |
| `robotcfg.JointCfg.is_dof / has_qpos` | 区分"自由度"与"有 qpos" | 不适用（**这是通用概念**，只是本机第一次用上） |

### 1.10 哪些代码已经具备通用性

| 层 | 位置 | 通用性证据 |
|---|---|---|
| 模型层 | `model/{RobotModel,Link,Joint,Actuator,Pose}.ts` | 无一处出现 "mearm" 之外的具体数值；`validateRobotModel` 是通用拓扑/一致性校验 |
| 遍历 | `RobotModel.kinematicChain()` / `fk.jointMatrices()` | 沿 `parentLink/childLink` 递归，**天然支持分支与叶关节** |
| FK | `fk.ts` | 零硬编码，纯树遍历；通用到"换一整套 links/joints 也照样跑" |
| 矩阵 | `kinematics/transform.ts` | 纯数学 |
| 坐标 | `kinematics/coordinate.ts` | 纯转换层（mm ↔ 场景单位） |
| 标定 | `calibration/calibration.ts` | 逐执行器仿射映射，**已支持多舵机关节**（同一 jointId 两条 actuator） |
| 传输 | `transport/{RobotTransport,MockTransport,WebSocketTransport}.ts` | 抽象 + 两实现，与机构无关 |
| 后端 | `backend/internal/{device,controller,wsserver,protocol}` | `device.Device` 只有 7 个方法；新增 MuJoCo 实现**没改 WebSocket/协议/前端任何一行** |
| 被动关节 | `Joint.type = 'passive'` + `coupling` | **通用机制**，不是 MeArm 特例；`validateRobotModel` 有一条通用校验链 |

**⇒ 结论：模型层与传输层已经通用；唯一"不通用"的是 `ik.ts` —— 而它恰恰是被要求保护的对象。**

---

## 2. 冻结边界（哪些能改、哪些不能）

| 层 | 判定 | 依据 |
|---|---|---|
| `links[].geometry` / `links[].details` / `appearance` | ✅ **自由** | ADR D55：语义核心哈希不含外观 |
| `links[].length` · `joints[]`（轴/限位/coupling）· `actuators[]`（标定） | ❌ **冻结** | 同上；改了就报错并逐字段列出差异 |
| `physics.yaml`（gravity / contact / timestep / servo / inertia） | ❌ **冻结** | spec §10 / §14 |
| FK / IK **算法** | ❌ **冻结** | spec §1.2 / §20 禁止 4 |
| Three.js 外观结果（比例/轴向/零位/运动方向） | ❌ **冻结** | spec §15 |
| 传输层 / 协议 / 后端分层 | ⚪ **只做接口整理** | spec §13「不要重新建立第二套 Transport 系统」 |
| 注释 / 文档 / **新增只读元数据字段** | ✅ **自由** | 不影响任何真值 |

> 冻结由两道守卫共同保证：
> ① `config/baseline-kinematics-physics.json` + `tools/freeze_baseline.py`（语义核心哈希）
> ② `tests/sim/test_baseline_frozen.py`（11 项辨识力测试：改外观放行、改 length 报错）

---

## 3. 抽象边界判断（spec §8–§15 的落地）

### 3.1 可以做

```text
① RobotDefinition   —— 把"已经有"的模型概念显式命名（不新建结构，只加一层类型名）
② KinematicsEngine  —— 统一 forward/inverse 的**调用面**，实现体仍是既有 fk.ts / ik.ts
③ IKResult          —— 统一**结果形状**，本机 orientationError 恒为 null（不伪装 6D）
④ MeArmKinematics   —— 上述接口在 MeArm-V1 上的实现，**纯委托，零算法**
```

### 3.2 不可以做（以及为什么）

| 诱惑 | 为什么不做 |
|---|---|
| 顺手把 `ik.ts` 改成 `GenericIK` | spec §10 明令：不要创建假的 GenericIK。而且改算法 = 改数学模型 = 违反 §1.2 |
| 给 IK 加 6DOF 位姿参数 | spec §12 明令：4/5/6 DOF 任务空间能力不同，写死 `(x,y,z,r,p,y)` 是错误抽象 |
| 把 `solveIk` 的返回改成 spec 示例的字段名 | 会**破坏现有 2000 组闭环断言与 e2e 探针**。正解是**适配**，不是替换 |
| 重命名现有文件以贴合建议目录树 | spec §23 明令："代码职责优先，目录形式其次"；且 §20 禁止 13（大规模重命名） |
| 抽 MuJoCo 的 physics 参数"顺便统一" | spec §10 禁止 10；且 physics.yaml 已冻结 |

### 3.3 最小抽象的形状

```text
              现有（不改）
   ┌──────────────────────────────────┐
   │ RobotModel  (= RobotDefinition)  │
   │ fk.ts       (= FK 算法)           │
   │ ik.ts       (= MeArm IK 算法)     │
   └──────────────────────────────────┘
                    ▲  委托（零算法）
                    │
              新增（本阶段）
   ┌──────────────────────────────────┐
   │ KinematicsEngine   (interface)   │
   │ IKResult           (type)        │
   │ RobotDefinition    (type + 适配)  │
   │ MeArmKinematics    (实现)         │
   └──────────────────────────────────┘
```

**判据**：抽象层调用与直接调用的输出必须**逐位一致**（不是"接近"）。
这条由 `tests/sim2sim` 回归强制。

---

## 4. 风险登记（本阶段需要盯住的）

| # | 风险 | 现状 | 对策 |
|---|---|---|---|
| R1 | 抽象层复制了 `effectiveJointAngle` 的耦合语义 | `buildRobotObject3D.ts` 与 `fk.ts` **共用**该函数 | 抽象层**禁止**重新实现耦合，只准 re-export / 委托 |
| R2 | 新增 `robot.model` 字段触发真值冻结告警 | `freeze_baseline.py` 用**白名单**（`id/name/units/tcp/homePose`）取字段 ⇒ **不进语义核心** | 已验证：语义核心哈希不变；L1 整文件哈希会变（属"放行"情形），跑 `--update` 刷新记录 |
| R3 | `nq=5 vs 自由度=4` 被抽象层混用 | 三处（前/Go/Py）已统一为 `type === 'revolute'` | 抽象层沿用 `movableJoints()`，**不得**写成 `!== 'fixed'` |
| R4 | e2e 数字随环境变化（84 vs 88） | 复用既有 8091 实例时跳过"断线重连"子项 + opt-in 真机项 | 基线文档**两种都记录并注明语义**，不当作回归 |
| R5 | 随机性让基线不可复现 | 现有测试已用固定 seed（`20260913` / `0x5eed01` / LCG `20260912`） | 黄金数据一律**落盘 JSON**，随机测试只作补充（spec §6） |

---

## 5. 与 spec 的逐条对照

| spec 要求 | 现状 | 本阶段动作 |
|---|---|---|
| §3 增加 `robot.model: MeArm-V1` | 无 | 加字段（白名单外，零风险） |
| §4 Baseline 记录 | 散落在 README | 汇总到 `mearm-v1-baseline.md` |
| §5 `tests/baseline/mearm-v1/` | 不存在 | 新建 4 个 JSON + 生成器 |
| §6 固定 seed | 已有 seed，但**期望值不落盘** | 黄金数据落盘 |
| §9 RobotDefinition | `RobotModel` 即此物 | 加类型别名 + 适配器 |
| §10 KinematicsEngine | 无 | 新增接口 + MeArm 实现 |
| §11 IKResult | 现有 `IkResult`（字段名不同） | 新增统一结果类型 + **适配器** |
| §16–18 Sim2Sim | 判据已存在但分散 | 集中到 `tests/sim2sim` + 前端 `tests/sim2sim` |
