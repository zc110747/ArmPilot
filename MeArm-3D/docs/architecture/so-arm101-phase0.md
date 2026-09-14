# SO-ARM101 接入 · Phase 0 只读分析

> 状态：**只读分析，未修改任何代码 / 配置 / 测试。**
> 本文件对应 spec §二十三 Phase 0 要求输出的五项，外加实施前必须裁决的决策点。
> 结论一句话：**抽象层已经存在（MeArm-3D 阶段就已建好），本任务缺的不是接口，而是「选择哪一份模型」的机制。**

---

## 1. 当前模型加载路径（三端）

| 端 | 入口 | 机制 | 关键弱点（本任务要解决的） |
|---|---|---|---|
| 前端 | `frontend/src/robot/model/loadRobotModel.ts:12` | **构建期内联**：`import robotYamlText from '@config/robot.yaml?raw'`；`:584 loadRobotModel()` 是**无参单例** | 只能有一份模型。选择发生在编译期，运行期无法切换 |
| 前端（消费方） | `frontend/src/store/robotStore.ts:211` | 模块级 `const model = loadRobotModel();` | store 绑死在 import 时刻的那一份模型上 |
| 前端（算法） | `kinematics/mearm/MeArmKinematics.ts:103,145` | 构造器缺省 `defineRobot(loadRobotModel())` | 同上 |
| Go 后端 | `backend/internal/cfg/cfg.go:197-216 ResolveRobotConfig` | 候选路径表 `[configured, ../config/robot.yaml, config/robot.yaml, ../MeArm-3D/config/robot.yaml]`，全部**相对 CWD** | 路径写死；`robot.Load` 硬要求 actuators 非空（`robot.go:102-103`）+ 每个 revolute 关节必须有执行器（`:126-134`） |
| Go 后端（消费方） | `internal/robot/robot.go:90 Load` | 解析 joints/actuators/homePose ⇒ `JointOrder` / `LimitTable` / `CalibrationTable` | 执行器语义 = **0..180° 舵机通道**（`robot.go:51`） |
| Python 仿真/测试 | `simulation/mujoco/robotcfg.py:20-26` | `PROJECT_ROOT/config` 下写死 `ROBOT_YAML` / `PHYSICS_YAML`；`load_robot(path)`/`load_physics(path)` **已支持显式路径** | 默认值写死，但改造成本最低 |
| Python（MuJoCo 模型） | `simulation/mujoco/model.py:45` | `DEFAULT_XML = PKG_DIR/"mearm.xml"`（由 `gen_model.py:581` 生成） | XML 路径写死；`server.py:308` 有 `--xml` 但 `main` 从未使用（`:325`） |

**跨端互检**：WS `hello` 携带 `model{id,name,jointOrder,limits,calibration,homePose}`（`protocol.go:128-138`），
前端只比对 jointOrder/limits/calibration、**不比 id**（`wsProtocol.ts:255-303`）⇒ 新增选择器字段/模型 id 对老前端向后兼容。

## 2. 当前 MeArm-V1 数据路径

| 类别 | 路径 | 约束 |
|---|---|---|
| 运动学与物理真值 | `config/robot.yaml` · `config/physics.yaml` | **冻结**：`config/baseline-kinematics-physics.json` 记语义核心哈希（`tools/freeze_baseline.py:46-48`，排除 `geometry/details/recording/deterministic`）⇒ 运动学一变就报错 |
| 黄金回归数据 | `tests/baseline/mearm-v1/{joint,fk,ik,workspace}_cases.json` | 由 `tools/gen_mearm_v1_baseline.py` 生成、`--check` 逐位复现；**必须逐字节不动** |
| 前端读取 | `frontend/tests/helpers/mearmV1Baseline.ts:19` | node `fs`，目录 `tests/baseline/mearm-v1` 写死 |
| Python 读取 | `tests/sim2sim/mearmV1Baseline.py:18` | 同目录写死 |
| MJCF（产物） | `simulation/mujoco/mearm.xml` | 由 `gen_model.py` 生成，`tests/sim/test_simulation.py:452-477` 逐字节核对入库 XML 与现算结果 |

## 3. 当前抽象接口

```text
RobotModel（数据，唯一真值）        model/RobotModel.ts
      │  defineRobot()  —— 分节视图，零转换零拷贝（引用相等有测试钉住）
      ▼
RobotDefinition                    definition/RobotDefinition.ts
      │  implements
      ▼
KinematicsEngine（行为）           kinematics/KinematicsEngine.ts
   ├── capability: KinematicsCapability {positioningDof, supportsOrientation,
   │                                     solverKind:'analytic'|'numeric'|'none'}
   ├── forward(joints)  / forwardAll(joints)   —— 输出 Pose（mm + degree）
   └── inverse(target: Vec3, options)          —— 只收位置（spec §12）
      └── 结果形状 IKResult          kinematics/IKResult.ts
            ikFailure(code, msg) ⇒ positionError=null, solutionType='none'
   └── 当前唯一实现：MeArmKinematics（纯委托既有 fk.ts / ik.ts）
```

**已经具备**：能力声明（可诚实说"我不支持姿态"）、`solverKind`（可声明"没有求解器"）、
统一失败形状（`ikFailure` 可表达"未实现"，不需要伪造 0 误差）、"抽象前后逐位一致"的回归
（`tests/unit/kinematicsEngine.test.ts`、`tests/sim2sim/mearm-v1-baseline.test.ts`）。

**目前没有**（本任务要补，且都属"新增"而非"改 MeArm"）：

1. `RobotRegistry` / `loadRobot(id)` —— 无多模型调度层；
2. `LinkGeometry` 无 `mesh` 类型 —— 前端 3D 只支持 primitive（box/plate/cylinder/sphere/servo/jaw），**没有网格加载路径**；
3. `capability` 没有 `supportsIK` 字段（用 `solverKind:'none'` 表达等价语义）；
4. Sim2Sim 框架按机器人写死（`mearm-v1-baseline.test.ts` / `test_mearm_v1_baseline_mujoco.py`），不是 `runSim2Sim(robot)`。

## 4. 官方 SO-ARM101 模型实测（已下载到 `.workbuddy/probe/so101/`，仅作分析）

| 项 | 实测 |
|---|---|
| 来源 / 许可 | TheRobotStudio/SO-ARM100 · **Apache-2.0** |
| 关节 | `shoulder_pan` / `shoulder_lift` / `elbow_flex` / `wrist_flex` / `wrist_roll` / `gripper` = **5 DOF + 夹爪** |
| 轴 / 限位 | 6 个铰链全部 `axis="0 0 1"`；限位（rad）±1.91986 · ±1.74533 · ±1.69 · ±1.65806 · −2.74385/2.84121 · −0.17453/1.74533 |
| 单位 | `compiler angle="radian" meshdir="assets"`，长度**米**（ArmPilot 是 mm） |
| 执行器 | `<actuator>` 6 个 `<position>`，`ctrlrange` = 关节范围 ⇒ **关节空间位置伺服，无标定偏移**；`<equality/>` 空；无 `<keyframe>` |
| 末端 | 官方自带 **`gripperframe` site**（pos `[-0.0079,-0.000218121,-0.0981274]`，quat）⇒ **TCP 有官方定义，无需改模型** |
| 零点 | 官方 README：**new_calib 各关节虚拟零点 = 行程中点** ⇒ `homePose` 取全 0 **有官方依据** |
| 夹爪语义 | 官方 README 明说 LeRobot 的"0..100 线性"**尚未**反映进 URDF/MJCF ⇒ 不得自行发明 |
| 资源 | 13 个 **二进制** STL，单个 0.06–2.7 MB，**合计 16,129,292 字节（≈15.4 MiB）**，**322,564 三角形** |

> ⚠️ 勘误：本节初稿把 STL 记作 "ASCII"，P1 下载后实测为**二进制**（13 个文件长度全部满足 `84 + 50×n`，
> 头部 80 字节为空）。**结论因此更好**：官方资源本就是二进制 ⇒ **零转换、零适配**，D4 决策自动消解。

## 5. 最小接入点

| # | 接入点 | 做法 | 位置 |
|---|---|---|---|
| A | **选择器** | `config/robots.yaml`：`active: mearm-v1 \| so-arm101` + `registry` 映射到各机器人**自己的**定义文件（只做路由，不复制参数） | 新增 |
| B | **三端 Registry** | 前端 `loadRobotModel(ref?)`（默认读选择器）+ `import.meta.glob` 静态登记；Go `ResolveRobotConfig` 读选择器；Python `robotcfg` 按选择器覆盖默认路径 | 改 3 处入口，MeArm 的 yaml 不动 |
| C | **SO-101 定义** | `config/robots/so-arm101/robot.yaml`：官方数值 m→mm（×1000）、rad→deg，**派生不改数**；`tcp` 取官方 `gripperframe`；`homePose` 全 0 | 新增 |
| D | **SO-101 运动学** | `kinematics/soarm101/SoArm101Kinematics.ts` 实现 `KinematicsEngine`：`forward/forwardAll` 按官方 origin/axis 累乘；`inverse → ikFailure('NOT_IMPLEMENTED')`；`capability.solverKind='none'` | 新增文件，MeArm 不动 |
| E | **3D** | `LinkGeometry` 增 `mesh`（STL）+ 官方 visual geom 的 pos/quat；MeArm 的 primitive 路径不变 | `Link.ts` / `loadRobotModel.ts` / `buildRobotObject3D.ts` 增量 |
| F | **MuJoCo** | 直接加载官方 `so101_new_calib.xml`（**零修改**），末端读官方 `gripperframe` site | 新增 sim 适配 + 参数化 XML 路径 |
| G | **Sim2Sim** | 统一 `runSim2Sim(robotId)`，两个机器人共用同一框架；SO-101 的 IK/Workspace 行显式标 `-` | 新框架 + 现有 MeArm 用例改为其一个实例 |

## 6. 需要修改的文件列表

**新增（不碰 MeArm）**

```text
config/robots.yaml                                  选择器（active + registry）
config/robots/so-arm101/robot.yaml                  SO-101 RobotDefinition（mm/deg，派生自官方）
config/robots/so-arm101/physics.yaml                SO-101 物理量（全 SI），供 MuJoCo
assets/models/so-arm101/official/                   官方 urdf+xml+13 STL，逐字节原样
assets/models/so-arm101/SOURCE.md                   来源/revision/sha256/许可/适配记录
frontend/src/robot/registry/RobotRegistry.ts        定义登记 + 按 id 取 RobotDefinition/Engine
frontend/src/robot/kinematics/soarm101/SoArm101Kinematics.ts
frontend/src/components/RobotScene/meshGeometry.ts  STL → three 对象（含无 DOM 守卫）
tests/baseline/so-arm101/{joint_cases,fk_cases}.json  只建可靠数据；不伪造 ik/workspace
tests/sim2sim/run_sim2sim.py（或 tests/sim2sim/framework.py）  统一框架
frontend/tests/sim2sim/so-arm101.test.ts
docs/architecture/so-arm101-*.md                    实施记录
```

**修改（增量、缺省行为逐值不变）**

```text
frontend/src/robot/model/Link.ts + loadRobotModel.ts   geometry 增 mesh 类型（可选字段）
frontend/src/robot/model/loadRobotModel.ts:584         loadRobotModel(ref?)；默认 = 选择器；新增 registry glob
frontend/src/store/robotStore.ts:211                   改用 active 模型
frontend/src/components/RobotScene/buildRobotObject3D.ts   dispatch 增 mesh 分支（primitive 路径不变）
frontend/src/robot/index.ts                            导出 registry / SoArm101Kinematics
backend/internal/cfg/cfg.go                            读选择器（新增解析，候选表保留）
backend/internal/device/mujoco.go:102-114              传 --xml/--robot
simulation/mujoco/robotcfg.py:20-26                    默认路径可被选择器覆盖
simulation/mujoco/model.py + server.py:306-325         参数化 XML/EE site
tools/gen_so_arm101_baseline.py（可复用 gen_mearm_v1_baseline.py 抽取）  cases 生成
约 20 处 loadRobotModel() 调用点                       显式钉上机器人 id（见决策 D5）
```

**必须逐字节不动（红线）**

```text
config/robot.yaml · config/physics.yaml · config/baseline-kinematics-physics.json
tests/baseline/mearm-v1/*.json
simulation/mujoco/mearm.xml
frontend/src/robot/kinematics/{fk.ts,ik.ts,mearm/MeArmKinematics.ts}
backend/internal/robot/robot.go 的业务逻辑 · protocol 编解码
```

## 7. 实施前必须裁决的决策点

| # | 问题 | 推荐方案（含理由） |
|---|---|---|
| **D1** | 选择器放哪、叫什么 | **新增 `config/robots.yaml`**（`active` + `registry`）。**不要**塞进 `config/robot.yaml`：那份文件进冻结语义哈希，且其中 `robot.model: MeArm-V1` 已表示"模型版本标签"，语义会冲突 |
| **D2** | 单位 | 官方是**米/弧度**，ArmPilot 是 **mm/度**。取 **×1000 精确换算**（数值不改，只换单位），在 `SOURCE.md` 记录"派生自官方，未改数值" |
| **D3** | **SO-101 的执行器/标定**（最关键） | 官方执行器是**关节空间位置伺服**（`ctrlrange` = 关节范围，无 offset/scale/reverse 标定），而 ArmPilot 的 `Actuator` 语义是"**0..180° 舵机通道**"，且 Go 侧**硬要求 actuators 非空**、TS 侧 `ACTUATOR_LIMIT_180` 也卡 0..180。⇒ 三种走法：<br>**①（推荐）**给 `Actuator` 增**可选** `unit: 'deg' \| 'joint'`（缺省 `'deg'` ⇒ MeArm 逐值不变），范围校验改为"按本机器人声明的执行器范围"，SO-101 声明为关节空间（degrees）；<br>② SO-101 不声明 actuators ⇒ 必须放宽 Go 与 TS 的两条既有硬校验（**会动到共享不变量，风险更高**）；<br>③ 用 scale 把 ±110° 压进 0..180 ⇒ **等于伪造标定**，与 spec §22 精神冲突（不推荐） |
| **D4** | 3D 网格 | ~~先按原样加载官方 ASCII STL~~ → **实测官方即二进制 STL**（P1 勘误）⇒ **按原样加载，零转换零适配**。**不重做几何** |
| **D5** | 测试参数化 | `loadRobotModel()` 无参改为"**读选择器**"，并把 ≈20 处调用点显式写成 `loadRobotModel('mearm-v1')`。**这不是修改期望值**，而是把"被测对象"由隐式变为显式 —— 只有这样才能同时满足 spec §17（两种配置各跑完整测试）与 §24A（MeArm 原测试全过）。需你认可这属于"参数化"而非"改测试" |
| **D6** | 关节 `role` | 现有枚举 `['base','shoulder','elbow','tool','gripper']` 放不下 `wrist_flex`/`wrist_roll`。⇒ **先不加 role**（该字段本就可选，校验只要求"同 role 不重复"），确有需要再增量扩枚举 |

## 8. 已知风险（Phase 0 就看见的）

1. **红色区域比想象的大**：`robot.yaml` 同时被冻结哈希、黄金数据生成器、Go 校验、Python 生成器、前端构建期读取共用 ⇒ 任何"为了统一而重构"都会同时触发两个判据失败。本方案**只新增、不改 MeArm** 就是为避开这一点。
2. **前端 mesh 是异步资源**，而 vitest 跑在 **node 环境**（无 DOM、无 `document`）⇒ 必须沿用既有 `plateTexture` 的"无 DOM 不加载 + 失败回退"双守卫，否则会连带打挂几何类测试。
3. **`import.meta.glob` 能否用 `@config` 别名**需在实施第一步用小探针验证（不行就退回相对路径写法）。
4. **仓库会增大约 15.4 MiB 二进制资源**，需你确认可以入库。
5. 官方 `qpos` 顺序（6 关节）与 ArmPilot `JointState`（可动关节）在当前模型下**恰好一致**，但仍要按既有纪律显式声明顺序，不能默认。

## 9. 建议实施顺序（对应 spec Phase 1–8）

```text
P1 官方模型入库（逐字节原样 + SOURCE.md + sha256）
P2 SO-101 RobotDefinition + SoArm101Kinematics（capability: solverKind='none'）
P3 选择器 + 三端 Registry（MeArm 默认值不变 ⇒ 先跑一遍全量验收确认零回归）
P4 3D mesh 接入（MeArm 视觉行为逐值不变）
P5 SO-101 FK + joint/fk cases（不伪造 IK/workspace）
P6 MuJoCo 用官方 XML 原样 + gripperframe site
P7 统一 runSim2Sim(robot) + §19 回归矩阵
P8 切换压力回归（MeArm→SO101→MeArm→SO101）+ MeArm before/after 逐位比对 + 终报告
```

每个 Phase 收尾都跑一次验收（tsc / vitest / vite build / go test / pytest sim + sim2sim），
P3 与 P8 另跑 e2e 与冻结/黄金复现命令。
