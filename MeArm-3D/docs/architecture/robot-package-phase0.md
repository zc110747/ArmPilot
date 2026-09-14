# Robot Package + Working Robot 架构重构 · Phase 0 只读分析

> 对应 spec《ArmPilot Robot Package + Working Robot 架构级重构》§42 / §28。
> **本阶段不改任何代码**。产物 = ① 现状事实 ② 文件归属判定表 ③ 冲突登记 ④ 待裁决决策点。
>
> 冻结约束（spec §1）：**MeArm-V1 Golden Baseline 优先级最高**，任何迁移不得改其算法/数据/行为。

---

## 0. 一句话结论

> **现有工程已经落地了"上一版 spec"的逻辑抽象（`RobotDefinition` / `KinematicsEngine` /
> `RobotRegistry` / 选择器 / 统一 Sim2Sim），而且 Core 内部确实没有 `if robot == …` 特判。
> 缺的不是"抽象"，而是三样具体的东西：**
>
> 1. **Robot Package 的物理边界**（`manifest.yaml` + 包内自带 model/physics/kinematics/tests）
> 2. **Core 侧的执行组件**（`RobotLoader` / `RobotRuntime` / `PackageValidator` / `TestRunner`）
> 3. **Working Robot 运行实例**（+ `update.bat` 的 hash 检测与原子替换）

因此本次重构的性质是：**把"已经逻辑解耦"的代码，补上"物理边界 + 构建/运行闭环"**，
而不是重新做一遍解耦。这对"保护 MeArm 冻结基线"是重大利好 —— 算法层不需要动。

---

## 1. 现状：三端调用链（实测）

```text
                    config/robots.yaml          ← 选择器（只放指针：id/name/config/physics/mjcf/tcpSite）
                            │
        ┌───────────────────┼───────────────────┐
        ↓                   ↓                   ↓
  frontend              backend             python
robotConfigRegistry.ts  robot/registry.go   robotcfg.py
 (import.meta.glob        (LoadSelector)    (load_robot_selector)
  + ?raw 静态登记)
        │                   │                   │
        ↓                   ↓                   ↓
  loadRobotModel(id)   Entry{ConfigPath,    RobotCfg + PhysicsCfg
        │               PhysicsPath,             │
        ↓               MJCFPath,TCPSite}        ↓
   RobotModel                                  RobotSim(model.py)
        │                                          │
        ├──→ defineRobot() → RobotDefinition       ├── MuJoCo MjModel
        │        │                                 │   (mearm.xml 生成 / so101_new_calib.xml 官方原样)
        │        ↓                                 │
        │   loadRobot(id) ──→ RobotRegistry ──→ KinematicsEngine
        │        │                                 │
        │        ├─ MeArmKinematics      ──→ fk.ts（**通用**）+ ik.ts（**MeArm 专属解析解**）
        │        └─ SoArm101Kinematics   ──→ fk.ts（**同一份通用 FK**）+ inverse() = NOT_IMPLEMENTED
        │
        └──→ buildRobotObject3D(model) ──→ Three.js 渲染树（**通用**，直接消费 RobotModel）
                    │
              robotStore（活动机器人 = store 状态，含守卫与切换）
                    │
              transportBridge → RobotTransport（Mock / WebSocket）
```

**读法**：横向三端各读同一份配置；纵向 Core（定义/模型/FK/渲染/传输/regstore）与
Robot 专属（`ik.ts` / MJCF / 标定值 / 测试数据）**已经分开**，只是**都在同一个扁平目录里**。

---

## 2. Phase 0 十四问逐条作答（spec §42）

| # | 问题 | 结论 |
|---|---|---|
| 1 | 当前目录结构 | `frontend/`(104) `backend/`(27) `tests/`(25) `assets/`(25) `tools/`(23) `simulation/`(13) `docs/`(12) `config/`(6) `3d-models/`(1) `protocol/`(1) + 根 3 文件（`README.md` / `start.bat` / `.gitignore`） |
| 2 | MeArm-specific 文件 | 见 §3 表中 `PKG` 与 `PKG-TOOL` 行：`ik.ts`、`MeArmKinematics.ts`、`IKResult.fromMeArmIkResult`、`config/robot.yaml`、`config/physics.yaml`、`mearm.xml`、`gen_model.py`、`assets/textures/mearm/**`、`3d-models/mearm3Dasm.STEP`、`tests/baseline/mearm-v1/**`、11 个 MeArm 硬件/纹理工具 |
| 3 | Shared Core 文件 | 见 `CORE` 行：`model/*`、`fk.ts`、`transform.ts`、`coordinate.ts`、`KinematicsEngine.ts`、`IKResult.ts`(类型部分)、`registry/*`、`transport/*`、`teach/*`、`interaction/*`、`calibration/*`、`components/**`、`store/**`、`simulation/mujoco/{model,server,run,record,units,limits,fkref,sim2sim,robotcfg}.py`、`tests/sim/{conftest,harness,ikbridge}.py`、`backend/**` |
| 4 | Generated 文件 | `simulation/mujoco/mearm.xml`（由 `gen_model.py` 从 robot.yaml 生成，**禁手改**）；`config/robots/so-arm101/robot.yaml`（由 `gen_so_arm101_robot_yaml.py` 生成）；`frontend/dist/**`；`backend/bin/*.exe` |
| 5 | Robot-specific tests | `tests/baseline/mearm-v1/{fk,ik,joint,workspace}_cases.json`、`tests/baseline/*/sim2sim.json`、`frontend/tests/sim2sim/{mearm-v1,so-arm101}-baseline.test.ts`、`frontend/tests/helpers/mearmV1Baseline.ts`、`frontend/tests/acceptance/{fk-three-alignment,ik-fk-roundtrip}.test.ts`、`tests/sim2sim/test_mearm_v1_baseline_mujoco.py` |
| 6 | Shared tests | `tests/sim/{conftest,harness,ikbridge}.py`（**执行框架**）、`tests/sim/test_{collision,gravity,actuator,mujoco_model,server,simulation,capture_judge}.py`（**通用机制**，当前以 MeArm 当夹具）、`frontend/tests/{acceptance/{robot-switch,mode-transport-link,transport-loop,ws-transport-loop,drag-tracking}, unit/{autoConnect,dragPlane,linkFeedback,plateUv,teachPlayback,teachTrack,wsProtocol,webSocketTransport,mockTransport}, e2e/*}` |
| 7 | 当前 Build 流程 | `frontend`: `tsc -b && vite build` → `frontend/dist`；`backend`: `go build -o bin/armpilot-backend.exe .`；无跨端统一构建入口（这是 Working Robot 要补的） |
| 8 | 当前 Frontend 流程 | `vite` dev（5273, `server.fs.allow=[frontend, repoRoot]`）+ alias `@robot`/`@config`；生产 `vite build` → `dist`（`start.bat` 跑的是 **dev**，不是 dist） |
| 9 | 当前 Backend 流程 | `armpilot-backend.exe -c config.yaml\|config.serial.yaml`；`internal/robot.LoadSelector()` 按**选择器文件所在目录的上一级**解析 `RepoRoot` ⇒ 直接依赖仓库布局 |
| 10 | 当前 MuJoCo 流程 | `simulation/mujoco/server.py --robot <id>`（`robotcfg.py` 按 `PROJECT_ROOT = PKG_DIR.parents[1]` 定位 `config/`）→ `model.py:RobotSim` 吃 MJCF → WebSocket/stdio 桥给 Go 后端 |
| 11 | 当前 FK/IK 调用链 | `robotStore` → `loadRobot(id)` → `entry.kinematics.forward/inverse` → `fk.ts` / `ik.ts`；**旁路**：`buildRobotObject3D.ts` 与 `transport/{Mock,WebSocket}Transport.ts` **直接 import `fk.ts`**（不走引擎！见 §6-C2） |
| 12 | 当前 3D 调用链 | `RobotScene` → `buildRobotObject3D(model)`（通用，消费 `RobotModel`）→ `meshRegistry`（STL 走 `import.meta.glob`）+ `textureRegistry`（照片纹理同理） |
| 13 | 当前配置加载流程 | 选择器 → 每台的 `robot.yaml`/`physics.yaml`/`mjcf`/`tcpSite`；前端用 **`?raw` 构建期静态登记**，Go/Python 用**运行期文件 IO** ⇒ 同一份文件两种读法（这是 Working Robot 必须统一的地方） |
| 14 | 所有 MeArm 耦合点 | 前端 20 文件命中 `mearm`（最多是 `MeArmKinematics.ts` 22 次，本身即包代码）；Python 7 文件（多为 docstring 对比表）；Go 13 文件（多为注释/测试夹具）。**真正的耦合只有 4 处**，见 §6 |

---

## 3. 文件归属判定表（spec §28）

口径：`CORE` = 换机器人不用改；`PKG` = 换机器人必须改；`GEN` = 产物；`TEST-C` = 通用测试框架/机制；`TEST-P` = 机器人专属测试数据。

### 3.1 前端 `src/`

| File | Old Location | New Location | Category | Reason |
|---|---|---|---|---|
| `RobotModel.ts` `Link.ts` `Joint.ts` `Actuator.ts` `Pose.ts` `RobotState.ts` `RobotCommand.ts` | `frontend/src/robot/model/` | `core/frontend/model/` | CORE | 纯数据结构；SO-101 也吃同一份（`loadRobotModel` 已参数化） |
| `loadRobotModel.ts` | 同上 | `core/frontend/model/` | CORE | 配置→模型的**唯一**解析器，已按 id 分派 |
| `robotConfigRegistry.ts` | 同上 | `core/frontend/loader/` | CORE | 选择器读取器（构建期静态登记） |
| `robotIds.ts` | 同上 | `core/frontend/model/` | CORE | id 常量；但 `MEARM_V1_ROBOT_ID` 是否该进 Core 需裁决（见 D-D） |
| `meshRegistry.ts` `textureRegistry.ts` | 同上 | `core/frontend/registry/` | CORE | `import.meta.glob` 登记器，机器人无关（新增纹理只丢文件） |
| `configError.ts` `linkFeedback.ts` | 同上 | `core/frontend/model/` | CORE | 通用错误/反馈 |
| `fk.ts` | `frontend/src/robot/kinematics/` | `core/frontend/kinematics/` | CORE | **通用链式 FK** —— 证据：`SoArm101Kinematics.forward()` 也纯委托它 |
| `transform.ts` `coordinate.ts` | 同上 | `core/frontend/kinematics/` | CORE | 纯矩阵/坐标系数学 |
| `KinematicsEngine.ts` | 同上 | `core/frontend/kinematics/` | CORE | 接口 + 能力声明（spec §10/§12 的落地） |
| `ik.ts` | 同上 | `robot-package/mearm-v1/kinematics/ik.ts` | **PKG** | MeArm 平面 2R + 偏航**解析解**，含 `assertPlanar2R`/`assertPlanarWrist` 适用性断言（spec §8：不强行统一 IK） |
| `IKResult.ts` | 同上 | **拆两半**：`core/frontend/kinematics/IKResult.ts`（类型 + `ikFailure` 等通用构造）+ `robot-package/mearm-v1/kinematics/fromMeArmIkResult.ts` | CORE + **PKG** | 类型是契约（Core）；`fromMeArmIkResult` 是 MeArm 原生形状→契约的**适配器**（spec §39 允许，但应待在包内） |
| `mearm/MeArmKinematics.ts` | `.../kinematics/mearm/` | `robot-package/mearm-v1/kinematics/engine.ts` | **PKG** | 纯委托实现，但"属于哪台"是包的事实 |
| `soarm101/SoArm101Kinematics.ts` | `.../kinematics/soarm101/` | `robot-package/so-arm101/kinematics/engine.ts` | **PKG** | 同上 |
| `registry/RobotRegistry.ts` | `frontend/src/robot/registry/` | `core/frontend/registry/RobotRegistry.ts` | CORE | 唯一分派表；改造点：手写 `KINEMATICS_FACTORIES` → `import.meta.glob` 自动发现包（见 D-D） |
| `calibration/calibration.ts` | `frontend/src/robot/calibration/` | `core/frontend/calibration/` | CORE | 通用算法；**标定值**在包的 `robot.yaml` |
| `interaction/dragPlane.ts` | `frontend/src/robot/interaction/` | `core/frontend/interaction/` | CORE | 拖拽平面数学 |
| `teach/teachTrack.ts` `teach/TeachPlayer.ts` | `frontend/src/robot/teach/` | `core/frontend/teach/` | CORE | 示教播放机制 |
| `transport/{RobotTransport,socket,timer,wsProtocol,MockTransport,WebSocketTransport}.ts` | `frontend/src/robot/transport/` | `core/frontend/transport/` | CORE | 通信基础设施 |
| `store/robotStore.ts` `store/transportBridge.ts` | `frontend/src/store/` | `core/frontend/store/` | CORE | 活动机器人是 **store 状态**（D79）；含能力门（`NO_SOLVER`） |
| `components/**`（23 文件） | `frontend/src/components/` | `core/frontend/components/` | CORE | UI 全部机器人无关（`RobotModelPanel` 只是把生效值摊开给人看） |
| `components/RobotScene/buildRobotObject3D.ts` `meshObject.ts` `plateEnvironment.ts` | 同上 | `core/frontend/scene/` | CORE | 通用建树/网格/环境反射（逐材质方案） |
| `App.tsx` `main.tsx` `styles.css` `vite-env.d.ts` `index.html` | `frontend/` | `core/frontend/` | CORE | 应用外壳 |
| `hooks/useAutoConnect.ts` | `frontend/src/hooks/` | `core/frontend/hooks/` | CORE | 自动连接（环境变量驱动） |

### 3.2 配置 / 资产 / 模型

| File | Old | New | Category | Reason |
|---|---|---|---|---|
| `config/robots.yaml` | `config/` | `core/packages/selector.yaml` **或** `working-robot/manifest.yaml` | CORE | 它是"目录表"，不是任何一台机器人的属性（**放哪 → D-C**） |
| `config/robot.yaml` | `config/` | `robot-package/mearm-v1/model/robot.yaml` | **PKG** | 运动学/标定/限位真值 |
| `config/physics.yaml` | `config/` | `robot-package/mearm-v1/physics/physics.yaml` | **PKG** | 物理量真值（MeArm 是我们估算 ⇒ 该文件即真值） |
| `config/robots/so-arm101/{robot,physics}.yaml` | `config/robots/so-arm101/` | `robot-package/so-arm101/model/` + `physics/` | **PKG** | 同上（生成物，`--check` 盯同步） |
| `config/baseline-kinematics-physics.json` | `config/` | `core/baseline/`（**保持语义哈希不变**） | CORE | 它盯的是"真值没被偷改"这个**机制**，不属任何一台机器人 |
| `simulation/mujoco/mearm.xml` | `simulation/mujoco/` | `working-robot/generated/mearm.xml` | **GEN** | 由 `gen_model.py` 产出，禁手改 |
| `assets/models/so-arm101/official/**`（20 文件） | `assets/` | `robot-package/so-arm101/model/official/**` | **PKG** | 官方模型逐字节原样（`meshdir="assets"` 必须同层级保留） |
| `assets/textures/mearm/**` | `assets/` | `robot-package/mearm-v1/assets/textures/**` | **PKG** | MeArm 照片纹理 |
| `3d-models/mearm3Dasm.STEP` | `3d-models/` | `robot-package/mearm-v1/model/source/mearm3Dasm.STEP` | **PKG** | MeArm 的源 CAD |
| `serial-v1.md` | `protocol/` | `docs/`（**实际落点**，非本表 Phase 0 原拟的 `core/protocol/`） | CORE | 通用串口协议契约 |

### 3.3 Python 仿真层

| File | Old | New | Category | Reason |
|---|---|---|---|---|
| `model.py` | `simulation/mujoco/` | `core/python/mujoco/model.py` | CORE | 文件头明确"机器人无关"；能力五件套（load/reset/step/command/state） |
| `robotcfg.py` | 同上 | `core/python/loader/robotcfg.py` | CORE | 配置读取器；⚠️ 含 MeArm 遗留路径常量 `ROBOT_YAML`/`PHYSICS_YAML`（迁移时改为纯选择器驱动） |
| `sim2sim.py` | 同上 | `core/python/sim2sim/sim2sim.py` | CORE | 统一验收框架（D77）；容差按机器人**登记**，不含具体算法 |
| `fkref.py` | 同上 | `core/python/kinematics/fkref.py` | CORE | **独立参考 FK**，机器人无关（读任一份 robot.yaml 的几何） |
| `limits.py` `units.py` | 同上 | `core/python/` | CORE | 限位换算 / 单位与关节角映射（`JointAngleMap`） |
| `server.py` `run.py` `record.py` | 同上 | `core/python/runtime/` | CORE | 服务/入口/录制 |
| `calibrate.py` | 同上 | `core/python/vision/`（或留 tools） | CORE | 相机标定，与机器人型号无关 |
| `gen_model.py` | 同上 | `robot-package/mearm-v1/tools/gen_model.py` | **PKG-TOOL** | **MeArm 专属**：把 MeArm 的几何翻译成 MJCF（SO-101 用官方 XML，不需生成） |

### 3.4 Go 后端

| File | Old | New | Category | Reason |
|---|---|---|---|---|
| `internal/robot/registry.go` `robot.go` | `backend/internal/robot/` | `core/backend/robot/` | CORE | 选择器读取器 = Core 的 Loader；**改造点**：`RepoRoot` 依赖布局（§6-C3） |
| `internal/cfg/cfg.go` | `backend/internal/cfg/` | `core/backend/cfg/` | CORE | 运行参数（端口/设备模式），无模型真值 |
| `internal/device/{device,mujoco,serial,sim,port_*}.go` | `backend/internal/device/` | `core/backend/device/` | CORE | 设备抽象（Mujoco/Serial/Sim） |
| `internal/controller/controller.go` | `backend/internal/controller/` | `core/backend/controller/` | CORE | 控制循环 |
| `internal/protocol/protocol.go` | `backend/internal/protocol/` | `core/backend/protocol/` | CORE | 串口协议 |
| `internal/wsserver/{server,ws}.go` | `backend/internal/wsserver/` | `core/backend/wsserver/` | CORE | WebSocket 基础设施 |
| `main.go` | `backend/` | `core/backend/` | CORE | 入口；**改造点**：告警按 `physics_kind` 分支（已完成） |
| `backend/config.yaml` `config.serial.yaml` | `backend/` | `core/backend/`（运行参数） | CORE | 只放端口/设备模式 |

### 3.5 测试层（spec §9 / §10 / §29）

| File | Old | New | Category | Reason |
|---|---|---|---|---|
| `tests/sim/{conftest,harness,ikbridge}.py` | `tests/sim/` | `core/tests/framework/` | TEST-C | **测试执行框架属 Core**（spec §10） |
| `frontend/tests/helpers/{fakeTimer,fakeSocket,backendModel}.ts` | `frontend/tests/helpers/` | `core/tests/framework/` | TEST-C | 测试替身 |
| `frontend/tests/tools/kinematics-bridge.mjs` | `frontend/tests/tools/` | `core/tests/framework/` | TEST-C | py↔ts 桥 |
| `tests/sim/test_{collision,gravity,actuator,mujoco_model,server,simulation,capture_judge}.py` | `tests/sim/` | `core/tests/`（**参数化**） | TEST-C | 断言的是**通用机制**（接触/重力/驱动/模型结构/服务/闭环）；当前以 MeArm 当夹具 ⇒ 迁移后应改为"按活动包参数化"，**保留原断言强度** |
| `tests/sim/test_fk.py` `test_ik.py` `test_joint_limits.py` | `tests/sim/` | 拆：机制断言→`core/tests/`；**MeArm 期望值→包内** | TEST-C + **TEST-P** | 见 §6-C4（这是最容易搬错的一组） |
| `tests/sim/test_baseline_frozen.py` | `tests/sim/` | `core/tests/` | TEST-C | 冻结**机制** |
| `tests/sim/test_texture_pipeline.py` | `tests/sim/` | `robot-package/mearm-v1/tests/` | **TEST-P** | 盯的是 MeArm 照片纹理管线 |
| `tests/sim/test_sim2sim_matrix.py` | `tests/sim/` | `core/tests/` | TEST-C | 统一矩阵（D77） |
| `tests/sim2sim/test_mearm_v1_baseline_mujoco.py` + `sim2sim/mearmV1Baseline.py` | `tests/` | `robot-package/mearm-v1/tests/` | **TEST-P** | 名字即归属 |
| `tests/baseline/mearm-v1/{fk,ik,joint,workspace}_cases.json` `sim2sim.json` | `tests/baseline/` | `robot-package/mearm-v1/tests/cases/` | **TEST-P** | spec §29 明确要求 |
| `tests/baseline/so-arm101/sim2sim.json` | 同上 | `robot-package/so-arm101/tests/cases/` | **TEST-P** | 同上 |
| `frontend/tests/unit/ik.test.ts` `calibration.test.ts` `linkGeometry.test.ts` `plate{Texture,Appearance,Uv}.test.ts` | `frontend/tests/unit/` | `robot-package/mearm-v1/tests/` | **TEST-P** | IK 解析解 / MeArm 标定 / 平行连杆几何 / **MeArm 底板照片纹理** |
| `frontend/tests/acceptance/{fk-three-alignment,ik-fk-roundtrip}.test.ts` | `frontend/tests/acceptance/` | `robot-package/mearm-v1/tests/` | **TEST-P** | 用 `mearmV1Baseline` 助手 + 断言 MeArm 具体数值 |
| `frontend/tests/sim2sim/{mearm-v1,so-arm101}-baseline.test.ts` + `helpers/mearmV1Baseline.ts` | `frontend/tests/` | 各自包 `tests/` | **TEST-P** | 名字即归属 |
| `frontend/tests/{unit,acceptance}/**`（其余 14 文件） | `frontend/tests/` | `core/tests/` | TEST-C | store/传输/示教/协议/自动连接/AI 探针 —— 通用机制（**注**：其中若干当前依赖 store 的**缺省模型**，见 §6-C1） |
| `frontend/tests/e2e/{ui-smoke,screenshot}.mjs` | `frontend/tests/e2e/` | `core/tests/e2e/` | TEST-C | 浏览器 E2E，机器人无关 |

### 3.6 工具层

| File | Old | New | Category | Reason |
|---|---|---|---|---|
| `freeze_baseline.py` | `tools/` | `core/tools/` | CORE | 冻结机制（路径常量要随布局改，**语义哈希必须不变**） |
| `run_sim2sim.py` | `tools/` | `core/tools/` | CORE | 统一矩阵 CLI |
| `gen_so_arm101_robot_yaml.py` `inspect_so101_physics.py` | `tools/` | `robot-package/so-arm101/tools/` | **PKG-TOOL** | 只服务 SO-101 的派生与审计 |
| `gen_mearm_v1_baseline.py` | `tools/` | `robot-package/mearm-v1/tools/` | **PKG-TOOL** | 生成 MeArm 自己的黄金数据（框架部分可下沉 Core） |
| `mearm_hw.py` `capture_texture.py` `make_texture.py` `measure_roi.py` `fit_pivot.py` `fit_pose.py` `analyze_sweep.py` `segment_arm.py` `project_plates.py` `cam_stability.py` `verify_calib_repro.py` | `tools/` | `robot-package/mearm-v1/tools/` | **PKG-TOOL** | 全部是 MeArm 的相机标定 / 纹理重投影 / 真机工具 |
| `verify_pose.py` `verify_serial_e2e.mjs` `ws_probe.mjs` `lan_e2e_probe.mjs` `first_load_probe.mjs` `park_sim_pose.mjs` `set_joints.mjs` | `tools/` | `core/tools/` | CORE | 通用探针（真机/仿真/首帧/泊位） |
| `start.bat` | 根 | `core/update/`（+ 根保留薄封装） | CORE | 一键拉起；`update.bat` 是它的**前置**（spec §20） |

### 3.7 新建物（当前不存在）

| 新建 | 位置 | 作用 |
|---|---|---|
| `manifest.yaml` × N | `robot-package/<id>/manifest.yaml` | 包身份 + 版本 + 能力 + 入口指针（spec §6 / §26） |
| `RobotLoader` | `core/` | 读 manifest → 组装"当前机器人"的完整视图（spec §24） |
| `RobotRuntime` | `core/` | load/init/build/validate/start/stop/state/command（spec §25） |
| `PackageValidator` | `core/` | 包结构/字段/引用/能力自洽校验（spec §18） |
| `TestRunner` | `core/` | **共享**执行器：读活动包的 `tests/` 并执行（spec §10） |
| `working-robot/` | 仓库根 | 当前机器人的运行实例（spec §11–§13） |
| `update.bat` | 仓库根 | 检测 → 构建 → 测试 → 校验 → **原子替换**（spec §20/§21） |

---

## 4. 重点问题：若直接 `cp -r` MeArm 进 Robot Package，**哪些不该复制**

这是 spec §42 点名的核心问题。答案（每条都给证据）：

| 不该复制的 | 为什么 |
|---|---|
| `robot/model/*`（RobotModel/Link/Joint/…/loadRobotModel） | 已经是**参数化**解析器；SO-101 正在用同一份。复制 ⇒ 两套模型解析，必然漂移 |
| `robot/kinematics/fk.ts` | **通用链式 FK**。反证：`SoArm101Kinematics.forward()` 就是委托它 |
| `transform.ts` `coordinate.ts` | 纯数学 |
| `kinematics/KinematicsEngine.ts` `IKResult.ts`(类型) | 接口即契约，机器人都要实现它 |
| `registry/RobotRegistry.ts` | 分派表属 Core（spec §38 的反面教材正是把它塞进包） |
| `transport/**` `teach/**` `interaction/**` `calibration/calibration.ts` | 通用机制 |
| `components/**` `store/**` `App.tsx` | UI/状态属 Core |
| `simulation/mujoco/{model,server,run,record,units,limits,fkref,sim2sim,robotcfg}.py` | 全部机器人无关；`model.py` 文件头已写明"没有一处 `if robot == …`" |
| `tests/sim/{conftest,harness,ikbridge}.py` | **测试执行框架属 Core**（spec §10 的反面教材：给每个包复制一份 TestRunner） |
| `backend/**` `docs/serial-v1.md` | 通用基础设施 |
| `tools/freeze_baseline.py` `run_sim2sim.py` + 通用探针（7 个） | 机制/探针属 Core |
| `frontend/src/components/RobotScene/buildRobotObject3D.ts` | 通用建树器；它**直接 import `fk.ts`**（这条旁路见 §6-C2） |
| **`tests/sim/test_fk.py` 这类"用 MeArm 当夹具的机制测试"** | 断言的是机制，不是 MeArm。搬进包 = 机制失去通用测试，且新机器人没人测（spec §31 禁止降低标准） |

**反过来，必须搬进包里的**（否则包不成其为包）：
`ik.ts`、`MeArmKinematics.ts`、`fromMeArmIkResult`、`robot.yaml`、`physics.yaml`、
`mearm.xml`(→generated)、`gen_model.py`、MeArm 纹理与 CAD、`tests/baseline/mearm-v1/**`、
MeArm 专属测试（`ik.test.ts` / `calibration.test.ts` / `linkGeometry.test.ts` /
`plate*.test.ts` / `fk-three-alignment` / `ik-fk-roundtrip` / `*baseline.test.ts`）、
11 个 MeArm 硬件/纹理工具。

---

## 5. 目标结构与现状的差距

| spec 要求 | 现状 | 差距 |
|---|---|---|
| §2 Core/包共享划分 | ✅ 逻辑上已分开 | 缺**物理边界** |
| §3.3 `working-robot/` | ❌ 不存在 | 全新建 |
| §6 `manifest.yaml` | ⚠️ 半个：`config/robots.yaml` 只有指针 | 缺版本/能力/入口/树形 | 
| §7/§8 FK/IK 归包 | ✅ `mearm/` `soarm101/` 已分包 | 物理位置仍在 `frontend/src/` 内 |
| §9 机器人测试归包 | ⚠️ 数据在 `tests/baseline/<id>/`（已分） | 测试**代码**未分；框架与专属混在 `tests/sim/` |
| §10 共享 TestRunner | ❌ 现在是 pytest+vitest 硬绑仓库布局 | 需抽象执行器 |
| §16 Backend 只认 Working Robot | ❌ `RepoRoot` 按选择器位置推导 | 需改造路径解析（**双读期**：兼容旧布局） |
| §17 Frontend 从 `working-robot/dist` 服务 | ❌ `start.bat` 跑 dev (5273) | 需决定"dev 还是 dist"（见 D-E） |
| §18/§19 hash 检测 | ❌ 无 | 全新建 |
| §20/§21 `update.bat` + 原子替换 | ❌ 无（`start.bat` 只做端口预检+拉起） | 全新建 |
| §23/§24/§25 Registry/Loader/Runtime | ⚠️ Registry ✅，Loader 半个（`loadRobotModel`），Runtime ❌ | 补 Loader/Runtime |
| §26 Capability 归包 | ⚠️ 在引擎里（`KinematicsCapability`）而非 manifest | 提到 manifest + 引擎读它 |

---

## 6. 冲突登记（与"MeArm 冻结优先"正面相关的四条）

### C1 · 「缺省模型」这个开关同时是三端所有"未显式指定"的缺省
**事实**：本轮实测 —— 把 `config/robots.yaml → default` 改成 `so-arm101`，`vitest` **37 处失败**、
`pytest` **13 处失败**。根因：① 前端 `robotStore` 的模块初值 `loadRobotModel(defaultRobotId())`；
② Python 测试不传 `--robot`。
**与 spec 的冲突**：spec §37 说"切换只改 `robot: model: xxx`，然后 Working Robot Update→Build→Test→Runtime"。
但**当前那个开关一改，测试观测对象就变**，于是"切换后测试结果不同"无法区分
"新机器人测试失败"与"测试测错了对象"。
**→ 处置**：见决策 **D-C**（把"当前机器人"搬到 `working-robot/manifest.yaml`，让 `default` 降级）。

### C2 · 存在一条**绕过引擎**的 FK 旁路
**事实**：`buildRobotObject3D.ts`（渲染）与 `MockTransport`/`WebSocketTransport`（反馈）
**直接 `import fk.ts`**，不经过 `KinematicsEngine`。
**含义**：如果哪天某个包提供"自己的 FK"（spec §7 允许 `fk.entry` 指向包内实现），
这条旁路会用**通用 FK**算，而引擎用**包的 FK**算 ⇒ 渲染与判据分家，且**不会报错**。
**→ 处置**：迁移时把这条旁路收进包作用域，或明确写死"FK 一律通用、包只提供几何"。

### C3 · Go/Python 的路径解析绑定仓库布局
**事实**：`registry.go` 的 `RepoRoot = 选择器目录的上一级`；`robotcfg.py` 的
`PROJECT_ROOT = PKG_DIR.parents[1]`。两者都假设"选择器在 `config/`，代码在固定深度"。
**与 spec §16 的冲突**：spec 要求 Backend 只认 `working-robot/`。
**→ 处置**：引入"路径锚点"参数（`--robot-root`），迁移期**双读**（先找 `working-robot/`，
回退旧布局），并加一条测试证明两种布局解析到同一批文件。

### C4 · `tests/sim/test_fk.py` / `test_ik.py` / `test_joint_limits.py` 的"机制 vs 期望值"混装
**事实**：这些文件同时包含（a）通用机制断言（链式公式自洽、限位截断行为）
与（b）MeArm 的具体期望值。
**与 spec §31 的冲突**：整份搬进包 ⇒ 机制失去通用覆盖（降低标准）；
整份留 Core ⇒ 包里没有测试数据（违反 §9）。
**→ 处置**：**按断言拆文件**，机制留 Core 并参数化，期望值随包。**不许整份搬，也不许整份留。**

---

## 7. 待裁决的决策点

| # | 决策 | 选项 | 我的建议 |
|---|---|---|---|
| **D-A** | 三分结构是"物理搬迁"还是"逻辑声明" | **A1** 真搬（仓库根出现 `core/` `robot-package/` `working-robot/`）/ **A2** 只建声明层，源码原地 | **A1**，但**分步**：一次一个子系统，每步跑冻结校验；理由是 A2 无法满足 §16/§17（Backend/Frontend 只认 working-robot） |
| **D-B** | `working-robot/` 是否入库（git） | 入库 / **gitignore** | **gitignore**：它是构建产物，等价于 `dist`；入库会制造第二份真值 + 每次切换产生巨大 diff |
| **D-C** | "当前机器人"的开关放哪（见 §6-C1） | ① 仍用 `config/robots.yaml → default`（三端共用）/ ② 提升为 `working-robot/manifest.yaml → id`，`default` 降级为"裸运行缺省" | **②**：这样"切机器人"不再静默改变测试观测对象；`default` 只服务"没有 working-robot 时"的测试/工具 |
| **D-D** | 包内前端代码物理位置 + Registry 发现方式 | ① `robot-package/<id>/frontend/kinematics/*.ts` + Registry 改 `import.meta.glob` **自动发现** / ② 留在 `frontend/src/robot/kinematics/<id>/`，manifest 用 `entry:` 指针 | **①**：它同时消掉 Registry 里手写的工厂表（现在加一台机器人要改两处代码 + 一处配置，改成 glob 后"只丢一个包"），正是 spec §41 想要的 |
| **D-E** | Frontend 运行方式 | ① 保持 `vite dev`（现状，热更新、LAN 可用）/ ② 改为服务 `working-robot/dist`（spec §17 字面） | **①+②并存**：`update.bat` 负责 build 到 `working-robot/dist`；**开发用 dev，交付用 dist**。否则开发期每次改一行都要等构建 |

---

## 8. 迁移前基线（已实测冻结 · spec §30）

> 采集时间 2026-09-14 13:5x，工作区纯净（`git status` 空，`default: mearm-v1`）。
> **迁移后要求 `Before == After`，不是"大概能跑"。**

| 项 | 实测值 |
|---|---|
| `tsc -b --force` | **0 error** |
| `vitest run` | **409 passed / 29 文件** |
| `vite build` | OK；`dist` 18 文件（13 个 STL）23M；`.js` 产物 `__armPilot` **0 命中** |
| `go build ./...` / `go test ./...` | 0 问题 / 全 ok（**75 个 Test 函数**） |
| `pytest tests/sim tests/sim2sim -q` | **170 passed**（161 + 9） |
| `node tests/e2e/ui-smoke.mjs` | **84/84 PASS**（隔离端口 8091/5276 实测；需按配反复跑） |
| `freeze_baseline.py` | ✓ 与冻结基线一致 |
| `gen_mearm_v1_baseline.py --check` | ✓ 4 份逐位一致 |
| `gen_so_arm101_robot_yaml.py --check` | ✓ |
| `inspect_so101_physics.py --check` | ✓ 46 项吻合 |

**本阶段没有产生任何代码改动**：`git status` 仅剩本文档（未跟踪）。

---

## 9. 建议的 Phase 1 起手（待确认后执行）

若 D-A 选 **A1**、D-D 选 **①**，则 Phase 1 是**纯增量、零搬迁**，可立即做：

1. `robot-package/mearm-v1/manifest.yaml` + `robot-package/so-arm101/manifest.yaml`
   —— 先只**声明**（字段覆盖现状能提供的全部：id/name/version/format/dof/actuator_count/
   model/kinematics/capabilities/tests 路径），**指向现有文件**，不搬任何文件。
2. `core/` 下新增 `PackageValidator` + 一致性测试：证明
   **manifest 声明的路径全部存在**、**manifest 与 `config/robots.yaml` 一一对应**、
   **能力声明与引擎的 `capability` 逐字段一致**（这条会立刻抓出"配置说支持、代码没实现"）。
3. 上述两步**不改任何现有文件** ⇒ 冻结基线天然不受影响，`Before == After` 自动成立。

Phase 2（真搬迁）在此之前不启动。
