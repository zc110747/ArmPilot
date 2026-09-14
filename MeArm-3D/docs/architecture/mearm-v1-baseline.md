# MeArm-V1 基线冻结 · 验收结论

> **本文档是本阶段的结论记录**：冻结了什么、抽象了什么、判据是什么、实测多少。
> 动手前的现状审查见 [`mearm-v1-baseline-analysis.md`](./mearm-v1-baseline-analysis.md)；
> 与抽象无关的问题登记在 [`mearm-v1-followups.md`](./mearm-v1-followups.md)。
>
> - 模型标识：**`MeArm-V1`** / `version: 1.0.0`（`config/robot.yaml`）
> - 冻结判据：`config/baseline-kinematics-physics.json` + `tests/sim/test_baseline_frozen.py`
> - 黄金数据：`tests/baseline/mearm-v1/*.json`（seed `20260914`）
> - 完成日期：2026-09-14

---

## 1. 交付摘要

**一句话**：MeArm 现有实现本来就已经是"一个 Robot Model + 一套运动学"。
本阶段没有新建架构，只补了三样东西 —— **版本标识**、**可复现的黄金数据**、**一层薄接口** ——
并用 Sim2Sim 回归证明"抽象前后行为逐位一致"。

| # | 交付物 | 位置 |
|---|---|---|
| ① | 模型版本标识（只读元数据，不进冻结白名单） | `config/robot.yaml` · `robotcfg.py` · `RobotModel.ts` |
| ② | 黄金测试数据集 + 采集器 | `tests/baseline/mearm-v1/*.json` · `tools/gen_mearm_v1_baseline.py` |
| ③ | 最小抽象层（4 个文件，纯委托零算法） | `frontend/src/robot/{definition,kinematics}/` |
| ④ | Sim2Sim 回归（前端 3 文件 / Python 侧 3 文件，另改 `tests/sim/harness.py`） | `frontend/tests/sim2sim/` · `tests/sim2sim/` |
| ⑤ | 现状审查 + 验收结论 + 无关问题登记 | `docs/architecture/mearm-v1-*.md`（3 份） |

**没有做的事（刻意的）**：

- ❌ 没有重写 IK / FK —— 一个算法行都没动
- ❌ 没有创建 `GenericIK` —— 那是"看起来通用"的假抽象
- ❌ 没有给求解器加 5DOF / 6DOF 参数（如 `solveIK(x,y,z,r,p,y)`）
- ❌ 没有改 MuJoCo 物理参数、没有放宽任何误差阈值、没有删除任何原有测试
- ❌ 没有大规模重命名
- ❌ 没有把"真机测试"当验收条件（本阶段全部判据都在仿真域内闭合）

---

## 2. 冻结边界

### 2.1 MeArm-V1 是什么

```text
4 DOF 位置型机构
├─ base      —— 绕世界 Z 的偏航（±60°）
├─ shoulder ── 矢状面内平面 2R 的第一杆（世界 Y 轴）
├─ elbow     —— 平面 2R 的第二杆（世界 Y 轴）；★ 存的是**离开天顶的绝对倾角**
├─ tool      —— **被动腕**：type=passive + coupling{gain:-1}，被平行四连杆锁成水平
│               nq 里有它的 qpos（否则爪不会被带着转），但它**没有自由度**
└─ gripper   —— 夹爪，**不参与末端定位**（tcp.joint 取的是 tool）

自由度 4 ≠ qpos 维数 5
joint_ids（qpos 序）≠ dof_ids（状态帧序）—— 混用会得到五元组的 JR
```

判定"可动关节"的唯一规则：**`joint.type === 'revolute'`**（不是 `!== 'fixed'`），
前端 `isMovableJoint()` / Go `JointOrder()` / Python `is_dof` 三端同源。

### 2.2 哪些被钉死、哪些自由

| 层 | 判定 | 依据 |
|---|---|---|
| `links[].geometry` / `.details` / `appearance` | ✅ **自由** | 语义核心哈希不含外观（ADR D55） |
| `links[].length` · `joints[]`（轴/限位/coupling）· `actuators[]`（标定） | ❌ **冻结** | 改了报错并逐字段列出差异 |
| `physics.yaml`（gravity / contact / timestep / servo / inertia） | ❌ **冻结** | 全 SI，运动学量不得出现 |
| FK / IK **算法** | ❌ **冻结** | 本阶段一行未改 |
| Three.js 外观结果（比例 / 轴向 / 零位 / 运动方向） | ❌ **冻结** | — |
| 注释 / 文档 / **新增只读元数据字段** | ✅ **自由** | `robot.model` / `robot.version` 属此类 |

### 2.3 新增的 `model` 字段为何不触发冻结告警

`tools/freeze_baseline.py` 用**白名单**（`id` / `name` / `units` / `tcp` / `homePose`）
提取语义核心，`model` 与 `version` 不在其中 ⇒ 语义核心哈希**不变**。
整文件哈希（L1 记录）会变，属"放行"情形，跑 `--update` 刷新记录即可。

> 实测：`freeze_baseline.py` → 语义核心一致 + L1 漂移提示；`--update` 后完全干净；
> `test_baseline_frozen.py` 11 项辨识力测试全过。
> 同步守卫 `test_generated_mjcf_is_in_sync_with_config` 亦全过（MJCF 无需重生成）。

---

## 3. 最小抽象层

### 3.1 形状

```text
              现有（**一行未改**）
   ┌──────────────────────────────────────┐
   │ RobotModel   （= RobotDefinition）    │
   │ fk.ts        （= FK 算法）            │
   │ ik.ts        （= MeArm IK 算法）      │
   │ buildRobotObject3D.ts（渲染层）        │
   └──────────────────────────────────────┘
                    ▲  委托（零算法、零转换、零拷贝）
                    │
              新增（本阶段）
   ┌──────────────────────────────────────┐
   │ definition/RobotDefinition.ts         │   分节视图 + defineRobot() + isModel()
   │ kinematics/IKResult.ts                │   统一结果形状 + 适配器
   │ kinematics/KinematicsEngine.ts        │   能力声明 + 调用面接口
   │ kinematics/mearm/MeArmKinematics.ts   │   MeArm-V1 实现：纯委托
   └──────────────────────────────────────┘
```

### 3.2 三条设计约束（都是被"回归要有意义"逼出来的）

**① 零转换、零拷贝。** `RobotDefinition` 的 `links` / `joints` / `actuators` / `tcp` /
`homePose` 全是**原对象引用**，`robotModel` 也是同一个对象（`expect(geo).toBe(model)` 钉住）。
一旦某天有人在这里做深拷贝，"抽象前后一致"就不再是同一个对象的两次读取。

**② 绝不复制 `effectiveJointAngle()`。** 渲染层 `buildRobotObject3D.ts` 与 `fk.ts`
**共用同一个耦合表达式**（前者直接 import 后者）—— 这是 Phase 3「FK == Three.js」能到
`8.7e-14 mm` 的根本原因（风险 R1）。抽象层只准 re-export / 委托。

**③ `orientationError` 恒为 `null`，不是 `0`。** 本机没有姿态自由度，
填 `0` 会**同时骗过调用方和测试**（看起来"姿态误差为零"）。`positionError` 在失败时同样
是 `null` 而非 `0`。

### 3.3 唯一一处"看起来多余"的严格性

`MeArmKinematics.toIkOptions()` 对未知 `prefer` **显式抛错**。
理由：`ik.ts` 对无法识别的 `prefer` 会**静默退化**为 `elbow-up`。
适配器如果把未知值直接透传，"抽象层传错了参数"就会变成一条安静的、结果仍然"正确"的路径。

---

## 4. 黄金测试数据集

### 4.1 为什么需要它

项目既有测试大多是「**当场算两遍、互相比对**」（FK 参考实现 vs MuJoCo）。
那能证明"两套实现对得上"，**证明不了"今天的行为和上周一样"** —— 重构之后两套实现
可能一起变，判据照样全绿。黄金数据把当前行为**落盘**，补的就是这一环。

### 4.2 构成（`tests/baseline/mearm-v1/`）

| 文件 | 例数 | 载荷 | 期望值来源 |
|---|---|---|---|
| `joint_cases.json` | 116 | FK / Three.js / MuJoCo 三条判据的**公共输入** | 关节位形枚举 |
| `fk_cases.json` | 116 | Joint → FK 三侧 | `tcpFrontend`（前端 `fk.ts`）/ `tcpRef`+`framesRef`（`fkref.py`）/ `tcpMujoco` |
| `ik_cases.json` | 121 | XYZ → IK →（FK / MuJoCo） | 前端真实 `ik.ts` 经桥 |
| `workspace_cases.json` | 121 | 可达性 / 错误码 / 2R 几何 | 同上 |

用例分类（`joint_cases.json → categories`）：

```text
zero / home                 2     ← 零位与固件 RESET 位
per-joint min/mid/max      12     ← 4 关节各取限位两端与中点
boundary corners            2     ← 全最小 / 全最大
seeded random             100     ← numpy default_rng(20260914)
```

IK 用例另含**显式越界**（3 个：远超连杆总长 / 三轴同时远离 / 远低于底座）
与**方位角越界**（3 个：90° / 180°，几何可达但超出 base ±60°）——
错误码也是被冻结的行为，必须有覆盖。

### 4.3 采集原则

1. **期望值一律来自实跑，绝不手算。** 前端侧经 `kinematics-bridge.mjs`（Vite SSR 加载
   **同一份** `fk.ts` / `ik.ts` 源码）；Python 侧只递 JSON，**不解释任何运动学语义**。
2. **固定 seed（`20260914`），全流程可复现。** 允许变动的只有 `generated_at` 时间戳；
   `--check` 会逐位比对复现结果与已提交文件。
3. **随机只作补充**，主数据集是显式枚举的定点用例。
4. 落盘保留 **12 位小数**（`-0.0 → 0.0`），远严于最小容差 `1e-9`，同时 JSON 稳定、diff 友好。

---

## 5. 验收结果

### 5.1 五件套（+ 两项本阶段新增）

| # | 命令 | 结果 |
|---|---|---|
| 1 | `tsc -b --force` | ✅ **exit 0**（0 error） |
| 2 | `vitest run` | ✅ **345 passed / 26 files** |
| 3 | `vite build` | ✅ 通过；产物 JS bundle `__armPilot` **0 命中** |
| 4 | `go test ./...` | ✅ **5 包全 ok** |
| 5 | `pytest tests/sim -q` | ✅ **147 passed** |
| 6 | `pytest tests/sim2sim -q` ★新增 | ✅ **9 passed** |
| 7 | `node tests/e2e/ui-smoke.mjs`（隔离端口 5276 + 8091） | ✅ **84 / 84 PASS，0 FAIL** |

> **关于 e2e 的 84 与 88**：完整跑是 88 项。复用既有后端实例时，
> 「断线重连」子项与 opt-in 的真机项会走 `skip()` 而非 `check()` ——
> 这是**语义上的跳过**，不是回归（见 followups / 风险 R4）。
> 本次是**自起干净实例**（`device=sim`），因此断线重连子项照跑。
>
> **关于 `__armPilot` 的判据范围**：既有契约是
> `grep -o "__armPilot" dist/assets/*.js | wc -l` = 0 —— 即**发布的 JS** 干净。
> `.js.map` 里仍会出现该字样（源映射内联了源码文本与守卫注释），这不是缺陷，
> 但**判据扫描范围要写清楚**，否则下次会有人对着 `grep -r dist/` 的 1 命中发呆。

### 5.2 Sim2Sim 判据（spec §17）

**前端侧** `frontend/tests/sim2sim/mearm-v1-baseline.test.ts`（13 项）

| 判据 | 规模 | 实测 max\|Δ\| | 容差 |
|---|---|---|---|
| ① Joint → FK（`fk.ts`） | 116 | **7.176e-13 mm** | 1e-9 |
| ② Joint → FK 逐关节坐标系 | 116×5 | **7.418e-13 mm** | 1e-9 |
| ③ Joint → Three.js `matrixWorld` | 116 | **7.141e-13 mm** | 1e-6（项目契约 0.1） |
| ④ XYZ → IK → FK 闭环 | 121（115 成 / 6 败） | **4.010e-13 mm** | 1e-9 |
| ⑤ 关节角 vs 基线 | 121 | **5.862e-13 °** | 1e-9 |
| ⑥ 抽象层等价（引擎 vs 直接调用） | 237 | **差异 0**（逐位） | 0 |
| ⑦ sweep500 Joint→Three.js | 500 | **7.105e-14 mm** | 1e-6 |
| ⑧ sweep1000 XYZ→IK→FK | 1000（失败 0） | **1.148e-13 mm** | 1e-9 |

**MuJoCo 侧** `tests/sim2sim/test_mearm_v1_baseline_mujoco.py`（9 项）

| 判据 | 规模 | 实测 max\|Δ\| | 容差 |
|---|---|---|---|
| ① Joint → MuJoCo TCP | 116 | **4.974e-13 mm** | 1e-9 |
| ② 参考实现 `fkref` vs 基线 TCP / 关节坐标系 | 116 / 116×5 | **4.974e-13 mm** | 1e-9 |
| ③ Joint → MuJoCo **关节锚点**（含被动腕） | 116×5 | **5.258e-13 mm** | 1e-6 |
| ④ XYZ → IK → MuJoCo 闭环 | 121（115 / 6） | **4.027e-13 mm** | 1e-9 |
| ⑤ vs 基线 `tcpMujoco` | 115 | **1.037e-12 mm** | 1e-9 |
| ⑥ 工作空间判定 | 121（可达 115 / 越界 3 / 限位 3） | 错误码全match | — |
| ⑦ 矢状面距离换算 | 121 | **1.009e-12 mm** | 1e-9 |
| ⑧ sweep500 Joint→MuJoCo | 500 | **8.527e-14 mm** | 1e-6 |
| ⑨ sweep1000 XYZ→IK→MuJoCo | 1000（失败 0，全 `elbow-up`） | **1.180e-13 mm** | 1e-9 |

**这些数字意味着什么**：1e-13 mm = **0.1 飞米**，是双精度浮点在 10² mm 量级上的
固有噪声。任何真实的几何/旋转约定错误（intrinsic XYZ 写成 extrinsic、
耦合项漏掉、绝对角当局部角）都会把偏差推到**毫米以上**。

### 5.3 抽象层等价（本阶段专属判据）

spec §19 的字面要求是「旧代码 VS 新封装」，所以另立一条：

| 对比 | 规模 | 结果 |
|---|---|---|
| `engine.forward()` vs `endEffectorPosition()` | 116 | 逐位一致（`Object.is` 全 true） |
| `engine.inverse()` vs `solveIk()` | 121 | 成功/分支/残差/错误码全部一致 |
| `engine.definition.robotModel` vs `model` | — | **同一对象** |

覆盖 `tests/sim2sim` 9 项 + `frontend/tests/unit/kinematicsEngine.test.ts` 14 项。

---

## 6. 未决项与后续

### 6.1 明确留给下一阶段

| 项 | 为什么现在不做 |
|---|---|
| `GenericIK` / 多机种抽象 | spec 明令禁止；而且**真实需求还没出现** —— 假抽象的代价高于收益 |
| 5DOF / 6DOF 任务空间接口 | 能力声明（`KinematicsCapability`）已就位，但本机只有 3 个定位自由度；参数化位姿接口现在写一定是错的 |
| MuJoCo 侧也用同一套 `KinematicsEngine` | MuJoCo 轨走的是"独立参考实现"路线，让它去调前端接口会**破坏独立裁判机制**（见 `tests/sim/ikbridge.py` 文件头） |

### 6.2 已知但不在本阶段范围的问题

见 [`mearm-v1-followups.md`](./mearm-v1-followups.md)：
`MeArm-RemoteControl` 缺 `go.mod` · Windows 工具语义陷阱（`sort -u` / `grep \b` /
`taskkill //PID`）· 一处关节锚点辅助函数的重复实现 · `docs/serial-v1.md §5.1` 文档漂移。

### 6.3 提交划分

```text
① freeze:   establish MeArm-V1 baseline
            模型版本标识 + 黄金数据集 + 采集器 + 桥的 FK 批量求值分支

② refactor: introduce minimal robot model interfaces
            RobotDefinition / KinematicsEngine / IKResult / MeArmKinematics（+ 导出）

③ test:     add MeArm-V1 sim2sim regression
            前端 13 项 + MuJoCo 侧 9 项 + 基线文档 + followups
```

三个提交**各自可独立检出并通过对应判据**：① 之后黄金数据可复现、② 之后抽象层可用
（但尚无回归证明）、③ 之后回归闭合。这也正是"先冻结、再抽象、最后证明"的顺序意义所在。
