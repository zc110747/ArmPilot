# Robot Package 重构 · Phase 1 完成记录

> 对应 spec §43（建立 Core 边界）。前一份是 `robot-package-phase0.md`（只读分析）。
> 记录格式遵循 spec §56：**修改 → Build → Test → 验证 → 记录**。

---

## 0. 一句话

> **Phase 1 是纯增量：新增 8 个文件 / 2 个 manifest，未修改任何现有文件。**
> Core 边界（`PackageValidator` + `RobotLoader` + manifest 契约 + 内容哈希）已经立起来，
> 且**立刻抓到了一处真实的跨端缺陷**（见 §4）。

`git status` 证据（MeArm-3D 下）：

```text
 M MeArm-3D/.workbuddy/memory/2026-09-14.md     ← 记忆（本阶段之前的回退轮写的）
?? MeArm-3D/core/                               ← 新增
?? MeArm-3D/robot-package/                      ← 新增
?? MeArm-3D/docs/architecture/robot-package-phase0.md
?? MeArm-3D/frontend/tests/unit/robotManifest.test.ts
```

**没有一行现有代码被改** ⇒ MeArm-V1 冻结基线在结构上不可能被本阶段影响
（这比"跑完测试恰好没红"是更强的证据）。

---

## 1. 新增物

| 文件 | 作用 | 为什么需要它 |
|---|---|---|
| `core/README.md` | Core 边界声明（什么进 Core、什么禁止进）+ 能力语义表 | spec §2/§3 的判据必须**写在文件里**，否则下一轮又会长出特判 |
| `robot-package/mearm-v1/manifest.yaml` | 包身份 + 能力 + 指针 | spec §6 |
| `robot-package/so-arm101/manifest.yaml` | 同上 | spec §6 |
| `core/python/robopkg/root.py` | **唯一**一处算 `PROJECT_ROOT` + `load_robotcfg()` 临时桥 | 现状已有两处各自推算仓库根（`robotcfg.py` / `registry.go`），Phase 2 每搬一次就要同步改两处 ⇒ 收敛成一处 |
| `core/python/robopkg/errors.py` | `PackageError` | 与 `ConfigError` 分开："配置错了"与"封装错了"是不同层的事实 |
| `core/python/robopkg/manifest.py` | manifest 加载 + **严格**校验（含未知字段拒绝） | 见 §3「未知字段检查不是洁癖」 |
| `core/python/robopkg/loader.py` | `RobotLoader`：id → 推导后的完整视图 | spec §24 |
| `core/python/robopkg/content_hash.py` | 内容哈希（哈希源、排除产物） | spec §19；陈旧检测的全部价值建立在此 |
| `core/python/robopkg/validator.py` | `PackageValidator`：**四份声明互相对账** | spec §18 |
| `core/python/robopkg/cli.py` | `list / show / validate / hash / selftest` | `update.bat`（Phase 5）与人共用的入口 |
| `core/tests/test_package_contract.py` | 25 项契约测试 | 见 §3「反面测试」 |
| `frontend/tests/unit/robotManifest.test.ts` | 16 项**跨端**一致性测试 | Python 读不到 TS 的引擎 ⇒ 有一条跨端缝，见 §1.1 |

### 1.1 为什么必须有 TS 侧那一半

manifest 的能力声明由 Python 校验，但**能力声明的另一半在 TS**：
`KinematicsEngine.capability`（`solverKind` / `supportsOrientation` / `positioningDof`）。

```text
manifest.yaml  capabilities.ik: true        ← 声明
KinematicsEngine.capability.solverKind: 'none' ← 实现
```

两边各自"自洽"，合起来是矛盾的：上层拿 manifest 判断能力、拿引擎去求解，
就会去调一个不存在的求解器。Python 读不到 TS、TS 读不到 Python ⇒ 必然有一条缝。
**TS 测试就是那条缝的守卫。**

---

## 2. 与 spec §43 的对照

| spec 要求 | 状态 | 说明 |
|---|---|---|
| `Robot`（接口） | ⏳ Phase 2 | 实体现存为 `RobotDefinition`（等价物，已在用） |
| `RobotRegistry` | ✅ 已存在 | 前端 `robot/registry/RobotRegistry.ts`（P8 已落地） |
| `RobotLoader` | ✅ **本阶段新增** | `core/python/robopkg/loader.py` |
| `RobotRuntime` | ⏳ Phase 3 | 要有 Working Robot 才有意义（它管 load/start/stop） |
| `KinematicsEngine` | ✅ 已存在 | 前端 `kinematics/KinematicsEngine.ts` |
| `PackageValidator` | ✅ **本阶段新增** | `core/python/robopkg/validator.py` |
| `TestRunner` | ⏳ Phase 3 | 同上：它要跑的是 Working Robot 的包内测试 |
| 不重写 MeArm | ✅ | 零改动（见 §0） |

> ⚠️ **`RobotRuntime` 与 `TestRunner` 有意留到 Phase 3**。它们的作用对象是
> "当前运行实例"（`working-robot/`），现在还没有那个东西。提前写只能写成一个
> 凭空猜测的抽象 —— 而 spec §43 自己也说"不要重写 MeArm"，
> 同理也不该先写一个没有使用者的运行时。

---

## 3. 实测：`Before == After`

| 项 | 迁移前（Phase 0 冻结） | 现在 | 判定 |
|---|---|---|---|
| `tsc -b --force` | 0 error | **0 error** | ✅ |
| `vitest run` | 409 passed / 29 文件 | **425 passed / 30 文件**（+16） | ✅ 原有 409 一项未少 |
| `vite build` | OK，`__armPilot` 0 命中 | **OK，`__armPilot` 0 命中** | ✅ |
| `go build` / `go vet` / `go test` | 0 / 0 / 75 Test 全 ok | **0 / 0 / 全 ok** | ✅ |
| `pytest tests/sim tests/sim2sim` | 170 passed | **170 passed** | ✅ |
| `pytest core/tests` | （不存在） | **25 passed** | ✅ 新增 |
| `freeze_baseline.py` | ✓ 与冻结基线一致 | **✓ 与冻结基线一致** | ✅ |
| `gen_mearm_v1_baseline.py --check` | ✓ 4 份逐位一致 | **✓ 4 份逐位一致** | ✅ |
| `gen_so_arm101_robot_yaml.py --check` | ✓ | **✓** | ✅ |
| `inspect_so101_physics.py --check` | ✓ 46 项 | **✓ 46 项** | ✅ |

复现命令（新增的只多一条）：

```bash
python core/python/robopkg/cli.py selftest
python core/python/robopkg/cli.py list
python core/python/robopkg/cli.py show mearm-v1
python core/python/robopkg/cli.py validate --all
python -m pytest core/tests -q            # 25
```

---

## 4. ★ 新发现：C5 · `role` 的**跨端语义分歧**（真实缺陷，本阶段不修）

### 事实（新契约测试顶出来的）

| 端 | `role` 的语义 | 对 SO-101（关节无 `role` 字段）的结果 |
|---|---|---|
| Python `robotcfg.py:288` | `role = j.get("role", j["id"])` ⇒ **缺省 = 关节 id** | `gripper` 关节 → `role = "gripper"` ⇒ `has_gripper = **true**` |
| TS `loadRobotModel.ts:439` | `role?: JointRole`（**封闭 5 值联合**，缺省 = 无） | → `role === undefined` ⇒ 推导 `has_gripper = **false**` |

同一份 `robot.yaml`，两端给出**相反**的结论。

### 影响（不是纸面问题）

- `jointByRole(model, 'gripper')` → `undefined`
- ⇒ `gripperJointId(model)` → **`null`**（`robotStore.ts:820`）
- ⇒ **SO-ARM101 在网页上拿不到夹爪关节**（滑杆/标签/示教里凡是按 role 定位夹爪的地方都会静默失效）

MeArm 不受影响（它的每个关节都显式写了 `role`，且 `role === id`）。

### 为什么不顺手修

修法只有两条，都要动**公共类型/公共语义**：

| 方案 | 内容 | 代价 |
|---|---|---|
| R1 | 前端对齐 Python：`role = raw.role ?? id`，并把 `JointRole` 从封闭联合放宽为 `string` | 动公共类型；`JOINT_ROLES` 校验也要放宽（否则 SO-101 直接抛错） |
| R2 | Python 对齐前端：不默认 `role = id` | 则 SO-101 的 `has_gripper` 变 false，与事实不符；且 `gripperJointId` 依然是 null（缺陷没修，只是两端一起错） |

⇒ 按 spec §56「如果发现架构设计存在冲突：优先保护 MeArm-V1 Golden Baseline，
**不要自行扩大任务范围**」，本阶段**只登记、不修改**。
契约测试先用**与 Python 同一条规则**（`role ?? id`）判事实，并在注释里写清这条分歧的来源。

**建议**：放到 Phase 6（SO-ARM101 包）一起做，因为那时本来就要回答
"SO-101 的 6 个关节各自是什么角色"（它的腕有 2 个自由度，MeArm 的 5 值联合装不下）。

---

## 5. 有意偏离 spec 的三处（都给了理由）

| spec 原文 | 本项目做法 | 理由 |
|---|---|---|
| §6 manifest 示例含 `dof: 4` / `actuator_count: 4` | **不写**，由 `loader` 推导；期望值放测试里 | 第二条铁律"只放指针，不放数值"。抄来的数不会自己发现自己过期，推导出来的会 |
| §6/§7 示例把 MeArm 的 `fk.type` 写成 `custom` + `entry` | 写 `type: engine`（无 entry） | **证据**：`SoArm101Kinematics.forward()` 也纯委托 `fk.ts` ⇒ 它是**通用**的。若写成 custom 会出现两份 FK（渲染走通用、引擎走包内）且**不报错** |
| §26 只列 6 个能力 | 同 6 个，但**收紧了语义**：`position` = 支持位置目标（需 IK）⇒ SO-101 为 `false` | 否则 `position: true` + `ik: false` 会让上层以为"拖个目标点就能动"。SO-101 只能关节空间定位，这是**诚实声明**而不是缺陷 |

### 5.1 「未知字段检查」不是洁癖

`manifest.py` 拒绝 manifest 里的任何未知键。现实后果：
`tcpSite`（选择器的驼峰写法）误写进 manifest 时，若不做未知字段检查，
它会被当成"没给"，而 `tcp_site` 有缺省 ⇒ **SO-101 的 TCP 落到一个官方 MJCF 里不存在的
site 名上**，运行期才炸。字段名拼错是本项目最贵的一类静默错误。

### 5.2 反面测试（没有它等于没测校验器）

`test_package_contract.py` 里有一组**故意构造坏 manifest** 的测试：
未知字段 / 缺字段 / 非布尔能力 / format 不匹配 / `position: true` 但 `ik: false` /
`type: engine` 却给了 entry / 未知 id 不回退 / 选择器有而包没有。
只测"好 manifest 通过"的话，**一个永远返回 ok 的校验器也能通过**。

---

## 6. 下一步（Phase 2 真搬迁，待本阶段确认后启动）

按"一次一个子系统 + 每步跑冻结校验"的顺序（风险从低到高）：

1. **包内测试数据**（`tests/baseline/<id>/` → `robot-package/<id>/tests/cases/`）
   —— 只动数据路径，同步改 manifest，跑 `gen_*_baseline.py --check`
2. **配置真值**（`config/robot.yaml` / `config/physics.yaml` / `config/robots/so-arm101/`）
   —— 动它要同步：vite `@config` alias、`robotcfg` 的三个路径常量、Go `RepoRoot`、
   `freeze_baseline.py` 的路径、`start.bat` 前置检查。
   **判据：语义哈希逐位不变**（`freeze_baseline.py` 是护栏）
3. **Python 侧的机器人专属工具**（`gen_model.py` 等 11 个）
4. **前端包内实现**（`kinematics/mearm/` `kinematics/soarm101/` `ik.ts` + `IKResult` 拆分）
   —— 同时把 `RobotRegistry` 的手写工厂表换成 `import.meta.glob` 自动发现（决策 D-D）
5. **包内测试代码**（`tests/sim/test_fk.py` 等按**断言**拆文件：机制留 Core、期望值随包）
