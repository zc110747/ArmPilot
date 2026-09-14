# Robot Package 重构 · Phase 2（真搬迁）

> 对应 spec「ArmPilot Robot Package + Working Robot 架构级重构」的 Phase 2。
> 上承 [`robot-package-phase1.md`](robot-package-phase1.md)（Phase 1：只读分析与契约建立）。
> **本阶段结束时，`robot-package/<id>/` 成为自包含的"机器人包"**：真值、引擎、黄金数据、
> 专属工具、专属测试全部住在包里；Core 只剩跨机器人共用的机制。

---

## 0. 一句话结论

| 项 | 结果 |
|---|---|
| 步① 包内测试数据 | ✅ `robot-package/<id>/tests/cases/` |
| 步② 配置真值随包走 | ✅ `robot-package/<id>/{model,physics}/` |
| 步③ Python 包内工具 | ✅ 24 个工具就位（Core 8 / mearm-v1 14 / so-arm101 2），仓库根 `tools/` 已消失 |
| 步④ 前端包内实现 | ✅ `ik.ts` / `engine.ts` 进包；`RobotRegistry` 改 `import.meta.glob` **自动发现** |
| 步⑤ 包内测试代码 | ✅ 两条**测试通道**（pytest / vitest）+ 8 条型号构造事实随包 |
| MeArm-V1 基线 | ✅ **未变**（语义核心哈希逐位不变；黄金数据数值载荷零改动） |
| 三端验收 | ✅ pytest **204** · vitest **33 文件 / 441 例** · tsc **0 error** · vite build ✅ · go build/vet/test **0** |

---

## 1. 判据：什么随包，什么留 Core

本阶段反复用到同一条判据，它比任何分类表都好用：

> **把这条断言 / 这份数据 / 这个工具放到另一台机器人上，它还是"对的说法"吗？**

| | 例子 | 去处 |
|---|---|---|
| **成立**（只是数据不同） | 随机位形 FK 与 MuJoCo 逐位一致；限位校验的文案格式；几何↔运动学解耦；纹理比例守卫 | **Core**（`tests/`、`frontend/tests/`） |
| **不成立**（换台就是错话） | 零位竖直段 = `column+upper_arm+forearm`；`elbow` 存绝对角 + 平行四连杆耦合；被动腕派生出的 2° 反例窗口 | **包**（`robot-package/<id>/tests/`） |

「成立」的那一类**仍然是靠包的数据驱动的** —— 所以 Core 里不该出现任何型号**字面量**，
只该出现型号**id 作为选择器**。

---

## 2. 步③：工具搬迁（Core 8 / mearm-v1 14 / so-arm101 2）

```
core/tools/                       ← 真值冻结 · 统一 Sim2Sim · 真机与链路探针（跨机器人）
robot-package/mearm-v1/tools/     ← gen_model.py · gen_mearm_v1_baseline.py · 相机标定/纹理/真机操作
robot-package/so-arm101/tools/    ← gen_so_arm101_robot_yaml.py · inspect_so101_physics.py
```

### ★ 搬迁暴露的三个**真 bug**（不是搬迁引入的，是搬迁**揭穿**的）

这三个都属于同一类：**"算路径"的写法在文件搬家后会指向一个仍然存在的错误位置**，
报错出现在离原因很远的地方。

| 文件 | 原写法 | 搬到 `core/tools/` 后 | 表现 |
|---|---|---|---|
| `run_sim2sim.py` | `Path(__file__).parent.parent` | 变成 `core/` | `ModuleNotFoundError: sim2sim` |
| `freeze_baseline.py` | 同上 | 变成 `core/` | 「基线文件不存在」「真值读不到」 |
| `run_sim2sim.py --freeze` | 默认目标 `PROJECT_ROOT/tests/baseline` | 旧目录**被重新建出来** | **"冻结成功了"却什么都没冻** —— 真值目录一字节未变 |

**统一修法**：`_find_repo_root()` —— 向上找**同时含 `core/` 与 `robot-package/` 的那一层**。
它**不依赖文件所在层数**，所以下一次搬家不会再坏。

第三个的性质更恶劣（**静默假成功**），单独记一条：

> 一个"写入型"工具的**默认目标**如果硬编码，那么布局搬迁之后它会照常成功退出码 0，
> 只是把产物写到**旧位置**、把旧目录**重新建出来**。没有报错、没有提示，
> 而消费者的测试读的是新位置 ⇒ 永远读的是旧数据。
> ⇒ 写入目标必须**从数据源声明解析**（这里是 `declared_path(id, "tests.cases")`）。

### 另一半：`freeze_baseline.py` 的 API 不完整

`tests/sim/test_baseline_frozen.py`（Phase 2 步② 写的）要求的是**按 robot_id 参数化**的接口：

```python
fb.paths(robot_id)        -> (robot.yaml, physics.yaml, 冻结基线 json)   # 全部来自 manifest
fb._rel_names(robot_id)   -> (robot_rel, physics_rel)                    # 基线 files 的键
fb.snapshot(robot_id) / fb.check(robot_id) / fb._read_baseline(robot_id)
```

而工具侧只落了"改快照的键"这一半 —— **键已指向包内路径，读的还是 `config/`**。
这正是本项目最忌讳的**路径二义**：两份"真相"各自自洽，合起来矛盾，而"键"看上去完全正确。
现已按测试文件的规格补齐，并加了一条防呆：基线里记的 `robot_id` 与请求的不一致时**显式报错**，
而不是拿 A 的哈希去判 B（那会报成"真值被改了"这样的假故障）。

### 一个反面教训：迁移文档里的**陈旧命令**

`README.md` / `assets/textures/mearm/README.md` / 各工具 `usage` 里大量写着
`python tools/xxx.py`。这类字符串**不会被任何测试抓住**，但它会把人送到一个不存在的路径。
⇒ 本轮做了一次**有白名单、有历史词豁免**的对齐（见 §5 的纪律）。

---

## 3. 步④：前端包内实现

```
robot-package/mearm-v1/kinematics/{engine.ts, ik.ts, fromMeArmIkResult.ts}
robot-package/so-arm101/kinematics/engine.ts
```

- `RobotRegistry` 从**手写工厂表**改为 `import.meta.glob('../../../../robot-package/*/kinematics/engine.ts')`
  **自动发现**。两条刻意保留的响亮失败：包 id 重复 ⇒ 抛错；命中文件但没有约定导出
  `createKinematicsEngine` ⇒ 抛错（"自动发现"绝不能退化成"发现不到就算了"）。
- `fromMeArmIkResult` 拆进包：Core 的 `IKResult` 只留**统一形状**，不再认识 MeArm 的字段名。
- `Core 里不得出现任何型号名` 由新的护栏 `frontend/tests/unit/corePackageBoundary.test.ts` 盯住：
  它列出**全部**"Core → 包"的 import，白名单外一律失败，并对白名单做逐符号核对。

### 登记的层次倒置（**刻意不修**）

`frontend/src/store/robotStore.ts` 直接 import 了 MeArm 解析解的 `solveIk` 与四个原生类型。
**本轮不动**：它暴露的是原生诊断（`candidates` / `azimuth` / `relativeAngle` / `joint`），
而统一 `IKResult` 刻意不带这些字段 ⇒ 改成走引擎会**减少界面信息**，属于行为变更，
与「MeArm 冻结优先 / 抽象前后逐位一致」冲突。按 spec 要求**报告冲突而不是自行扩大范围**。

处置路径（Phase 6/7）：给 `IKResult` 加可选 `diagnostics` 袋子，包内适配器填充，Core 只透传。
在那之前，`corePackageBoundary.test.ts` 的白名单就是它的看门人（白名单腐烂也会被它发现）。

---

## 4. 步⑤：包内测试代码

### 4.1 两条通道（本步的使能件）

没有通道，"期望值随包"就无处可去。两条通道各自配了一条**守卫**，
因为「测试存在」与「测试被执行」是两件事：

| 通道 | 机制 | 守卫 |
|---|---|---|
| Python | 仓库根 `pytest.ini` 的 `testpaths` 含 `robot-package` | `test_package_contract.py::test_pytest_testpaths_covers_package_dirs` |
| 前端 | `vite.config.ts` 的 `test.include` 含 `../robot-package/*/tests/**/*.test.ts` | `packageTestChannel.test.ts` |

两条守卫都做过**反面验证**（临时关掉通道 ⇒ 守卫必须变红并点名），确保它们不是"永远绿"。

> ⚠️ 刻意**不**给 pytest 开 `--import-mode=importlib`：现有测试大量依赖
> `from conftest import ...` / `from harness import ...` 这类裸模块名导入，
> 它们成立的唯一原因是 prepend 模式会把测试文件所在目录塞进 `sys.path`。

### 4.2 随包走的 8 条型号构造事实

| Python（`tests/test_mearm_v1_structure.py`） | 前端（`tests/mearmV1Structure.test.ts`） |
|---|---|
| 零位竖直段 = column+upper_arm+forearm | 同（用**前端真实 FK**） |
| 夹爪开合不影响 TCP（被动腕） | 同 |
| `elbow.min > shoulder.max` ⇒ 合法域是斜的 | 同 |
| 耦合重灾区批量对比 | — |
| 被动腕派生的 2° 反例窗口 + 配对拒绝 | — |
| — | `tcp.joint === 'tool'`；`tool` 是被动关节 |

两侧用的是**完全不同的实现**（MuJoCo + `fkref.py` vs 前端 `engine.ts`），
所以同时绿才是"两个独立裁判都同意" ⇒ 刻意**不**删成"只留一份"。

### 4.3 顺手修掉的**重复真值**

搬迁时发现三类"第二份真值"，已改为从真值派生：

| 位置 | 原写法 | 风险 |
|---|---|---|
| `test_server.py` 线序 `ORDER = ("base","shoulder","elbow","gripper")` | 抄了一遍 `JointOrder` | 改关节序 ⇒ 报成"解析错位" |
| `test_server.py` / `test_joint_limits.py` 的 `"ERR JOINT elbow 90.00 (limit 108.44..141.86)"` | 抄了一遍限位 | 重新标定 ⇒ 报成"文案不对" |
| `sim2sim.json` 的 `"generator"` | 抄了一遍工具路径 | 搬迁后无人核对 |

判据写法随之升级为 `re.fullmatch(...)` / `f"{:.2f}"` —— **数值从真值派生，格式契约仍被钉死**
（只比数值的话，把 `%.2f` 改成 `%g` 就没人拦得住了）。

---

## 5. 执行纪律（本轮踩出来的）

1. **同文件连续/并行编辑会丢写入**。`manifest.yaml` 的两处 `generated_by`、
   `__init__.py` 的导入块、`freeze_baseline.py` 的 API 都实测丢过 ⇒
   **改完立刻用独立命令核对落盘**（`grep -n` / `sed -n`），不要相信"编辑成功"的回执。
2. **批量改写文本文件必须保留行尾**。`Path.write_text()` 在 Windows 上会把 `\n` 翻成 `\r\n`，
   而仓库是 **LF-only + `core.autocrlf=false`** ⇒ 整个文件都会算作改动
   （实测 README 的"62 行改动"里混进了全文）。用 `open(..., newline="")`。
3. **批量路径对齐要有白名单 + 历史词豁免**。仓库里大量"搬迁前真值住在 `config/robot.yaml`"
   是**历史描述**，一并替换会把"当时"改写成谎言。
4. **搬迁后要跑一遍"全部工具的 `--help` 冒烟"**。三个真 bug 中有两个就是这么抓到的。
5. **写入型工具的产物要验证"写到哪了"**，而不是只看退出码。
   最好的判据是 **diff 的规模**：`--freeze` 重跑后快照 diff 只有 `generator` + `frozenAt` 两行
   ⇒ 40/46 例数值逐位复现 + 写入位置正确，一次断言两件事。

---

## 6. 验收（本阶段末实测）

```bash
cd frontend
./node_modules/.bin/tsc -b --force         # 0 error
./node_modules/.bin/vitest run             # 33 文件 / 441 例（含 ../robot-package/mearm-v1/tests/）
./node_modules/.bin/vite build             # ✅（产物中 __armPilot / 旧真值路径命中 0）

$PY -m pytest -q                           # 204 passed（走 pytest.ini 的 testpaths）
$PY core/python/robopkg/cli.py validate --all          # 2 通过 / 0 问题
$PY robot-package/mearm-v1/tools/gen_mearm_v1_baseline.py --check   # 逐位一致
$PY robot-package/so-arm101/tools/gen_so_arm101_robot_yaml.py --check  # 与官方模型同步
$PY robot-package/so-arm101/tools/inspect_so101_physics.py --check     # 46 项吻合
$PY core/tools/freeze_baseline.py                      # ✓ 与冻结基线一致
$PY core/tools/run_sim2sim.py --all                    # ✓ 全部机器人 FK 在登记容差内
cd backend && go build ./... && go vet ./... && go test ./...
```

**MeArm-V1 基线未被削弱**：`gen_mearm_v1_baseline.py --check` 逐位复现；
`freeze_baseline.py` 语义核心哈希未变；`sim2sim.json` 重跑只有元信息两行差异。

---

## 7. 未做 / 留给后续

| 项 | 去处 |
|---|---|
| 资产搬迁（`assets/models/so-arm101/official/**`、`assets/textures/mearm/**`、`3d-models/*.STEP`） | Phase 3+（体积大，风险独立） |
| `robotStore` → 包内 `ik.ts` 层次倒置 | Phase 6/7（`IKResult.diagnostics` 袋子） |
| `C5` `role` 跨端语义分歧（`role ?? id` vs 封闭 5 值联合） | Phase 6（要动 `JointRole` 公共类型） |
| `C6` `project_plates.py` 的 `JOBS` 硬编码板件尺寸 | Phase 6（构成 `robot.yaml` 之外的第二份真值） |
| `test_ik.py` 剩余的 MeArm 耦合 | 目前是**"只把型号当数据键"**的形态（`robot.joint("elbow")`），可接受；完全泛化属 Phase 7/8 |
| `frontend/tests/unit/robotModel.test.ts` 断言的 MeArm 连杆名清单 | 同上：它是**加载器**的测试，MeArm 只是样本 |
| `docs/architecture/mearm-v1-*.md` 里的旧路径 | **刻意保留**：那几份是"当时"的只读分析，改写会篡改历史结论 |
| `cam_stability.py` 不接受 `--help`（把 `argv[1]` 当数字） | 一次性工具，不改 |
