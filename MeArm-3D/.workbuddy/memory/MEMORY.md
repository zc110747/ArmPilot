# MeArm-3D · 项目长期记忆（索引）

> **本文件只是索引**：铁律 + 验收命令 + 去哪找细节。明细在 `playbook.md`，
> 决策理由在 `docs/decisions.md`（D1–D79），验收数据在 `README.md` §6，每日过程在 `YYYY-MM-DD.md`。
> ★ 注入阈值实测 ≈6.3k 字符，超出会被**静默截断**（2026-09-14 从 10.6k 拆成「索引 + playbook」）。

## 一、唯一真值源（铁律）

- **真值只有一份，位置由包自己声明**：`robot-package/<id>/model/robot.yaml`（运动学/标定/限位/零位）
  + `robot-package/<id>/physics/physics.yaml`（**只放物理量，全 SI**；运动学量一律从 robot.yaml 读，
  `test_config_truth_is_not_duplicated` 盯）。**禁止硬编码尺寸/角度/限位**；`backend/config.yaml` 只放运行参数。- ★★ **路径只在 manifest 声明、只由 `declared_path()` 解析**（Phase 2 建立）：
  `manifest.yaml → model.config / model.physics / simulation.mjcf / tests.cases / tests.frozen / tests.local`
  → 三端同一契约（Python `robopkg.declared_path` / TS `frontend/tests/helpers/robotPackage.ts` /
  前端 `robotConfigRegistry` 的 `import.meta.glob`）。**新增路径字段必须登记**（未登记即报错）。
  动机：搬迁时"自己拼路径的读者"漏改**不一定报错**——可能解析到一个**仍然存在的**同名文件
  （只是已属另一台机器人）。仓库根 `config/` 现在**只剩** `robots.yaml` 选择器。
- **真值已冻结**：`core/baseline/baseline-kinematics-physics.json` + `core/tools/freeze_baseline.py`
  （`--robot` 缺省 `mearm-v1`）+ `tests/sim/test_baseline_frozen.py`。判据是**语义核心哈希**（非整文件，D55）
  ⇒ 改**外观**（`links[].geometry`/`details`）**放行**；改**运动学**（`length`/轴限位耦合/`actuators`）或
  **物理量** ⇒ **报错并逐字段列差异**。有意改后：`freeze_baseline.py --update` **且**重跑
  `robot-package/mearm-v1/tools/gen_model.py`（MJCF 是产物，`test_generated_mjcf_is_in_sync_with_config` 盯）。
  ★ 冻结基线的键 = 真值文件的**仓库相对路径** ⇒ 路径一变键就变，`--update` 后**哈希必须逐位不变**。
- **黄金数据同样冻结**：`robot-package/<id>/tests/cases/*.json`（D71），`gen_mearm_v1_baseline.py`
  生成、`--check` 逐位复现；**改运动学/物理后必须重跑**，否则 Sim2Sim 回归必红。
- `mode`（simulation/real）**不是 UI 开关**：决定"要不要发给真实机械臂"，取值须校验全通过才改，
  失败要 pushLog 说明原因与修法（D41/D43）。
- **机器人相关状态一律进 store**，组件不持局部副本（含 `teachTrack`）。
- **整合版标准 URDF**：`robot-package/<id>/urdf/<id>.urdf` 由 `tools/gen_urdf.py` 从
  `robot.yaml`+`physics.yaml` 派生（含 `<armpilot>` 扩展块，标准解析器忽略未知顶层元素 ⇒ 整份仍合法）。
  MJCF/URDF 皆为生成物；`manifest.model.urdf` 声明位置、`manifest.model.generated_by` 指向生成器。
  ★ **`content_hash` 语义陷阱**：`model.generated_by` 描述「生成 **model.urdf** 的生成器」，**不是** model.config
  （robot.yaml 永远是真值、永远进哈希）⇒ `compute_content_hash` 里 `model.config` 必须 `is_generated=False`。
- ★★ **读复杂标准格式一律用参考实现，禁止自研解析器**（2026-09-15，明细 `playbook.md` §10）：
  自研正则解析器读 `mearm3Dasm.STEP` 误判"只有层级没有几何、0/241 可达"并写成 `BLOCKED`；
  用户指出"不要自己写解析器，用完整开源项目"——**用户对**。换 OCCT（`cadquery-ocp`）⇒ **111/111 全有几何**。
  **铁律**：自研工具给出**否定性结论**（"文件坏了"/"数据缺"）时**必须先用参考实现复核**，
  否则会把**工具的能力边界**误报成**数据的缺陷**。工具 `core/tools/step_report.py`（OCCT，唯一保留）。
- ★ **STEP = 单姿态快照：可作外观源，禁止作运动学真值源**（2026-09-15，明细 `playbook.md` §10）：
  4 个 SG90 输出轴（R=6.20）= 关节轴，轴向全与 robot.yaml 吻合 ⇒ **无结构冲突**；
  ★★ **肩轴 ∥ 肘轴（0.00°、间距 107.39）**= 平行四连杆签名 ⇒ `elbow.coupling.gain=-1` 从照片拟合
  升级为 **CAD 结构实证**。⚠️ **`length` 不可由 STEP 反推**（已证 STEP 姿态 ≠ 零点位）⇒ 唯一路径仍是**标尺实拍**。
- **CAD→URDF 那条路已放弃**（2026-09-15）：`local_mu28fwc1_g8139u_urdf_stl/robot.urdf` 只有
  1 link / 0 joint / 111 件未指派 + mm 尺寸配 m 原点的 1000× 单位错。- ★ **包边界 + 测试通道，各配一条「能红」的守卫**（Phase 2 步④⑤，明细 `playbook.md` §8）：包内测试必须
  **真的被执行** —— Python 靠 `pytest.ini` `testpaths`（含 `robot-package`）、前端靠 `vite.config.ts`
  `test.include`（`'../robot-package/*/tests/**'`），守卫 `packageTestChannel.test.ts`；运动学引擎由
  `import.meta.glob('robot-package/*/kinematics/engine.ts')` **自动发现**。
  ⚠️ **更正实际覆盖面**（2026-09-14 核实）："Core 不得出现型号名/型号路径"是**纪律**；机器判据只有
  `frontend/tests/unit/corePackageBoundary.test.ts`，它扫 **`frontend/src/**` 的 `import` 说明符**，
  **不覆盖** `core/**`、也不看路径字符串 ⇒ `core/tools/*.mjs` 里写死型号名**不会报错**。
- **多机器人轨（2026-09-14，P0–P8 已完成）**：`config/robots.yaml` 选择器（**只放指针**）→ 三端各自解析
  **同一份** → `RobotRegistry`（唯一分派表，禁止 `if robot == ...`）；第二台 = 官方 SO-ARM101，
  资产**逐字节原样、禁止改**；统一验收只有 `run_sim2sim.py --all`。明细见 `playbook.md` §7。

## 二、验收七件套

**完整命令见 `playbook.md` §11**（前端三件：tsc / vitest / vite build / ui-smoke；
Python 六件：pytest / run_sim2sim --all / 两条 `--check` / `validate --all`）。
实测基线（2026-09-15）：`pytest -q` **196 passed** · `test_baseline_frozen.py` **11 passed** · `validate --all` 通过。

⚠️ **`pytest` 只在仓库根成立**（2026-09-14 实测）：从 `frontend/` 跑 `pytest -q` 会输出
**`no tests collected` 且 exit 0** —— 只看退出码的脚本会把它读成「通过」。跑验收前先确认 **cwd = 仓库根**；
`pytest.ini` 的 `testpaths` 相对 **rootdir** 解析，**不要**给 pytest 传目录参数（传了就绕过 testpaths）。
**前端那套覆盖不到** MJCF / 纹理管线 / 文档↔脚本一致性 / 黄金基线 ⇒ 都在 pytest 里。
**造前提**：`node core/tools/park_sim_pose.mjs <wsUrl> --joints '{...}'`（文件内**不含任何限位/角度常量**）。
⚠️ **限位端点不可精确到达**（舵机整数度量化后可能落到限位外被后端拒绝 `ERR JOINT …`）⇒ 取**限额内部**的值。

- 环境硬约束（明细 `playbook.md` §5）：`npx <tool>` 触发 WSL 黑名单 ⇒ 一律 `./node_modules/.bin/<tool>`；
  ★ **裸名 `bash` 也会触发**（PATH 首项 `/tmp/system32/bash`）⇒ 用 **`/usr/bin/bash x.sh`**；
  `(cmd &)` 后台进程只活到本次工具调用结束 ⇒ 起服务与跑 e2e 必须**在同一次调用里**；
  `/tmp/*.log` 重定向被沙箱拦 ⇒ 日志落 `.workbuddy/captures/`；e2e **必须隔离端口**（5276 + 8091）；
  Python 用 `~/.workbuddy/binaries/python/envs/default/Scripts/python.exe` + `PYTHONIOENCODING=utf-8`；
  跑 `.bat`/`cmd` 只能靠 Python `subprocess` 且 **stdout/stderr 必须重定向到文件**（`capture_output=True` 会挂起）。
  ★★ **绝不要用 `cat >> 文件 << EOF` 改记忆文件**（2026-09-15 实测：重定向**从头截断**而非追加，
  一次就毁掉了 playbook §1–§3）⇒ 用 Edit/Write 工具或 Python `io.open(...,'w')`。
- ★ **真机验收有一条容易自欺的口径**：链路层全绿（`hello.device=serial`/`OK JR`/`joint_state`）只证明
  "指令送到固件并被接受"，**不证明物理到位**（真机无位置反馈，`joint_state` 是**开环目标值**）。
  物理证据只有相机（`verify_pose.py`），且要先按 Phase 4.5 把台面（白分割板 + 锁曝光）就位。详见 `playbook.md` §9。
- ★ **`start.bat` 已不再"只负责起窗口"**：`[0/3]` 陈旧闸门（源码比 exe 新就 `go build`，无 `go` 就拒绝启动）
  + 启动后 `/healthz` 探活。别绕过它去直接起旧二进制。详见 `playbook.md` §9。

## 三、细节入口（按需读）

| 主题 | 去处 |
|---|---|
| §1 测试/探针纪律（D44/D65/D67） · §2 机制与关节判定（轴/nq=5/绝对角/斜合法域） | `playbook.md` §1 · §2 |
| §3 外观·纹理轨（D56–D69） · §4 测量方法论与能力边界（D34/D47/D59/D64） | `playbook.md` §3 · §4 |
| §5 运行环境与 Windows 陷阱 · §6 协作约定与真机链路（COM 号会变/DTR/`--home`） | `playbook.md` §5 · §6 |
| §7 多机器人轨（选择器/注册表/mesh/SO-101 陷阱） · §8 Robot Package 重构边界与搬迁纪律 | `playbook.md` §7 · §8 · `docs/architecture/robot-package-phase*.md` |
| §9 `start.bat`（陈旧闸门/探活/块内括号坑）+ 真机验收两条证据链 | `playbook.md` §9 |
| §10 CAD/STEP 数据接入（OCCT 铁律/关节轴/平行四连杆实证/外观边界） · §11 验收七件套全文 | `playbook.md` §10 · §11 |
| **STEP 校验三件套（含一次误判复盘）** | `docs/STEP_KINEMATICS_VALIDATION.md` · `STEP_ASSEMBLY_ANALYSIS.md` · `KINEMATIC_CHANGE_PROPOSAL.md` |
| 冻结与基线决策理由 | `docs/decisions.md` D55 / D71–D73 |
| 多机器人统一验收 / 运行期切换（D77–D79） | ADR D77 / D78 / D79 |
| 首帧"重影" / 幽灵臂渲染（D74） | ADR **D74** · `core/tools/park_sim_pose.mjs` · skill `webgl-first-frame-forensics` |
| 未修的无关问题 F1–F9 | `docs/architecture/mearm-v1-followups.md` |
