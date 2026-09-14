# MeArm-3D · 项目长期记忆（索引）

> **本文件只是索引**：铁律 + 验收命令 + 去哪找细节。明细在 `playbook.md`，
> 决策理由在 `docs/decisions.md`（D1–D79），验收数据在 `README.md` §6，每日过程在 `YYYY-MM-DD.md`。
> ★ 注入阈值实测 ≈6.3k 字符，超出会被**静默截断**（2026-09-14 从 10.6k 拆成「索引 + playbook」）。

## 一、唯一真值源（铁律）

- **真值只有一份，位置由包自己声明**：`robot-package/<id>/model/robot.yaml`（运动学/标定/限位/零位）
  + `robot-package/<id>/physics/physics.yaml`（**只放物理量，全 SI**；运动学量一律从 robot.yaml 读，
  `test_config_truth_is_not_duplicated` 盯）。
  **禁止硬编码尺寸/角度/限位**；`backend/config.yaml` 只放运行参数。
- ★★ **路径只在 manifest 声明、只由 `declared_path()` 解析**（Phase 2 建立）：
  `manifest.yaml → model.config / model.physics / simulation.mjcf / tests.cases / tests.frozen / tests.local`
  → 三端同一契约（Python `robopkg.declared_path` / TS `frontend/tests/helpers/robotPackage.ts` /
  前端 `robotConfigRegistry` 的 `import.meta.glob`）。**新增路径字段必须登记**（未登记即报错）。
  动机：搬迁时"自己拼路径的读者"漏改**不一定报错**——它可能解析到一个**仍然存在的**同名文件
  （只是已属另一台机器人）。仓库根 `config/` 现在**只剩** `robots.yaml` 选择器。
- **真值已冻结**：`core/baseline/baseline-kinematics-physics.json` + `core/tools/freeze_baseline.py`
  （`--robot` 缺省 `mearm-v1`）+ `tests/sim/test_baseline_frozen.py`。判据是**语义核心哈希**（非整文件，D55）
  ⇒ 改**外观**（`links[].geometry` / `details`）**放行**；改**运动学**（`length` / 轴限位耦合 / `actuators` 标定）
  或**物理量**（gravity/contact/timestep/servo/inertia）⇒ **报错并逐字段列差异**。有意改后：
  `$PY core/tools/freeze_baseline.py --update` **且**重跑 `$PY robot-package/mearm-v1/tools/gen_model.py`
  （MJCF 是产物，`test_generated_mjcf_is_in_sync_with_config` 盯同步）。
  ★ 冻结基线里的键 = 真值文件的**仓库相对路径** ⇒ 路径一变键就变，`--update` 后**哈希必须逐位不变**。
- **黄金数据同样冻结**：`robot-package/<id>/tests/cases/*.json`（D71），`robot-package/mearm-v1/tools/gen_mearm_v1_baseline.py`
  生成、`--check` 逐位复现；**改运动学/物理后必须重跑**，否则 Sim2Sim 回归必红。
- `mode`（simulation/real）**不是 UI 开关**：决定"要不要发给真实机械臂"，取值须校验全通过才改，
  失败要 pushLog 说明原因与修法（D41/D43）。
- **机器人相关状态一律进 store**，组件不持局部副本（含 `teachTrack`）。
- ★ **包边界 + 测试通道，各配一条「能红」的守卫**（Phase 2 步④⑤，明细 `playbook.md` §8）：Core 里
  不得出现任何**型号名 / 型号路径**（`corePackageBoundary.test.ts` 盯）；包内测试必须**真的被执行**
  —— Python 靠 `pytest.ini` `testpaths`（含 `robot-package`）、前端靠 `vite.config.ts` `test.include`
  （`'../robot-package/*/tests/**'`），守卫 `packageTestChannel.test.ts`。运动学引擎由
  `import.meta.glob('robot-package/*/kinematics/engine.ts')` **自动发现**（声明只用于对账）。
- **多机器人轨（2026-09-14，P0–P8 已完成）**：`config/robots.yaml` 选择器（**只放指针**：`name/config/
  physics/simulation.mjcf/tcpSite`，白名单 schema 断言盯着）→ 三端各自解析**同一份** →
  `RobotRegistry`（唯一分派表，禁止 `if robot == ...`）。第二台 = 官方 SO-ARM101，
  资产 `assets/models/so-arm101/official/`（**逐字节原样，禁止改**）。
  ★ **注册表 id ≠ 模型 id**（`mearm-v1` vs `robot.id = mearm`）⇒ 只按**路径**反查。
  ★ `physics.yaml` 两种形态：**顶层有无 `driver:` 段** = `legacy`(MeArm) / `driver`(SO-101，真值在官方 MJCF)。
  ★ 物理量真值 = **官方 MJCF**；限位与 TCP 帧朝向也取 MJCF（URDF 那两份不可信 —— 截断 / 差 90°）。
  ★ SO-101 **无 IK**：`solverKind:'none'` ⇒ `moveTo` 在**下发层**返回 `NO_SOLVER`（与"试过不行"并列）。
  统一验收只有一份：`run_sim2sim(robot_id)` / `core/tools/run_sim2sim.py --all`，**不为第二台另写一套**。

## 二、验收七件套

```bash
# ── 前端三件（在 frontend/ 下执行）──
cd frontend
./node_modules/.bin/tsc -b --force            # 0 error
./node_modules/.bin/vitest run                # 全绿（含 ../robot-package/*/tests/*.test.ts）
./node_modules/.bin/vite build                # dist/assets/*.js 中 __armPilot 命中 0（*.js.map 必然含，不算）
node tests/e2e/ui-smoke.mjs                   # 必须隔离端口
# ── Python 六件（★ 必须在**仓库根**执行）──
$PY -m pytest -q                              # ★ 走 pytest.ini testpaths：core/tests + tests + robot-package（204）
$PY core/tools/run_sim2sim.py --all           # ★ 统一 Sim2Sim 矩阵（选择器全部机器人，D77）
$PY robot-package/mearm-v1/tools/gen_mearm_v1_baseline.py --check    # 黄金数据逐位复现（改运动学/物理必跑）
$PY robot-package/so-arm101/tools/gen_so_arm101_robot_yaml.py --check # SO-101 配置 ↔ 官方模型同步（改生成器必跑）
$PY robot-package/so-arm101/tools/inspect_so101_physics.py --check   # SO-101 物理快照 ↔ 官方 MJCF（46 项）
$PY core/python/robopkg/cli.py validate --all # 选择器 / manifest / 真值 / 包目录 四方对账
```

- ⚠️★ **`pytest` 只在仓库根成立**（2026-09-14 实测）：从 `frontend/` 跑 `pytest -q` 会输出
  **`no tests collected` 且 exit 0** —— 只看退出码的脚本会把它读成「通过」。跑验收前先确认 **cwd = 仓库根**；
  另外 `pytest.ini` 的 `testpaths` 是相对 **rootdir** 解析的，**不要**给 pytest 传目录参数（传了就绕过 testpaths）。
- **前端那套覆盖不到** MJCF / 纹理管线 / 文档↔脚本一致性 / 黄金基线 ⇒ 都在 pytest 里。
- **造前提**：`node core/tools/park_sim_pose.mjs <wsUrl> --joints '{...}'` 把 sim 后端停到指定位姿（文件内
  **不含任何限位/角度常量**）。⚠️ **限位端点不可精确到达**（舵机整数度量化后可能落到限位外被后端如实
  拒绝 `ERR JOINT …`）⇒ 取**限额内部**的值。
- 环境硬约束（明细见 `playbook.md` §5）：`npx <tool>` 触发 WSL 黑名单 ⇒ 一律 `./node_modules/.bin/<tool>`；
  ★ **裸名 `bash` 也会触发**（PATH 首项 `/tmp/system32/bash`）⇒ 用 **`/usr/bin/bash x.sh`**；
  `(cmd &)` 后台进程只活到本次工具调用结束 ⇒ 起服务与跑 e2e 必须**在同一次调用里**；
  `/tmp/*.log` 重定向被沙箱拦 ⇒ 日志落 `.workbuddy/captures/`；e2e **必须隔离端口**（5276 + 8091）；
  Python 用 `~/.workbuddy/binaries/python/envs/default/Scripts/python.exe` + `PYTHONIOENCODING=utf-8`。

## 三、细节入口（按需读）

| 主题 | 去处 |
|---|---|
| 测试 / 探针纪律（D44 / D64–D67） | `playbook.md` §1 |
| 机制与关节判定（revolute / nq=5 / 绝对角 vs 局部角） | `playbook.md` §2 |
| 外观 · 纹理轨（RoundedBoxGeometry / flipY / 渲染参数语义，D56–D69） | `playbook.md` §3 |
| 测量方法论与能力边界（D34 / D37 / D47 / D59 / D64） | `playbook.md` §4 |
| 运行环境与 Windows 工具陷阱（sort / grep `\b` / taskkill） | `playbook.md` §5 |
| 协作约定与真机链路（COM16 / DTR 复位 / `--home`） | `playbook.md` §6 |
| **多机器人轨（选择器 / 注册表 / mesh / SO-101 陷阱）** | `playbook.md` §7 |
| **Robot Package 重构（Core/Package/WorkingRobot 边界与搬迁纪律）** | `playbook.md` §8 · `docs/architecture/robot-package-phase*.md` |
| 冻结与基线决策理由 | `docs/decisions.md` D55 / D71–D73 |
| 多机器人统一验收 / 运行期切换（D77–D79） | ADR D77 / D78 / D79 |
| 首帧"重影" / 幽灵臂渲染问题（D74） | ADR **D74** · `core/tools/park_sim_pose.mjs` · skill `webgl-first-frame-forensics` |
| 未修的无关问题 F1–F9 | `docs/architecture/mearm-v1-followups.md` |
