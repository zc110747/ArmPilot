# MeArm-3D · 项目长期记忆（索引）

> **本文件只是索引**：铁律 + 验收命令 + 去哪找细节。明细在同目录 `playbook.md`，
> 决策理由在 `docs/decisions.md`（D1–D74），验收数据在 `README.md` §6，每日过程在 `YYYY-MM-DD.md`。
> ★ 注入阈值实测 ≈6.3k 字符，超出会被**静默截断**（2026-09-14 从单文件 10.6k 拆成「索引 + playbook」）。

## 一、唯一真值源（铁律）

- 模型/标定/限位/零位**只有一份**：`config/robot.yaml`（前端与 Go 后端都读，`hello` 在线互检）。
  **禁止硬编码尺寸/角度/限位**；`backend/config.yaml` 只放运行参数；`config/physics.yaml` 只放物理量（全 SI），
  运动学量一律从 `robot.yaml` 读（`test_config_truth_is_not_duplicated` 盯）。
- **真值已冻结**（`config/baseline-kinematics-physics.json` + `tools/freeze_baseline.py` +
  `tests/sim/test_baseline_frozen.py`）：判据是**语义核心哈希**（非整文件，D55）⇒ 改**外观**放行；
  改**运动学**（`length` / joints 轴限位耦合 / `actuators` 标定）或**物理量**（gravity/contact/timestep/servo/inertia）
  ⇒ **报错并逐字段列差异**。有意改后：`$PY tools/freeze_baseline.py --update` **且**重跑
  `python simulation/mujoco/gen_model.py`（MJCF 是产物，`test_generated_mjcf_is_in_sync_with_config` 盯同步）。
- **黄金数据同样冻结**：`tests/baseline/mearm-v1/*.json`（D71），由 `tools/gen_mearm_v1_baseline.py` 生成、
  `--check` 逐位复现；**改运动学/物理后必须重跑**，否则 Sim2Sim 回归必红。
- `mode`（simulation/real）**不是 UI 开关**：它决定"要不要发给真实机械臂"，取值须校验全通过才改，
  失败要 pushLog 说明原因与修法（D41/D43）。
- **机器人相关状态一律进 store**，组件不持局部副本（含 `teachTrack`）。
- **多机器人轨（2026-09-14 起）**：`config/robots.yaml` 选择器（**只放 id/name/config**）
  → `loadRobotModel(id?)` → `RobotRegistry`（唯一分派表，禁止业务代码 `if robot == ...`）。
  第二台 = 官方 SO-ARM101，资产 `assets/models/so-arm101/official/`（**逐字节原样，禁止改**）。
  ★ 它的**物理量真值 = 官方 MJCF**（`config/robots/so-arm101/physics.yaml` **不复制任何数值**）；
  限位与 TCP 帧朝向均取 **MJCF**（URDF 那两份都不可信 —— 截断 / 差 90°）。
  能力：`solverKind: 'none'`，`inverse()` 诚实返回 `NOT_IMPLEMENTED`（**禁止伪造 IK**）。

## 二、验收七件套

```bash
cd frontend
./node_modules/.bin/tsc -b --force            # 0 error
./node_modules/.bin/vitest run                # 全绿
./node_modules/.bin/vite build                # dist/assets/*.js 中 __armPilot 命中 0（*.js.map 必然含，不算）
node tests/e2e/ui-smoke.mjs                   # 必须隔离端口
$PY -m pytest tests/sim -q                    # ★ 跨端改动（config / geometry 类型）必跑
$PY -m pytest tests/sim2sim -q                # Sim2Sim 基线回归（D71–D73）
$PY tools/gen_mearm_v1_baseline.py --check    # 黄金数据逐位复现（改过运动学/物理必跑）
$PY tools/gen_so_arm101_robot_yaml.py --check # SO-101 配置 ↔ 官方模型同步（改生成器/模型必跑）
$PY tools/inspect_so101_physics.py --check    # SO-101 物理快照 ↔ 官方 MJCF（46 项）
```

- **前端那套覆盖不到** MJCF / 纹理管线 / 文档↔脚本一致性 / 黄金基线 ⇒ 都在 pytest 里。
- **造前提**：`node tools/park_sim_pose.mjs <wsUrl> --joints '{...}'` 把 sim 后端停到指定位姿
  （文件内**不含任何限位/角度常量**，位姿由调用方给）。⚠️ **限位端点不可精确到达**——舵机整数度量化
  后可能落到限位外被后端如实拒绝（`ERR JOINT …`）⇒ 取**限额内部**的值。
- 环境硬约束（明细见 `playbook.md` §5）：`npx <tool>` 触发 WSL 黑名单 ⇒ 一律 `./node_modules/.bin/<tool>`；
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
| 冻结与基线决策理由 | `docs/decisions.md` D55 / D71–D73 |
| **首帧"重影" / 幽灵臂类渲染问题**（D74） | ADR **D74** · `tools/park_sim_pose.mjs` + `tools/first_load_probe.mjs` · 跨项目 skill `webgl-first-frame-forensics` |
| 未修的无关问题 F1–F9 | `docs/architecture/mearm-v1-followups.md` |
