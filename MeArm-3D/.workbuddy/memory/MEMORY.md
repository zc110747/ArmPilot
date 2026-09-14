# MeArm-3D · 项目长期记忆（curated）

> 只放**跨会话长期有效的铁律与速查**。每日过程写 `YYYY-MM-DD.md`。
> **验收数据看 `README.md` §6，决策理由看 `docs/decisions.md`（D1–D73，以 docs 为准）**
> —— 本文件只留结论；**控制在 8k 字符以内**，超出会在注入时被静默截断（2026-09-14 已从 10.6k 压到 7.9k）。

## 一、唯一真值源（铁律）

- 模型 / 标定 / 限位 / 零位**只有一份**：`config/robot.yaml`（前端与 Go 后端都读它，`hello` 带限位/标定在线互检）。
  **禁止硬编码尺寸/角度/限位**；`backend/config.yaml` 只放运行参数。第二份真值 `config/physics.yaml`
  只放物理量（全 SI），运动学量一律从 `robot.yaml` 读（`test_config_truth_is_not_duplicated` 盯）。
- **真值已冻结**：`config/baseline-kinematics-physics.json` + `tools/freeze_baseline.py` +
  `tests/sim/test_baseline_frozen.py`。判据是**语义核心哈希**（**不是**整文件哈希，D55）：
  改**外观**（`links[].geometry`/`details`）放行；改**运动学**（`length`/joints 轴限位耦合/`actuators` 标定）
  或**物理量**（gravity/contact/timestep/servo/inertia）报错并逐字段列差异。
  有意改参数后：`$PY tools/freeze_baseline.py --update` **并且**重跑
  `python simulation/mujoco/gen_model.py`（MJCF 是产物，`test_generated_mjcf_is_in_sync_with_config` 盯同步）。
- `mode`（simulation/real）**不是 UI 开关**，它决定"要不要发给真实机械臂"；取值必须校验全通过才改，
  失败要 pushLog 说明原因与修法（D41/D43）。
- **机器人相关状态一律进 store**，组件不持局部副本（含 `teachTrack`）。

## 二、验收七件套

```bash
cd frontend
./node_modules/.bin/tsc -b --force        # 0 error
./node_modules/.bin/vitest run            # 全绿
./node_modules/.bin/vite build            # dist/assets/*.js 中 __armPilot 命中 0（*.js.map 必然含，不算）
node tests/e2e/ui-smoke.mjs               # 必须隔离端口
$PY -m pytest tests/sim -q                # ★ 跨端改动（config / geometry 类型）必跑
$PY -m pytest tests/sim2sim -q            # Sim2Sim 基线回归（D71–D73）
$PY tools/gen_mearm_v1_baseline.py --check   # 黄金数据逐位复现（改过运动学/物理必跑）
```

- **前端那套覆盖不到** MJCF / 纹理管线 / 文档↔脚本一致性 / 黄金基线四类契约 ⇒ 都在 pytest 里。
- `npx <tool>` 触发 WSL 黑名单 ⇒ **一律 `./node_modules/.bin/<tool>` 直调**。
- **`(cmd &)` 后台进程只活到本次工具调用结束** ⇒ 起服务与跑 e2e 必须**在同一次调用里**。
- **`/tmp/*.log` 重定向被沙箱拦** ⇒ 日志落 `.workbuddy/captures/`（已 gitignore）；
  `node "/e/cnb/..."` 会被解析成 `E:\e\cnb\...` ⇒ 传 `E:/cnb/...` 形式。
- **e2e 必须隔离端口**：干净 dev server（5276，**不注入 `VITE_AUTO_CONNECT`**）+ 独立后端（8091，
  配置由 `backend/config.yaml` 派生）+ 用 `BACKEND_HTTP`/`BACKEND_WS` 指过去。
  ⚠️ 必须全写在**同一次工具调用里**；目前仍是手写长命令（followups **F8**）。
- Python：系统 `python3` 无 numpy。用 `~/.workbuddy/binaries/python/envs/default/Scripts/python.exe`
  （numpy/Pillow/pyserial/**mujoco**/pyyaml/pytest），设 `PYTHONIOENCODING=utf-8`（否则中文乱码）。

## 三、测试与探针纪律

- **探针只准用 `matrixWorld` / DOM 真实现象做证据，不许把数学再算一遍** —— 用 `FK(actual)` 证明"渲染了"
  是自证（挂错树 / 可见性误关 / 全透明都会让断言全绿）。
- **探针（含 `delete window.__xxx`）必须 `import.meta.env.DEV` 守卫**，否则字面量进生产包。
- ★ **位置型读取（`rows[i].children[j]`）必须限定到同一容器内**（D67 一）：`.sidebar table.grid tbody tr`
  会同时命中 `ConnectionControl` 的"指标/值"表（它排在 `StatusPanel` **前面**）⇒ 一旦渲染，`rows[1]`
  静默漂移到"丢帧/拒绝"行。用 `cardByTitle('状态 · Status')` 先限定。**"读到值" ≠ "读到想读的对象"。**
  ⚠️ 这类探针**必须写成函数**（模块级模板字面量在 import 时求值 ⇒ `cardByTitle` 还在 TDZ ⇒ ReferenceError）。
- **时间相关的多个读数必须合并在同一次 `cdp.evaluate`**（分两次往返会让快收敛量读到归零值 ⇒ 间歇失败）。
- ★ **端口预检不许用「分组 + `\b`」**（D65）：本机 `\b` 紧跟 `)` 失效 + 漏 `-E` 时 `(` `|` `)` 是字面字符
  ⇒ 恒返回"干净"。改用字段级比较
  `netstat -ano | tr -d '\r' | awk '$4=="LISTENING"{n=split($2,p,":"); if (p[n]==8090) print $5}'`；
  `taskkill //PID` 同样**静默失败** ⇒ 用 `taskkill -F -PID <pid>`（单横线）。
- **`start.bat` 起的是"一对"进程**（D67 三）：真机后端（8090）+ 带 `VITE_AUTO_CONNECT=ws` 的 vite（5273），
  只清一个还会踩。症状：e2e 读到 `Command 列跟随滑杆 — 0 / 0`、滞后类断言读到已收敛值。
- **不许用推导量当独立判据**（D44：`TransportStats.moving = lagDeg > eps` 判"卡死"永远无结论）；
  **没有正面证据不下断言**，宁可停在保守态（`tracking`）。
- ★ **固定 ROI 只在同一机位下可比**（D64 七）：换机位必须重定 ROI 并**目视复核 ROI 在物体上**；
  量小面时别用"掩膜 bbox 画矩形"（会把确实该变的邻件圈进来）⇒ 在**基线图算出的固定像素集合**上量。
- **e2e 有 SKIP 语义**（D65）：复用既有实例且 `device=serial` 时，「末端非 serial ⇒ 拒绝切换」前提不成立
  ⇒ 走 `skip()` 而非 `check()`，**不许把环境差异伪造成代码回归**。

## 四、测量与标定方法论（D37 / D47）

- **单批"最优" = 过拟合**：定 ROI / 阈值 / 骨架模型**必须跨 ≥2 批验证**。
- **"改模型让它对上"与"调参数让它对上"是同一类错误**：必须先证**测量可复现**
  （`tools/verify_calib_repro.py`：增益跨批极差 ≤5%），才有资格改 `robot.yaml`。**一次只改一条**。
- 量包围盒必须**目视复核**（线缆/桌沿污染）；**手入镜 / 相机位移**是头号污染源。
- ★★ **带区间的搜索必须显式检测"最优解是否贴边"**（D59 ①）：最优落在区间端点 2% 以内 ⇒
  该值不是测量结果，是区间的人为截断（实测 `s=3.80` 恰 = 当年 `S_LO=3.8`）。
  **两个读数互相矛盾时先查区间是不是设窄了，别急着改模型。**
- ★★ **纯几何推理走不通时改用「致动并观察」（motion-diff）**（D59 ④）：动关节 → 拍前后两帧 → 差分
  ⇒ 差分区域 = 该关节及其下游零件，不依赖任何相机/位姿假设。⚠️ 但给不出干净的单板四角 ⇒ 只做粗定位。
- ★ **「无位置回读」≠「没动作」**：MG90S 无位置反馈只说明**不能证明"到位"**，不等于"没动"。
- 当前状态：A1 / A2 **均不改**；唯一入口是 A3 台面重测（`docs/hardware-measurement.md` §7）。

## 五、能力边界（必须在文档与 UI 显式声明）

- **MG90S 无位置回读**：`OK SET` / `STATUS` / 后端 `joint_state` 全都在说"我打算去哪"，没有一条能证明
  "它实际上在哪"。串口回执**原理上无法验证物理到位**，唯一外部地面真值是**相机**（D34）。
- Phase 11 误差面板反映的是**链路时延与限位截断**，不是"物理臂到位没有"。

## 六、机制速查

- 命令下发：store `commandJoints` → `transportBridge` **尾沿合并 30~33Hz** → `RobotTransport`。
  **回推只写 `actualJoints`**（回写 command 即无限回环）。
- 安全门：`mode === 'simulation'` **且**是真机链路（websocket + `device === 'serial'`）⇒ 拒发；
  mock / `device=sim` 照常放行。时钟：一切时间逻辑走 `TimerLike`。
- 端口：后端 **8090**（`/ws/joint`、`/healthz`）与 `MeArm-RemoteControl` 的 8080 摇杆**并存**。
- **判定可动关节的唯一规则：`joint.type === 'revolute'`**（**不是** `!== 'fixed'`）。
  `nq = 5` 而自由度 = 4（被动腕 `tool` 有 qpos 但被 tendon 锁死）；`joint_ids`（qpos 序）
  ≠ `dof_ids`（状态帧序），混用会得到五元组的 JR。

## 六·五、运行环境（2026-09-14 一次性搭好）

| 用途 | 位置 / 版本 |
|---|---|
| 前端 | `frontend/node_modules`（**不入库**）· Node 22.22.2 · vite 8 · vitest 5 · tsc 5.9 |
| 后端 | Go 1.27 → `backend/bin/armpilot-backend.exe`（已 gitignore）· module `armpilot/backend`，唯一外部依赖 `gopkg.in/yaml.v3 v3.0.1` |
| Python | `~/.workbuddy/binaries/python/envs/default` · py3.13 · numpy 2.5 · Pillow 12.3 · pyyaml 6.0 · **mujoco 3.13** · pytest 9.1 |

- ★ **`go.mod` 曾被父仓 `.gitignore` 的 `*.mod*` 吞掉**（`MeArm-RemoteControl` 至今仍缺，followups **F1**）
  ⇒ 见到 `cannot find main module`，先跑 `git check-ignore -v <path>/go.mod`，别急着 `go mod init`。
- ★★ **本机 `sort` 解析到 Windows `System32\sort.exe`**（不认 `-u`）⇒ `... | sort -u` **静默返回空**
  （曾让"按 PID 清理进程"一个都没杀）。去重用 `awk '!seen[$0]++'` 或 `/usr/bin/sort`。
  **与 §三 `grep \b`、`taskkill //PID` 同类：工具语义没验证。**
- `backend/config.yaml → device.mujoco.python` 是**本机绝对路径**，换机器必改（followups **F5**）。

## 七、外观 / 纹理轨（D56–D69）· 速查

> 完整口径在 `docs/decisions.md` D56–D69 + `docs/texture-capture-guide.md`，此处只留最容易再犯的。
> 纹理 key 走 `links[].geometry.texture`，`textureRegistry.ts` 用 `import.meta.glob` 静态登记
> ⇒ **新增纹理只丢文件**；守卫：无 DOM 不加载、key 未登记 ⇒ warn + 回退纯色。

- ★★ **`RoundedBoxGeometry` 的 UV = "每面各自铺满 [0,1]"、与长宽比无关** ⇒ 必须按面用材质数组；
  `materialIndex` `0=+X 1=-X 2=+Y 3=-Y 4=+Z 5=-Z`；它是**非索引几何**（`g.index === null`）
  ⇒ group 的 `start/count` 是**顶点范围**，按索引遍历直接 `TypeError`。
- ★★ **`TextureLoader.flipY` 默认 `true`** ⇒ **图像顶行 ⇔ `uv_v = 1`**。不报错、类型检查也查不出，
  漏掉会让 v 方向推理**整体反号**。⚠️ 定朝向**不许目视/推断** ⇒ 用四象限探针纹理定案。
- ★ **绝不改回 `scene.environment`**：全局，且 three 用 `scene.environmentIntensity` **覆盖**
  `material.envMapIntensity`（仅当 `material.envMap === null`）⇒ 逐材质关反射**根本不生效**。
- 几何铁律：plate 大面法向 = `size` 里**最小那一维**；摆动平面 = X–Z（`base` 轴 = 世界 Z、
  `shoulder`/`elbow` 轴 = 世界 Y）⇒ **相机在 −Y 一侧**。⚠️ **局部轴 ≠ 世界轴**（`tool_link` 局部 Z
  → 世界 X）⇒ **别背轴向，拿实物转一圈找面积最大的面。**
- ⚠️⚠️ **`mujoco.Renderer(model, height, width)` —— 顺序是 (height, width)**（D61 一）。
  传反不报错、不告警，却让**所有目视结论都错**。⇒ **工具调用的参数语义必须先验证。**
- ★ **三态 `ok`/`reject`/`undecidable`；「判不了」≠「不合格」** ⇒ 此时**禁止**输出"像结论"的数字。
  ⚠️ **禁用 PIL `Image.transform(QUAD)`**（+12~17px **静默**平移），用 `make_texture.py` 的 `homography`。

## 八、协作约定

- `git push` **由用户自行执行**；agent 只做本地 commit / diff。
- **破坏性操作先列清单确认**；建新目录先跑 `git check-ignore -v <path>/probe.txt` 探针。
- 每轮收尾：README 阶段表/§6 + `docs/decisions.md` ADR + memory **同步更新**。
- **真机链路**：`backend/bin/armpilot-backend.exe -c config.serial.yaml` · **COM16 CH340 @115200 8N1** ·
  开机四舵机全 90°(= HOME)。调试入口 `tools/set_joints.mjs`（`--status`/`--home`/`name=value`），
  **别拿验收脚本当摇杆**。
  - ★ `hello` **没有** `connected` 字段（`protocol/serial-v1.md` §5.1 属**文档漂移**，followups **F4**）
    ⇒ 判真机只能看 `hello.device === 'serial'`。
  - ★ `hello` 与 `joint_state` 同时到达且 `hello` 在前 ⇒ 发送点必须在收齐 `joint_state` 之后，
    否则"保持不变"的关节会 fallback 到 `homePose` = **把臂拉回 home**。`--settle` 的"保持不变"
    会让状态**跨实验累积** ⇒ 对比实验必须先 `--home`。
  - ★ 打开串口会拉低 DTR 复位 ATmega328P（舵机弹回 90°）⇒ 实测脚本必须在**单次连接**内完成
    `[set → 稳定 → 抓拍]`。
