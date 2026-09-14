# MeArm-3D · 项目 playbook（长期记忆明细）

> 由 `MEMORY.md`（索引）按需引用的明细。跨会话长期有效，改这里要同步 `MEMORY.md` 的入口表。

## §1 测试与探针纪律

- **探针只准用 `matrixWorld` / DOM 真实现象做证据，不许把数学再算一遍**（用 `FK(actual)` 证明"渲染了"
  是自证：挂错树 / 可见性误关 / 全透明都会让断言全绿）。
- **探针（含 `delete window.__xxx`）必须 `import.meta.env.DEV` 守卫**，否则字面量进生产包。
- ★ **位置型读取（`rows[i].children[j]`）必须限定到同一容器**（D67 一）：`.sidebar table.grid tbody tr`
  会同时命中 `ConnectionControl` 的"指标/值"表（排在 `StatusPanel` **前**）⇒ 一旦渲染，`rows[1]`
  静默漂移到"丢帧/拒绝"行。用 `cardByTitle('状态 · Status')` 先限定。**"读到值" ≠ "读到想读的对象"。**
  ⚠️ 这类探针**必须写成函数**（模块级模板字面量在 import 时求值 ⇒ `cardByTitle` 还在 TDZ ⇒ ReferenceError）。
- **时间相关的多个读数必须合并在同一次 `cdp.evaluate`**（分两次 CDP 往返会让快收敛量读到归零值 ⇒ 间歇失败）。
- ★ **端口预检不许用「分组 + `\b`」**（D65）：本机 GNU grep 3.0 里 `\b` 紧跟 `)` 失效，且**漏 `-E` 时
  `(` `|` `)` 是字面字符** ⇒ 两缺陷叠加让预检**恒返回"干净"**。用字段级比较
  `netstat -ano | tr -d '\r' | awk '$4=="LISTENING"{n=split($2,p,":"); if (p[n]==8090) print $5}'`；
  `taskkill //PID` 亦**报错并静默失败** ⇒ 用 `taskkill -F -PID`（单横线）或 `Stop-Process`。
- **`start.bat` 起的是"一对"进程**（D67 三）：真机后端（8090）+ 带 `VITE_AUTO_CONNECT=ws` 的 vite（5273），
  只清一个还会踩。症状：e2e 读到 `Command 列跟随滑杆 — 0 / 0`、滞后类断言读到已收敛值。
- **不许用推导量当独立判据**（D44：`TransportStats.moving = lagDeg > eps` 判"卡死"永远无结论）；
  **没有正面证据不下断言**，宁可停在保守态（`tracking`）。
- **e2e 有 SKIP 语义**（D65）：复用既有实例且 `device=serial` 时，「末端非 serial ⇒ 拒绝切换」前提不成立
  ⇒ 走 `skip()` 而非 `check()`，**不许把环境差异伪造成代码回归**。
- 断言语义要分清：`defineRobot()` 是**视图**（`toBe` 钉同一对象）；`forwardKinematics()` 每次调用都
  **新构造**结果 ⇒ 只能逐位比数值（`toContain`/`toBe` 会假失败）。
- ★ **逐帧记录器记得"是谁先跑"**（D74）：`useFrame` 回调**先于** `gl.render` ⇒ 读到的矩阵是
  **上一帧画出去的**；首帧读到的空对象是**仪器伪影，不是 bug**（主臂那帧正确只因它每 0.25 s 显式
  `updateMatrixWorld`）。**改探针前先分辨"真现象"与"采样时刻"**。

## §2 机制与关节判定速查

- 命令下发：store `commandJoints` → `transportBridge` **尾沿合并 30~33Hz** → `RobotTransport`；
  **回推只写 `actualJoints`**（回写 command 即无限回环）。**唯一例外 = 首次接管那一帧**（D74）：
  `commandJoints` 初值是对机器现状的**假设**（= `homePose`）⇒ 首次 `connected` 置 `attachPending`，
  由第一帧回推消费 → `attachToActual()` 对齐、**不下发**、**让位于用户意图**
  （`commandAuthoredSinceConnect`）；重连仍走 D32「补发命令」。
- 安全门：`mode === 'simulation'` **且**是真机链路（websocket + `device === 'serial'`）⇒ 拒发；
  mock / `device=sim` 照常放行（否则打死整条仿真闭环）。时钟：一切时间逻辑走 `TimerLike`。
- 端口：后端 **8090**（`/ws/joint`、`/healthz`）与 `MeArm-RemoteControl` 的 8080 舵机级摇杆**并存**。
- ★ **可动关节的唯一判定：`joint.type === 'revolute'`**（**不是** `!== 'fixed'`），三端同源
  （`isMovableJoint()` / Go `JointOrder()` / Python `is_dof`）。`nq = 5` 而自由度 = 4
  （被动腕 `tool` 有 qpos 但被 `<tendon><fixed>` + `<equality><tendon>` 锁死）；
  `joint_ids`（qpos 序）**≠** `dof_ids`（状态帧序），混用会得到五元组 JR。
- ★ `elbow` 存**离开天顶的绝对倾角**（HOME 112.6185771989°），MuJoCo `hinge qpos` 是**相对父 body 的局部角**
  ⇒ `relative = value + gain × otherValue` **只在 `units.JointAngleMap` 一处**转换。
- **抽象层性质**（D72）：`defineRobot()` 是对 `RobotModel` 的**分节视图**（所有字段是原对象引用，`toBe` 钉住，
  零转换零拷贝）；`orientationError` 恒 `null`（本机无姿态自由度，填 `0` 会同时骗过调用方与测试）；
  `IKResult` 失败时 `positionError: null`；`MeArmKinematics` 是**零算法纯委托**，仅对未知 `prefer` 显式抛错。
- **独立裁判机制**：`tests/sim/ikbridge.py` → `frontend/tests/tools/kinematics-bridge.mjs` 用 Vite SSR
  加载器加载**同一份** `ik.ts`/`fk.ts`；Python 侧只递 JSON、**不解释任何运动学语义**。

## §3 外观 / 纹理轨（D56–D69）· 速查

> 完整口径在 `docs/decisions.md` D56–D69 + `docs/texture-capture-guide.md`。

- 定义：`robot.yaml → links[].geometry.texture` 填 key（相对 `assets/textures/`），`textureRegistry.ts`
  用 `import.meta.glob` 静态登记 ⇒ **新增纹理只丢文件，无需改代码**；
  两个必要守卫：① 无 DOM 不加载（`TextureLoader` 要 `document`，vitest 是 node 环境）；
  ② key 未登记 ⇒ `console.warn` + 回退纯色（**贴图失败是静默的**，只有测试能兜住）。
- ★★ **`RoundedBoxGeometry` 的 UV = "每面各自铺满 [0,1]"、与长宽比无关** ⇒ 不能整块挂一张图，
  **必须按面用材质数组**（大面贴照片、其余四面保持板色）。`materialIndex`
  `0=+X 1=-X 2=+Y 3=-Y 4=+Z 5=-Z`，6 个 group 覆盖全部顶点。⚠️ 它是**非索引几何**（`g.index === null`）
  ⇒ group 的 `start/count` 是**顶点范围**，按索引遍历直接 `TypeError`。
- **定位**：两个大面的 Δu/Δv 必有一个相反（盒体展开的必然）⇒ 同一张照片**必有一面需要镜像**。
- ★★ **`TextureLoader` 的 `flipY` 默认 `true`** ⇒ **图像顶行 ⇔ `uv_v = 1`**（不是 0）。不报错、类型检查
  也查不出，漏掉会让所有 v 方向推理**整体反号**。⚠️ **定朝向不许靠目视/推断** ⇒ 用**四象限探针纹理**定案。
- ★ **绝不改回 `scene.environment`**：它全局，且 three 用 `scene.environmentIntensity` **覆盖**
  `material.envMapIntensity`（仅当 `material.envMap === null`）⇒ 逐材质关反射**根本不生效**。
  逐材质方案（`plateEnvironment.ts` 程序化 PMREM，零外部 HDR）底座 ×1.0000。
- 几何铁律：plate 大面法向 = `size` 里**最小那一维**；摆动平面 = X–Z（`base` 轴 = 世界 Z，
  `shoulder`/`elbow` 轴 = 世界 Y）⇒ **相机在 −Y 一侧**。⚠️ **局部轴 ≠ 世界轴**（`tool_link` 局部 Z → 世界 X）
  ⇒ **别背轴向，拿实物转一圈找面积最大的面。**
- ⚠️⚠️ **`mujoco.Renderer(model, height, width)` —— 顺序是 (height, width)**（D61 一）。
  传反不报错、不告警，却让**所有目视结论都错** ⇒ **工具调用的参数语义必须先验证。**
- ★ **三态 `ok` / `reject` / `undecidable`；「判不了」≠「不合格」** ⇒ 此时**禁止**输出"像结论"的数字。
  ⚠️ **禁用 PIL `Image.transform(QUAD)`**（+12~17px **静默**平移），用 `make_texture.py` 的 `homography`。

## §4 测量方法论与能力边界（只留结论）

- **单批"最优" = 过拟合**：定 ROI / 阈值 / 骨架模型**必须跨 ≥2 批验证**。
- **"改模型让它对上"与"调参数让它对上"是同一类错误**：必须先证**测量可复现**
  （`tools/verify_calib_repro.py`：增益跨批极差 ≤5%），才有资格改 `robot.yaml`。**一次只改一条**，改完重跑判据。
- ★★ **带区间的搜索必须显式检测"最优解是否贴边"**（D59 ①）：最优落在区间端点 2% 以内
  ⇒ **该值不是测量结果，是区间的人为截断**。**两个读数互相矛盾时，先查区间是不是设窄了，别急着改模型。**
- ★★ **纯几何推理走不通时，改用「致动并观察」（motion-diff）**（D59 ④）：动一个关节 → 拍前后两帧 → 差分
  ⇒ **差分区域 = 该关节及其下游零件**，不依赖任何相机/位姿假设。⚠️ 但**给不出干净的单板四角** ⇒ 只做归属判定 + 粗定位。
- 量包围盒必须**目视复核**（线缆 / 桌沿会污染 bbox）；**手入镜 / 相机位移**是头号污染源。
  ★ **固定 ROI 只在同一机位下可比**（D64 七）：换机位必须重定 ROI 并**目视复核 ROI 在物体上**；
  量一个小面时不要用"掩膜 bbox 画矩形"（会把**确实该变**的邻件圈进来）⇒ 在**基线图算出的固定像素集合**上量。
- **MG90S 无位置回读**：`OK SET` / `STATUS` / 后端 `joint_state` **全都在说"我打算去哪"**，
  没有一条能证明"它实际上在哪"；唯一外部地面真值是**相机**（D34）。
  ★ **「无位置回读」≠「没动作」**：只说明**不能证明"到位"**，**不等于"没动"**。
  Phase 11 误差面板反映的是**链路时延与限位截断**，不是"物理臂到位没有"。
- 当前状态：A1（骨架双杆）/ A2（`coupling.gain` → −0.81）**均判定为不改**；
  唯一入口是 A3 台面重测（`docs/hardware-measurement.md` §7）。

## §5 运行环境与 Windows 陷阱

| 用途 | 位置 / 版本 |
|---|---|
| 前端 | `frontend/node_modules`（`npm install` 产物，**不入库**）· Node 22.22.2（managed）· vite 8 · vitest 5 · tsc 5.9 |
| 后端 | Go 1.27 → `backend/bin/armpilot-backend.exe`（`/backend/bin/` 已 gitignore）· module `armpilot/backend`，唯一外部依赖 `gopkg.in/yaml.v3 v3.0.1` |
| Python | `~/.workbuddy/binaries/python/envs/default` · py3.13 · numpy 2.5 · Pillow 12.3 · pyyaml 6.0 · **mujoco 3.13** · pytest 9.1 · pyserial 3.5 |

- ★ **`backend/go.mod` 曾根本不存在**：父仓 `D:/user_project/git/ArmPilot/.gitignore` 里那条 `*.mod*`
  （从 Linux 内核模板抄来）把 `go.mod` 一起吞了 ⇒ `go build` 报 `cannot find main module`。已加例外 `!go.mod`
  （注意 `*.mod` / `*.mod*` 都必然匹配 `go.mod`，只能靠负向规则救）。**`MeArm-RemoteControl` 是同一个坑**
  （其 `go.mod` 至今未入库，followups **F1**）⇒ 见 `cannot find main module`，先跑
  `git check-ignore -v <path>/go.mod`，别急着 `go mod init`。
- ★★ **裸名 `bash` 解析到 WSL 启动器，会被沙箱拦**（2026-09-14 实测）：`which -a bash` 首项是
  `/tmp/system32/bash` = `C:\Windows\System32\bash.exe` ⇒ `bash x.sh` 直接报
  `PROGRAM BLOCKED BY SECURITY POLICY … wsl.exe`，**且 stdout 被整段丢弃**（表现为"脚本什么都没输出"，
  极易误判成脚本自身的问题）。**跑脚本一律 `/usr/bin/bash x.sh` 或 `sh x.sh`。**
  ⚠️ 脚本 shebang 写 `#!/usr/bin/env bash` 也会踩同一个坑 ⇒ 写 `#!/usr/bin/bash`。
- ★★ **`./node_modules/.bin/vite` 同样会被拦**（npm 生成的 shim 先 `cygpath -w "$basedir"`，
  再 `exec node "D:\…\node_modules\.bin/../vite/bin/vite.js"`，反斜杠路径走歪）⇒
  **`node node_modules/vite/bin/vite.js …`**。`vitest` 的 shim 不受影响（同上位脚本实测可用）。
- ★★ **本机 `sort` 解析到 Windows `System32\sort.exe`**（不认 `-u`）⇒ `... | sort -u` **静默返回空**，
  表现为"按 PID 清理进程"的循环一个都没杀。去重用 `awk '!seen[$0]++'`，或走 `/usr/bin/sort`。
  **与 §1 的 `grep \b`、`taskkill //PID` 是同一类坑：工具语义没验证。**
- `backend/config.yaml → device.mujoco.python` 是**本机绝对路径**，换机器必改（followups **F5**）。
- ★★ **`git push` 在本沙箱"推送已生效，但退出非 0 / 或干脆不退出"**（2026-09-14 实测两次）：
  stderr 末尾是 `fatal: unable to write credential store: Permission denied` +
  `[sandbox] 命令被沙箱拦截 … C:\Users\lx176\.git-credentials (写 · 剥写)`，
  但**上一行已经打印了 `6981c77..d98f7ea  Develop -> Develop`** ⇒ **推送其实成功了**，
  挂掉的只是"把凭据回写缓存"这一步（另一次同场景表现为进程 6 分钟无输出被转后台，
  `GIT_TERMINAL_PROMPT=0` 拦不住）。
  ⇒ **判定推送是否成功只认 `git ls-remote origin refs/heads/<branch>` 与本地 HEAD 比对**，
  **不要看退出码**；非 0 时先读 stderr 最后一行，分辨"真失败"与"收尾被拦"。
  连通性自检：`curl -s -o /dev/null -w "%{http_code}" --max-time 8 https://github.com`（返回 200 即网络正常，
  curl 自身 exit 23 是沙箱写 `/dev/null` 被拦，**不是网络故障**）。
- `start.bat` 前置检查只依赖：`backend/bin/armpilot-backend.exe` · `backend/config.yaml` ·
  `config/robots.yaml`（**注意：只有 `robots.yaml`，旧的 `config/robot.yaml` 已随选择器化废弃**）·
  PATH 上的 `node` · `frontend/node_modules/.bin/vite.cmd`。
  ★ `[0/3]` 之前的**陈旧闸门**会在 `.go` 源比 exe 新时自动 `go build`；launch 后还会再探一次 `/healthz`。
  两者都存在的原因：**"启动了"和"真的在服务"是两个命题**（详见 §9）。
- ★★ **本机 `grep`（msys2 `GNU grep 3.0`）里 `\[` / `\]` 不等于字面方括号**（2026-09-14 实测）：
  `grep -c '\[0/3\]' start.bat` 返回 **59**（= 文件里含 `0`/`/`/`3` 的**行数**，说明 `\[…\]` 被当成
  **方括号表达式**），而该串真实出现 **1** 次；`grep -cF '[0/3]'` 与 Python `str.count()` 都返回 1。
  用 `\|` 把两个模式或起来更糟：`grep -c "robot-package\^)\|\[0/3\]"` 返回 **0**
  （而两个模式**都确实在文件里**）——这是本轮唯一一次把工具 bug 误读成"仓库被回退"的源头。
  ⇒ **数精确字符串一律 `grep -cF`，或直接 Python `str.count()`**；先拿"已知答案的探针文件"
  验一次工具语义（与 §1 的 `grep \b`、§5 的 `sort -u` 是同一类坑：**工具语义没验证就下结论**）。
  > 假警报的完整还原：`grep` 报 0 → 我读成"修复被 `checkout` 回退"，于是去查 reflog/index mtime。
  > 真相是：`git diff -- MeArm-3D/start.bat` **为空**（工作区 == 暂存区**内容一致**），
  > 313/343 字节差 = 343 行 × 1 字节 = **纯 CRLF 行尾**，属正常；
  > 工作区与暂存区都含 `robot-package^)` ×1、`[0/3]` ×1、`STALE` ×3、`healthz` ×5。
  > **教训：判"文件被回退"用 `git diff` + 逐字节哈希，不要用 `grep` 的计数。**

## §6 协作约定与真机链路

- `git push` **默认由用户自行执行**；**用户显式要求时可代推**（2026-09-14 起，用户明确下达过一次）。
  ⚠️ 推送的判定与坑见 §5「push 成功但不退出」。
- **破坏性操作先列清单确认**；建新目录先跑 `git check-ignore -v <path>/probe.txt` 探针。
- 每轮收尾：README 阶段表/§6 + `docs/decisions.md` ADR + memory **同步更新**。
- 提交按逻辑拆分（freeze / refactor / test 各自独立），不混在一起。
- **真机链路**：`backend/bin/armpilot-backend.exe -c config.serial.yaml` · **CH340 @115200 8N1** ·
  ★ **COM 号不是常量**，它会随 USB 枚举变：2026-09-14 实测 = **`COM18`**（此前 `COM16`，已成
  `Present=False` 的幽灵条目）。**唯一改动点 = `config.serial.yaml` 的 `device.serial.port`**，
  别信记忆/文档里的旧值 —— 先 `serial.tools.list_ports.comports()` 看一眼实际是谁。
  开机四舵机全 90°(= HOME)。调试入口 `core/tools/set_joints.mjs`（`--status`/`--home`/`name=value`），
  **别拿验收脚本当摇杆**。
  - ★ `hello` **没有** `connected` 字段（`docs/serial-v1.md` §5.1 属**文档漂移**，followups **F4**）
    ⇒ 判真机只能看 `hello.device === 'serial'`。
  - ★ `hello` 与 `joint_state` 同时到达且 `hello` 在前 ⇒ 发送点必须在收齐 `joint_state` 之后，
    否则"保持不变"的关节会 fallback 到 `homePose` = **把臂拉回 home**。`--settle` 的"保持不变"
    会让状态**跨实验累积** ⇒ 对比实验必须先 `--home`。
  - ★ 打开串口会拉低 DTR 复位 ATmega328P（舵机弹回 90°）⇒ 实测脚本必须在**单次连接**内完成
    `[set → 稳定 → 抓拍]`。

---

## §7 多机器人轨（SO-ARM101）· 速查（2026-09-14 起）

**通用层**（与具体机器人无关，MeArm / SO-101 共用）：

- 选择链：`config/robots.yaml`（**只**放 id/name/config，禁止放参数）
  → `model/robotConfigRegistry.ts`（`import.meta.glob` 构建期登记 yaml 原文）
  → `model/loadRobotModel(id?)`（按 id 缓存；未知 id **抛错不回退**）
  → `registry/RobotRegistry.ts`（**唯一**分派表：工厂表 + `assertRegistryCoverage()`）。
- 业务代码**禁止** `if robot === ...`；"谁有 IK"由 `kinematics.capability` **声明**（数据）。
- 叶子模块（防循环导入）：`model/configError.ts`（`RobotConfigError`，`source` 可传文件标签）、
  `model/robotIds.ts`（`MEARM_V1_ROBOT_ID` / `SO_ARM101_ROBOT_ID`）。
- `Actuator.unit`：`'deg'`（缺省，0..180 舵机行程）/ `'joint'`（关节空间，ctrlrange ≡ 关节 range）。
  `ACTUATOR_LIMIT_180` 只在**非**关节空间生效。
- `RotationConvention`：`'xyz'`（缺省，intrinsic，`Rx·Ry·Rz`）/ `'rpy'`（URDF `<origin rpy>`，
  fixed-axis，`Rz·Ry·Rx`；three.js 侧用 Euler order `'ZYX'`，**数值原样只换 order**）。
- mesh：`model/meshRegistry.ts` 只登记 **URL**（`?url` + eager）；渲染在
  `RobotScene/meshObject.ts`，两条守卫（无 DOM 不加载 / key 未登记则 warnOnce + 回退）。

**SO-ARM101 专属**：

- 官方资产 `assets/models/so-arm101/official/`（**逐字节原样，禁止改**）；配置在
  `config/robots/so-arm101/{robot.yaml,physics.yaml}`；生成器 `tools/gen_so_arm101_robot_yaml.py`。
- ★ **物理量真值 = 官方 MJCF** ⇒ `physics.yaml` **不复制任何数值**，只放
  真值声明 + 驱动参数 + 审计快照 + **官方未声明项**。
  `tools/inspect_so101_physics.py --check` 从 **MjModel** 读生效值（**不是读 XML 文本** ——
  XML 里 class 写的 `forcerange` 是 ±2.94，被 6 个 `<position>` 逐个覆盖成 ±3.35）。
- ★ **两条 90° / 精度陷阱**：TCP 帧朝向取 **MJCF**（URDF `rpy=[0,π,0]` vs MJCF `quat=Ry(π/2)`）；
  关节限位取 **MJCF 满精度**（URDF 截断到 6 位小数）。理由见 `SOURCE.md §4.2`。
- ★ **FK↔MuJoCo 残差**的根因是**官方两份文件自身的精度差**（URDF `rpy` 截断到 6 位有效数字
  `1.5708`≠π/2；MJCF 四元数归一化后恰好 90°），**不是换算错误**。
  实测冻结值：MeArm **7.7e-14 mm** / SO-101 **3.3e-3 mm**。
- **能力声明**：`{positioningDof: 5, supportsOrientation: false, solverKind: 'none'}`；
  `inverse()` → `ikFailure('NOT_IMPLEMENTED')`（`success:false` / `joints:{}` /
  `positionError:null`）。**禁止**抄 MeArm 的平面 2R 解或塞数值解 —— 见 spec「不伪造」。
- 官方**未声明**（别以为有）：无 floor/table、**无 `<contact><exclude>`**（相邻连杆会互相碰撞）、
  无关节速度上限、无独立标定段、质量来自 CAD 非称重。
- 执行器：官方 6 个 `<position>`、`gear=1` ⇒ **无 offset/scale/reverse 标定**（`unit: joint`）。

**P3'–P8 落地事实**（2026-09-14 完成，实测数据见 `README.md` §6）：

- **三端同源**：`config/robots.yaml` 由**前端**（`import.meta.glob`）、**Go**（`internal/robot.LoadSelector`
  /`LoadByID`）、**Python**（`robotcfg.py`）**各自解析同一份文件** —— 三端都有 `resolve_robot_entry_by_config()`。
- ★★ **选择器只放"指针"**：允许 `name` / `config` / `physics` / `simulation.mjcf` / `simulation.tcpSite`，
  顶层只允许 `version` / `default` / `robots`；**尺寸、限位、标定、物理量、TCP 偏移一律禁止**。
  用**白名单 schema 断言**守（Go `TestSelectorSchemaAllowsPointersOnly`），不是靠自觉。
- ★★ **注册表 id ≠ 模型 id**：选择器 key `mearm-v1`，而 `robot.yaml → robot.id` 是 `mearm`
  ⇒ **不能用 `robot.id` 反查注册表**；`SO_ARM101_ROBOT_ID = 'so-arm101'`（注意是 `so-arm101` 不是 `so-arm101-*`）。
- ★ **`physics.yaml` 两种形态，判据是"顶层有没有 `driver:` 段"**（刻意不加配置项）：
  `legacy`（MeArm，顶层即驱动参数）/ `driver`（SO-101，物理量真值在官方 MJCF）。
  Go 侧告警**必须按 `physics_kind` 分支** —— 对官方模型说"惯量是估算值"是**错的告警**，
  比没有告警更糟（`backend/main.go`）。
- ★★ **`max_velocity` 优先级 = `rad_per_s` 先**：MeArm 同时写了 `deg_per_s: 573` 与 `rad_per_s: 10`，
  573°/s 换算回来 = 10.000737 rad/s ⇒ **取 `rad_per_s` 才是改前行为**。写反会静默改变运动速度。
- ★★ **timestep 不一致是静默错误**：物理时间只由 `model.opt.timestep` 决定（MeArm `0.001` / SO-101 `0.002`）
  ⇒ `RobotSim` 构造时硬自检，`server.run()` 再对 `--phys-hz` 告警"以 MJCF 为准"。
- **能力门必须落在真正下发命令的层**（D78）：`store.moveTo` 在 `solverKind !== 'analytic'` 时
  **不调求解器**，返回 `NO_SOLVER`。它与 `OUT_OF_WORKSPACE` / `JOINT_LIMIT` **并列而不合并** ——
  前者是"**没算**"，后两者是"**试过不行**"。合并会让用户以为要去调目标点。
  拒绝时仍写入 `target`，UI 才能说清"我想去哪"和"为什么没动"。
- **统一 Sim2Sim 只有一份判据**（D77）：`run_sim2sim(robot_id)`（`simulation/mujoco/sim2sim.py`）
  + CLI `tools/run_sim2sim.py --all` + `sim2sim_matrix()`（默认取选择器**全部**机器人）。
  **绝不**为第二台机器人另写一套验收（两套标准各自都会绿）。
- ★★ **容差按机器人登记、必须写明理由、未登记一律 `raise`**（不给缺省值）：
  `FK_TOL_MM = {mearm-v1: 1e-6(实测 7.7e-14，留 7 个数量级), so-arm101: 5e-2(实测 3.3e-3，留 1 个数量级)}`。
  悄悄给个"够大"的缺省 = 把"没人想过它的精度来源"藏起来，而那正是放宽阈值以通过测试的开端。
- ★ **交叉一致性**：统一框架的快照（`run_sim2sim.py` 采）与旧黄金基线（`gen_mearm_v1_baseline.py` 采）
  在两批**独立采集**产物的**同名用例**上必须逐位相同 —— 否则抽象过程会把基准期望值悄悄改掉。
- ★ **同名关节会骗人**：两台机器人**都有** `gripper`（MeArm `0..90` vs SO-101 `-10..100`）
  ⇒ "键集合相等"**不是**模型一致的判据，只有**限位**能兜住。切换后的断言应写
  "MeArm 独有键 `not.toHaveProperty`" + 专节声明同名不同义。
- **活动机器人是 store 状态**（D79）：初值取选择器 `default`；`setRobot` 有守卫
  （`transportDriven` 时拒切并说明"请先断开"）+ 未知 id **拒绝不回退**；
  `hello` 只做**在线互检**、**不静默切换**。★ 它**不是** `model.id`（见上）。
  运行期一律用 `get().model`，**不要再引用模块级 `initialModel`**。
- **测试落点**：`tests/sim` · `tests/sim2sim` · `backend/internal/robot` `go test`
  · 前端 `vitest`（含 `tests/acceptance/robot-switch.test.ts` + `tests/sim2sim/so-arm101-baseline.test.ts`）。
  基线快照：`robot-package/{mearm-v1,so-arm101}/tests/cases/sim2sim.json`（`--freeze`，`--n-random 24` 冻结）。

## §8 Robot Package 重构（Core / Robot Package / Working Robot，2026-09-14 起）

**目标形态**：`core/`（机制，**不含任何型号名**）· `robot-package/<id>/`（一台机器人的全部：manifest +
`model/` + `physics/` + `tests/cases/` + `tools/` + 资产）· `working-robot/`（"现在轮到谁"，构建产物，gitignore）。
spec = `docs/MeArm_3D_Prompter_06.md`（56 节）；只读分析 = `docs/architecture/robot-package-phase0.md`。

**文档落点（2026-09-14 定，用户拍板）**：串口 / WS 协议基线 = **`MeArm-3D/docs/serial-v1.md`**
（原 `MeArm-3D/protocol/serial-v1.md`，`protocol/` 目录已撤）。phase0 映射表原拟的 `core/protocol/`
**未采用**，该行已就地标注实际落点。全仓 51 处引用 + 2 处仓库树 + 1 处映射表已同步
（唯一故意不改：父仓 `docs/MeArm_3D_Prompter_01.md` 的**历史设计快照树**）。

**搬迁纪律（不可跳）**：一次只搬一个子系统 → `Build → Test → 验证 → 记录`，**每步交付一次实测**。
判据按子系统选：
- **配置真值**：`freeze_baseline.py --update` 后**语义核心哈希逐位不变**（证明"只挪了位置，一个字节没动"）。
- **黄金数据**：`gen_mearm_v1_baseline.py --check` 四份 `逐位一致`。
- **包契约**：`core/python/robopkg/cli.py validate --all`（选择器 / manifest / 真值 / 包目录 四方对账）。

### 本轮（Phase 2 步① ②）踩到的坑 —— 全是"静默"类

- ★★ **「编辑成功」≠「改到盘上了」**：`tools/gen_so_arm101_robot_yaml.py` 的 `OUT_PATH` 上一轮改过、
  **实际没落盘**。它不报"我还在用旧路径"，而是报 `缺少 config\robots\so-arm101\robot.yaml`
  —— 读起来像"文件丢了"。⇒ **每处编辑后用独立命令核对磁盘**（`grep -n` / `--check` / `Read`），
  同文件多处编辑尤其要复核。
- ★★ **`bash` 的 `cat >> file << EOF` 在本沙箱会写坏文件**（2026-09-14 实测）：追加 45 行，
  结果**文件头部被覆盖**（标题 / §1 / §2 标题丢失、尾部混入半行残片），而字节数**恰好不变**
  （22156 → 22156），`wc`/`ls` 完全看不出来。**唯一可靠的发现方式是 `Read` + `grep -n "^#"` 看章节**。
  ⇒ **追加长文本一律用 Edit 工具（带锚点）或 Write（整文件重写），并在写完后立刻读回核对**。
  ⇒ 记忆文件受 git 跟踪（仓库根在父目录 `ArmPilot/`，故 `git show HEAD:MeArm-3D/.workbuddy/memory/playbook.md`）
  ⇒ **写坏可直接 `git checkout --` 恢复**（本次即如此救回）。
- ★★ **"漏改的读者"清单要用全仓 grep 建立，不能靠记忆**：本轮漏掉的是
  ① 前端 `loadRobotModel.ts` 的 `import robotYamlText from '@config/robot.yaml?raw'`
  （Vite 的 `?raw` **必须静态** ⇒ 它是唯一绕开 Registry 自己拼路径的读者；一处漏改 →
  **29 个 vitest 红 + 13 个 pytest 红**，vitest 报 `ENOENT`、ikbridge 报 `Cannot find module`）；
  ② `tools/{verify_pose,fit_pose,make_texture}.py` 的 `YAML` / `ROBOT_YAML` 常量；
  ③ `tests/sim/test_simulation.py`（`ROOT/"config"/"physics.yaml"`）；
  ④ `tests/sim/test_baseline_frozen.py` 的 `fb.BASELINE`（`freeze_baseline` 函数化后属性名变了）。
  ⇒ **扫描词**：`"config"` · `config/robot.yaml` · `config/physics.yaml` · `config/robots/` ·
  `tests/baseline` · `baseline-kinematics-physics`。
- ★ **报错文案决定定位速度**：`freeze_baseline.check()` 原先在路径变更时抛 `KeyError: '…/model/robot.yaml'`
  —— 读起来像"工具坏了"。改成可读的"**键已过期**"并直接给出修法（`--update`）后，一眼归因。
- ★ **不要把当时的目录布局写进断言**：`robotRegistry.test.ts` 的 `config.startsWith('config/')`
  问的是"路径在不在 `config/` 下"，而它**想**问的是"是不是三端都能解析的仓库相对路径"。
  搬迁时改它**不是放宽判据**，但必须**同时加强** —— 本轮补了"选择器声明的路径 ≡ 该包 manifest 的
  `model.config` 声明（resolve 后是同一个文件）"，把"两处声明各自漂移"也钉住。
- ★ **用户可见字符串里的路径同样不许写死**：`backend/main.go` 的物理量告警改为取
  `entry.PhysicsPath` / `entry.ConfigPath`；`ik.ts` 抛错文案里的 `config/robot.yaml` 是**进生产包**的
  —— 靠 `grep -ho "config/robot.yaml\|config/physics.yaml" dist/assets/*.js | wc -l`（须为 0）才发现。
- ★ **Core 里不该留下"像是真值目录"的名字**：`robotcfg.py` 的 `CONFIG_DIR` 已删（它现在只服务选择器，
  留着这个名字会让人以为真值还在仓库根 `config/` 下）。
- **`robopkg` 是包不是模块** ⇒ `python -m robopkg` 不可用；入口是
  `python core/python/robopkg/cli.py {list|show|validate|hash|selftest}`（退出码 0/1，不吞错误）。

### Phase 2 步③ ④ ⑤（2026-09-14 收口）—— 三个真 bug + 两条测试通道

**搬迁结果**：仓库根 `tools/` 消失，24 个工具就位 —— `core/tools/`（8：`freeze_baseline.py` /
`run_sim2sim.py` / `park_sim_pose.mjs` / `set_joints.mjs` / `ws_probe.mjs` / `first_load_probe.mjs` /
`lan_e2e_probe.mjs` / `verify_serial_e2e.mjs`）· `robot-package/mearm-v1/tools/`（14）·
`robot-package/so-arm101/tools/`（2）。前端：`ik.ts` / `MeArmKinematics.ts` → 包内 `kinematics/`，
`RobotRegistry` 改 `import.meta.glob` **自动发现**。测试：按**断言**拆（机制留 `tests/sim/`，期望值随包）。

**搬迁揭穿的四个真 bug（全是"静默"类）**：

| # | 症状 | 根因 | 修法 |
|---|---|---|---|
| 1 | `run_sim2sim.py --help` → `ModuleNotFoundError: No module named 'sim2sim'` | `Path(__file__).resolve().parent.parent` 搬到 `core/tools/` 后指向 `core/` | `_find_repo_root()` |
| 2 | `freeze_baseline.py` 实跑报"基线文件不存在：`config\baseline-…json`" | 同上 + `BASELINE`/`ROBOT_YAML`/`PHYSICS_YAML` 三条硬编码 | `_find_repo_root()` + 全改 `declared_path()` |
| 3 | `freeze_baseline.py` 是**半成品迁移**：`snapshot()` 的**键**已是包内路径、**读**的还是 `config/` ⇒ 路径二义 | 键与读取来自两个来源 | 统一为 `repo_relative(declared_path(...))` |
| 4 | `run_sim2sim.py --freeze` **静默假成功**：退出码 0，真值目录**一字节未变** | 默认目标 `PROJECT_ROOT/tests/baseline` 把旧目录**重新建出来** | 默认目标改 `declared_path(rid,"tests.cases")` + `rm -rf tests/baseline` |

★★ **`_find_repo_root()` = 「向上找标记」**：向上找**同时含 `core/` 与 `robot-package/`** 的那一层。
它**替代一切 `parents[N]` / `Path(__file__).parent.parent`** —— 后者的层数是**被搬迁改写的隐式契约**，
写错时**不报错**，只是解析到另一个目录（于是症状伪装成"文件丢了"）。

**两条测试通道 + 两条守卫（"测试存在" ≠ "测试被执行"）**：

- Python：`pytest.ini`（**仓库根**）的 `testpaths = core/tests / tests / robot-package`。
  ★ 刻意**不**设 `--import-mode=importlib`（现有测试依赖裸模块名导入 + prepend 模式）。
  ★★ **不要给 pytest 传目录参数** —— 传了就用 args **覆盖** testpaths，包内测试被**静默跳过**
  （旧记忆里的 `pytest tests/sim tests/sim2sim core/tests -q` 正是这种写法）。
- 前端：`vite.config.ts` 的 `test.include` 增 `'../robot-package/*/tests/**/*.test.ts'`。
  包在 `frontend/` **之外** ⇒ 包内 TS 测试的 `import 'vitest'` 向上走不到 node_modules，
  故 `tsconfig.app.json` 的 `paths` 显式加 `"vitest"` / `"vitest/*"`（否则 tsc 报 `Cannot find module 'vitest'`）。
- 守卫 A（通道）：`core/tests/test_package_contract.py::test_pytest_testpaths_covers_package_dirs`
  + `frontend/tests/unit/packageTestChannel.test.ts`（读 `vite.config.ts` 与各包 manifest 的 `tests.local`，
  断言目录存在 / 在自己包内 / 有 `*.test.ts` / 与 Core 测试不重名）。
- 守卫 B（边界）：`frontend/tests/unit/corePackageBoundary.test.ts` 扫 `src/**/*.{ts,tsx,vue}`
  （**先剥注释再匹配 `import`**），白名单**只有** `store/robotStore.ts` 的 IK 类型
  （`solveIk` / `IkBranch` / `IkPreference` / `IkReason` / `IkResult`），并做逐符号双向核对 +
  白名单腐烂检测 + 旧路径检测。★ `robotStore` ← 包内 `ik.ts` 的**层次倒置**留到 Phase 3，
  用 `IKResult.diagnostics` 袋子解。

★★ **`pytest` 只在仓库根成立**（2026-09-14 实测）：从 `frontend/` 跑 `pytest -q` 输出
**`no tests collected` 且 exit 0** —— 只看退出码的脚本会把它读成"通过"。跑验收前先确认 cwd。

**本轮新踩的本机沙箱坑（三条，都改工作方式）**：

- ★★ **同文件连续 / 并行 `Edit` 会丢写入**：⇒ 改完**立刻用独立 `grep -n` / `Read` 核对落盘**；
  一处以上修改优先写**一次性脚本**（对每条替换断言"命中次数 == 1"，零命中/多命中都报错）而非连续 Edit。
- ★★ **`Path.write_text()` 在 Windows 把 `\n` → `\r\n`**，而仓库是 **LF-only + `core.autocrlf=false`**
  ⇒ 批量改文件必须 `open(..., newline="")`，否则"1 行改动"里混进全文行尾变换
  （实测 `assets/textures/mearm/README.md` 1276 → 1303 字节、27 处 CRLF）。出锅后 `git checkout --` 回退重做。
- ★ **在途运行的 pytest 会用旧模块**：后台 pytest 在我改 `manifest.py` **之前**已 import 旧 `_unknown`
  白名单 ⇒ 拿新字段 `local` 报 10 failed。**不是缺陷** —— 改完代码必须重跑。

**守卫被反向验证过**（"能红"才算守卫）：往 `frontend/src/robot/` 插一个越界 import ⇒
`corePackageBoundary.test.ts` 报错并**点名** `__boundary_probe.ts`（随后删探针）；
临时去掉 `vite.config.ts` 的 `include` 项 ⇒ `packageTestChannel.test.ts` 变红并**给出修法**。

**三处重复真值被消除**（second source of truth）：`ORDER` 线序（`tests/sim/test_server.py` →
`dev.robot.joint_order()`）· 限位文案 `108.44..141.86`（改为**从真值派生** + `re.fullmatch` 钉格式）·
`sim2sim.json` 的 `generator`（`run_sim2sim.py` 写入目标改 `declared_path(rid,"tests.cases")`，
不再自带第二份生成器声明）。

**Phase 2 实测验收**：`pytest -q` **204 passed** · `vitest run` **33 文件 / 441 例** ·
`tsc -b --force` **0 error** · `vite build` 产物中 `__armPilot` / 旧真值路径命中 **0** ·
`go build/vet/test` **0** · `cli.py validate --all` **2 通过 / 0 问题** ·
`freeze_baseline.py` **与冻结基线一致** · `run_sim2sim.py --all` **全部机器人 FK 在登记容差内**。
文档：`docs/architecture/robot-package-phase2.md`（判据 / 真 bug / 纪律 / 验收命令 / 未做清单）。

**已登记未做（Phase 3+）**：资产搬迁（`assets/models/so-arm101/official/**`、`assets/textures/mearm/**`、
`3d-models/*.STEP`）· `IKResult.diagnostics` 袋子（解 `robotStore` → 包内 `ik.ts` 的层次倒置）·
C5（`role` 跨端语义分歧）· C6（`project_plates.py` 硬编码板件尺寸）· `test_ik.py` 残留的 MeArm 耦合
（目前是"把型号当数据键"的可接受形态）。

## §9 一键启动器（`start.bat`）与真机验收口径（2026-09-14 起）

### 9.1 `start.bat` 现在不止"起两个窗口"

| 阶段 | 做什么 | 为什么必须有 |
|---|---|---|
| `[0/3]` 陈旧闸门 | 任一 `backend/**/*.go` 比 `bin/armpilot-backend.exe` 新 ⇒ 自动 `go build`；**没有 `go` 就拒绝启动** | 旧二进制不报"我过期了"：它照常启动、毫秒内因配置错误死掉、窗口一闪而过。实测就这么发生的（exe 落后**整整一次重构**，仍在找已废弃的 `config/robot.yaml`） |
| `[1/3]` 端口预检 | 8090 / 5273 占用则列 PID 并**问**是否清理 | （原有） |
| `[2/3]` 启动 | 两个独立窗口 | （原有） |
| 启动后探活 | 主动请求 `/healthz`，答不上就在 **launcher 窗口里**报出来 | "窗口出现了" ≠ "服务起来了"。后端自己的报错在被回收的窗口里，用户在 launcher 里根本看不到 |

陈旧判据用 `%%~tT` 字符串比较（格式 `yyyy/MM/dd HH:mm` ⇒ 字典序 == 时间序）。
★ **别用 `dir /o-d` 多目标排序**（实测报"文件名、目录名或卷标语法不正确"），也别用 `forfiles /d`（只到"天"）。

### 9.2 `.bat` 里的括号：只有"块内裸括号"会炸

| 位置 | 安全? | 说明 |
|---|---|---|
| `if (...)` / `for ... (...)` **块内**的裸 `(` `)` | ❌ **炸** | cmd 的块读取器扫到第一个 `)` 就闭合块，余下文本成游离 token |
| 同一行**双引号内**的括号 | ✅ | 引号保护（实测） |
| **顶层**（不在任何块内）echo 里的括号 | ✅ | 不参与块配对（实测） |

症状：`此时不应有 .`（`. was unexpected at this time`）+ `RC=255`，脚本死在 preflight。
**定位手段（比二分快得多）**：把脚本复制成 `@echo on` 版本跑一遍，trace 会停在出错那一行。
（同理：块内 echo 里裸 `>` 会被当成**整块**的重定向，写文本要 `^>`。）

### 9.3 真机验收有**两条独立证据链**，不能混着读

| 证据链 | 能证明 | **不能**证明 |
|---|---|---|
| 链路层（`hello.device=serial` / `OK JR` 回执 / `joint_state` / `verify_serial_e2e.mjs --no-camera`） | 指令**确实送到了固件并被接受** | ❌ **物理到位** —— 真机无位置反馈，`joint_state` 是固件**内部目标值**（开环） |
| 相机（`verify_pose.py` 反解肩/肘绝对角） | **物理**在哪（唯一外部地面真值） | —— |

⇒ **链路层全绿 ≠ "真机验证通过"**。要判物理到位，必须让相机取景按 `docs/hardware-measurement.md`
的 **Phase 4.5** 就位（白分割板 + 画面内标尺 + 正交侧视 + **锁死曝光**）**且人员离场**。
台面没就位时，`verify_pose.py` 的**绝对误差含反解公共偏置**，不可当机械精度读 —— 只有
**重复性（同位姿两帧之差）**是干净的噪声底。取景里连臂都没有时，只能报"物理层未取到证据"，别硬算。

### 9.4 （已修）Core 重构后工具路径的 off-by-one

工具从 `<repo>/tools/` 搬进 `core/tools/` 时，`ROOT = resolve(TOOLS_DIR, '..')` **没多退一层** ⇒
`ROOT` 变成 `<repo>/core`。后果不是报错，而是**静默指错地方**：backend 找不到、输出写进
`core/.workbuddy/`、`verify_pose.py` 按旧目录找 ⇒ `[fatal]`。
**搬工具时同步检查**：`ROOT`/`REPO_DIR` 的层数 · 工具之间的相对引用 · 文档里写下的路径。
另一条同族纪律：型号专属工具**随包走**（`manifest.tests.tools` 是完整清单）⇒ Core 侧要
**由 `config/robots.yaml` 的 `default` 定位包**，不要写死 `mearm-v1`（写死 = 每接一台机器人回 Core 改一行）。
**这类修复的可测判据**：`node core/tools/verify_serial_e2e.mjs --dry-run`（不碰硬件，验路径 + 动作计划 + 限位）。
