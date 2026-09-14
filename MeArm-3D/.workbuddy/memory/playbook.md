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
  `config/robot.yaml` · PATH 上的 `node` · `frontend/node_modules/.bin/vite.cmd`。

## §6 协作约定与真机链路

- `git push` **默认由用户自行执行**；**用户显式要求时可代推**（2026-09-14 起，用户明确下达过一次）。
  ⚠️ 推送的判定与坑见 §5「push 成功但不退出」。
- **破坏性操作先列清单确认**；建新目录先跑 `git check-ignore -v <path>/probe.txt` 探针。
- 每轮收尾：README 阶段表/§6 + `docs/decisions.md` ADR + memory **同步更新**。
- 提交按逻辑拆分（freeze / refactor / test 各自独立），不混在一起。
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
- **测试落点**：`tests/sim`（161）· `tests/sim2sim`（9）· `backend/internal/robot` `go test`（75）
  · 前端 `vitest`（409，含 `tests/acceptance/robot-switch.test.ts` 13 项 + `tests/sim2sim/so-arm101-baseline.test.ts` 12 项）。
  基线快照：`tests/baseline/{mearm-v1,so-arm101}/sim2sim.json`（`--freeze`，`--n-random 24` 冻结）。
