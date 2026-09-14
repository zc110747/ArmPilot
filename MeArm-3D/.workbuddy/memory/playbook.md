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
- ★★ **本机 `sort` 解析到 Windows `System32\sort.exe`**（不认 `-u`）⇒ `... | sort -u` **静默返回空**，
  表现为"按 PID 清理进程"的循环一个都没杀。去重用 `awk '!seen[$0]++'`，或走 `/usr/bin/sort`。
  **与 §1 的 `grep \b`、`taskkill //PID` 是同一类坑：工具语义没验证。**
- `backend/config.yaml → device.mujoco.python` 是**本机绝对路径**，换机器必改（followups **F5**）。
- `start.bat` 前置检查只依赖：`backend/bin/armpilot-backend.exe` · `backend/config.yaml` ·
  `config/robot.yaml` · PATH 上的 `node` · `frontend/node_modules/.bin/vite.cmd`。

## §6 协作约定与真机链路

- `git push` **由用户自行执行**；agent 只做本地 commit / diff。
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
