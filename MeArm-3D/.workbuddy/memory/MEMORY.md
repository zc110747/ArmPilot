# MeArm-3D · 项目长期记忆（curated）

> 只放**跨会话长期有效的铁律与速查**。每日过程写 `YYYY-MM-DD.md`。
> **详细验收数据看 `README.md`，决策理由看 `docs/decisions.md`（ADR D1–D70，以 docs 为准）** —— 本文件只留结论。

## 一、唯一真值源（铁律）

- 模型 / 标定 / 限位 / 零位**只有一份**：`config/robot.yaml`。前端与 Go 后端**都读它**（`hello` 带限位/标定在线互检）。
  **禁止在代码里硬编码尺寸 / 角度 / 限位。** `backend/config.yaml` **只放运行参数**（端口/设备模式/模拟器参数）。
- **第二份真值**：`config/physics.yaml` **只放物理量、全 SI**；运动学量一律从 `robot.yaml` 读
  （`test_config_truth_is_not_duplicated` 盯）。
- **真值已冻结**：`config/baseline-kinematics-physics.json` + `tools/freeze_baseline.py` +
  `tests/sim/test_baseline_frozen.py`。判据是**语义核心哈希**（**不是**整文件哈希，D55）：
  - 改**外观**（`links[].geometry` / `details`）→ ✅ 放行
  - 改**运动学**（`length` / `joints` 轴限位耦合 / `actuators` 标定）或**物理量**
    （gravity / contact / timestep / servo / inertia）→ ❌ 报错并逐字段列出差异
  - 有意改参数后：`$PY tools/freeze_baseline.py --update` **并且**重跑
    `python simulation/mujoco/gen_model.py`（MJCF 是产物，`test_generated_mjcf_is_in_sync_with_config` 盯同步）
  - ⇒ **外观层完全自由**（几何 primitive / 颜色 / 照片纹理 / 未来重建网格都可随便迭代），运动学与物理被钉死。
    **这是做视觉建模的前提。**
- **`mode`（simulation/real）不是 UI 开关**，它决定"要不要发给真实机械臂"。**取值必须校验全通过才改**，
  失败要 pushLog 说明**为什么**和**怎么修**（D41 / D43）。
- **机器人相关状态一律进 store**，组件不持局部副本（含 `teachTrack`）—— e2e 探针要读原始数值。

## 二、验收五件套

```bash
cd frontend
./node_modules/.bin/tsc -b --force        # 0 error
./node_modules/.bin/vitest run            # 全绿
./node_modules/.bin/vite build            # 并确认产物中 __armPilot 命中 0
node tests/e2e/ui-smoke.mjs               # 见下方"隔离端口"
<python> -m pytest tests/sim -q           # ★ 跨端改动（config / geometry 类型）必跑
```

- **前端那套覆盖不到 MJCF / 纹理管线 / 文档↔脚本一致性三类契约** —— 它们都在 pytest 里。
- `npx <tool>` 会触发 WSL 黑名单 ⇒ **一律 `./node_modules/.bin/<tool>` 直调**。
- **`(cmd &)` 起的后台进程只活到本次工具调用结束** ⇒ 起服务与跑 e2e 必须**在同一次调用里**，
  否则下一条必然 `ERR_CONNECTION_REFUSED`。
- **`/tmp/*.log` 重定向在沙箱内会被拦**（文件不生成）⇒ 日志落工作区内 `.workbuddy/captures/`（已 gitignore）。
- `node "/e/cnb/..."` 会被解析成 `E:\e\cnb\...` ⇒ 传 **`E:/cnb/...`** 正斜杠盘符形式。
- **e2e 必须在隔离端口跑**（8090/5273 常被用户**正在驱动真机**的实例占着，不能杀）：
  自起干净 dev server（如 5276，**不注入 `VITE_AUTO_CONNECT`**）+ 独立后端
  （由 `backend/config.yaml` 派生 `.workbuddy/e2e-sim.yaml`，端口 8091），
  再用 `BACKEND_HTTP` / `BACKEND_WS` 把 `ui-smoke.mjs` 指过去（它识别为"复用实例"并跳过断线重连子项）。
- Python：系统 `python3` 无 numpy。用 `~/.workbuddy/binaries/python/envs/default/Scripts/python.exe`
  （numpy / Pillow / pyserial / **mujoco** / pyyaml / pytest），并设 `PYTHONIOENCODING=utf-8`（否则中文乱码）。

## 三、测试与探针纪律

- **探针只准用 `matrixWorld` / DOM 层真实现象做证据，不许把数学再算一遍** ——
  用 `FK(actual)` 证明"渲染了"是自证（挂错树 / 可见性误关 / 全透明都会让断言全绿）。
- **探针（含 `delete window.__xxx` 那一行）必须 `import.meta.env.DEV` 守卫**，否则字面量进生产包。
- **★ 位置型读取（`rows[i].children[j]`）必须限定到同一个容器内**（D67 一）：
  `.sidebar table.grid tbody tr` 会同时命中 `ConnectionControl` 的"指标/值"表，而它排在 `StatusPanel` **前面**
  ⇒ 一旦那张表渲染，`rows[1]` 就静默漂移到"丢帧/拒绝"行。**"读到一个值" ≠ "读到了想读的那个对象"。**
  用 `cardByTitle('状态 · Status')` 先限定。⚠️ 这类探针**必须写成函数**：模块级模板字面量会在 import 时求值
  ⇒ `cardByTitle` 还在 TDZ，直接 ReferenceError。
- **时间相关的多个读数必须合并在同一次 `cdp.evaluate`** —— 分两次 CDP 往返会让快收敛的量读到归零值，
  变成**间歇性失败**。自洽性判据 = 各读数必须**互相印证**。
- **★ 端口预检不许用「分组 + `\b`」**（D65）：本机 GNU grep 3.0 里 `\b` 紧跟 `)` 会失效，
  且**漏 `-E` 时 `(` `|` `)` 是字面字符** ⇒ 两缺陷叠加让预检**恒返回"干净"**（我复犯过）。
  必须字段级比较：`netstat -ano | tr -d '\r' | awk '$4=="LISTENING"{n=split($2,p,":"); if (p[n]==8090) print $5}'`；
  省事写法 `netstat -ano | grep -E ":8090|:5273" | grep LISTENING`（**不带 `\b`**）。
  同理 `taskkill //PID` 在本机 Git Bash 下**报错并静默失败** ⇒ 用 **`taskkill -F -PID <pid>`（单横线）** 或 `Stop-Process`。
- **`start.bat` 起的是"一对"进程**（D67 三）：真机后端（8090）**和**带 `VITE_AUTO_CONNECT=ws` 的 vite（5273）。
  只清一个还会踩；页面加载即自动连上真机链路会让 **Mock 闭环类断言整体失真**。
  判别残留是否在害你的**症状**：e2e 读到 `Command 列跟随滑杆 — 0 / 0`、滞后类断言读到已收敛值。
- **不许用推导量当独立判据**：`TransportStats.moving = lagDeg > eps`，用它判"卡死"永远得不到结论（D44）。
- **没有正面证据不下断言**（如"卡死"）：宁可停在保守态（`tracking`）。
- **★ 固定 ROI 只在同一机位下可比**（D64 七）：换机位必须重定 ROI 并**目视复核 ROI 在物体上**。
  量一个小面时不要用"掩膜 bbox 画矩形"（会把**确实该变**的邻件圈进来）⇒ 正解是**用基线图算像素集合、在固定集合上量**。
- **e2e 有 SKIP 语义**（D65）：复用 8090 既有实例且 `device=serial` 时，「末端非 serial ⇒ 拒绝切换」前提不成立
  ⇒ 走 `skip()` 而非 `check()`，**不许把环境差异伪造成代码回归**。

## 四、测量与标定方法论（D37 / D47）

- **单批"最优" = 过拟合**。定 ROI / 阈值 / 骨架模型**必须跨 ≥2 批验证**。
- **"改模型让它对上"与"调参数让它对上"是同一类错误**。必须先证**测量可复现**
  （`tools/verify_calib_repro.py`：增益跨批极差 ≤5%），才有资格改 `robot.yaml`。
- **一次只改一条**，每改一条重跑判据 —— 一次改多条 = 无法归因。
- 量包围盒必须**目视复核**（线缆 / 桌沿会污染 bbox）；**手入镜 / 相机位移**是头号污染源。
- ★★ **带区间的搜索必须显式检测"最优解是否贴边"**（D59 ①）：最优落在区间端点 2% 以内
  ⇒ **该值不是测量结果，是区间的人为截断**（实测 `s=3.80` 恰好 = 当年的 `S_LO=3.8`）。
  **两个读数互相矛盾时，先查区间是不是设窄了，别急着改模型。**
- ★★ **纯几何推理走不通时，改用「致动并观察」（motion-diff）**（D59 ④）：动一个关节 → 拍前后两帧 → 差分
  ⇒ **差分区域 = 该关节及其下游零件**，不依赖任何相机/位姿假设。实测三个关节各改变画面 14~20% 像素。
  ⚠️ 但**给不出干净的单板四角**（肩座/平行杆/线缆连成一片）⇒ 只做**归属判定 + 粗定位**。
- ★ **「无位置回读」≠「没动作」**：MG90S 没有位置反馈，只说明**不能证明"到位"**，**不等于"没动"**。
- 当前状态：A1（骨架双杆）/ A2（`coupling.gain` → −0.81）**均判定为不改**；
  唯一入口是 A3 台面重测（`docs/hardware-measurement.md` §7）。

## 五、能力边界（必须在文档与 UI 里显式声明）

- **MG90S 无位置回读**：`OK SET` / `STATUS` / 后端 `joint_state` **全都在说"我打算去哪"**，
  没有一条能证明"它实际上在哪"。串口回执**原理上无法验证物理到位**，唯一外部地面真值是**相机**（D34）。
- Phase 11 误差面板反映的是**链路时延与限位截断**，不是"物理臂到位没有"；Phase 12 幽灵臂证明**链路走通了**。

## 六、机制速查

- 命令下发：store `commandJoints` → `transportBridge` **尾沿合并 30~33Hz** → `RobotTransport`。
- **回推只写 `actualJoints`**（回写 command 即无限回环）。
- 安全门：`mode === 'simulation'` **且**是真机链路（websocket + `device === 'serial'`）⇒ 拒发。
  mock / `device=sim` 照常放行（否则打死整条仿真闭环）。
- 时钟：一切时间相关逻辑走 `TimerLike`（生产 `realTimer` / 测试 `FakeTimer`）。
- 后端端口 **8090**（`/ws/joint`、`/healthz`）与 `MeArm-RemoteControl` 的 8080 舵机级摇杆**并存**。
- ★ **绝不要把外观层改回 `scene.environment`**：它全局（底座蓝板 ×3.3757），且 three 会用
  `scene.environmentIntensity` **覆盖** `material.envMapIntensity`（仅当 `material.envMap === null`）
  ⇒ 逐材质关反射**根本不生效**。逐材质方案（`plateEnvironment.ts` 程序化 PMREM，零外部 HDR）底座 ×1.0000。

## 六·五、运行环境（2026-09-14 一次性搭好）

| 用途 | 位置 | 版本 |
|---|---|---|
| 前端 | `frontend/node_modules`（`npm install` 产出，**不入库**） | Node 22.22.2（managed）· vite 8 · vitest 5 · tsc 5.9 |
| 后端 | Go 1.27 → `backend/bin/armpilot-backend.exe`（`/backend/bin/` 已 gitignore） | module `armpilot/backend`，唯一外部依赖 `gopkg.in/yaml.v3 v3.0.1` |
| Python（tools / 仿真 / pytest） | `~/.workbuddy/binaries/python/envs/default` | py3.13 · numpy 2.5 · Pillow 12.3 · pyyaml 6.0 · **mujoco 3.13** · pytest 9.1 · pyserial 3.5 |

- ★ **`backend/go.mod` 曾根本不存在**：父仓 `D:/user_project/git/ArmPilot/.gitignore` 里那条
  `*.mod*`（从 Linux 内核模板抄来）把 `go.mod` 一起吞了 ⇒ `go build` 报 `cannot find main module`。
  已加例外 `!go.mod`（**注意：`*.mod` / `*.mod*` 都必然匹配 `go.mod`，只能靠负向规则救**）。
  **`MeArm-RemoteControl` 是同一个坑**，其 `go.mod` 同样未入库。
  ⇒ 今后看到 `cannot find main module`，先跑 `git check-ignore -v <path>/go.mod`，别急着 `go mod init`。
- ★ `backend/config.yaml → device.mujoco.python` 是**本机绝对路径**，换机器必改
  （原值指向另一个用户目录 ⇒ mujoco 模式必然起不来）。
- ★★ **本机 `sort` 解析到 Windows `System32\sort.exe`**（不认 `-u`）⇒ `... | sort -u`
  **静默返回空**，表现为"按 PID 清理进程"的循环一个都没杀（我据此误以为清理成功，靠复查端口才发现）。
  去重用 `awk '!seen[$0]++'`，或走 `/usr/bin/sort`。**与 §三 的 `grep \b`、`taskkill //PID` 是同一类坑：工具语义没验证。**
- `start.bat` 前置检查刚好只依赖：`backend/bin/armpilot-backend.exe` · `backend/config.yaml` ·
  `config/robot.yaml` · PATH 上的 `node` · `frontend/node_modules/.bin/vite.cmd`。

## 七、外观 / 照片纹理轨（D56–D69）

- **定义**：`robot.yaml → links[].geometry.texture` 填 key（相对 `assets/textures/`），
  `textureRegistry.ts` 用 `import.meta.glob` 静态登记 ⇒ **新增纹理只丢文件，无需改代码**。
- ★★ **`RoundedBoxGeometry` 的 UV 是"每面各自铺满 [0,1]"、与长宽比无关** ⇒ 不能整块挂一张图，
  **必须按面用材质数组**（大面贴照片、其余四面保持板色）。实测 `materialIndex`：
  `0=+X 1=-X 2=+Y 3=-Y 4=+Z 5=-Z`，6 个 group 覆盖全部顶点。
  ⚠️ 它是**非索引几何**（`g.index === null`）⇒ group 的 `start/count` 是**顶点范围**，按索引遍历直接 `TypeError`。
- ★★ **`TextureLoader` 的 `flipY` 默认 `true`** ⇒ **图像顶行 ⇔ `uv_v = 1`**（不是 0）。
  **不报错、类型检查也查不出**的隐式默认值，漏掉会让所有 v 方向推理**整体反号**（我因此返工一次）。
  ⚠️ **定朝向不许靠目视 / 推断**（真实纹理近黑、特征模糊）⇒ 用**四象限探针纹理**做受控实验定案。
- **定位**：两个大面的 Δu/Δv 必有一个相反（盒体展开的必然）⇒ 同一张照片**必有一面需要镜像**。
- **两个必要守卫**：① **无 DOM 不加载**（`TextureLoader` 要 `document`，而 vitest 是 **node 环境**
  ⇒ 不加守卫会让 5 个几何/验收测试直接 `ReferenceError`）；② **key 未登记 ⇒ `console.warn` + 回退纯色**
  （贴图失败是**静默**的，只有测试能兜住）。守卫测试 `tests/unit/plateTexture.test.ts`
  （含"纹理长宽比必须与板大面一致"抓配错图）。
- **几何铁律**：plate 大面法向 = `size` 里**最小那一维**。坐标系基准：`base` 轴 = 世界 Z、
  `shoulder`/`elbow` 轴 = 世界 Y ⇒ **摆动平面 = X–Z** ⇒ **相机在 −Y 一侧（站侧面）**。
  ⚠️ **"转 base 90°"是错的**两处：① `base` 限位**只有 ±60°**，② 转向对分辨率**零和**（净 1.015）。
  ⚠️ **局部轴 ≠ 世界轴**：`upper_arm/forearm_link` 局部 Y → 世界 Y；`forearm_brace`/`tool_link`
  局部 Z → **世界 X（沿臂伸出方向）**。**别背轴向，拿实物转一圈找面积最大的面。**
- ★★ **动关节改变不了 `px/mm`，只由物理距离决定**（2026-09-13 定论）：摆动平面平行于像平面
  ⇒ 不改距离也不改投影长度，只改板在画面内的朝向。**要提高分辨率只能物理靠近相机。**
  `s` 由三个独立方法收敛到 **≈3.7–4.1**；中途得到的 `2.58` 是**欠定下的假解**（D59 ① 只对了一半）。
  相机上限 **1280×720**（`ffmpeg -f dshow` 直取，无需 OpenCV）。
- ★★ **「自动分割出单块板」不可行，白卡也救不了**：板/立柱/底座/舵机/控制板**全是同一种黑**
  （暗像素占整帧 25.7%），且板与板**由螺栓物理相连** ⇒ 不存在"只含一块板"的连通域。
  **白卡挡得住背景，挡不住结构性连通。**
- ★★ **目标板四角 = 模型投影给初值 + 图像梯度吸附**（`snap_quads.py`，D61 五）。
  单靠投影不够（整臂剪影 IoU 只 43%），单靠目视也不够（±15px）；合起来可行：
  实测边分 8.78→**19.30**（大臂）/ 4.65→**18.74**（小臂），**长宽比精确吻合真值**。
  ⚠️ 用此法前**必须先确认手性**（互为镜像时 IoU 永远上不去：30.2% → 43.2%）。
  ⚠️ 投影时**必须排除 `table_top`**（会撑成全屏矩形使 IoU 退化）；三项相乘目标函数**欠定**（曾 97.4% 假性满分）。
- ⚠️⚠️ **`mujoco.Renderer(model, height, width)` —— 参数顺序是 (height, width)**（D61 一）。
  传反不报错、不告警，却让**所有目视结论都错**（我曾据此得出"azimuth=270 一致"与"模型有系统性差异"，
  **二者均已撤回**；正确是 90）。⇒ **工具调用的参数语义必须先验证。**
- **真机臂板是镂空桁架**，`geometry.size` 只是**外接盒** ⇒ 照片纹理**必须做孔洞掩膜**（已默认开启）。
  **三层判据缺一不可**：① **连通性**（纯亮度会把板面上的**螺栓一起填掉**）② **形状**
  （只判面积会把被暗区包围的**线缆小段**留下）③ **色差** `R − B > 15`
  （暗线缆**亮度低于阈值**，只判亮度**永远去不掉**）。填充色取**实测板面中位色 `RGB(1,1,20)`**
  （近黑**带蓝**，不是纯黑）。实测纹理亮度**双峰**（板面 0~20 / 非板面 110~125），阈值 60 落在平台上。
- ★ **判「四角贴合吗」只有一条定义上无歧义的判据：纹理四条边的亮比例**（贴合 ⇒ 应接近 0）。
  实测框比板大约 30%（`upper_arm` 45/18/31/46%、`forearm` 58/34/62/20%，两块偏心方向还相反）。
  ⚠️ "框外/镂空"二分统计**不能单独判贴合** —— 镂空若开口到板端同样接触边界（**两种解释都能套上同一个数**）
  ⇒ **该换判据，不是换参数**。⚠️ 但四角偏差**不阻塞**：板面近黑、可见内容主要就是那几枚螺栓
  ⇒ **先贴上去看效果**，再决定要不要继续投入。
- ★ **判据只有在其前提成立时才有意义**（D60）：`project_plates.py` 的 `ramp`（边缘过渡）是在**投影框边缘**测的
  ⇒ 框定位错了它就是噪声（曾误报"失焦 10.78px"，目视真实边缘仅 ~2px）。**别拿下游指标反推上游前提。**
- **采集闭环**：`tools/capture_texture.py`（抓帧 → 三态判定 → 动作建议）→ `tools/make_texture.py`（校正 → 纹理）。
  判定工具**降采样**（只判几何），纹理必须用**全分辨率**原图；抓帧取**最后一帧**（启动会吐黑帧）。
- ★ **三态 `ok` / `reject` / `undecidable`。「判不了」≠「不合格」** —— 此时**禁止**输出任何"看起来像结论"的数字。
- ★ **指标要量在正确的对象上**：清晰度不能量板**内部**（均匀黑板对不上焦也是 0）⇒ 量**轮廓**梯度，
  且必须**归一化** `ramp_px = ΔI / (2·斜率)`（绝对值会被**光照**缩放）。上限 2.5。
- ⚠️ **禁用 PIL 的 `Image.transform(QUAD)` 做透视校正**：会引入 +12~17px 的**静默**平移（已加源码扫描测试防倒退）。
  用 `make_texture.py` 里的 `homography` / `_warp`。
- **取景**：HOME 位臂会被画面顶边裁掉 ⇒ **不要在 HOME 位出纹理**。用 `shoulder=45` 这类姿态
  （整臂完整入画，且板面仍正对相机）。
- **`RoundedBoxGeometry` 之外**：`jaw_link` 改为 `geometry.type: jaw` 的**参数化平面轮廓**（D66）。
  `ExtrudeGeometry` **不做布尔并集** ⇒ 齿轮盘与爪指**必须画成一条连续闭合折线**（否则 z-fighting / 露接缝）。
  三条派生关系在渲染层推、yaml 不配平：中心距 = `2 × 分度圆半径`；爪指中线内偏 `δ = 分度圆半径 − width/2`。
- ⚠️ **`details` 里的 plate 会被 `load_plate_sizes()` 枚举成待拍板，且用 `enumerate` 下标做别名键**
  ⇒ 补料必须用 **`box`**（不在枚举内）**且追加在 `details` 末尾**（插中间会把后续下标挤错，测试当场报错）。
- ⚠️ **不要改回 `scene.environment`**（见 §六）；`environmentIntensity` / `roughness` 实测**只缩放亮度、
  不改变细节能量**（`L` 动 4 倍时 `hp_std` 只动 11%）⇒ "再亮一点"换不来更多细节。
  **渲染层已到上限**：板面在 tile 里只用到 41 档码值，想更"像"必须改**拍摄端**（对板测光 / 补光）。

## 八、协作约定

- `git push` **由用户自行执行**；agent 只做本地 commit / diff。
- **破坏性操作先列清单确认**；建新目录先跑 `git check-ignore -v <path>/probe.txt` 探针。
- 每轮收尾：README 阶段表 + `docs/decisions.md` ADR + memory **同步更新**。
- **真机链路**：`backend/bin/armpilot-backend.exe -c config.serial.yaml` · **COM16 CH340 @115200 8N1** ·
  开机四舵机全 90°(= HOME)。调试入口 `tools/set_joints.mjs`（`--status` / `--home` / `name=value`），
  **别拿验收脚本当摇杆**。
  ★ **`hello` 里没有 `connected` 字段**（`protocol/serial-v1.md` §5.1 属**文档漂移**）⇒ 判真机只能看 `hello.device === 'serial'`。
  ★ **`hello` 与 `joint_state` 同时到达且 `hello` 在前** ⇒ 发送点必须在收齐 `joint_state` 之后，
  否则"保持不变"的关节会 fallback 到 `homePose` = **把臂拉回 home**。
  ★ `--settle` 的"保持不变"会让状态**跨实验累积** ⇒ 对比实验必须先 `--home`。
  ★ 打开串口会拉低 DTR 复位 ATmega328P（舵机弹回 90°）⇒ 实测脚本必须在**单次连接**内完成
  `[set → 稳定 → 抓拍]`。
