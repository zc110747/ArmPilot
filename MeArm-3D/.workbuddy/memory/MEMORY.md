# MeArm-3D · 项目长期记忆（curated）

> 只放**跨会话长期有效**的约定与铁律。每日过程写 `YYYY-MM-DD.md`。

## 一、唯一真值源与铁律

- **模型 / 标定 / 限位 / 零位的真值只有一份**：`config/robot.yaml`。前端与 Go 后端**都读它**
  （`hello` 带限位/标定做在线互检）。**禁止在代码里硬编码尺寸 / 角度 / 限位。**
- `backend/config.yaml` **只放运行参数**（端口 / 设备模式 / 模拟器参数），**禁写限位与标定**。
- **`mode`（simulation/real）不是 UI 开关**，它决定"要不要发给真实机械臂"。取值必须**校验通过才改**，
  失败要 pushLog 说明**为什么**和**怎么修**（拒绝不是目的，让用户知道在驱动谁才是）。见 D41 / D43。
- **机器人相关状态一律进 store**，组件不持局部副本（含 `teachTrack`）。理由之一很实际：
  e2e 探针要读原始数值，组件内状态只能读 DOM 的格式化文本。

## 二、验收四件套（每次改动必跑）

```bash
cd frontend
./node_modules/.bin/tsc -b --force        # 0 error
./node_modules/.bin/vitest run            # 全绿
./node_modules/.bin/vite build            # 并确认产物 __armPilot 命中 0
# e2e：必须自起干净 dev server（同一次工具调用内 起服务→跑→杀）
./node_modules/.bin/vite --port 5273 --strictPort & sleep 8
node tests/e2e/ui-smoke.mjs
```
- `npx <tool>` 会触发 WSL 黑名单 ⇒ **一律 `./node_modules/.bin/<tool>` 直调**。
- 跑 e2e 前**必须清掉带 `VITE_AUTO_CONNECT` 的残留 dev server**，否则"环境差异"被读成"代码回归"（D40/D43）。
- Python：系统 `python3` 无 numpy。用 `~/.workbuddy/binaries/python/envs/default/Scripts/python.exe`
  （numpy / PIL），并设 `PYTHONIOENCODING=utf-8`（否则中文乱码）。

## 三、测试与探针纪律（血泪换来的）

- **探针只准用 `matrixWorld` / DOM 层真实现象做证据，不许把数学再算一遍**：
  用 `FK(actual)` 证明"渲染了"是自证（挂错树 / 可见性误关 / 全透明都会让断言全绿）。
- **探针（含 `delete window.__xxx` 那一行）必须 `import.meta.env.DEV` 守卫**，
  否则字面量进生产包（D27 起的固定检查项）。
- **时间相关的多个读数必须合并在同一次 `cdp.evaluate`** —— 分两次 CDP 往返会让快收敛的量读到归零值，
  变成**间歇性失败**。
- **不许用推导量当独立判据**：`TransportStats.moving = lagDeg > eps`，用它判"卡死"永远得不到结论（D44）。
- **没有正面证据不下断言**（如"卡死"）：宁可停在保守态（`tracking`）。

## 四、测量与标定方法论（D37 / D47）

- **单批"最优" = 过拟合**。定 ROI / 阈值 / 骨架模型**必须跨 ≥2 批验证**。
- **"改模型让它对上"与"调参数让它对上"是同一类错误**。必须先证**测量可复现**
  （`tools/verify_calib_repro.py`：增益跨批极差 ≤5%），才有资格改 `robot.yaml`。
- **一次只改一条**，每改一条重跑判据；一次改多条 = 无法归因。
- 量包围盒必须**目视复核**（线缆 / 桌沿会污染 bbox）；**手入镜 / 相机位移**是头号污染源（D39/D40）。
- 当前状态：A1（骨架双杆）/ A2（`coupling.gain` → −0.81）**均判定为不改**；
  下一步唯一入口是 A3 台面重测（`docs/hardware-measurement.md` §7）。

## 五、能力边界（必须在文档与 UI 里显式声明）

- **MG90S 无位置回读**：`OK SET` / `STATUS` / 后端 `joint_state` **全都在说"我打算去哪"**，
  没有一条能证明"它实际上在哪"。串口回执**原理上无法验证物理到位**，唯一外部地面真值是**相机**（D34）。
- 因此 Phase 11 误差面板反映的是**链路时延与限位截断**，不是"物理臂到位没有"；
  Phase 12 幽灵臂证明**链路走通了**，不证明物理到位。

## 六、常用机制速查

- 命令下发：store `commandJoints` → `transportBridge` **尾沿合并 30Hz** → `RobotTransport`。
- 回推**只写 `actualJoints`**（回写 command 即无限回环）。
- 安全门：`mode === 'simulation'` **且**真机链路（websocket + `device === 'serial'`）⇒ 拒发。
  mock / `device=sim` 照常放行（否则打死整条仿真闭环）。
- 时钟：一切时间相关逻辑走 `TimerLike`（生产 `realTimer` / 测试 `FakeTimer`）。

## 七、协作约定

- `git push` **由用户自行执行**；agent 只做本地 commit / diff。
- **破坏性操作先列清单确认**；建新目录先跑 `git check-ignore -v <path>/probe.txt` 探针。
- 每轮收尾：README 阶段表 + `docs/decisions.md` ADR + memory **同步更新**。
