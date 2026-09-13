# ArmPilot · mARM 虚拟建模 + 真实机械臂同步控制系统

让**虚拟机械臂**与**真实机械臂**共享同一个 `RobotModel`，做到：

> 用户看到的虚拟 mARM，就是现实 mARM 的实时数字映射。

本仓库（`MeArm-3D`）是 ArmPilot 的**数字孪生前端 + 后端**；真实机械臂固件与服务在
同级仓库 [`MeArm-Device`](../MeArm-Device)（AVR / PlatformIO）与
[`MeArm-RemoteControl`](../MeArm-RemoteControl)（Go）中。

> **机构参数以真机实测为准**：`config/robot.yaml` 的关节角色、标定（offset/scale/reverse）、
> 限位与零位均由「相机照片 + 舵机逐度扫描」反解得到（2026-09-12），
> 推翻了最初按固件 `SERVO_LEFT/RIGHT` 推定肩/肘的假设 —— 详见
> [`docs/hardware-measurement.md`](docs/hardware-measurement.md) 与 `docs/decisions.md` D15–D17。

![ArmPilot 控制台 · HOME / RESET 位](docs/images/armpilot-console.png)

<sub>上图为 HOME 位（四舵机全 90° = 固件 RESET 位）的虚拟臂渲染，与实拍照片目视一致；J1 0.0° / J2 0.8° / J3 112.6° / Gripper 50.0°，TCP `112.0, 0, 93.8 mm`。</sub>

---

## 1. 架构

```
                      ArmPilot
                         │
                   RobotModel          ← config/robot.yaml（唯一数据源）
                         │
             ┌───────────┴───────────┐
             ↓                       ↓
       Virtual Robot             Real Robot
        (Three.js)              (mARM / AVR)
             │                       │
             └───────────┬───────────┘
                         │
                    Joint State
                         │
                       FK/IK
```

**正向（操作虚拟臂 → 控制真实臂）**

```
滑杆/鼠标 → Joint State → 安全检查 → Actuator Mapping(标定) → Transport → Go → Serial → AVR → 舵机
```

**反向（真实臂状态 → 同步虚拟臂）**

```
舵机 → AVR → Serial → Go → WebSocket → RobotState → Virtual Robot
```

> Phase 11 把这条反向链的**时延**显式化（链路误差面板：趋势 + 健康结论），
> Phase 12 把它**画出来**（实际臂幽灵 = 画面里第二条半透明臂，露出的部分就是滞后量）。
> ⚠️ 但 `actual` 在无位置回读的固件下**等于目标值**（下位机没有回读能力）——
> 它证明**链路走通了**，**不**证明物理到位（唯一的地面真值是相机，见 D34）。

**传输层分层（Phase 7–8 已落地，Phase 9 接真串口，Phase 13 可录制回放）**

```
store.commandJoints
   │ 尾沿合并 33ms（transportBridge）
   ▼
RobotTransport ──┬─ MockTransport        （Phase 7：纯虚拟闭环，不碰网络）
                 └─ WebSocketTransport    （Phase 8：JSON ↔ 真实 Go 后端）
                          │
                          ▼
     Go: wsserver → controller → device（sim | serial | mujoco）
                          │  JR / OK JR / STATE（文本行）
                          ▼
              内置 sim「假固件」（Phase 8）  |  AVR（Phase 9）  |  MuJoCo 物理仿真（MuJoCo 轨 ✅）
```

| 环节 | Phase 7 | Phase 8 | Phase 9 |
|------|---------|---------|---------|
| 前端 ↔ 传输 | `MockTransport` | **`WebSocketTransport`（JSON）** | 同 Phase 8 |
| 传输 ↔ 机械臂 | —（同进程） | **内置 sim 假固件，走真实字节流** | 串口 115200 + ACK 门控 |
| 标定 / 限位来源 | `robot.yaml` | **同一份文件**（后端也读它，`hello` 在线互检） | 同 |

### MuJoCo 物理仿真轨（末端第三实现）

在 `device.Device` 上再挂一个实现：**MuJoCo 刚体动力学**。因为 `device.Device`
只有 7 个方法，所以新增它**没有改动 WebSocket 层 / controller / 协议 / 前端任何一行**。

```
Go: device.MujocoDevice ──stdio(同一套 JR/OK JR/STATE 文本协议)──▶ python simulation/mujoco/server.py
                                                                          │
                                                              MuJoCo 1kHz 物理 + 100Hz 控制环
```

> ⚠️ **诚实声明（spec §37）**：当前是 **Level 3→4 参数化物理仿真**，
> `config/physics.yaml` 里的值全部是**公开值 / 估算值**，**不是**对本台 meArm 的标定模型。
> `calibration.calibrated` 恒为 `false`，且由测试强制（见 `docs/decisions.md` D52）。
> 详见 [`simulation/README.md`](simulation/README.md) §0。

## 2. 技术栈

| 层 | 选型 |
|----|------|
| 前端 | React 19 · TypeScript · Vite · Three.js · React Three Fiber · @react-three/drei · Zustand |
| 后端 | **Go 1.21+（自包含 module `armpilot/backend`）· 标准库 RFC6455 WebSocket · HTTP `/healthz`**（Phase 8 ✅） |
| 真实臂 | AVR (ATmega328P) · 串口 115200 8N1（Phase 9） |
| **物理仿真** | **MuJoCo 3.13（Python，MJCF 由 `config/robot.yaml` + `config/physics.yaml` 生成）**（MuJoCo 轨 ✅） |
| 测试 | Vitest（单元 / 验收）· `go test`（后端）· pytest（物理仿真）· 零依赖 CDP e2e（无头 Edge/Chrome） |

## 3. 目录结构

```
MeArm-3D/
├── config/robot.yaml             # ★ 唯一模型定义（links / joints / actuators / home / tcp）
├── config/physics.yaml           # ★ MuJoCo 物理参数（纯物理量，全 SI；**禁写限位与标定**）
├── config/baseline-kinematics-physics.json  # ★ 真值冻结基线（语义核心哈希；守卫见 D55）
├── simulation/                   # ★ MuJoCo 物理仿真后端（device.Device 第三实现）
│   ├── README.md                 #   ★ 怎么跑 / 判据纪律 / 验收数据 / Level 声明
│   └── mujoco/
│       ├── gen_model.py          #   robot.yaml + physics.yaml → mearm.xml（MJCF 是产物，禁手改）
│       ├── mearm.xml             #   生成的 MJCF（nq=4 nbody=8 ngeom=33 nu=4）
│       ├── units.py              #   单位 + **角度语义**（elbow 绝对角 ↔ hinge 局部角）单点换算
│       ├── model.py              #   MeArmSim：reset / step / settle / state / gravity_torque
│       ├── limits.py             #   限位校验（与 Go Validate 同语义、同文案）
│       ├── server.py             #   无头设备服务（stdio 上跑 arm-device 协议）
│       ├── run.py                #   独立 Viewer（spec §27：FPS / sim time / 关节角 / TCP / 接触数）
│       ├── record.py             #   JSONL / CSV 记录（spec §34，不落数据库）
│       ├── calibrate.py          #   标定接口（spec §38）
│       └── fkref.py              #   参考 FK（独立实现，**仅用于验收**）
├── docs/
│   ├── coordinate-system.md      # ★ 坐标系、单位、运动学链规则、标定表（权威文档）
│   ├── model-structure.md        # ★ 显式几何（plate/servo/details）与运动学的边界
│   ├── hardware-measurement.md   # ★★ 真机实测记录：角色映射 / 绝对角解耦 / 标定 / 不确定度
│   ├── ARCHITECTURE_ANALYSIS.md  # ★ MuJoCo 轨 Phase 1：自由度清点 / 五种角度对照 / 接入方案
│   ├── decisions.md              # 设计决策 ADR（D1–D65；D48–D54 = MuJoCo 物理轨 · D55 = 真值冻结 · D56 = 纹理校正 · D57 = 采集判定 · D63 = 按面材质贴图 · D64 = 暗端材质与光照 · D65 = 验收纪律）
│   ├── texture-capture-guide.md  # ★ 图像采集指南（拍哪块板 / 大面朝向 / 采集闭环 / 四项硬性要求 / 自查清单）
│   └── images/                   # 界面截图（armpilot-console.png 由 e2e 自动重出；
│                                 #   armpilot-phase11-13.png 由 tests/e2e/screenshot.mjs 出）
├── protocol/serial-v1.md         # ★ 串口 / WS 协议基线（§4 固件侧待 Phase 9；§5 上位机侧 Phase 8 已实现）
├── tools/                        # ★ 真机实测工具链（Python 3 + numpy，独立于前端）
│   ├── mearm_hw.py               # 串口控制 + 相机抓拍（单进程，避开 DTR 复位陷阱）
│   ├── analyze_sweep.py          # 逐度扫描图 → 白底暗件分割 → 面积/重心/最高点/红绿差分
│   ├── segment_arm.py            # 公共静止区自动分段：底座 / 大臂 / 小臂 + PCA 定方向
│   ├── fit_pivot.py              # 圆拟合枢轴（Kasa）+ 爪尖极角转角 + 不动区质心互证
│   ├── fit_pose.py               # ★★ FK 骨架 ↔ 实拍照片拟合（对称 Chamfer + Hooke-Jeeves）
│   │                             #    `--rod-gap MM` 可切换小臂单折线 / 平行双杆骨架（默认 0，向后兼容）
│   ├── verify_pose.py            # ★ 相机反解关节角 + 与期望值比对；`--selftest` 含自检 C（双杆模型表达力）
│   ├── verify_calib_repro.py     # ★A1 判定：跨批 × 底座掩膜 × 骨架杆距的增益散布矩阵 + 极差 ≤5% 判据
│   ├── verify_serial_e2e.mjs     # Phase 9 真机端到端（opt-in）
│   ├── cam_stability.py          # 相机/台面稳定性抽检
│   ├── freeze_baseline.py        # ★ 冻结/校验运动学+物理真值（两级判据；改外观放行，改真值报错）
│   ├── make_texture.py           # ★ 实拍照片 → 板件纹理（PCA 估四角 + homography 校正 + 归一化；--selftest）
│   ├── capture_texture.py        # ★ 采集判定：摄像头抓帧 → 三态判定(ok/reject/undecidable) → 给出该往哪动
│   └── ws_probe.mjs · lan_e2e_probe.mjs   # WS / 局域网链路探针
├── assets/textures/mearm/        # ★ 板件纹理资产（raw/ = 原图 · tiles/ = 校正后的贴图）
├── frontend/
│   ├── src/robot/
│   │   ├── model/             # RobotModel · Link · Joint · Actuator · Pose · RobotState · RobotCommand
│   │   │                      # linkFeedback（Phase 11 误差语义：趋势 / 健康判定）· teachTrack（Phase 13 轨迹）
│   │   ├── kinematics/        # coordinate（转换层）· transform（矩阵）· fk · ik（逆解）
│   │   ├── interaction/       # dragPlane：拖动平面 + 射线求交（纯数学、零依赖）
│   │   ├── calibration/       # 关节角 ↔ 舵机角 标定
│   │   ├── teach/             # TeachPlayer（Phase 13 回放器，可注入时钟；复用 TimerLike 约定）
│   │   └── transport/         # RobotTransport 抽象 · MockTransport（有限角速度/延迟/丢帧/限位拒绝）
│   │                          # WebSocketTransport（Phase 8）· wsProtocol（纯函数编解码 + 链路精度）
│   │                          # socket（SocketLike 可注入）· timer（可注入时钟）
│   ├── src/components/        # RobotScene（buildRobotObject3D · DragHandle · ActualGhostArm · TestProbe dev探针）
│   │                          # RobotControl（JointControl · TargetControl · TeachPanel · ConnectionControl）
│   │                          # StatusPanel（StatusPanel · ErrorPanel Phase 11 · LogPanel）
│   ├── src/store/             # Zustand 唯一状态仓库 · transportBridge（尾沿节流 / 回推落地 / 回环打破 / 重连补发）
│   └── tests/                 # unit / acceptance / e2e
│                              #   tools/kinematics-bridge.mjs = ★ 把真实 ik.ts / fk.ts 暴露成 CLI
│                              #     （Vite SSR 加载器；供 tests/sim/test_ik.py 当独立裁判用，见 D53）
├── backend/                   # ★ Phase 8：关节级 WebSocket 服务（自包含 Go module）
│   ├── main.go                # 组装：cfg → robot.Model → device → controller → wsserver
│   ├── config.yaml            # **只放运行参数**（端口 / 设备模式 / 模拟器参数），禁写限位与标定
│   ├── internal/robot/        # 读 robot.yaml；关节↔舵机换算；限位校验（唯一"真值"入口）
│   ├── internal/protocol/     # JSON / JR / OK JR / STATE / ERR 编解码（不认识机械结构）
│   │                          #   + simulation_mode（spec §25：只加一个可选字符串，前端零改动）
│   ├── internal/controller/   # ★ 唯一"懂机械臂"处：ACK 门控 · latest-wins · 标定核对 · 状态发布
│   ├── internal/device/       # sim.go（假固件）· serial.go（真串口）· mujoco.go（★ Python 子进程）
│   ├── internal/wsserver/     # 标准库 RFC6455 服务端 · 路由 · 广播 · 两层心跳
│   └── README.md              # 架构图 · 与 MeArm-RemoteControl 的分工 · 测试矩阵
└── tests/sim/                 # ★ MuJoCo 轨验收（pytest，106 项）
    ├── harness.py             #   共用采样/求值工具（FK 与 IK 两条判据不各写一份）
    ├── ikbridge.py            #   前端运动学 CLI 桥的 Python 门面
    └── test_*.py              #   模型 / 重力 / 执行器 / 限位 / 碰撞 / 协议 / FK / IK / 系统级
```

## 4. 快速开始

### 一键启动（推荐）

根目录下的 `start.bat` 会拉起后端 + 前端，并让页面**自动连上后端**（不再是默认的
浏览器内 Mock 仿真）：

```bat
start.bat              :: SIM 模式（默认，不碰硬件）
start.bat --real       :: REAL 模式（config.serial.yaml，会真的动舵机）
start.bat --help       :: 用法
```

它做的事：检查 `backend/bin/armpilot-backend.exe` / `config/robot.yaml` / node 是否存在、
探测并（经你确认后）清理占用 8090 / 5273 的残留进程、按模式起两个窗口、
打印本机与局域网访问地址。

`start.bat` 会往前端注入两个环境变量，这正是"一键启动后页面自动进入 ws 模式"的实现：

| 变量 | SIM 模式 | REAL 模式 | 作用 |
|------|---------|----------|------|
| `VITE_AUTO_CONNECT` | `ws` | `ws` | 页面挂载后自动连 WebSocket 后端（而非浏览器内 Mock） |
| `VITE_AUTO_REAL` | 不设 | `1` | 连接成功后自动切到 Real Robot（带校验，见下） |

> ⚠️ **REAL 模式下若后端链路末端不是 serial**（例如没插机械臂、串口号不对），
> 自动切换会**被拒绝**（按钮保持 Simulation），日志给出拒绝原因与实际末端。
> 这是刻意的：静默降级比明确报错更危险。
>
> 同理，**SIM 模式下手动点 Real Robot 也会被拒绝** —— 后端末端是 `sim` 就不该显示
> "正在驱动真实机械臂"。判据见 ADR **D43**。
>
> 不想要自动连接时，直接 `cd frontend && npm run dev` —— 未注入环境变量时行为完全不变
> （仍停在 Mock，等你手动点 Connect）。

### 手动启动

```bash
# 前端
cd frontend
npm install
npm run dev            # 本机 http://localhost:5273；局域网 http://<本机IP>:5273
npm run typecheck      # tsc -b，零错误
npm test               # vitest（单元 + 验收），233 项
npm run test:e2e       # 真浏览器冒烟（需先 npm run dev；见下方参数说明）
npm run build          # 生产构建

# 后端（Phase 8）
cd ../backend
go build -o bin/armpilot-backend.exe .
./bin/armpilot-backend.exe -c config.yaml     # 监听 0.0.0.0:8090，端点 /ws/joint
curl http://127.0.0.1:8090/healthz            # {"ok":true,"device":"sim","linked":true,"state":{…}}
go test ./...                                 # 56 项
```

> **局域网访问**：Vite 已设 `server.host: true`、后端已监听 `0.0.0.0:8090`，
> 同一 WiFi 下的手机/平板打开 `http://<本机IP>:5273` 即可。
>
> 前端**不写死** `ws://localhost:8090` —— 那样局域网访问时浏览器会把 `localhost`
> 解析成**访问者自己那台设备**，表现为"页面能开、但一直重连"。实际按
> `window.location.hostname` 推导（见 `ConnectionControl.tsx` 的 `getDefaultWsUrl`）：
> 本机访问 → `ws://localhost:8090`，局域网访问 → `ws://<本机IP>:8090`。
> 需要指向别的后端时用 `VITE_WS_URL=ws://host:port/ws/joint` 显式覆盖。
>
> 首次从局域网访问若连不上，检查 Windows 防火墙是否放行
> `node.exe` / `armpilot-backend`（本机已有这两条入站规则，生效于**公用**配置档）。

> e2e 脚本（零依赖 CDP，直接驱动无头 Edge/Chrome）参数为
> `node tests/e2e/ui-smoke.mjs [url] [debugPort] [screenshotPngPath]`；
> 传第 3 个参数时会**在 HOME 位自动重建** `docs/images/armpilot-console.png`：

```bash
node tests/e2e/ui-smoke.mjs http://localhost:5273 9333 ../docs/images/armpilot-console.png
```

> **e2e 会自动拉起后端**（`backend/bin/armpilot-backend.exe`）并在结束后清理进程。
> 若 8090 上已有实例，则复用该实例并**跳过**"断线重连"子项（脚本杀不掉别人的进程）。

### 驱动真实机械臂（Real Robot）

**一条命令**（`start.bat` 会做完下面手动三步的全部事情）：

```bat
start.bat --real
```

确认后端窗口出现 `[serial] 已连接 COM16 @ 115200 8N1`，
页面提示条显示「★ 正在驱动真实机械臂（链路末端 serial）」即可。
串口号在 `backend/config.serial.yaml` 的 `device.serial.port` 里改。

<details>
<summary>手动三步（不用脚本时）</summary>

```bash
# 1. 用真机配置启动后端（config.yaml 是 sim，不会碰硬件）
cd backend && ./bin/armpilot-backend.exe -c config.serial.yaml

# 2. 前端连上这个后端
cd frontend && npm run dev
#    在 Connection 面板切到 WebSocket → 输入 ws://localhost:8090/ws/joint → Connect

# 3. 在「关节控制」卡片点 Real Robot
```

</details>

**「命令去向」提示条的含义**（`data-testid="mode-routing"`）：

| 提示 | 含义 |
|------|------|
| 纯仿真（未连接…） | 没连传输，命令只改虚拟臂 |
| 已拦截：Simulation 模式下不下发给真机 | **连着真机但模式是 Simulation** ⇒ 安全门生效，真机不动 |
| ★ 正在驱动真实机械臂（链路末端 serial） | 真机模式 + 真机链路，命令会下发给硬件 |
| Real 模式下…非真机 / 连接的是 Mock / 末端未知 | 模式选对了但链路不对，检查后端是否用 `config.serial.yaml` 启动 |

> ⚠️ **真机没有位置反馈**（无编码器）：界面上的 `Actual` 是**固件内部目标值反算**，
> 不代表已物理到位。唯一的外部真值是相机 —— 见下方 Phase 9 验收与 `tools/verify_pose.py`。

### 用 MuJoCo 物理仿真作末端（MuJoCo 轨）

不接硬件，也不走"纯运动学回显"，而是让末端变成**真实的刚体动力学**：

```bash
# 1) 装依赖（隔离环境）
~/.workbuddy/binaries/python/envs/default/Scripts/python.exe -m pip install mujoco pyyaml pytest

# 2) 起一个 MuJoCo 末端（python 子进程由 Go 侧拉起，stdio 上跑同一套文本协议）
cd backend && ./bin/armpilot-backend.exe        # ★ 需先把 config.yaml 的 device.mode 改成 "mujoco"
curl http://localhost:8090/healthz              # {"device":"mujoco","linked":true,"ok":true,...}

# 3) 前端连上即可（Connection 面板 → ws://localhost:8090/ws/joint → Connect）
#    前端零改动：mujoco 按"仿真"放行安全门，hello.simulation_mode = "mujoco"

# 4) 独立 Viewer（不进 Web，直接看动力学）
<python> simulation/mujoco/run.py --demo
```

> ⚠️ **当前是 Level 3→4 参数化物理仿真**，`config/physics.yaml` 的值全是公开值/估算值，
> 不是对本台 meArm 的标定模型。详见 [`simulation/README.md`](simulation/README.md) §0。
> 改 `config/robot.yaml` 的几何后**必须重跑** `python simulation/mujoco/gen_model.py`。

## 5. 阶段进度

| Phase | 内容 | 状态 | 验收证据 |
|-------|------|------|----------|
| 1 | RobotModel（唯一数据源） | ✅ | 单测 10 项；改 yaml 单字段即生效 |
| 2 | Three.js 3D 机械臂 | ✅ | 无头浏览器截图 + e2e 渲染断言；**按实物 meArm 参数化重做外观**（件色现为**近黑** + 4 舵机 + 轴销，见 `docs/model-structure.md`；件色于 2026-09-13 由蓝改近黑、视口由深色改灰底，见 ADR **D68**）；**舵机布置按真机实测修正为 S9 底座 / S7 肩 / S8 肘 / S6 夹取** |
| 3 | FK | ✅ | **FK↔Three.js 最大误差 8.673e-14 mm**（要求 < 0.1） |
| 4 | Joint Control | ✅ | 单测 7 项 + e2e 滑杆交互 |
| **4.5** | **真机参数实测（相机反解机构）** | ✅ | 白底分割 + 舵机逐度扫描 + FK 骨架拟合：修正 S7/S8 角色映射、确证**小臂绝对角（平行四连杆，耦合 gain=-1）**、反解标定/限位/零位并写入 `config/robot.yaml`（见 `docs/hardware-measurement.md`、`docs/decisions.md` D15–D17） |
| 5 | IK（XYZ → J1/J2/J3） | ✅ | **FK(IK(XYZ)) 2000 组随机位姿最大残差 1.401e-13 mm**；错误码 `OUT_OF_WORKSPACE` / `JOINT_LIMIT`；多解 `elbow-up/elbow-down/nearest`（默认就近）；几何量全部从模型求导，改 yaml 即生效（见 `docs/coordinate-system.md` §3.1、D18/D19） |
| 6 | XYZ / 鼠标拖动末端 | ✅ | XYZ 直输 + **鼠标真实拖拽**（e2e 用 CDP 派发真实鼠标事件命中场景把手，Δ 17.47mm）；三种拖动平面 xy/xz/camera 在 pointerdown **冻结**；**越界不钳位**（关节逐位不变）；400 点轨迹穿越工作空间边界验收（见 `docs/coordinate-system.md` §3.2、D20–D22） |
| 7 | MockTransport 闭环 | ✅ | 完整双向闭环（命令 → 尾沿节流 → Mock → 回推 → Actual）；Mock **如实模拟舵机有限角速度 / 传输延迟 / 丢帧 / 限位拒绝**（非等值回显）；回推**只写 Actual**（回环打破，400 点轨迹引用从未改变）；Connection 面板可实时调参（见 `docs/coordinate-system.md` §3.3、D23–D26） |
| 8 | **Go WebSocket** | ✅ | **后端独立 module `backend/`（8090）+ 内置「假固件」sim**：命令走 `JSON → JR 文本 → 舵机角 → 反算关节角 → STATE` 真实往返，非等值回显；`OK JR` **只做标定核对不发布状态**；ACK 门控 + latest-wins；`hello` 带模型真值在线互检；两层心跳；断线指数退避重连**并补发当前命令**。`go test` 56 项 · 前端新增 66 项单测（`wsProtocol` 25 / `WebSocketTransport` 30 / 接线验收 11）· e2e 新增 21 项真实 WS 端到端（见 `docs/coordinate-system.md` §3.4、`protocol/serial-v1.md` §5、D27–D33） |
| **9** | **Serial（真机）** | ✅ | `internal/device/serial.go` 落地真串口（Windows 非重叠 I/O，**不用 `bufio`**）；Uno DTR 复位静默窗口 `connect_settle_ms=2600` + 暖机包。**真机端到端闭环实测 PASS 18 / FAIL 1**：`hello=serial` · `homePose` 与 `robot.yaml` 逐位一致 · 7 步链路回推 `max\|Δ\| ≤ 0.004°` · 相机反解重复性肩 `0.26°`/肘 `0.01°`。见 `tools/verify_serial_e2e.mjs`、`docs/decisions.md` D34–D36 |
| 10 | Real Robot | ✅ | 机构角色 / 标定 / 限位 / 零位**已实测就绪**（Phase 4.5 + `config/robot.yaml`）；Serial 已落地 ⇒ 浏览器拖动能**真实驱动物理机械臂**。**2026-09-12 修复 mode↔transport 联动缺口**：`Real Robot` 按钮原先只改 UI 样式、命令照样走当前 transport（"点了真机不动 / 切回仿真仍在动真机"），现补准入校验 + 安全门 + 去向提示（ADR **D41**）。⚠️ 相机验收已测出**肩标定增益偏差 −13.1%**（肘 +1.4% 已证实），需按锁死曝光重布台面后重测 |
| **10.5** | **一键启动 `start.bat`** | ✅ | 根目录 `start.bat`：前置检查（backend exe / robot.yaml / node）、端口探测+确认清理（8090/5273）、按模式起前后端、打印本机+局域网地址。**关键**：注入 `VITE_AUTO_CONNECT=ws`（+ `--real` 时 `VITE_AUTO_REAL=1`）让页面**自动连后端并切 Real Robot** —— 原先页面默认停在 MockTransport 且不会自动连接，"脚本起好了但只动仿真臂"（ADR **D42**）。`npm run dev` 不注入，手动调试行为不变 |
| **10.6** | **Real Robot 准入改为"拒绝"** | ✅ | 用户报障「前端显示 Real 模式但后端末端是 sim」。根因：`setMode('real')` 把 `set({ mode })` 写在准入校验**之前**，校验只 pushLog、状态照改（D41 只修了一半，且旧测试还把该行为固化成契约）。现改为**校验全通过才切换**，否则保持 Simulation 并说明原因；`device` 未知（hello 未到）也拒绝，`useAutoConnect` 相应改为等 device 到达再切（ADR **D43**） |
| 11 | Real Feedback（**链路误差反馈面板**） | ✅ | 逐关节**带符号偏差条** + 误差**趋势 sparkline** + 一句**健康结论**（已到位 / 跟踪中 / 异常）。判据全在 `@robot/linkFeedback`（纯函数，19 项单测）。**关键**：`TransportStats.moving` 是 lag 的同义重写（`moving = lag > eps`），拿它判"是否在追"**永远推不出"卡死"** —— 判据只能从时间序列得出，且趋势用**四分位中位数**（首末值/均值会被单帧尖峰翻面）。纪律：没有正面证据不下"卡死"断言，`unknown`/`shrinking` 一律判 `tracking`（ADR **D44**） |
| 12 | 虚拟 / 真实同步（**实际臂幽灵**） | ✅ | 场景同时渲染**两条臂**：主臂跟 `commandJoints`（意图）、半透明幽灵跟 `actualJoints`（现状），未被遮挡时露出的就是**滞后量** —— 比读数表更快。幽灵用**半透明**而非醒目色（本项目「无装饰色」，信号是位置分离本身）；`depthWrite=false` 防半透明脏面。e2e **取渲染后 `matrixWorld`** 而非重算 FK —— 挂错父节点/可见性误关/材质全透明都会让画面空掉而断言全绿（ADR **D45**） |
| **13** | **示教录制 / 回放** | ✅ | `Record / Play / Pause / Stop / Clear / Export / Import`。录的是 **`commandJoints`**（不是 `actual` —— 那会把链路时延焊进轨迹）；回放**复用 `store.setCommandJoints()`**，于是尾沿节流与安全门自动生效，**不另开直发通道**。采样 20Hz + 静止去抖 0.5° + 上限 2000 帧**拒绝新帧**（不丢开头）+ 停录**强制补末帧**（否则轨迹终点 ≠ 臂当前位置）。回放**关节空间线性插值**保证命令连续，但结束时刻**精确取末帧** ⇒ 「回放终点 == 录制终点」是逐值不变量（e2e 以 1e-9 断言，ADR **D46**） |
| **A3** | **标定精度闭环（待操作者执行）** | ⏳ | A1（骨架改双杆）/ A2（`coupling.gain` → −0.81）**双双判定为"不改"**：自检 C 显示双杆改善仅 0.3° 量级且**无单调趋势**；实拍矩阵 `rod_gap=4` 局部"修好"、`rod_gap=10` 让 `dir_S7` 崩到 **−46.3%**。`tools/verify_calib_repro.py` 判定跨批极差 **肩 10.8% / 肘 52.9% > 5% 容差 ⇒ 测量本身不可复现**，此时把偏差归因给模型或标定表都不成立。**台面锁变量清单 + 采集 + 判据**见 `docs/hardware-measurement.md` §7（ADR **D47**） |

### MuJoCo 物理仿真轨（M1–M10，spec §39 的独立编号）

> 这是一条**与上表并列的独立轨**（原 spec 自己编了 Phase 1–10），
> 编号加 `M` 前缀以免与上面的 Web / 真机轨混淆。

| Phase | 内容 | 状态 | 验收证据 |
|-------|------|------|----------|
| **M1** | 架构勘察 | ✅ | [`docs/ARCHITECTURE_ANALYSIS.md`](docs/ARCHITECTURE_ANALYSIS.md)（14 节，**数字全部实读**）：可动 DOF = 4 / 定位 DOF = 3；五种角度（Servo / Joint / Physical / UI / IK）对照表；FK↔刚体树对应表；三个接入方案对比（推荐 A = 第三 `device.Device`）；7 条风险 R1–R7（含 `docs/model-structure.md` §4 把 S8/S7 写反 —— **已在本轮修正**） |
| **M2** | MJCF 刚体树 | ✅ | `gen_model.py` 从 `robot.yaml` + `physics.yaml` 生成 `mearm.xml`（8550 B · nq=4 nv=4 nbody=8 ngeom=33 nu=4 nexclude=5 · 总质量 0.2173 kg）。**MJCF 是产物，禁手改**；VISUAL / COLLISION / PHYSICS 几何三类分离，几何优先 primitive |
| **M3** | 质量 / 惯量 / 重力 | ✅ | 重力对照实验：无驱动 3 s，有重力 Δ 肩 35.96° / 肘 48.10°，**关重力 Δ 全为 0**；稳态误差与「重力矩 ÷ kp」自洽（肩 0.36° = 0.0316 ÷ 5.0） |
| **M4** | 执行器 / 位置控制 / 限位 | ✅ | 单/多关节控制正确；`elbow` 绝对角语义（局部角 85.311° = 125.977 − 40.666）；阶跃 60° 在 0.1 s 内只转 0.83°（速率限制生效）；**限位四方一致**：MuJoCo hinge range 刻意外扩 padding 2° ⇒ **不是限位真值**，把关人是 Go controller + `limits.py`（ADR **D49**） |
| **M5** | 碰撞 / 摩擦 / 自碰撞 | ✅ | HOME 位 `ncon=0`（修掉"立柱戳在地上"与"爪伸出 TCP 34 mm"两个伪接触）；下压时 `jaw↔table` 且**无穿透**；5 对相邻连杆**实测重叠**（−20.0 / −1.576 / −7.123 / 0 / −5.0 mm）⇒ exclude 是必要的；摩擦单一组合来自配置；接触下 5 s 漂移 0.0013°。**纪律**：「有接触记录」和 `dist` 都不是证据，必须看 `qfrc_constraint` / 满力矩 / 对照位移（ADR **D51**） |
| **M6** | Python Backend | ✅ | `server.py`（无头设备服务，协议逐字节正确、实时倍率 1.000）· `run.py`（Viewer，统计行含 FPS / sim time / 关节角 / TCP / 接触数）· `record.py`（JSONL/CSV）· `calibrate.py`（`--show/--template/--apply`） |
| **M7** | Go 侧接入 | ✅ | `internal/device/mujoco.go` 起 Python 子进程；`exec.LookPath` 失败与 `No module named 'mujoco'` 都给明确修复指引；**启动握手 `PING → OK PING`**（否则"解释器不对 / 未装 mujoco / XML 编译失败"会表现成"设备可用但永远没回执"）。`go build`/`vet`/`test` 全绿；`healthz` = `"device":"mujoco","linked":true`；WebSocket 全链路探针 PASS（62 帧 `joint_state`，shoulder 1.21° → 20.49° **渐进收敛**） |
| **M8** | `simulation_mode` | ✅ | spec §25「不要重新设计协议」：`ServerMessage` **只加一个可选字符串** `simulation_mode`（`omitempty`），`SimulationModeFor()` 映射 `sim→kinematic` / `mujoco→mujoco` / `serial→real`。**不改消息类型、不改 `joints` 结构 ⇒ 前端零改动**（ADR **D48**） |
| **M9** | FK / IK 一致性 | ✅ | **FK**：参考实现 `fkref.py`（独立读 `robot.yaml` 原始几何）vs MuJoCo —— 零位 / HOME / 120 随机 + 256 限位角点，`max\|Δ\| = 7.1e-14 mm`。**IK**：**加载真实 `ik.ts`**（Vite SSR 桥，不用 Python 重写）—— 120 随机可达点成功率 **120/120**、`max = 1.137e-13 mm`；顺带把**前端 `fk.ts` 与 MuJoCo** 也对撞（376 点，`max = 1.137e-13 mm`）。两条**机构学结论**被算术+实测双重钉住：`elbow-down` 支恒不可行；**可达工作空间内锥为空**（最小水平半径 65.62 mm ⇒ 真机够不到自己的中轴线）（ADR **D49/D53**） |
| **M10** | 系统级集成 + 文档 | ✅ | 三层时间步解耦（`step(1)×10 ≡ step(10)` 按位 · `fps=5 vs 240` qpos 按位相同）· 复位可重复 + 热启动不漏 · **跨进程确定性**（`run.py --demo` 跑两遍，12 行数值载荷逐字相同）· 数据记录回读 · Test D 快速运动峰值 4.92 rad/s（限速 10）· **Level 声明机器可检查**（ADR **D52**） |

**MuJoCo 轨实测汇总**：pytest **106 passed**（10 文件）· `go test` 全绿 · vitest **293 passed** · `tsc -b` 0 error ·
`vite build` OK（JS 产物 `__armPilot` 0 命中）。

### 本阶段明确**不实现**

> MuJoCo 轨把「MuJoCo」从"不实现"移到了"已实现"，但**只实现了物理仿真本身** ——
> 下面这些**仍然不做**（spec §30 明令禁止）。

AI · 机器学习 · 强化学习（PPO/SAC）· 自训练 · 视觉识别 · 摄像头 · 目标检测 · 数据集 ·
模型训练 · ONNX · 语音控制 · 动作学习 · **策略训练** · **自动数据采集** · Sim2Real 实机迁移。

架构已按 spec §三十八 预留 `RobotCommand` / `RobotState` / `RobotModel` / `RobotTransport`
四个扩展边界；MuJoCo 轨另按 §29/§31 预留了 Sim2Real 接口
（`state()` / `set_target_joints()` / `reset()` / `step()` ↔ `Observation` / `Action` / `Reset` / `Step`）。

### Phase 11–13 界面（同框可见三个新面板）

![ArmPilot 控制台 · Phase 11–13](docs/images/armpilot-phase11-13.png)

<sub>左侧 3D 视口右上角新增 **Actual Arm** 勾选框（Phase 12 幽灵臂开关）。右侧栏自上而下：
**关节控制** → **末端目标** → **示教 · TEACH**（Phase 13）→ **连接 · TRANSPORT** →
**状态 · STATUS** → **链路误差 · LINK ERROR**（Phase 11：Command→Actual 逐关节偏差条 +
误差趋势 sparkline + 健康结论）→ **模型 · ROBOT MODEL**。</sub>

## 6. 当前验收数据（Phase 1–13 + MuJoCo 轨 M1–M10）
```
类型检查      tsc -b                    0 error
单元测试      vitest run                293 / 293 PASS（21 文件；含 13 项几何回归 · 19 项 IK · 19 项拖动平面 ·
                                       13 项目标语义 · 30 项 wsProtocol · 33 项 WebSocketTransport ·
                                       17 项自动连接意图与切换时序 · 12 项 mode↔transport 联动 ·
                                       19 项链路误差语义 · 4 项幽灵臂渲染 · 20 项示教轨迹 · 17 项示教回放）
后端单测      go test ./...             56 / 56 PASS（5 包：robot · protocol · device · controller · wsserver）
                                        + go vet 干净 · gofmt -l 无输出
浏览器 e2e    node tests/e2e/ui-smoke   88 / 88 PASS（含 25 项 Phase 8 真实 WebSocket 端到端
                                        + 4 项 Phase 10.6 Real Robot 准入拒绝 + 7 项 Phase 11 误差面板
                                        + 6 项 Phase 12 幽灵臂 + 19 项 Phase 13 示教录制/回放）
生产构建      vite build                1,310.04 kB (gzip 369.33 kB)
FK↔Three.js  200 组随机关节状态         末端位置最大误差 8.673e-14 mm
                                       关节矩阵最大元素误差 8.527e-14
FK(IK(XYZ))  2000 组随机可达位姿         末端位置最大残差 1.401e-13 mm（失败 0 组）
IK 拖动连续性 300 点就近跟随             最大误差 1.017e-13 mm，支解切换 0 次
拖动轨迹      400 点穿越工作空间边界      边界定位到一格（1.5mm）内；越界段关节**零变化**
真实鼠标拖拽  无头 Edge + CDP 真实事件    Δ 17.47 mm；被锁轴 Z **逐位相同**（93.83984236589028）
测试探针      生产构建产物                JS bundle 中 `__armPilot` **0 命中**（dev-only 门控生效；`data-testid` 是有意保留的稳定选择器）
运行态       真实浏览器（swiftshader）   FK↔3D = 3.18e-14 mm @ 初始位姿
几何↔运动学  抹掉全部 geometry/details   endEffectorPosition 逐位不变
真机一致性    HOME 位（四舵机全 90°）    虚拟臂渲染姿态与实拍照片目视一致
真值冻结      运动学+物理语义核心         与基线一致（L2 哈希未变）；改 geometry 放行 · 改 length/gravity 报错
                                       （D55；`tools/freeze_baseline.py` + 11 项辨识力测试）
纹理校正      合成"模拟照片"往返          四角估计 max|Δ| = 1.00 px；刻度线偏差 1/1/1 px；板色 58≈60
                                       （D56）
采集判定      合成帧 + 真机实测值          真机帧轮廓过渡 1.17 px（上限 2.5）；整条臂连通域 ⇒ 判 undecidable 且不误报倾斜
                                       （D57；`tools/capture_texture.py`，pytest 共 147 项）
贴图可见性    近黑纹理渲染（ROI 30680 px）  板面 L 0.08 → **12.21**、细节能量 hp_std 0.086 → **0.293**（×3.41）；
                                       仍**暗于背景 22.7** ⇒ 剪影保留；非贴图件**逐像素不变**
                                       （底座蓝板 ×1.0000、max|Δ| = 0.00）（D64）
──────────── Phase 7 · 传输闭环（Mock） ────────────
Mock 物理约束 240°/s · 15ms 延迟 · tick 20ms   每 tick 恰好 4.8°；阶跃 40° 约 182ms 收敛
延迟语义      命令送达前                  不回推任何状态；送达瞬间走出第一步
丢帧语义      dropRate = 1                机械臂照常收敛到目标，但 received = 0（丢帧只影响上报）
限位拒绝      越界命令                    ERR JOINT 且**位置保持不动**；rejected +1
节流          400 次命令变化              实际下发 10–80 条（尾沿合并 33ms + 结束补发）
回环打破      400 点拖动轨迹              回推期间 `commandJoints` **引用从未改变**（toBe 断言）
断开语义      断开连接                    Actual 拉回 Command，不留假误差
e2e 实测      滑杆跳 40°                  即时 cmd 39.9° / act 0.8°（差 39.1°）→ 收敛 39.9° / 39.9°
收敛即停      到位后 tick 归零            无残留定时器空转（FakeTimer.pending() === 0）
──────────── Phase 8 · 真实 Go 后端 + 内置假固件 ────────────
后端真值      读 ../config/robot.yaml     /healthz.state = HOME 位（shoulder 0.8498937633 / elbow 112.6185771989）
标定往返      JR 1 位小数 → 舵机角 → 反算   HOME 位逐位自洽；改标定表即表现为 Actual ≠ Command
非等值回显    后端集成测试                命令 → 6 步中间态 4.18→7.52→10.85→14.18→17.51→20.80（等值回显必失败）
OK JR 语义    回执是**目标**角             只用于核对标定（容差 0.1°）；状态只认 STATE（舵机实际角反算）
ACK 门控      同一时刻 1 条在途            在途期间新命令只覆盖"待发槽"，不排队（latest-wins）
链路精度      0.1°（JR 1 位小数）          e2e `stat-lag` = 0.00°（修复前恒为 0.02° 假误差，见 D31）
两层心跳      应用层 ping/pong + RFC6455   e2e 实测 RTT 19–22 ms；服务端 40s 静默判死
模型互检      hello 带限位/标定/通道       前后端读同一份 yaml ⇒ 无告警；差异会被逐项报出
断线重连      指数退避 500→1000→2000…      e2e 杀掉后端再重启（新 sim 从 HOME 起步）⇒ Actual 回到 29.9°
e2e 实测      滑杆跳 30°                  即时 cmd 29.9° / act 0.8°（差 29.1°）→ 收敛 29.9° / 29.9°
──────────── Phase 9 · 真串口 + 相机地面真值 ────────────
链路末端      后端 hello                  device=serial（真机，不再是内置假固件）
标定单一真值  homePose 逐位比对           base 0 / shoulder 0.8498937633 / elbow 112.6185771989 / gripper 50
开机就绪门    Uno DTR 复位静默窗口         connect_settle_ms=2600 + 暖机包 ⇒ 不再出现 DEVICE_UNAVAILABLE
链路回推      7 步 JR 命令                max|Δ(意图角, 回执角)| ≤ 0.004°（纯链路自洽，**不含物理**）
物理到位      **相机**反解（唯一真值）      PASS 18 / FAIL 1（19 项）
相机重复性    同位姿两帧（00_reset vs 06_reset2）  肩 0.26° / 肘 0.01° —— 噪声底
增益复核      反解 Δ关节/Δ舵机 vs yaml    肘 −0.4235 vs −0.4177 ⇒ **+1.4%，肘标定被独立证实**
                                        肩 +0.6033 vs +0.6944 ⇒ **−13.1%，肩标定需重测**
曝光漂移      ⚠️ 同台面相隔 2 分钟两批   锚点绝对角偏置 +2.69° → +7.75°（漂 5°），Otsu 两批均 164
                                        ⇒ **自动曝光是绝对角主导误差源，高精度验收前必须锁死曝光**
──────────── Phase 11 · 链路误差反馈面板 ────────────
滞后判定      命令跳 34° 瞬间             结论「跟踪中（滞后 34.26°，命令到位前属正常）」——**不说"已到位"**
收敛判定      松手后收敛                 结论变为「已到位（最大偏差 0.00°）」；TCP 位置误差 ≤0.05 mm；超差条 0 条
断开判定      断开连接                   结论回到「未接入传输 · Actual ≡ Command，误差恒为 0」（不留假链接状态）
纪律验证      没有正面证据不下断言        命令静止 < 700ms / 样本 < 4 / 趋势仍下降 ⇒ 一律 `tracking`，不误报卡死
──────────── Phase 12 · 实际臂幽灵 ────────────
分离量        滞后瞬间                   幽灵 ↔ 主臂分离 **47.1 mm**（= 滞后量 34.26°），滞后一消失即重合
跟随正确性    时间无关不变量             幽灵 ↔ store 的 `actualEndEffector` 距离 **0**（渲染矩阵 vs 纯数学 FK，两条独立路径互证）
开关接管      关 → 开                    关闭后探针返回 `null`（对象树确实停止渲染）；重开后恢复
──────────── Phase 13 · 示教录制 / 回放 ────────────
录制          连摆三个姿态（间隔 130ms）   **4 帧 / 436 ms**（首帧 + 3 次变化；面板读数与 store 一致）
末帧捕获      退出录制                   末帧姿态 = 臂当时所在位置（停录时 `force` 补帧）
不自录        回放期间                   帧数 **4 vs 4** —— 回放不产生新帧（录制与回放共用命令通道）
终态精度      回放走完 0.44s 时间轴        命令 **逐值**等于录制末帧（1e-9 判定）；与"回放前挪开的 5°"明显不同
未绕安全门    回放经既有命令路径          下游日志可见 `joint_command`（尾沿节流与安全门照常生效）
清空          清空轨迹                   0 帧 / 状态回到「已就绪」；导出按钮重新禁用
──────────── A1/A2 判定 · 骨架双杆与 coupling.gain ────────────
自检 C        合成真值（不依赖实拍）       杆距 4mm 改善仅 +0.26°（肘）；8mm 改善方向不一致 ⇒ **无单调趋势**
实拍矩阵      rod_gap 0 / 4 / 10          gap=4 局部"修好"（+0.7% / −0.5%）；gap=10 让 dir_S7 崩到 **−46.3%**（拟合退化）
向后兼容      rod_gap=0                  w2_S7 仍报 `+0.6033 / −13.1%`（逐值复现）⇒ 默认关闭不影响既有结论
跨批极差      标定增益可复现性            肩 **[−13.1%, −2.3%] 极差 10.8%** · 肘 **[+0.9%, +53.8%] 极差 52.9%**（容差 5%）
最终判定      —                          **测量本身不可复现** ⇒ A1/A2 双双不做；A3 上台面重测是唯一入口（见 `hardware-measurement.md` §7）
──────────── MuJoCo 轨 · 物理仿真（M1–M10） ────────────
生成器        gen_model.py              8550 B · nq=4 nv=4 nbody=8 ngeom=33 nu=4 nexclude=5 · 总质量 0.2173 kg
FK 交叉验证   参考实现 vs MuJoCo        零位 [0,0,260] · HOME [111.9569,0,93.8398] · 120 随机 + 256 角点
                                       → max|Δ| = 7.1e-14 mm（阈值 1e-6）
IK 交叉验证   真实 ik.ts → MuJoCo 复算   目标由 MuJoCo FK 生成 ⇒ 120/120 成功（100%）
                                       max = 1.137e-13 mm · mean = 3.76e-14 mm
前端 fk.ts    与 MuJoCo 逐点对撞          376 点（含 256 限位角点）max = 1.137e-13 mm
越限诊断      几何可达但限位不允许        IK 报 JOINT_LIMIT（不是 OUT_OF_WORKSPACE），最接近支差 55.0°
可达内锥      合法域内最小水平半径         65.6208 mm（角点 θs=−6.0937°, θe=141.8582°）⇒ 够不到中轴线
解支唯一性    θe − θs ≥ 58.9866°        ⇒ elbow-down（α<0）恒不可行；四对限位换算后恰好等价
重力测试      无驱动 3s（有/无重力对照）  Δ base 3.873° / shoulder 35.962° / elbow 48.104° / gripper 31.691°
                                       关重力对照：Δ 全部 == 0.0
稳态误差自洽  qpos == ctrl − τ/kp        肩 0.36° = 0.0316 N·m ÷ 5.0
接触力判据    dist = +1.439 mm           qfrc_constraint = [0, −0.23325, −0.145184, 0] · 执行器满力矩 0.1765
                                       禁用台面 ⇒ TCP z 由 44.05 掉到 15.79 mm（Δz = −28.26 mm）
自碰撞几何距  相邻连杆实测重叠            −20.0 / −1.576 / −7.123 / 0.0 / −5.0 mm ⇒ exclude 是必要的
接触稳定性    下压 5s                   漂移 0.0013° · 无深穿透（min dist > −2 mm）
时间步分层    1kHz / 100Hz / 33.3ms      step(1)×10 ≡ step(10)（按位）；第 10 步限速目标恰好走 1 格
渲染解耦      fps=5 vs fps=240           qpos 按位相同（渲染帧率不决定物理步长）
复位可重复    reset × 2 · 热启动泄漏       qpos/qvel 按位相同；新实例与复用实例亦一致
跨进程确定性  run.py --demo 跑两遍         12 行数值载荷逐字相同（seed=0）
记录回读      JSONL / CSV               列与 config 一致 · 数值等于当时状态 · sim_time 严格递增
快速运动      Test D 峰值角速度           4.9161 rad/s（限速目标 10 rad/s，0.49×）· 无 NaN · 不越 range
Level 声明    calibration.calibrated      false + 七项全空（由测试强制，ADR D52）
 ─── pytest 106 passed（10 文件）· go test 56 全绿 · vitest 293 passed · tsc 0 error · vite build OK ───
```

> **Phase 9 最重要的一条结论**：真机固件**没有位置反馈**（`arm_get_angle()` 回的是固件记着的
> **目标值**）。所以 `OK SET` / `STATUS` / 后端 `joint_state` **全都在说「我打算去哪」**，
> 没有一条能证明「它实际上在哪」—— 哪怕机械臂卡死在桌上，回执依然一字不差。
> **串口回执原理上无法验证物理到位，唯一的外部地面真值是相机。**
> 见 `tools/verify_serial_e2e.mjs`（闭环主控）与 `tools/verify_pose.py`（反解内核），
> 决策记录见 `docs/decisions.md` D34–D38。

**2026-09-12 复测补记（19:25 批，D37/D38）**

| 项 | 值 |
|----|-----|
| 脚本可诊断性 | `verify_serial_e2e.mjs` 加**逐步日志**（`run.log`，带相对时间+耗时）+ 顶层错误打印**完整堆栈**，并区分 **exit 2 = 脚本崩溃 / exit 1 = 验收 FAIL**（此前两者都被当成"崩了"） |
| **ROI 不改** | 用户要求"用新 ROI 更新数据"；单批实验确曾把锚点偏置从 −2.20° 改善到 **−0.45°**，但**跨批验证推翻**：另一批各帧误差均值 6.29 → **12.17**（翻倍恶化）⇒ 单批"最优"是过拟合，**维持 `(330,150,1040,530)`** |
| **主误差源定位** | 反解标定增益偏差 **肩 +34% / 肘 −50%**，且**随行程放大**（+5° 命令误差 ~0.5°，+15° 涨到 +3.6~4.5°）⇒ 与 `hardware-measurement.md` §2 早已挂起的**平行四连杆耦合增益残差（实测 ≈ −0.81 而非 −1）** 完全吻合 |
| 根因（目视复核） | 骨架拟合贴的是臂的**外轮廓边**而非连杆轴线；meArm 小臂是**两根平行杆**，三连杆骨架模型**结构上表达不了它** ⇒ **须改模型，不是调 ROI** |
| 新增工具 | `tools/measure_roi.py`：量臂紧包围盒 + 给 ROI 建议值 + 可视化（**辅助目视工具**；数值会被线缆/桌沿污染成 `x0=0,x1=1279`，必须目视复核） |

### 真机端到端闭环（Phase 9）

```
verify_serial_e2e.mjs ──WebSocket(JSON，关节级)──▶ backend(serial) ──JR──▶ 固件 ──▶ 舵机 ──┐
        ▲                                                                                  │ 物理运动
        │  joint_state（开环**目标值**，只作链路自洽性参考，不作到位证据）                    │
        └── ffmpeg 抓帧 ◀────────────────────── 相机 ◀──────────────────────────────────────┘
                     │
                     ▼  tools/verify_pose.py（FK 侧视骨架 ↔ 实拍掩膜，反解肩/肘绝对角）
              与本步意图关节角比对 → PASS / FAIL
```

| 项 | 值 |
|----|-----|
| 动作计划 | `00_reset` / `01_sh_p5` / `02_sh_p15` / `03_el_p5` / `04_el_p15` / `05_combo` / `06_reset2`（单关节 ±5°/±15° 安全流程，结束回 RESET） |
| **期望值取量化角** | 固件只吃**整数舵机度**，物理落点是 `servoToJoint(round(jointToServo(θ)))`，与意图角天然差 `0.347°`(肩)/`0.209°`(肘) ⇒ 取意图角当期望 = 白送假误差（由 `verify_pose.py --quantize` 给出） |
| **判据一：绝对误差** | `--tol 5.0°`，会被**轮廓厚度系统偏置**污染 ⇒ **只作粗筛** |
| **判据二：帧间差** | `--dtol 1.5°`，对 RESET 锚点帧取差、**抵消公共偏置** ⇒ **锐利判据**（结论只认它） |
| 分割阈值 | `--thresh auto`（全批 Otsu 中位数）。实测新批 164/165/165、老批 111/114/114 |
| 底座掩膜 | `--base-region self`（**从本批自身派生**）。跨批复用掩膜会让锚点偏置 −5.79°→+2.69°、PASS 1/7→5/7 |
| base 约束 | 相机只能测矢状面 ⇒ **base 必须留 0°**，离面即判 SKIP（而不是硬算一个假角度） |

> **为什么绝对误差只能当粗筛**：对称 Chamfer 拟合的偏置**随轮廓厚度单调增长**
> （`--selftest` 自检 B：20px → 肩 −0.24°/肘 +0.10°；60px → +1.19°/+1.78°；100px → +3.40°/+5.49°）。
> 这与 `robot.yaml` 自述的「绝对角 ±5° 量级不确定度」吻合 —— 单看绝对误差，
> **分不清「标定错了 3°」和「轮廓画厚了 20px」**。
>
> ⚠️ **当前绝对角的不确定度还不足以判定 1° 级标定。** 要压到 1~2° 必须按
> `docs/hardware-measurement.md` 的 **Phase 4.5 标准重布台面**：
> 白分割板铺满视场 + 画面内放一把尺 + 尽量正交侧视取景 + **锁死相机曝光**（关闭自动曝光/白平衡）。
> **锁死曝光是硬要求，不是优化项** —— 实测自动曝光漂移就能让绝对角偏 5°。

#### 复测记录 · 2026-09-12 20:42（线缆固定后 · ADR D40）

按 D39 的前置要求固定线缆后重跑，**D39 的修复得到决定性验证**：

| 指标 | 20:31 批（相机漂 6px） | **20:42 批（线缆已固定）** |
|------|----------------------|--------------------------|
| 拟合尺度 `s` | 3.670 px/mm（−7.2%） | **3.969 px/mm** ✅ |
| 肩枢轴 | 漂移，锚点偏 −4.12° | **(548.7, 520.2)**，锚点偏 −1.62° ✅ |
| 重复性（肩） | 2.81° ❌ | **0.83°** ✅ |
| 反解 PASS | 1/7 | **3/7** |
| 总判定 | PASS 18 / FAIL 2 | **PASS 18 / FAIL 2** |

**新发现的第四类污染源：手入镜。** `06_reset2` 帧的三个异常信号同时出现 ——
JPEG 体积 +2.7%、反解残差 **69.76px**（其余帧 28~42）、肘帧间差 **−3.81°**；
目视确认画面右上**有一只正在调线缆的手**。手的像素被分割器并入臂掩膜，
把联合拟合拉到错误解。

> **这一条 FAIL 恰恰证明判据有效**：失败的是「重复性」—— 正是 D39 §2 定的健康检查。
> **判据抓到了污染源，而不是被污染误导。**

**开跑前的量化闸门**（新增 `.workbuddy/analysis/_cam_stability.py`，10 帧静止场景）：

```
逐帧 vs 首帧：dx=0 dy=0（全 10 帧）    相邻帧：dx=0 dy=0（全 9 对）
整图中位 155.0（波动 0.00）           整图均值波动 0.10
⇒ 位移恒为 0，可以开跑
```

⇒ **采集前置硬要求共四条**：锁曝光 · 锁相机 · 锁线缆 · **人员离场**（手/反光物不得入 ROI）。

端到端数值抽查（与解析解逐位吻合，`frontend/tests/e2e/ui-smoke.mjs` 自动断言）：

| 关节状态（θ 为**绝对角**，degree） | 末端 TCP（页面读数） | 解析解 |
|----------|----------|--------|
| **HOME** (0, 0.849894, 112.618577, 50) | (112.0, 0.0, 93.8) | (111.957, 0, 93.840) |
| J2 滑杆 → 39.9°（量程 −6.09..49.45 量化） | Z 75.2 | Z 75.222 |
| J2 39.9° + J3 滑杆 60° → **钳位到限位 min 108.441485** | (165.2, 0, 83.4) | (165.154, 0, 83.413) |
| 夹爪滑杆 90° | TCP 与上行差 < 0.05（夹爪不参与定位） | 同左 |

> `x = 80·sin(θ_肩) + 120·sin(θ_小臂绝对角)`，`z = 60 + 80·cos(θ_肩) + 120·cos(θ_小臂绝对角)`。
> 因小臂存**绝对角**（平行四连杆解耦），**不再叠加肩角** —— 这是本次实测修正的核心。
> 其中 `J3 滑杆 60°` 被 `elbow.limit.min = 108.441485` 钳位，正是「零位不可达 0°」的体现
> （见 `docs/coordinate-system.md`）。

### IK（Phase 5）

```ts
solveIk(model, [x, y, z])  // → { success: true, joints, branch, residual, azimuth, relativeAngle }
                           // | { success: false, reason: 'OUT_OF_WORKSPACE' | 'JOINT_LIMIT', joint?, message, candidates }
```

| 项 | 值 |
|----|-----|
| 几何量（运行时从模型求导） | `L1 = 80`（大臂）· `L2 = 120`（小臂 80 + TCP 偏移 40）· 枢轴 `z = 60`（立柱） |
| 可达球壳 | `40 ≤ D ≤ 200` mm（`D` = 肩枢轴到目标的距离） |
| 解算方式 | `J1 = atan2(y, x)` 偏航解耦 + 矢状面内 2R 解析解 |
| `elbow` 语义 | **绝对角**，解出即用，**不叠加肩角**（相对角由 FK 的 `effectiveJointAngle()` 唯一负责） |
| 多解 | `prefer: 'elbow-up' \| 'elbow-down' \| 'nearest'`（默认 `nearest`） |

> **本机结构上只有一支解**：`elbow.limits.min − shoulder.limits.max = 108.4415 − 49.4549 = 58.99° > 0`
> ⇒ 相对肘角恒为正 ⇒ `elbow-down` 恒不可达。2000 组随机可达位姿实测分支分布
> `{elbow-up: 2000, elbow-down: 0}`。多解策略是为**换机构**预留的通用性，Phase 6 拖动不会翻肘。
> 失败信息格式对齐 `protocol/serial-v1.md` 的 `ERR JOINT base 95 (limit -60..60)`；
> 模型轴/朝向不符平面 2R 前提时**抛 `IkModelError`**，不静默解出错解。

### 末端目标与拖动（Phase 6）

| 项 | 值 |
|----|-----|
| 三个量 | `target` 我想去哪 · `commandJoints` 实际去哪 · `endEffector` = `FK(commandJoints)` |
| 越界行为 | **只更新 `target` 与状态栏，关节逐位不变**（不钳位，见 D20） |
| 拖动平面 | `xy`（默认，锁 Z）· `xz`（锁 Y）· `camera`（全自由）；**`pointerdown` 时冻结**（见 D21） |
| 拖动链路 | R3F pointer 事件 → `dragPlane.dragTarget()`（纯数学层）→ `moveTo()` |
| 把手语义 | 把手跟随 `target`，TCP 圆点跟随关节；**两者分离即"够不到"**，中间画虚线 |
| 拖动期间 | `OrbitControls` 禁用 + `setPointerCapture`（指针移出 canvas 不中断） |
| e2e 证据 | dev-only 探针给出把手屏幕坐标 + CDP 派发真实鼠标事件（见 D22） |

> `moveTo()` 用 `prefer: 'nearest'` + `near: 当前命令角` 求解，并把当前关节状态作 `seed`，
> 使夹爪等未参与解算的关节原样透传 —— 返回值可直接喂 FK 闭环。
> 实测确认：从 HOME 向右拖，末端跟随 17.47mm，落点 `target[2]` 与按下瞬间的 TCP Z **逐位相同**
> （`93.83984236589028`），即"锁 Z"是**定义**而非数值巧合。

### 传输层与后端（Phase 7–8）

| 项 | 值 |
|----|-----|
| 抽象 | `RobotTransport`：`connect` / `disconnect` / `sendJointState` / `onState` / `onStatus` / `stats` |
| 实现 | `MockTransport`（进程内假机械臂）、`WebSocketTransport`（JSON ↔ Go 后端） |
| 命令→链路 | `transportBridge` 尾沿合并 33ms → `sendJointState` → 编码 → 发送 |
| 链路→状态 | `onState` → **只写 `actualJoints`**（回环打破）→ 状态面板 |
| 后端分层 | `wsserver → controller → device(sim \| serial)` + `protocol`（编解码）+ `robot`（标定/限位） |
| 后端端口 | **8090**（`MeArm-RemoteControl` 的舵机级摇杆是 8080，两者并存） |
| 唯一真值 | 后端也读 `../config/robot.yaml`，并在 `hello` 里回传 ⇒ 两端不一致会被逐项报出 |
| 链路末端 | `device.mode = sim`：内置「假固件」，**走真实字节流**（`JR`/`OK JR`/`STATE`），只有串口驱动是假的 |
| 可注入 | 时钟 `TimerLike`（`FakeTimer`）· socket `SocketLike`（`FakeSocketFactory`）⇒ 时序逻辑确定性可测 |

> **误差链路在这里第一次真的活起来**：`state` 由后端从**舵机实际角反算**，而不是回显命令。
> 所以标定表写错（offset/scale/reverse）、链路精度不够（0.1°）都会立刻表现为"Actual 追不上 Command"。
> 见 `docs/coordinate-system.md` §3.4 与 `docs/decisions.md` D28–D31。

## 7. 硬件基线

| 部位 | 参数 | 来源 |
|------|------|------|
| 机构 | 开源 meArm（4 舵机） | 标称尺寸 立柱 60 · 大臂 80 · 小臂 80 · 手部 40 mm（未直接量取，待标尺实测校核） |
| 舵机角色 | **S9 底座 · S7 肩(大臂) · S8 肘(小臂) · S6 夹取** | **2026-09-12 实测**（白底分割 + 逐度扫描 + 骨架拟合）；固件侧 `SERVO_LEFT/RIGHT` 只是安装位，**不等于**运动学角色 |
| 固件限位 | S9 30–150 / S8 20–100 / S7 80–160 / S6 40–130 | `MeArm-Device/bsp/servo.h` |
| 关节限位（反算） | shoulder −6.094..49.455 / elbow 108.441..141.858 | `config/robot.yaml`（舵机限位 × 实测标定增益） |
| 标定增益（实测） | S7 0.694 · S8 0.418 关节度/舵机度 | `docs/hardware-measurement.md` §3 |
| 固件开机位 | 全部 90°（= 本项目 HOME 位姿） | `MeArm-Device/core/arm_control.c` |
| 串口 | **COM16** · 115200 8N1 | `tools/mearm_hw.py` |
| 实测相机 | Windows 相机 + 机械臂与桌面之间的**白色分割板** | `tools/mearm_hw.py` / `analyze_sweep.py` |

> ⚠️ 打开 COM16 会拉低 DTR 使 ATmega328P 复位，固件随即把 4 个舵机驱到 90°。
> 所以「发一条指令就重开一次串口」会让机械臂每次都弹回 RESET 位 ——
> 实测脚本必须在**单次连接**内完成整段 `[set → 稳定 → 抓拍]`（见 `tools/mearm_hw.py`）。

> 机构尺寸仍是开源 meArm **标称值**，但**关节角色、标定 offset/scale/reverse、关节限位与零位
> 已全部改为实测反解值**（见 `docs/decisions.md` D15–D17）。后续再实测微调**只改
> `config/robot.yaml`**，代码与测试均无需改动 —— 测试期望值已改为跟随配置派生。

**显示外观同样只由 `robot.yaml` 决定**：连杆几何用 `plate`（倒角薄板）/ `box` / `cylinder` /
`sphere` / `servo`（舵机）参数化声明，附加件（舵机、螺栓、轴销）写在 `links[].details`。
渲染层只做「参数 → 网格」翻译，不含机构尺寸常量。改外观请只改 yaml —— 详见
`docs/model-structure.md`。

> 当前外观按开源 meArm 实物布置：底盘 + 转盘 + 两片立柱侧板 + 两片大臂/小臂侧板 +
> **4 个舵机**（S9 底座 / S7 肩 / S8 肘 / S6 夹取）+ 金属轴销 + 明显两片夹爪。
> 舵机归属已按实测修正：**肩舵机在立柱顶、肘舵机在大臂末端**（改 yaml 的 `details` 即可）。

### 真机实测流程（可复现，Phase 4.5）

**目标**：不靠人眼估读，用「相机照片 + 舵机角度」反解出机构角色、标定与零位，直接写回 yaml。

```bash
PY=<项目隔离 venv>/python      # numpy；与前端 node_modules 完全隔离

# 0) 台面：机械臂与桌面之间铺白色分割板 → 「灰度 < 90 = 机械臂」即可干净分割
# 1) 采集（单进程内 [复位 → 逐度 set → 稳定 → 抓拍]，避开 DTR 复位陷阱）
$PY tools/mearm_hw.py sweep 7 80 95 110 125 140 155 --out-dir .workbuddy/captures/w2_S7
$PY tools/mearm_hw.py sweep 8 30 45 60 75 90 100   --out-dir .workbuddy/captures/w2_S8
# 2) 帧间量化：掩膜面积 / 重心 / 最高点 / 红绿差分（最小角=红，最大角=绿，重合=黄）
$PY tools/analyze_sweep.py .workbuddy/captures/w2_S7 --overlay .workbuddy/analysis/ov_S7.png
# 3) 公共静止区自动分段：inter_S8 − inter_S7 ≈ 大臂像素（不用人肉认枢轴）
$PY tools/segment_arm.py --out .workbuddy/analysis/segments.png
# 4) 主证据：FK 骨架 ↔ 实拍照片拟合（对称 Chamfer + Hooke-Jeeves，RESET 帧锚定 s/ox/oy）
$PY tools/fit_pose.py .workbuddy/captures/w2_S7 --sweep both --anchor .workbuddy/captures/w2_S8/S8_090.jpg
```

三条**互相独立**的判据共同确定角色映射与解耦，任一单独成立都不足以定论：

| 判据 | 扫 S7（肩） | 扫 S8（肘） | 结论 |
|------|------------|------------|------|
| 画面最高点位移 | (570,220) → (736,321)，**194 px** | (594±4, 221±2)，**≈0 px** | S7 动整条臂 / S8 只动前臂 |
| 公共静止区差集 | `inter_S7` = 底座+立柱 | `inter_S8 − inter_S7` ≈ 6342 px = **大臂** | S8 不动大臂 |
| 骨架拟合绝对角 | 肩 Δ **+52.91°** 时小臂绝对角只漂 **+10.08°**（相对角却变 −42.83°） | — | **小臂是绝对角**（平行四连杆） |

完整数据（含拟合表、写入 yaml 的数值、自洽性验算、不确定度分级）见
[`docs/hardware-measurement.md`](docs/hardware-measurement.md)；决策记录见 `docs/decisions.md` D15–D17。

## 8. 与同级仓库的关系

| 仓库 | 角色 | 变更时机 |
|------|------|----------|
| **MeArm-3D**（本仓库） | 数字孪生**前端 + 关节级后端**（`backend/`，8090），模型/运动学/标定/状态权威 | — |
| `MeArm-Device` | AVR 固件：现有舵机级文本协议（`SET` / `JOY` / `STATUS` / `RESET`） | Phase 9：新增关节级 `JR` / `STATE`（见 `protocol/serial-v1.md` §4） |
| `MeArm-RemoteControl` | Go 串口↔Web/TCP 服务 + 双摇杆 UI（8080） | **不再计划改名**：Phase 8 的关节级通道落在本仓库 `backend/`（8090），与它的舵机级摇杆通道**并存**、互不干扰 |

**为什么关节级后端不塞进 `MeArm-RemoteControl`**（见 D27）：那是个**舵机级摇杆**服务，
混入关节级逻辑会让"谁负责标定"变得模糊 —— 而本项目的铁律是标定只有一份。
现在 `backend/` 直接读 `../config/robot.yaml`，并通过 WebSocket `hello` 把读到的
限位/标定回传给前端**在线互检**，两端真值不一致会当场暴露。

三者共享的**唯一标定表**放在本仓库 `config/robot.yaml`，固件侧标定表由它生成，
避免"上位机标一次、固件再标一次"的双份真值。

## 9. 后端（`backend/`）快速索引

| 想知道 | 看 |
|--------|-----|
| 怎么跑、`/healthz` 长什么样、与 8080 的分工 | [`backend/README.md`](backend/README.md) |
| JSON 消息全集、错误码、Go 分层职责 | `protocol/serial-v1.md` §5 |
| 为什么 `OK JR` 不能当 Actual、为什么要 latest-wins | `docs/decisions.md` D29 / D30 |
| 跟踪误差为什么是 0.02° 而不是 0（链路精度） | `docs/decisions.md` D31 · `docs/coordinate-system.md` §3.4 |
| Phase 9 接真串口的落点与实测坑 | `backend/internal/device/serial.go` 注释 · `protocol/serial-v1.md` §6.1 |
| 真机端到端怎么跑、相机怎么当唯一真值 | `tools/verify_serial_e2e.mjs` · `tools/verify_pose.py` 头注释 · D34–D36 |

## 10. MuJoCo 物理仿真（`simulation/`）快速索引

| 想知道 | 看 |
|--------|-----|
| 怎么跑、验收数据、Level 声明 | [`simulation/README.md`](simulation/README.md) |
| 架构勘察：几个自由度、五种角度、接入方案对比 | [`docs/ARCHITECTURE_ANALYSIS.md`](docs/ARCHITECTURE_ANALYSIS.md) |
| 为什么 MuJoCo 的 hinge range **不是**限位真值 | `docs/decisions.md` **D49** · `simulation/README.md` §4.2 |
| 「有接触记录」为什么不等于「有力」 | `docs/decisions.md` **D51** · `tests/sim/test_collision.py` 文件头 |
| 为什么 IK 验收必须加载真实 `ik.ts` | `docs/decisions.md` **D53** · `frontend/tests/tools/kinematics-bridge.mjs` |
| 改了几何后要做什么 | 重跑 `python simulation/mujoco/gen_model.py`（**MJCF 是产物，禁手改**） |
| 三个只在特定调用方式下暴露的静默错误 | `docs/decisions.md` **D54** |

## 11. 真值冻结与视觉层边界

用户要求「保存目前的运动学物理数据不变」。`config/robot.yaml` + `config/physics.yaml`
的**语义核心**已冻结（基线 `config/baseline-kinematics-physics.json`）：

```bash
$PY tools/freeze_baseline.py            # 校验（不符则退出码 1，并逐字段列出差异）
$PY tools/freeze_baseline.py --update   # 有意识改过参数后重新冻结
$PY tools/freeze_baseline.py --show     # 打印当前核心摘要
```

| 改动 | 结果 |
|------|------|
| `links[].geometry` / `links[].details`（外观：尺寸 / 颜色 / 细节件） | ✅ **放行** —— 视觉建模正需要这一层 |
| `links[].length` · `joints[]`（轴 / 限位 / `coupling`） · `actuators[]`（标定） | ❌ 报错 |
| `gravity` · `contact` · `timestep` · `servo` · `inertia` | ❌ 报错 |
| `recording` · `deterministic.seed`（非物理量） | ✅ 放行 |

> **判据是语义核心哈希，不是整文件哈希。** 整文件哈希会把"换外观"误判成违规，
> 于是守卫会被绕过或删掉 —— 一个会被绕过的守卫等于没有守卫（ADR **D55**）。

改真值后除 `--update` 外，**还必须**重跑 `python simulation/mujoco/gen_model.py`
让 MJCF 跟上（`test_generated_mjcf_is_in_sync_with_config` 会盯这件事）。

**推论（这是做视觉建模的前提）**：外观层自此完全自由 —— 几何 primitive、颜色、
以及未来的**照片纹理 / 重建网格**都可以随便迭代，而运动学与物理被钉死在基线上。

## 12. 照片纹理贴图（进行中）—— 让数字孪生更像真机

**问题**：`frontend/` 里**一个图像素材都没有**（无 png/jpg/glb/hdr），模型全靠程序化
primitive（`plate` / `cylinder` / `servo`）+ 十六进制单色 —— 这就是"不像真机"的根因。

**做法**：把实物照片贴到对应板件的**大面**上。

```
机械臂转到镜头前 → tools/capture_texture.py --plate <name>
   │                 三态判定：ok / reject / undecidable
   │                 不 ok 时给出**具体动作**（靠近多少 / 绕哪根轴转 / 长边横过来）
   ↓ 通过
assets/textures/mearm/raw/<name>.jpg
   → tools/make_texture.py（PCA 估四角 → homography 透视校正 → 按真实尺寸归一化）
   → assets/textures/mearm/tiles/*.png（+ 目视复核图，强制看图再信结果）
   → frontend 按面用材质数组贴图（仅 geometry 层，运动学/物理不动）
```

```bash
$PY tools/capture_texture.py --plate upper_arm_link   # 抓帧 → 判定 → 告诉你要怎么调
$PY tools/capture_texture.py --list-devices           # 列出 DirectShow 视频设备
$PY tools/make_texture.py --list                      # 待拍清单（文件名 + 大面尺寸，直接来自 robot.yaml）
$PY tools/make_texture.py --selftest                  # 合成数据自测，证明校正几何正确
$PY tools/make_texture.py --all                       # 处理 raw/ 里已有的照片
```

**五条决定成败的事实**（都是实测，不是推断）：

1. **大面法向 = `size` 里最小那一维，但「局部轴 ≠ 世界轴」。**
   大臂板 `[22,5,74]` 的局部薄轴是 Y，本位姿实测映射到**世界 Y**（垂直臂摆动平面）
   ⇒ 相机摆正前方只会拍到 5mm 窄边，**必须站侧面**。
   ⚠️ 早期写的"转 base 90°"是**错的**：`base` 限位只有 **±60°**（写 90° 会被后端拒绝），
   而且实测转 base 对分辨率是**零和**的（见第 2 条）。**当前机位本来就是侧面，不需要转。**
   ⚠️ 而 `forearm_brace` / `tool_link` 的局部薄轴同为 Z，实测却映射到**世界 X（沿臂方向）**
   ⇒ 早期表格按"局部 ±Z = 世界竖直"给出的"俯拍"是**错的**（D57 一）。
   **别背轴向 —— 拿实物转一圈，找面积最大的那个面。**
   （坐标系基准：`base` 轴 = 世界 Z、`shoulder`/`elbow` 轴 = 世界 Y ⇒ 摆动平面 = X–Z。）
2. **动关节改变不了分辨率 —— `px/mm` 只由物理距离决定。**
   相机在正前方（−Y），肩/肘的摆动平面（X–Z）**平行于像平面** ⇒ 摆动既不改变距离、
   也不改变板的投影长度，只改变板**在画面内的朝向**。实测三证：
   ① 仿真扫 shoulder 时大臂板倾角 89.2°→79.2°→59.2°→41°（**板随肩同步转**，投影长度不变）；
   ② 真机 HOME vs `shoulder=30` 两帧的**暗色投影面积 236345 → 230804（−2.3%**，差值来自遮挡变化）；
   ③ `base` 旋转实测远端放大 ×1.08，但投影缩短 `cos20°=0.94` ⇒ **净 1.015，约等于零**。
   ⇒ **要提高 px/mm，只能把机械臂挪近相机（或相机挪近），没有软件办法。**
   ⇒ 相机上限 **1280×720**。`px/mm` 现由**三个独立方法收敛到 `s ≈ 3.7–4.1`**
   （旧 CALIB 3.80 · 修正手性后的整臂剪影 IoU 4.05 · 梯度吸附后按长宽比反推 4.2~4.4），见 D61 三。
   ⚠️ 中途一度得到的 2.58 是**欠定下的假解**，不要当测量值（D59 ① 只对了一半）。
3. **「自动分割出单块板」不可行 —— 白卡也救不了。**
   ⚠️ 这条**推翻了早期的白卡方案**：实测板、立柱、底座、舵机、控制板**全是同一种黑**
   （暗像素占整帧 25.7%，左下象限更是 65%），且板与板**由螺栓物理相连**
   ⇒ 不存在"只含一块板"的连通域，最大暗色连通域必然是**整条臂 + 底座**。
   白卡只能挡背景，挡不住**结构性连通**。⇒ 目标板位置必须**由模型投影给出**（见第 4 条）。
4. **目标板四角 = 模型投影给初值 + 图像梯度吸附**（`.workbuddy/captures/snap_quads.py`），见 D61 五。
   单靠模型投影不够（整臂剪影 IoU 只到 43% ⇒ 误差几十像素），单靠目视也不够（±15px，且板与
   关节块连成一片）；但**投影给"大致位置与走向"、梯度吸附给"像素级贴合"**，两者合起来可行。
   实测边分 8.78→**19.30**（大臂）、4.65→**18.74**（小臂），且**长宽比精确吻合真值**：

   | 板 | 吸附后 短边×长边 | 长宽比 vs 真值 |
   |---|---|---|
   | `upper_arm_link` | 96.2 × 323.7 px | **3.37** vs 3.36 ✓ |
   | `forearm_link` | 75.8 × 286.4 px | **3.78** vs 3.78 ✓ |

   ⚠️ **走上这条路之前必须先确认手性**：模型与实拍**互为镜像**时 IoU 永远上不去
   （30.2% → 修正后 43.2%）。⚠️ 而目视判断手性极不可靠 —— 我之前正是被
   `mujoco.Renderer` 的**参数顺序坑**（`(height, width)` 我按 `(width, height)` 传）
   产出的拉伸图误导，得出了镜像的结论（D61 一）。
   ⚠️ D60 曾据同一批拉伸图断言"模型几何与 3D 打印实件有系统性差异"—— **该结论已撤回**（D61 二）。
   仍有效的旧教训：`project_plates.py` 的三项相乘目标函数**欠定**，只优化"框内暗色"时 `s=2.5`
   会把三块板挤进左下底座暗区拿到 97.4% 的**假性满分**（而 `oy=770` 已在画面外）。
5. **真机臂板是镂空桁架**，`geometry.size` 只是**外接盒**（可透过开口看到背后的墙）
   ⇒ 轮廓填充率天然只有 40~50%。**直接把照片烘到实心盒上会把背景色带进孔洞**，贴上去比现在更假。
   ⇒ **已实现孔洞掩膜**（`make_texture.py` 默认开启，ADR **D62**）：把非板面区域填成**板面材质**。
   三层判据缺一不可 —— ①**连通性**（纯亮度会把板面上的螺栓一起填掉）②**形状**（只判面积会把
   被暗区包围的线缆小段留下）③**色差**（`R−B>15`；暗线缆亮度低于阈值，只判亮度**永远去不掉**）。
   填充色取**实测板面中位色 `RGB(1,1,20)`**（近黑带蓝，**不是纯黑** —— 填 `(0,0,0)` 有色差补丁）。
   （`RoundedBoxGeometry` 的 UV 是"每个面各自铺满 [0,1]"，与长宽比无关 ⇒ 必须**按面用材质数组**；
   实测 6 个 group 覆盖全部顶点：`0=+X 1=-X 2=+Y 3=-Y 4=+Z 5=-Z`。）

**当前状态**：采集指南 + 透视校正管线（12 项测试）+ 采集判定（16 项测试）已就绪；
**真机链路已打通**（`COM16` CH340 实测在线，`tools/set_joints.mjs` 可脚本化驱动，
发送前用 `hello` 的限位做**本地校验**、非 `device=="serial"` 一律拒绝发送，
`--home` 保证实验从已知状态出发）。
用户已按 D59 的要求**重新取景并挪近** ⇒ 新机位下**整臂完整入画**（不再被顶边裁掉），
背景干净（背景最暗 **115**，目标板中位 2~8 ⇒ 阈值可用 **65**）。

**纹理已出 2 块**（`make_texture.py --corners`，四角来自 D61 的"投影初值 + 梯度吸附"；
**填孔已开启**，ADR D62）：

| 纹理 | 尺寸 | 填入 | 亮像素 | 质量 |
|---|---|---|---|---|
| `upper_arm_link` | 22×74mm → 220×740px | 33.4% | 1.25% | ✅ 近黑底 + 4 枚螺栓亮点 |
| `forearm_link` | 18×68mm → 180×680px | 35.2% | 0.85% | ✅ 近黑底 + 1 枚螺栓亮点 |

暖色残留（暗线缆）两块均为 **0.00%**。
⚠️ **上一轮"线缆悬在板前面、必须物理拨开"的判断已作废**（D61 四）：4× 放大后看清橙色线缆在板的
**背面**，是从**三角镂空里透出来**的 ⇒ **不需要动物理**。

⚠️ **但四角仍不完美** —— 新增诊断判据「**四边亮比例**」（框贴合板时四条边应都接近 0，ADR D62）：

| 纹理 | 上 | 下 | 左 | 右 |
|---|---|---|---|---|
| `upper_arm_link` | 45.2% | 18.5% | 31.1% | 46.4% |
| `forearm_link` | 57.6% | 33.7% | 61.5% | 19.7% |

⇒ **框比板大，纹理内容会被压缩**（两个框偏心方向还相反）。对观感的影响有限（板面近黑、
几乎不含空间信息，可见内容主要就是那几枚螺栓），但**不能假装它不存在**。

⚠️ 另注：早先的"框外 / 镂空"二分统计**不能单独用来判贴合** —— 三角镂空若开口到板端，
同样会接触纹理边界而被算进"框外"。**四边亮比例才是定义上无歧义的判据**（D62 教训）。

**还没做的 7 块**（`base_link` / `column_link` / `column_link_side` / `upper_arm_brace` /
`forearm_brace` / `tool_link` / `jaw_link`）：在当前姿态**被遮挡或不在视野**，
需要额外姿态 / 机位逐一补拍。

⚠️ 另纠一条**误报**：`project_plates.py` 报的 `ramp=10.78px 失焦` **是假的** ——
`ramp` 是在**投影框边缘**测"暗↔亮"过渡的，而旧标定定位本身就错（D59），框落在**暗区内部**
⇒ 边缘压根没有真实边界。放大目视：板边缘过渡约 **2px**，**并未失焦**。
⇒ **`ramp` 只有在四角正确时才有意义**，绝不能拿它反推"相机失焦 / 别再靠近"。

### 前端接入（ADR D63，✅ 已完成）

- **按面材质数组**：大面（`size` 里最小那一维所在的轴）贴照片，其余四面保持板色
  —— `RoundedBoxGeometry` 的 UV 是「**每个面各自铺满 [0,1]**」，用一个材质会让 5mm 窄边
  也被整张照片铺满，侧面出现被拉扁的螺栓，**比不贴更假**。
- `robot.yaml` 的 `links[].geometry` / `details[].texture` 填 **key**（相对 `assets/textures/`），
  新增纹理只需丢文件进 `assets/textures/`，**无需改代码**（`import.meta.glob` 静态登记）。
- 朝向：代码只负责「两个大面互为镜像」这条**几何必然**（由实测 UV 表推导），
  照片那一侧的不确定留给可选的 `textureFlipU/V`。
- ⚠️ **`TextureLoader` 的 `flipY` 默认 `true`**（图像顶行 ⇔ `uv_v = 1`）：漏掉它会让
  所有 v 方向的推理**整体反号** —— 本轮正是这么错了一次，最后靠**四象限探针纹理**
  受控实验定案（D63 三 / 四）。
- ⚠️ 测试跑在 **node 环境**（无 DOM），而 `TextureLoader` 需要 `document` ⇒
  无 DOM 时按「没配纹理」处理（纹理不参与任何几何判定）。
- 守卫测试 `tests/unit/plateTexture.test.ts`（7 项）：含**纹理长宽比必须与板大面一致**
  （抓"把 forearm 的图配到 upper_arm"）与"声明的 key 必须真的被登记"（贴图失败是**静默**的）。

### 暗端材质与光照（ADR D64，✅ 已完成）

**先定性**：板面"看不见"**不是偏暗，是信息被 8bit 量化抹掉了** ——
实拍按白桌曝光 ⇒ 黑亚克力落在 **1~3 码值**（tile 实测 `≤3` 占 72.2% / 77.0%，`L<40` 只用到 41 档码值），
所以**等比例提亮救不回来**：要把板面推到 `L≈60` 需 **+6.2 EV**，而那时板面摊成 `41/60/74` **三级平台**，
**比纯黑更假**（`exposureEv` 因此保持 `0`）。

真正有效的是**环境反射**：黑件的形状可读性来自**镜面项**（非金属 `F0≈0.04`，镜面项**不乘**那个 ≈0
的暗反照率）⇒ 取决于"反射了什么"，与暗端数据无关。

| 量（ROI `x[402,520) y[380,640)`，n=30680 px，同机位） | 改前 | 改后 | 变化 |
|---|---|---|---|
| `L` mean | 0.08 | **12.21** | ×152.6 |
| **`hp_std`（细节能量·主判据）** | 0.086 | **0.293** | **×3.41** |
| `p95−p05`（动态范围） | 0.07 | 1.86 | ×26.6 |
| `L≥20` 占比 | 0.0% | 0.0% | 仍**暗于背景** `22.7` ⇒ 剪影保留 |

- 实现：**逐材质 `envMap`**（`plateEnvironment.ts` 用程序化 `RoomEnvironment` + PMREM，
  **零外部 HDR 资产**），只有 `plateTexture` 材质被触碰；`envMap` 由调用方在 effect 里传入，
  故 `buildRobotObject3D` 保持**不依赖 DOM / WebGL**。
- ⚠️ **不要改回 `scene.environment`**：它是**全局**的（实测把底座蓝板一起点亮 **×3.3757**），
  且 three 在 `material.envMap === null && scene.environment !== null` 时
  **用 `scene.environmentIntensity` 覆盖 `material.envMapIntensity`**
  （`0.186.0 WebGLRenderer.js:2736`）⇒ 逐材质"关掉环境反射"的设计**根本不生效**
  （实测 V5/V6 统计**逐位相同**：`L=59.17 / p05=57.36 / max=78.2`）。
  逐材质方案实测**底座蓝板逐像素不变**（`×1.0000`、`max|Δ| = 0.00`）。
- 参数在 `config/robot.yaml → appearance`（**外观层**，与 FK / IK / 标定 / 限位无关）。
  `environmentIntensity` 与 `roughness` 实测**只缩放整体亮度、不改变细节能量**
  （`L` 动 4 倍时 `hp_std` 只动 11%）⇒ "再亮一点"换不来更多细节。

> ⚠️ **上限到此为止**：以上手段都只是"把**已经存在**的镜面信息显示出来"。
> 板面在 tile 里只用到 41 档码值，**真正能把细节送进数据的是拍摄端**（对板测光 / 补光重拍）
> —— 这正是下面 A 项必须一并解决的问题，不是渲染层能翻过去的。

### 夹爪形状：按实拍照片反解（ADR D66，✅ 已完成）

爪原先只是 **`plate` 占位（两片倒角方块 + 硬编码黄色）**，与实物差得最远。
实拍近景里它是**根部整圈方齿齿轮盘 + 内侧缘一排锯齿 + 末端斜切**的机加工件
⇒ 改为 `geometry.type: jaw` 的**参数化平面轮廓**（`config/robot.yaml → links[jaw_link].geometry`）。

- **能读的只有轮廓**：黑件近纯黑（`≤3` 码值占 72%~77%，与 D64 同一批照片），**没有贴图信息**；
  但**剪影可用** —— 激光切割的亮切口恰好勾出轮廓。所以本轮做的是**形状**，不是纹理。
- **画成一条连续闭合折线**：`ExtrudeGeometry` **不做布尔并集** ⇒ 齿轮盘与爪指不能各画一个 Shape
  （重叠区 z-fighting、分离处露接缝）；齿廓 + "脖子" + 外侧缘锯齿 + 斜切尖必须首尾相接。
- **三条派生关系在渲染层推、yaml 不配平**：中心距 = `2 × 分度圆半径`（标准啮合）；
  爪指中线相对齿轮中心内偏 `δ = 分度圆半径 − width/2`（让齿轮中心落在爪指**内侧缘**上）；
  ⇒ 由二者得 **`θ=0` 时两爪内侧缘正好贴合**。解析层对 `分度圆半径 < width/2` 直接抛 `RobotConfigError`。
- 尺寸链与照片同源：齿轮直径 / 爪长 = **0.69（照片）vs 0.71（模型）**。

⚠️ `serrations / serrationDepth / color` 三项是**观感目视调参、未做受控量测**。
⚠️ 运动学层未被触碰（`jaw_link.length` 恒 0，D55）。

| 判据 | 结果 |
|---|---|
| `tsc -b --force` / `vitest run` | **0 error** / **313-313** |
| `vite build` + 探针字面量扫描 | 构建通过 / **`__armPilot` 0 命中** |
| e2e `ui-smoke.mjs`（干净环境） | **88/88 PASS, 0 FAIL, 0 SKIP** |

### 视口与件色：灰底 + 近黑件 + 世界轴默认关（ADR D68，✅ 已完成）

- **视口背景** `#14171c` → **`#8b8e93`**（中性灰）；Z=0 参考圆盘**必须跟着改**
  （`#1d222a` → `#7a7e84`，只比背景暗一档）—— 否则灰底上会留下一个突兀的黑洞。
- **板件统一近黑 `#232830`**（6 个偏蓝色值、共 11 处）。前提是先确认：
  **贴图件两个大面用白色基色**，`geometry.color` 只染色那 4 个窄边
  ⇒ 改 `color` **不会**把照片纹理一起乘黑。反过来若大面也用 `color` 做 tint，同一改动结果完全不同。
- **世界轴默认关**：`WorldAxes` 是挂世界原点的 `axesHelper`，其 **Z 蓝轴与 J1 轴重合**
  ⇒ 看上去就是"机械臂**中轴**上一根蓝线"；它固定在世界系，J1 一转（立柱让开）就露出且**不跟着动**。
  **受控实验**（只翻 "World Axis" 一个开关）+ 像素 diff：关掉后 3D 里消失的只有 X / Y 两条长斜线；
  立柱上那道亮蓝竖条在 diff 里**没有任何标记** ⇒ 它是 `#4d8fe8` 的立柱侧板（已随本轮改黑），**不是轴**。
  开关**保留**（调试坐标系仍可打开），只是不再默认占用画面。
- **未改**：DragHandle 把手球 `#4c8dff` 与 TCP 标记的 Z 短柱 `#6699ff` —— 交互 / 调试元素，
  不是模型件（把手球的蓝 / 红还带"可达 / 超程"语义）。

| 判据 | 结果 |
|---|---|
| `tsc -b --force` / `vitest run` | **0 error** / **313/313** |
| `vite build` + 探针字面量扫描 | 构建通过 / **`__armPilot` 0 命中** |
| e2e `ui-smoke.mjs`（干净环境） | **88/88 PASS, 0 FAIL** |

**下一步（按优先级）**：

- **A. 补拍其余 6 块板**（`base_link` / `column_link` / `column_link_side` / `upper_arm_brace` /
  `forearm_brace` / `tool_link`）：需要额外姿态 / 机位，是"更像真机"**唯一**的主要增量。
  （原清单里的 `jaw_link` 已由 ADR **D66** 按实拍照片**反解形状**取代 —— 薄片、面积极小，
  "补拍纹理"对它收益最低，故移出。）
  ⚠️ **拍摄时必须一并解决黑件暗端**（对板测光 / 补光），否则照出来的照样是一堆 `≤3` 的码值；
- **B. 修四角**：当前框比板大约 30%（四边亮比例 18~62%）。需要一条**在定义上无歧义**的
  收紧方法，不能回到"调目标函数凑"（D59）。

> ⚠️ 上一轮给的选项"把线缆拨离板面"**已作废**：线缆本就在板背面，拨它没有意义（D61 四）。
> ⚠️ 上一轮的 **A（前端渲染接入）已完成**，原 **C（光照 / 材质调优）也已完成**（ADR D64），
> 二者均从待办中移除。
> ⚠️ **跑 e2e 前必须把 `start.bat` 拉起的那**一对**进程一起清掉**（后端 8090 + 带 `VITE_AUTO_CONNECT=ws`
> 的 vite 5273）：残留后端会让页面**加载即自动连上真机链路**，Actual 改由 WS 回推驱动，
> Mock 闭环类断言与位置型读数整体失真（实测 6 个假 FAIL，ADR **D67**）。清干净后 **88/88**。
> ⚠️ "照片纹理值不值"现在有**两层**答案：**形体更像了**；**信息量本身很低**（可见内容主要就是那几枚螺栓），
> 且**渲染层已到上限** —— 要不要继续投入取决于要不要按 A 走，且**必须同时改拍摄端**。
