# ArmPilot

**mARM 机械臂全栈平台** —— 从**裸机固件** → **串口 / Web 网关** → **数字孪生与真机同步控制**，
用**一份模型定义**贯穿仿真与物理硬件。

> ArmPilot is an open-source platform for robotic arm data collection, simulation, AI training,
> motion planning, vision-based manipulation, and real-robot deployment.

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Firmware](https://img.shields.io/badge/firmware-ATmega328P%20%C2%B7%20avr--libc-8a2be2)](MeArm-Device)
[![Gateway](https://img.shields.io/badge/gateway-Go%201.21-00ADD8)](MeArm-RemoteControl)
[![Frontend](https://img.shields.io/badge/frontend-React%2019%20%C2%B7%20Three.js-61DAFB)](MeArm-3D)
[![Physics](https://img.shields.io/badge/physics-MuJoCo%203.13-brightgreen)](MeArm-3D/simulation)

![ArmPilot 控制台 · HOME / RESET 位](MeArm-3D/docs/images/armpilot-console.png)

<sub>HOME 位（四舵机全 90° = 固件 RESET 位）的数字孪生渲染，与实拍照片目视一致。</sub>

---

## 目录

- [1. 这是什么](#1-这是什么)
- [2. 系统架构](#2-系统架构)
- [3. 三个子项目](#3-三个子项目)
- [4. 功能矩阵](#4-功能矩阵)
- [5. 技术栈总览](#5-技术栈总览)
- [6. 完成情况](#6-完成情况)
- [7. 快速开始](#7-快速开始)
- [8. 仓库结构](#8-仓库结构)
- [9. 能力边界（诚实声明）](#9-能力边界诚实声明)
- [10. 文档索引](#10-文档索引)
- [11. License](#11-license)

---

## 1. 这是什么

ArmPilot 是一台 **meArm 型 4 自由度舵机机械臂**的完整开源实现，覆盖「硬件固件 → 网关服务 → 数字孪生」全链路：

- **真实硬件**：Arduino Uno（ATmega328P）裸机固件驱动 4 路舵机，支持串口指令、硬件摇杆（ADC）、红外遥控（NEC）与动作序列。
- **虚拟模型**：浏览器里的 3D 机械臂与真机共享同一个 `RobotModel`，做到「**你看到的虚拟臂，就是现实机械臂的实时映射**」。
- **真实物理**：MuJoCo 物理仿真作为**末端第三实现**接入，与运动学实现共享同一套协议与前端，**零改动**切换。

### 设计原则

| 原则 | 落地方式 |
|---|---|
| **唯一真值源** | 尺寸 / 关节 / 限位 / 标定只存在于 `MeArm-3D/config/robot.yaml`，前端、Go 后端、MuJoCo 生成器**都读它**，代码里禁止硬编码 |
| **先量后改** | 机构参数由「相机照片 + 舵机逐度扫描」反解得到，**实测推翻假设**（最初按固件命名推定的肩/肘角色是错的） |
| **不许伪造** | 物理仿真显式声明「标定层次」，能力边界（如无位置回读）在文档与 UI 中明确写出 |
| **真值冻结** | 运动学 / 物理语义核心哈希入库，**改外观放行、改真值报错**，必须有意识地解冻 |
| **可验收** | 每个阶段都有可复现的 pass/fail 判据与量化数字，不靠目视 |

---

## 2. 系统架构

三个子项目共用**同一片 Arduino Uno**，但提供**两条互相独立、端口刻意错开的控制路径**：

```
                         ┌──────────────── 浏览器 ────────────────┐
                         │                                        │
        ┌────────────────┴──────────────┐        ┌────────────────┴──────────────┐
        │  MeArm-3D  · 数字孪生          │        │  MeArm-RemoteControl · 遥控台   │
        │  React 19 + Three.js           │        │  Go + 内嵌 Web（three.js 双摇杆）│
        │  关节级：滑杆 / XYZ / 拖动 / 示教│        │  舵机级：归一化摇杆帧            │
        │  :8090 (/ws/joint)  :5273 dev  │        │  :8080 (WebSocket)  :9001 (TCP) │
        └────────────────┬──────────────┘        └────────────────┬──────────────┘
                         │ WebSocket（JSON，关节级）                │ WebSocket（JSON，摇杆帧）
                         ▼                                        ▼
        ┌──────────────────────────────────┐      ┌──────────────────────────────┐
        │  MeArm-3D/backend （Go 1.21）      │      │  arm-web （Go 1.21）          │
        │  device：sim │ serial │ mujoco    │      │  serial ➜ hub ➜ tcp/web       │
        │  ACK 门控 · latest-wins · 标定核对 │      │  ACK 门控 · latest-wins       │
        └────────────────┬─────────────────┘      └──────────────┬───────────────┘
                         │ 串口 115200 8N1                          │ 串口 115200 8N1
                         └──────────────────┬───────────────────────┘
                                            ▼
                         ┌──────────────────────────────────────┐
                         │  MeArm-Device · ATmega328P 裸机固件    │
                         │  串口指令 / 硬件摇杆(ADC) / 红外(NEC)   │
                         │  Timer1 PWM 四路舵机 · Timer2 1ms 时基 │
                         └──────────────────────────────────────┘
                                            ▼
                          底座(S9) · 左舵(S8) · 右舵(S7) · 夹取(S6)
```

- **8080 与 8090 刻意错开** ⇒ 两套前端可同时运行。
- 两条路径最终都归一化为**同一套文本指令**下发固件，因此行为完全一致。
- MuJoCo 物理仿真作为 `device` 接口的**第三实现**（`sim | serial | mujoco`）接入，
  协议层 / WebSocket / controller / 前端**全部零改动**。

---

## 3. 三个子项目

| 子项目 | 角色 | 技术栈 | 状态 |
|---|---|---|---|
| [**MeArm-Device**](MeArm-Device) | ATmega328P 裸机固件 | C/C++ · avr-libc · 直接寄存器 · PlatformIO(工具链) · avrdude | ✅ 零警告，指令回归 67/67 |
| [**MeArm-RemoteControl**](MeArm-RemoteControl) | 串口 → Web / TCP 网关 | Go 1.21 标准库 · `//go:embed` · RFC6455 · 内嵌 three.js 前端 | ✅ 门控 ~13ms，端到端 ~10ms |
| [**MeArm-3D**](MeArm-3D) | 数字孪生 + 关节级后端 + 物理仿真 | React 19 · TS · Vite · Three.js(R3F) · Zustand · Go 1.21 · MuJoCo 3.13 | ✅ Phase 1–14 + M1–M10 |

### 3.1 MeArm-Device · 裸机 AVR 固件

**不使用 Arduino 框架**，纯 C/C++ + avr-libc + 直接寄存器编程。

| 模块 | 功能 |
|---|---|
| `bsp/servo` | Timer1 CTC（TOP=ICR1=39999，20 ms 帧），COMPA 中断状态机**顺序调度 4 路脉宽**，按各舵机强制范围钳位 |
| `bsp/systick` | Timer2 **1 ms 时基**，主循环彻底**零阻塞**（取代忙等 `_delay_ms`） |
| `bsp/uart` | USART0 **115200 8N1（U2X）**，RX/TX 环形缓冲 + 中断驱动，`uart_printf` 基于 `vsnprintf_P` |
| `bsp/adc` | 4 路摇杆模拟量采样（AVCC 参考，预分频 /128，10-bit） |
| `bsp/ir` | **NEC 32-bit 解码**（PD2/INT0 双边沿 + Timer0/64 4 µs 时基，地址与命令双重取反校验） |
| `bsp/eeprom` | 红外按键**运行时学习**结果持久化，断电不丢 |
| `bsp/led` | 板载 D13 心跳指示（500 ms 翻转） |
| `core/arm_control` | 角度斜坡平滑（3°/20 ms）、目标/当前双角、`arm_all_reached()` 到位判定 |
| `core/cmd` | 串口指令解析：`SET` / `S<n>=` / `STOP` / `AUTO` / `JOY` / `IR` / `SEQ` / `JOYHW` / `IRHW` / `ADC` / `RESET` / `STATUS` / `IRLEARN` / `IRCODES` / `IRCLEAR` / `HELP` |
| `core/joystick` | 硬件摇杆扫描 + **比例步进**（偏移越大越快，2–10°/20 ms） |
| `core/ir_ctrl` | 13 键表（8 微调 + 4 动作集 + 1 停止）→ `arm_nudge` / 序列触发 |
| `core/ir_seq` | **动作序列引擎**：4 套各 7 帧关键帧（存 `PROGMEM`），"到位 + 保持间隔"才进下一帧，可循环 / 切换 / 被摇杆打断 |

- **命令-应答（ACK）契约**：每条指令**必须且仅有一次**应答行（`OK ...` / `ERR ...`）；
  异步事件统一以 `# ` 开头，供上位机门控区分 —— 这是网关侧 ACK 门控能成立的前提。
- **⚠️ AVR 内存铁律**：ATmega328P 仅 2 KB RAM，`avr-gcc` 默认把字符串字面量放进 `.data`（RAM）。
  本工程所有字面量走 `PSTR()` + `_P` 变体函数、所有常量表走 `PROGMEM` + `pgm_read_*`。
  修复后：`.data` ≈ 98 B · `.bss` = 344 B · **RAM ≈ 442 B，栈余量 ≈ 1606 B** ·
  **FLASH ≈ 11052 B（34%）**，零警告。
- 上位机回归：`python tools/host_verify.py COM4 115200` → **67/67 PASS**。

### 3.2 MeArm-RemoteControl · 串口 → Web / TCP 网关（`arm-web`）

- **串口层**：纯标准库实现（Windows 走 `syscall` 直连 kernel32，Linux/macOS 走 `stty`）。
  自动重连、**ACK 门控**（同一时刻仅 1 条在途）、**连接静默窗口 + 暖机包**
  （等 Uno bootloader 交权，吞掉首包再下发真指令）。
- **TCP 转发**：局域网设备 raw TCP 连接后可直接下发固件指令，为远程控制预留统一接口。
- **Web 控制台**：Three.js **双 3D 摇杆**（左：底座 + 左舵；右：夹取 + 右舵）、
  **动作死区 5°**（四轴全居中 ⇒ 整帧不发，串口零流量）、按住偏转 **30 ms 心跳持续步进**、
  S6–S9 角度面板（从所有含角度的应答**合并更新**）、**30 行滑动窗口**回显终端。
- **性能根治**：Windows 非重叠 I/O 读写互斥曾导致每条命令 218–932 ms、丢帧 75%；
  改用「立即返回」读模式 + 手动行缓冲（弃用 `bufio`，同时修掉空闲 20 s 必断连）
  ⇒ **门控单条周期 ~13 ms · WebSocket 端到端 ~10 ms**。
- 端口：**8080**（Web / WebSocket）· **9001**（TCP）· 均可在 YAML 配置。

### 3.3 MeArm-3D · 数字孪生 + 关节级后端 + 物理仿真

让**虚拟机械臂**与**真实机械臂**共享同一个 `RobotModel`，实现双向同步。

**前端（React + Three.js）**

| 能力 | 说明 |
|---|---|
| 唯一模型源 | `RobotModel` 从 `robot.yaml` 加载，几何 / 运动学 / 限位 / 标定全部由配置派生 |
| 3D 场景 | `buildRobotObject3D` 按件生成（倒角板 / 舵机 / 轴销 / 夹爪轮廓），支持**实拍照片纹理贴图** |
| 正向运动学 | `fk.ts` 与 Three.js 渲染矩阵**互相独立**、交叉验证 |
| 逆运动学 | `ik.ts` 平面 2R 解析解，错误码 `OUT_OF_WORKSPACE` / `JOINT_LIMIT`，多解 `elbow-up/down/nearest` |
| 关节控制 | 逐关节滑杆 + 角度限位 + 舵机标定换算 |
| 末端目标 | XYZ 直输 + **鼠标真实拖拽**（拖动平面在 pointerdown **冻结**，越界不钳位） |
| 传输抽象 | `RobotTransport` 接口：`MockTransport`（模拟有限角速度 / 延迟 / 丢帧 / 限位拒绝）与 `WebSocketTransport` |
| 链路反馈 | Phase 11 逐关节带符号偏差条 + 误差趋势 + 健康结论；Phase 12 **实际臂幽灵**（半透明第二条臂，露出的就是滞后量） |
| 示教 | Phase 13 `Record / Play / Pause / Stop / Clear / Export / Import`，录 `commandJoints`（不含链路时延），回放复用既有命令通道 |
| 状态仓库 | Zustand 单一状态源 + `transportBridge`（尾沿合并 33 ms / 回推只写 `actual` / 回环打破 / 重连补发） |

**后端（Go 1.21，自包含 module）**

| 包 | 职责 |
|---|---|
| `internal/robot` | 读 `robot.yaml`；关节 ↔ 舵机换算；限位校验（唯一「真值」入口） |
| `internal/protocol` | JSON / `JR` / `OK JR` / `STATE` / `ERR` 编解码（不认识机械结构） |
| `internal/controller` | **唯一「懂机械臂」处**：ACK 门控 · latest-wins · 标定核对 · 状态发布 |
| `internal/device` | `sim.go`（内置假固件）· `serial.go`（真串口）· `mujoco.go`（Python 子进程） |
| `internal/wsserver` | 标准库 RFC6455 服务端 · 路由 · 广播 · 两层心跳 |

**物理仿真（MuJoCo 轨 M1–M10）**

- MJCF **由 `robot.yaml` + `physics.yaml` 生成**（`gen_model.py`），**禁止手改 XML**，并有测试盯同步。
- 三层时间步解耦（物理 1 kHz / 控制 100 Hz / 渲染只读），批量步进与渲染帧率**按位可复现**。
- 用 **Vite SSR 桥**把前端真实 `ik.ts` 当**独立裁判**（不是用 Python 再写一份自证）。
- Level 声明机器可检查（`calibrated == false`），不伪造「已标定」。

---

## 4. 功能矩阵

| 能力 | MeArm-Device | MeArm-RemoteControl | MeArm-3D |
|---|:---:|:---:|:---:|
| 4 路舵机 PWM 控制 | ✅ | — | ✅（虚拟） |
| 串口指令协议 | ✅（固件侧） | ✅（网关侧） | ✅（关节级） |
| ACK 命令-应答门控 | ✅（契约方） | ✅ | ✅ |
| 硬件摇杆（ADC） | ✅ | — | — |
| 红外遥控（NEC）+ 按键学习 | ✅ | — | — |
| 红外动作序列（4 套） | ✅ | ✅（可下发） | — |
| 以太网 / TCP 透传 | — | ✅ | — |
| Web 3D 摇杆（舵机级） | — | ✅ | — |
| 数字孪生（关节级） | — | — | ✅ |
| FK / IK | — | — | ✅ |
| 鼠标拖拽末端 | — | — | ✅ |
| 示教录制 / 回放 | — | — | ✅ |
| 链路误差面板 / 幽灵臂 | — | — | ✅ |
| MuJoCo 真实物理仿真 | — | — | ✅ |
| 实拍照片纹理贴图 | — | — | ✅ |
| 真值冻结（语义哈希） | — | — | ✅ |

---

## 5. 技术栈总览

| 层 | 选型 |
|---|---|
| 固件 | **C / C++ · avr-libc · 直接寄存器编程**（ATmega328P @16 MHz）；PlatformIO 仅作工具链与板卡管理，**不链接 Arduino 框架**；avrdude 烧录 |
| 网关 | **Go 1.21**（标准库 + `gopkg.in/yaml.v3`）· `//go:embed` 自包含单文件 · Windows `syscall` / Unix `stty` · RFC6455 WebSocket |
| 数字孪生前端 | **React 19 · TypeScript 5.9 · Vite 8 · Three.js · React Three Fiber · @react-three/drei · Zustand** |
| 关节级后端 | **Go 1.21**（标准库 `net/http` + RFC6455 WebSocket + `yaml.v3`） |
| 物理仿真 | **MuJoCo 3.13（Python）** · MJCF 由 YAML 生成 · `numpy` |
| 上位机工具 | Python 3（numpy / Pillow / pyserial / pytest）· Node.js（e2e 与链路探针） |
| 测试 | **Vitest**（单元 / 验收）· `go test` · **pytest**（物理仿真）· 零依赖 **CDP e2e**（无头 Edge/Chrome）· Node 端到端脚本 |
| 图像 | Otsu 分割 · 对称 Chamfer 拟合 + Hooke-Jeeves · PCA 定角 · homography 透视校正 |

---

## 6. 完成情况

### 6.1 里程碑

| 子项目 | 里程碑 | 状态 |
|---|---|---|
| MeArm-Device | 裸机固件（PWM / 时基 / 串口 / 摇杆 / 红外 / 序列 / EEPROM） | ✅ 零警告 · 67/67 PASS |
| MeArm-RemoteControl | 串口网关 · ACK 门控 · TCP 透传 · Web 双 3D 摇杆 | ✅ 端到端 ~10 ms |
| MeArm-3D | Phase 1–14（模型 / 3D / FK / IK / 拖动 / Mock / Go WS / 真串口 / 反馈 / 幽灵 / 示教 / 被动腕） | ✅ |
| MeArm-3D | MuJoCo 物理轨 M1–M10 | ✅ |
| MeArm-3D | 照片纹理贴图（D63 / D64 / D66 / D68 / D69） | ✅ |
| MeArm-3D | ADR 决策记录 D1–D70 | ✅ 67 条 |

### 6.2 验收数据（可复现）

**MeArm-3D**

```
类型检查      tsc -b                        0 error
单元/验收测试  vitest run                   318 / 318 PASS（24 文件）
后端单测      go test ./...                 65 / 65 PASS（5 包）+ go vet 干净
浏览器 e2e    node tests/e2e/ui-smoke.mjs   88 / 88 PASS
物理仿真      pytest tests/sim              147 / 147 PASS（12 文件）
生产构建      vite build                    1,332.32 kB（gzip 378.73 kB），探针 0 泄漏

FK ↔ Three.js   200 组随机位姿              末端最大误差 8.673e-14 mm
FK(IK(XYZ))     2000 组随机可达位姿          最大残差 1.180e-13 mm（失败 0 组）
MJCF 结构       gen_model.py                9240 B · nq=5 nv=5 njnt=5 nu=4
                                            nbody=8 ngeom=34 nexclude=5 neq=1 ntendon=1 · 0.2173 kg
HOME 位 TCP     [115.0335, 0, 109.2236] mm  闭式解与前端 IK 逐位吻合
重力对照        无驱动 3 s                  Δshoulder 32.095° / Δelbow 30.990°（Δbase = Δgripper = 0.000°）
接触力判据      dist = +1.353 mm            qfrc_constraint 非零 · 禁用台面后 TCP 下落 14.62 mm
跨进程确定性    run.py --demo 跑两遍         12 行数值载荷逐字相同（seed=0）
```

**MeArm-Device**

```
构建          avr-gcc + avrdude             零警告
FLASH                                        ≈ 11052 B / 32 KB（34%）
RAM                                          ≈ 442 B（.data 98 B + .bss 344 B）· 栈余量 ≈ 1606 B
指令回归      python tools/host_verify.py   67 / 67 PASS
```

**MeArm-RemoteControl**

```
静态检查      go vet ./...                  干净
门控性能      ACK 门控单条周期                ~13 ms（改造前 218–932 ms）
端到端        浏览器 → 服务器 → 串口           ~10 ms
受控流        50 ms 节奏连续 JOY              40 / 40 全部下发
```

### 6.3 关键结论（实测推翻假设）

- **舵机命名 ≠ 运动学角色**：最初按固件 `SERVO_LEFT/RIGHT` 推定肩/肘，被相机实测推翻，
  真实映射为 **S9 底座 / S7 肩 / S8 肘 / S6 夹取**。
- **小臂是平行四连杆（绝对角耦合，增益 `-1`）**：由逐度扫描独立确证。
- **爪被连杆锁成水平（Phase 14）**：小臂绝对倾角变 **29.24°** 时爪的画面倾角只变 **7.31°**
  ⇒ 旧模型「爪固连小臂」被否决，改为**被动腕关节**，连带重推 IK 几何、MuJoCo 约束、
  自由度计数口径与台面高度。
- **肩标定增益仍有 −13.1% 偏差**，但 `tools/verify_calib_repro.py` 判定**跨批测量本身不可复现**
  （极差 10.8% / 52.9% > 5% 容差）⇒ 此时改模型或改标定表都不成立，**唯一入口是重布台面重测**（A3）。

---

## 7. 快速开始

### 7.1 只跑数字孪生（不需要硬件）

```bash
cd MeArm-3D/frontend
npm install
npm run dev            # 打开提示的地址（默认 :5273）
```

前端默认停在 `MockTransport` 且**不自动连接**（刻意设计，防误驱真机）。

### 7.2 一键启动（前端 + 后端）

```bat
cd MeArm-3D
start.bat              # 前置检查 + 端口探测 + 起前后端 + 打印本机/局域网地址
```

### 7.3 驱动真实机械臂

```bash
# 1) 烧录固件（Uno 接在 COM4）
cd MeArm-Device && scripts\build_upload.bat COM4

# 2) 用真机配置启动关节级后端（config.yaml 默认是 sim，不会碰硬件）
cd ../MeArm-3D/backend && go run . -c config.serial.yaml

# 3) 前端 Connection 面板 → ws://localhost:8090/ws/joint → Connect → 「关节控制」点 Real Robot
```

### 7.4 物理仿真（MuJoCo）

```bash
# 起一个 MuJoCo 末端（Python 子进程由 Go 侧拉起，走同一套文本协议）
python MeArm-3D/simulation/mujoco/server.py
# 前端零改动连上即可；hello.simulation_mode = "mujoco"
```

### 7.5 串口网关（摇杆级遥控台）

```bash
cd MeArm-RemoteControl
build.bat            # Windows；Linux/macOS 用 bash build.sh 或 make
arm-web.exe -c config.yaml
# 浏览器 http://127.0.0.1:8080    局域网 telnet <IP> 9001
```

---

## 8. 仓库结构

```
ArmPilot/
├── README.md                     # 本文件（总览）
├── LICENSE                       # MIT
├── docs/                         # 平台级文档（开发提示词记录等）
├── MeArm-Device/                 # ① ATmega328P 裸机固件
│   ├── bsp/                      #    硬件驱动（uart / servo / led / adc / ir / eeprom / systick）
│   ├── core/                     #    应用（arm_control / cmd / joystick / ir_ctrl / ir_seq / main）
│   └── tools/                    #    host_verify.py（上位机指令回归）+ NEC 解码单测
├── MeArm-RemoteControl/          # ② 串口 → Web / TCP 网关
│   ├── internal/                 #    config / protocol / serial / hub / tcp / web
│   ├── web/static/               #    内嵌前端（three.js 双摇杆 + 角度面板 + 回显终端）
│   └── tools/                    #    e2e-sim.js · tcp-test.js · headless-joystick-test.js
└── MeArm-3D/                     # ③ 数字孪生 + 关节级后端 + 物理仿真
    ├── config/                   #    robot.yaml（★唯一模型）· physics.yaml · 真值冻结基线
    ├── frontend/                 #    React 19 + Three.js（robot / components / store / tests）
    ├── backend/                  #    Go 关节级服务（robot / protocol / controller / device / wsserver）
    ├── simulation/mujoco/        #    MJCF 生成器 + 模型 + 服务 + Viewer + 记录
    ├── tests/sim/                #    MuJoCo 轨验收（pytest）
    ├── protocol/serial-v1.md     #    串口 / WS 协议基线
    └── docs/                     #    坐标系 / 模型结构 / 真机实测 / ADR 决策记录 / 采集指南
```

---

## 9. 能力边界（诚实声明）

本项目刻意**不夸大**能力，以下限制在文档与 UI 中均显式写出：

- **舵机没有位置回读**。`OK SET` / `STATUS` / 后端 `joint_state` 回传的都是**目标值**
  （「我打算去哪」），**没有一条能证明「它实际上在哪」** —— 哪怕机械臂卡死在桌上，回执依然一字不差。
  唯一的外部地面真值是**相机**。
- 因此「链路误差面板」反映的是**链路时延与限位截断**，不是「物理臂到位没有」；
  「幽灵臂」证明**链路走通了**，**不**证明物理到位。
- **物理仿真的 Level 声明机器可检查**（`calibrated == false`）：当前是「参数化物理」，
  尚未做真机逐点标定，目标是 3 → 4 而非 5。
- **肩标定增益偏差 −13.1% 未解决**，且跨批测量不可复现（详见 §6.3）。

---

## 10. 文档索引

| 想了解 | 去看 |
|---|---|
| 平台构成 / 铁律 / skill 路由 | 本文件 + [`.workbuddy/memory/MEMORY.md`](.workbuddy/memory/MEMORY.md) |
| 固件指令协议与内存铁律 | [`MeArm-Device/README.md`](MeArm-Device/README.md) |
| 串口网关 / ACK 门控 / 性能 | [`MeArm-RemoteControl/README.md`](MeArm-RemoteControl/README.md) |
| 数字孪生全貌与阶段验收 | [`MeArm-3D/README.md`](MeArm-3D/README.md) |
| 坐标系 / 单位 / 运动学链 | [`MeArm-3D/docs/coordinate-system.md`](MeArm-3D/docs/coordinate-system.md) |
| 模型结构（几何 vs 运动学边界） | [`MeArm-3D/docs/model-structure.md`](MeArm-3D/docs/model-structure.md) |
| 真机实测记录与不确定度 | [`MeArm-3D/docs/hardware-measurement.md`](MeArm-3D/docs/hardware-measurement.md) |
| 设计决策 ADR（D1–D70） | [`MeArm-3D/docs/decisions.md`](MeArm-3D/docs/decisions.md) |
| 物理仿真怎么跑 / 判据纪律 | [`MeArm-3D/simulation/README.md`](MeArm-3D/simulation/README.md) |
| 串口 / WS 协议基线 | [`MeArm-3D/protocol/serial-v1.md`](MeArm-3D/protocol/serial-v1.md) |
| 图像采集指南 | [`MeArm-3D/docs/texture-capture-guide.md`](MeArm-3D/docs/texture-capture-guide.md) |
| 开发提示词记录 | [`docs/`](docs) |

---

## 11. License

[MIT](LICENSE) © 2026 听心跳的声音
