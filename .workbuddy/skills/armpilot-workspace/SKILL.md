---
name: armpilot-workspace
description: ArmPilot 大工程（MeArm-3D 数字孪生 / MeArm-Device AVR 固件 / MeArm-RemoteControl 摇杆服务）的**工程导航与跨项目铁律**。覆盖：三个子项目的分工与端口分配（8080 摇杆 / 8090 关节级）、单一真值源 config/robot.yaml 的跨端共享方式、前端默认停在 Mock 与自动连接的 env 驱动约定、e2e 跨批次状态残留的必查清单（跑前清残留 dev server）、测量/标定结论不得写死的约束、以及"该查哪个专项 skill"的路由表。当在 ArmPilot 下新建模块、跨子项目联调、改动配置真值、跑验收、或不确定某项知识沉淀在哪时**先读本文件**。
agent_created: true
---

# ArmPilot 大工程 · 导航与铁律

大工程 `ArmPilot/` 下有三个子项目。**本文件是入口**：先在这里定位"该改哪儿、该查哪个 skill"，
再进专项文档。专项 skill 的**权威正文不在本文件**（避免双重维护），只登记路由。

## 0. 三个子项目的分工（别搞混端口）

| 子项目 | 角色 | 语言/栈 | 端口 | 通信 |
|--------|------|---------|------|------|
| `MeArm-Device/` | **下位机固件** | C + avr-libc（裸机，**不用 Arduino 框架**），ATmega328P @16MHz | 串口 | 文本协议，每条合法指令回 `OK ...` / `ERR ...`，事件以 `# ` 前缀 |
| `MeArm-RemoteControl/` | **舵机级**摇杆服务 | Go（标准库）+ 内嵌 Web | **8080**（WS）+ 9001（TCP 局域网） | 把固件串口暴露成 Web/TCP；指令文本**严格依赖 arm-device 协议** |
| `MeArm-3D/` | **数字孪生 + 关节级后端** | Go（`armpilot/backend`）+ React/Three.js 前端 | **8090**（`/ws/joint`）+ 5273（vite dev） | 关节级 WebSocket(JSON)；`wsserver → controller → device(sim \| serial)` |

> ⚠️ **8080 与 8090 刻意错开**，两者可同时跑。新增服务前先确认端口不撞。

## 1. ★ 铁律：模型 / 标定 / 限位真值只有一份

**唯一真值源 = `MeArm-3D/config/robot.yaml`**（前后端共享，构建期读入）。

- 前端通过 vite alias `@config` 读它；后端启动时 `robot.Load()` 读它。
- **禁止**在任何地方硬编码连杆尺寸、角度 offset、限位、舵机映射。
- 测试的期望值必须**从配置派生**，不许写常数（写死会与真值漂移，且改动时不会报错）。
- 舵机命名 ≠ 运动学角色（已用相机实测确证）。改机构参数一律走「先量后改」，见 §3 路由。

## 2. 前端「默认不连硬件」是刻意设计，别擅自改默认值

`MeArm-3D/frontend` 的传输方式**默认停在 MockTransport**（浏览器内仿真），
且**不会自动连接**任何后端。理由：网页一打开就控制真实机械臂是危险的。

因此"一键启动后要接硬件"只能靠**环境变量注入**（`start.bat` 负责）：

| 变量 | 作用 |
|------|------|
| `VITE_AUTO_CONNECT=ws` | 页面挂载后自动连 WebSocket 后端（而非浏览器内 Mock） |
| `VITE_AUTO_REAL=1` | 连接成功后自动切 Real Robot（带准入校验） |

`npm run dev` **不注入** ⇒ 手动开发行为完全不变。**不要**把 websocket 改成全局默认值：
那会破坏手动调试、并让生产构建去连一个不存在的后端。

> 详细踩坑见 §3 路由（`arm-ws-joint-link` / `arm-mechanism-photogrammetry` §9.13–9.14）。

## 3. ★ 路由表：想查什么，去哪个 skill

skill 存放在用户级 `~/.workbuddy/skills/`（跨工程可用）。本工程相关三个：

| skill | 管什么 | 典型触发 |
|-------|--------|---------|
| **`arm-mechanism-photogrammetry`** | 用「相机 + 舵机逐度扫描」反解真实机构参数（offset/scale/reverse/限位），写回 YAML | 「虚拟模型与实物不一致」「校准机械臂参数」「哪个舵机管哪个关节」「相机标定」 |
| **`arm-robot-serial`** | 串口链路的陷阱：Uno 开机静默窗口、首条指令被丢弃、ACK 门控、Windows 非重叠 I/O 读写互斥、波特率对 RTT 的影响 | 「首条指令无应答」「摇杆失效」「web 卡顿」「串口时延优化」「固件-上位机联调」 |
| **`arm-ws-joint-link`** | 关节级 WebSocket 闭环：回执是**目标角**不是实际位置、ACK 门控 + latest-wins、量化精度=误差下限、断线重连补发、hello 带模型真值互检 | 「误差永远不归零」「面板一直显示正在逼近目标」「回执看着对但状态不对」 |

> 三个 skill **各管一段、无重叠**：机构（几何真值）→ 串口（字节链路）→ WS（语义链路）。
> 遇到跨段问题（如"命令发了真机不动"）通常要**同时**看 `arm-robot-serial` 与 `arm-ws-joint-link`。

## 4. ★ 验收节奏（每个模块完成即走一遍）

1. **先出实现计划**并获确认（任何新模块动手前）
2. 实现 → **Debug/Release 双构零警告**（前端为 `tsc` 0 error）
3. 单测：`cd MeArm-3D/frontend && npm test`（vitest，当前 **233** 项）
4. 生产构建：`npm run build`（`vite build`）
5. 端到端：`node tests/e2e/ui-smoke.mjs`（当前 **52** 项，**需先起后端 + 前端**）
6. 增量汇报 + 交付清单（✅ 收尾）+ 同步 README / ADR / memory

后端单测：`cd MeArm-3D/backend && go test ./...`（56 项）+ `go vet` + `gofmt -l`。

## 5. ⚠️ 跑 e2e 前必查：清掉残留进程与带 env 的 dev server

这是本项目**最常复发的假 FAIL 来源**（已记入 ADR D40/D41/D43 的"跨批次状态残留"章节）。

```bash
# 1) 清残留后端与 dev server（8090 / 5273）
taskkill //F //IM armpilot-backend.exe 2>/dev/null
netstat -ano -p TCP | grep -E ":8090|:5273" | grep LISTENING   # 逐个 taskkill //F //PID <pid>

# 2) ★ 关键：必须用**干净的** vite dev（不带 VITE_AUTO_* env）跑 e2e
cd MeArm-3D/frontend && ./node_modules/.bin/vite --port 5273
```

> **为什么**：若 5273 上跑着之前注入过 `VITE_AUTO_CONNECT=ws` 的实例，页面会自动连后端，
> e2e 读到的表格/状态会被回推干扰 ⇒ 出现**与代码无关的 FAIL**。
> 判据：FAIL 项集中在"读数类"断言且换干净 dev 后即消失。

**前端资源占用**（e2e 需真实浏览器渲染，勿与其它重任务并行）：
e2e 用 headless Edge 直连 CDP，零额外依赖（Node ≥22 自带 `fetch`/`WebSocket`）。

## 6. 沙箱/工具链环境坑（WorkBuddy 会话内）

| 现象 | 处理 |
|------|------|
| `npx <tool>` 触发 **WSL 黑名单**（`wsl.exe`） | 改用 `./node_modules/.bin/<tool>` 直调 |
| 从 Bash 调 PowerShell 被安全策略拦截 | 用 PowerShell 工具；若其输出通道不回显，用 Bash 查 `netstat`/`tasklist` 复核 |
| `/tmp` 写入静默失败 | 日志/探针文件写到**项目根**（用完即删） |
| 沙箱每次工具调用**回收子进程** | "服务能否启动"的验证必须在**同一次调用内**起服务 + 发请求 |
| 运行过的 `.exe` 被锁 | 构建到新文件名，或申请沙箱放行后重建 |
| Git Bash 的 `sed -i` 剥 CRLF | 用 Python 二进制模式替换；提交前 `git diff --numstat` 自查 |

## 7. 新增子项目 / 新目录前：先探针

大工程根 `.gitignore` 是给 STM32 工程写的**黑名单**（`Drivers` / `third_party` / `Debug`(含小写
`debug`) / `Build`(含小写) / `Release` / `obj` 等）。被忽略的文件 `git add` 会**静默跳过**。

```bash
mkdir -p <新目录> && touch <新目录>/probe.txt
git check-ignore -v <新目录>/probe.txt   # 有输出 = 被屏蔽，换名
```

`.workbuddy/skills/` 已实测**未被屏蔽**（2026-09-12 探针确认）。

## 8. 文档落点约定

| 内容 | 落点 |
|------|------|
| 关键设计决策（"为什么这么做"） | `MeArm-3D/docs/decisions.md`（ADR，**最新在前**，代码注释引编号如 `D43`） |
| 项目全貌 / 验收数据 / 使用说明 | 各子项目 `README.md` |
| 跨会话工作日志 | 各自的 `.workbuddy/memory/YYYY-MM-DD.md`（append-only，最新在前） |
| 可复用的方法论 | 提升为 skill（见 §3 路由表），**不放 memory** |
