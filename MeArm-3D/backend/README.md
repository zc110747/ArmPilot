# armpilot-backend · 关节级 WebSocket 服务（Phase 8–9）

ArmPilot 数字孪生的后端：把浏览器下发的**关节角**转成 arm-device 文本协议写到链路末端，
并把末端反算回来的关节角推回浏览器。

```
Browser ──WebSocket(JSON)──▶ wsserver ──▶ controller ──▶ device ──▶ AVR
         ◀──joint_state─────           ◀──            ◀──(STATE)
```

与 `MeArm-RemoteControl`（Go 摇杆服务，端口 8080）的分工：

| 服务 | 语言层 | 语义 | 端口 |
|------|--------|------|------|
| `MeArm-RemoteControl` | 舵机级文本（`SET` / `JOY`） | 摇杆遥控 | 8080 |
| **`armpilot-backend`** | **关节级 JSON ↔ `JR`** | **数字孪生 / IK / 拖动** | **8090** |

两者可同时运行，互不干扰。

## 1. 快速开始

```bash
cd MeArm-3D/backend
go build -o bin/armpilot-backend.exe .

# ① 仿真（默认，链路末端 = 内置假固件）
./bin/armpilot-backend.exe -c config.yaml

# ② 真机（Phase 9）：驱动物理舵机，串口见 config.serial.yaml
./bin/armpilot-backend.exe -c config.serial.yaml
```

> ⚠️ **真机模式会真的动舵机。** 两个配置的差别只有 `device.mode` / `device.serial.*`；
> **关节限位、标定（offset/scale/reverse）、HOME 位一律仍来自 `../config/robot.yaml`**
> —— 这是"标定只有一份"铁律在配置层的体现，真机模式不允许有自己的第二份标定。

启动后：

```
模型真值: E:\...\MeArm-3D\config\robot.yaml
robot: mARM (mearm)  base(S9 -60.00..60.00) shoulder(S7 -6.09..49.45) elbow(S8 108.44..141.86) gripper(S6 0.00..90.00)
链路末端: sim (舵机 240°/s · 延迟 15ms · tick 20ms · 限位校验 true)
就绪：浏览器连接 ws://127.0.0.1:8090/ws/joint
健康检查: http://0.0.0.0:8090/healthz
```

健康检查（供 e2e / 编排脚本等待就绪）：

```bash
curl -s http://127.0.0.1:8090/healthz
# {"clients":0,"device":"sim","linked":true,"ok":true,"state":{"base":0,"elbow":112.6185771989,...}}
```

## 2. 唯一真值：`config/robot.yaml`

本服务**不存任何限位或标定数值**。启动时读 `../config/robot.yaml`（前端、固件共用同一份），
派生：

- 关节顺序 `JointOrder()` = `JR` 四元组的位次（`base shoulder elbow gripper`，跳过 fixed 的 `tool`）
- 限位校验（越界返回 `ERR JOINT <id> <v> (limit <min>..<max>)`，文案与固件一致）
- 标定换算 `servo = reverse ? (-θ·scale + offset) : (θ·scale + offset)` 及其逆

> 路径支持回退：`../config/robot.yaml` → `config/robot.yaml` → `../MeArm-3D/config/robot.yaml`，
> 因此从 `backend/` 或 `MeArm-3D/` 启动都能找到。全部失败时会**列出所有尝试过的绝对路径**。
>
> 为什么强调这点：实测已证明"按固件命名推定舵机角色"会得到反着动的机械臂
> （S7=肩 / S8=肘 是靠相机实测纠正的，见 `docs/hardware-measurement.md`）。
> 三处各存一份标定表，必然漂移。

`backend/config.yaml` **只放运行参数**（端口、设备模式、模拟器参数、ACK 超时）。

## 3. 协议

### 浏览器 ↔ Go（JSON）

```jsonc
// 下行
{ "version": 1, "type": "joint_command", "timestamp": 1757654321000, "seq": 12,
  "joints": { "base": 0, "shoulder": 20.8, "elbow": 112.6, "gripper": 50 } }
{ "version": 1, "type": "ping" }
{ "version": 1, "type": "status_request" }

// 上行
{ "version": 1, "type": "hello", "model": { "id": "mearm", "jointOrder": [...],
  "limits": [...], "calibration": [...], "homePose": {...} }, "device": "sim" }
{ "version": 1, "type": "joint_state", "timestamp": 1757654321100,
  "joints": { "base": 0, "shoulder": 19.8, ... } }
{ "version": 1, "type": "error", "code": "JOINT_LIMIT",
  "message": "ERR JOINT elbow 95.00 (limit 108.44..141.86)" }
{ "version": 1, "type": "pong" }
{ "version": 1, "type": "device_status", "device": "sim", "connected": true }
```

错误码：`BAD_MESSAGE` / `VERSION_MISMATCH` / `JOINT_LIMIT` / `DEVICE_UNAVAILABLE` / `ACK_TIMEOUT` / `INTERNAL`。
`seq` 是 spec 字段之外的**可选扩展**（拖动时前端做去重/乱序丢弃）；旧客户端不发也能工作。

`hello` 里的 `model` 是**在线校验手段**：前端拿它与本地 `RobotModel` 比对限位/标定，
不一致就说明两侧读的不是同一份 `robot.yaml`。

### Go ↔ 设备（arm-device 文本，`protocol/serial-v1.md` §4）

```
JR 0.0 20.8 112.6 50.0                        下发（保留 1 位小数）
OK JR S9=90.00 S8=118.73 S7=90.00 S6=90.00    受理（携带**目标**舵机角，用于核对标定）
ERR JOINT elbow 95.00 (limit 108.44..141.86)  拒绝
STATE 0.00 20.80 112.60 50.00                 状态（由**舵机实际角反算**，主动上报）
```

心跳分两层，缺一不可：

- **传输层**：服务端每 15s 发 `ping`，浏览器自动回 `pong`；服务端看门狗据此判死连接
- **应用层**：浏览器发 `{"type":"ping"}` 判断服务端存活（浏览器 API 不暴露传输层 ping）

> ⚠️ **`JR` 的 1 位小数就是跟踪误差的可分辨下限。**
> 前端内部是全精度浮点（HOME 位 `elbow = 112.6185771989`），线上是 `112.6`；
> 拿两者直接相减会得到一个**永远收敛不到 0 的量化残差**（实测面板恒显 `0.02°`、
> `moving` 永不归零 —— 一个**假误差**）。所以前端的误差比对会先
> `quantizeForWire()` 把命令归整到 0.1°（`WIRE_JOINT_STEP_DEG`）。
> 这是**协议层**的事实，选 1 位小数也不是随手定的：0.1° 关节 ≙ 肩 `0.14°` / 肘 `0.24°`
> 舵机角，与舵机自身约 `0.18°` 的分辨率同量级。详见 `docs/decisions.md` D31。

## 4. 关键设计决策

| 决策 | 理由 |
|------|------|
| **sim 不做等值回显** | 内部维护舵机空间的 target/actual，以有限角速度逼近，并把 actual **反算**回关节角上报。等值回显会让"误差/收敛/丢帧"一条都没被验证，全部推迟到 Phase 11 第一次暴露 |
| **OK JR 不产生 `joint_state`** | 回执携带的是**目标角**（"我要去哪"），不是实际位置。拿它当 Actual 回推，误差会恒为 0 |
| **latest-wins，不排队** | ACK 门控下在途期间新命令只覆盖"待发槽"。排队会让拖动时命令无限堆积，回执永远在追赶历史（skill `arm-robot-serial` 关键坑 6） |
| **ERR 回执也放行门控** | 否则一次限位拒绝会把链路彻底卡死 |
| **间隔节流禁用 `time.Sleep`** | 睡眠会卡住读循环，而回执正需要读循环消费 ⇒ 自锁。改为撤下在途 + 放回待发槽 + 定时器补发 |
| **同步拒绝 vs 异步错误** | 限位越界在 `Apply` 的返回值里立刻拒（浏览器同步收到 `error`）；回执超时/设备拒绝走事件回调 |
| **WebSocket 层不碰 Device** | 所有控制意图必须经 controller（spec §二十二） |
| **切 serial 时接口不变** | `device.mode=serial` 显式报错而非静默退回 sim —— 静默降级比崩溃更危险 |
| **模型真值在线互检** | `hello` 回传本服务读到的限位/标定/通道，前端逐项比对。这是"标定只有一份"唯一能被在线检验的地方 |

## 5. 测试

```bash
cd MeArm-3D/backend
GOFLAGS=-mod=mod GOPROXY=off go test ./... -count=1      # 56 / 56 PASS（5 包）
# ⚠️ 本环境 go test -race 无法启动（0xc0000139），需要竞态检查请换机跑
go vet ./... && gofmt -l .                               # 无输出
```

前端的 e2e（`frontend/tests/e2e/ui-smoke.mjs`）会**自动拉起本服务**
（`bin/armpilot-backend.exe -c config.yaml`）并在结束时清理进程，
其中 21 项断言覆盖：连接 / `hello` 链路末端 / 模型一致性 / 送达前滞后 / 收敛 /
跟踪误差归零 / 心跳 RTT / 双向日志 / **杀掉后端后自动重连** / **重启后补发当前命令**。

| 包 | 覆盖 |
|----|------|
| `internal/robot` | yaml 解析、关节顺序、标定往返、限位边界（闭区间）、错误文案稳定性（map 迭代顺序随机但文案必须稳定）、舵机区间落在固件硬限位内 |
| `internal/protocol` | `JR` 编码（1 位小数量化）、`OK JR` / `STATE` / `ERR` 解析、`ERR` 判定先于 `OK JR`、舵机角反算**非恒等** |
| `internal/device` | 初始 HOME 位、限位拒绝且**位置不动**、延迟期内不回执、有限角速度逐步逼近、RESET 语义（舵机全 90°）、STATUS 回显、开机静默窗口、瞬时至位、Close 幂等 |
| `internal/controller` | 限位同步拒绝且不写设备、`JR` 编码、**ACK 不发布状态**、STATE 发布、latest-wins 合并、ERR 放行门控、ACK 超时上报并补发、设备不可用拒绝、部分帧保留其余关节、真 sim 集成（6 步收敛） |
| `internal/wsserver` | 握手 + `hello` 模型真值（含通道映射 S7=肩/S8=肘）、完整往返并**先经过中间态**、限位拒绝、版本不匹配、ping/pong、坏 JSON、多客户端广播、`/healthz` |

## 6. Phase 9：真串口（**已完成**）

`device.Device` 接口的价值在这里兑现：`NewSerial` 落地后 `controller` / `wsserver` **零改动**。

实现要点（来自 skill `arm-robot-serial` 的实测经验）：

1. Uno 开串口后 bootloader 有 ~2.5s 交权期，窗口内指令被吞 → `connect_settle_ms: 2600` 静默窗口
2. 连接后首包常丢 → 先发一个暖机包
3. Windows 非重叠 I/O 读写互斥（单条命令 200~900ms）→ 必须用「立即返回」读超时 + 手动行缓冲，
   且**不能用 `bufio`**（空闲 20s 会因空读抛错而断连）
4. 命令-应答门控下周期性 `STATUS` 查询会饿死控制流 → 状态改由固件**主动上报** `STATE`

### 6.1 ⚠️ 真机没有位置反馈：`joint_state` **不等于**物理到位

这是接真机后最容易踩的认知陷阱，也是本后端**能力边界**所在：

meArm 固件**没有编码器、没有电位器回读**。`arm_get_angle()` 返回的是固件变量里记着的
**目标值**。因此：

| 环节 | 它在说什么 | 能证明"物理到位"吗 |
|------|-----------|------------------|
| 固件 `OK SET` / `OK JR` | "我把目标设成了 X" | ❌ |
| 固件 `STATE` | "我记得我应该在 X" | ❌ |
| 后端 `joint_state` 回推 | 上面这条的转发 | ❌ |
| **`tools/verify_pose.py` 相机反解** | 它**实际上**在哪 | ✅ **唯一途径** |

**机械臂卡死在桌面上，上面四条回执依然一字不差。** 所以本后端的 `joint_state`
在验收口径里**只作链路自洽性参考**（证明命令确实穿过了整条链路），
**绝不作为「到位」证据**。要验收物理到位，必须走相机：

```bash
# 真机端到端闭环（会真的驱动机械臂 + 调用相机抓帧）
node MeArm-3D/tools/verify_serial_e2e.mjs
```

它串起 `WebSocket → 本服务 → 串口 → 固件 → 舵机 → ffmpeg 抓帧 → verify_pose.py 反解比对`。

### 6.2 Phase 9 首次真机闭环实测（PASS 18 / FAIL 1）

| 项 | 值 |
|----|-----|
| 链路末端 | `hello` → `device=serial`（真机） |
| 标定单一真值 | `homePose` 与 `config/robot.yaml` **逐位一致**（容差 `1e-6`） |
| 开机就绪门 | Uno DTR 复位静默窗口 2.7s；不等待会报 `DEVICE_UNAVAILABLE: 串口未就绪` |
| 链路回推 | 7 步 JR `max\|Δ\| ≤ 0.004°`（**纯链路自洽，不含物理**） |
| 相机重复性 | 同位姿两帧反解差 肩 `0.26°` / 肘 `0.01°` |
| 增益复核 · 肘 | 反解 `−0.4235` vs yaml `−0.4177` ⇒ **+1.4%，肘标定被独立证实 ✅** |
| 增益复核 · 肩 | 反解 `+0.6033` vs yaml `+0.6944` ⇒ **−13.1%，肩标定需重测 ❌** |
| ⚠️ 主导误差源 | 同台面同取景相隔 2 分钟两批，锚点绝对角偏置 `+2.69° → +7.75°`（**漂 5°**），Otsu 阈值两批同为 164 ⇒ **相机自动曝光** |

### 6.3 固件侧仍待办

- 固件 `core/cmd.c` 实现 `JR` / `STATE` 解析与回执（`protocol/serial-v1.md` §4）
- 固件内置标定表**由 `config/robot.yaml` 生成**，避免手抄造成双份真值

决策记录：`docs/decisions.md` **D34**（相机是唯一真值）· **D35**（帧间差为锐利判据）·
**D36**（自动曝光是主导误差源 ⇒ 必须锁死曝光）。
