# ArmPilot Serial Protocol v1

> **状态：§4 的固件侧 `JR`/`STATE` 仍待实现（Phase 9）；§5 的 Browser ↔ Go 侧已随
> Phase 8 落地并验收。** 本文是 Phase 9–12 的协议设计基线，供 `MeArm-Device`（AVR 固件）与
> `MeArm-RemoteControl`（Go 服务）同步扩展时对齐。Phase 1–7 完全不依赖本协议
> （Phase 7 走 `MockTransport`，不碰串口）。

## 1. 分层原则（不可破坏）

```
RobotModel  ──  唯一几何/限位来源（config/robot.yaml）
JointState  ──  唯一状态语言（关节角，degree）
Calibration ──  关节角 → 舵机角（offset / scale / reverse）
Protocol    ──  只负责把舵机角编成字节，不认识关节、不认识 IK
Transport   ──  只负责搬字节，不认识协议语义
```

**铁律**
- IK 永远只输出 `JointState`，绝不输出舵机角 / PWM / 通道号。
- 协议层不认识"关节"，只认识"舵机 ID + 角度"。
- 标定表只有一份（`config/robot.yaml`），固件与前端不得各存一份。

## 2. 关节 ↔ 舵机映射（v1 固定，来自 robot.yaml）

> ⚠️ **2026-09-12 实测修正**：舵机号与运动学角色**不是**一一对应的直觉关系。
> 早期版本按固件 `SERVO_LEFT/RIGHT` 的命名推定 `shoulder→S8 / elbow→S7`，**是错的**。
> 真机逐度扫描 + 白底暗件分割 + FK 骨架拟合（`docs/hardware-measurement.md`）确证：
> **S7 = 大臂/肩、S8 = 小臂/肘**。固件里的 left/right 只是**安装位**，不代表角色。
> 照旧映射写固件会得到一台反着动的机械臂。

| 关节 | 舵机 ID | 关节角范围 | 舵机角范围（固件硬限位） | 标定（关节 → 舵机） |
|------|---------|------------|--------------------------|---------------------|
| base | S9 | −60 … +60 | 30 … 150 | `θ + 90` |
| shoulder | **S7** | −6.09 … +49.45 | 80 … 160 | `1.44018·θ + 88.776` |
| elbow | **S8** | 108.44 … 141.86 | 20 … 100 | `−2.39401·θ + 359.61`（reverse） |
| gripper | S6 | 0 … 90 | 40 … 130 | `θ + 40` |

**elbow 的范围不是笔误**：小臂存的是**离开天顶的绝对倾角**（平行四连杆解耦），
真机可达区间就是 108°–142°，不存在"0° 小臂"。固件侧的限位校验必须照这张表，
不能套用"每个关节都 0–90"的通例。

标定表只有这一份真值，位于 `config/robot.yaml`。固件侧应由该文件**生成**，
而不是手抄 —— 抄两份必然漂移（见 §6 待办第一条）。

## 3. 现有固件协议（`MeArm-Device`，v0）

现状（`core/cmd.c`）是**舵机级文本协议**，作为 v1 的传输底座继续保留：

```
SET 9 120            # 单舵机
SET 9 120 8 90       # 组合（≤3 个舵机）
S7=90                # 简写
JOY 900 200 512 800  # 4 路摇杆 raw
STATUS               # 回 S6=.. S7=.. S8=.. S9=..
RESET                # 全部回 90°
```

回执契约：**每条指令恰好回一行**，合法 `OK ...`、非法 `ERR ...`；异步事件以 `# ` 开头
（`# IR RAW=...`）。上位机门控以"非 `# ` 开头的行 = 应答"判定。

## 4. v1 目标：关节级协议（新增，向后兼容）

ArmsPilot 的虚拟机械臂天然工作**关节空间**。v1 让固件也理解关节空间，
避免"上位机做标定、固件再做一次标定"的双份真值。

```
JR <j1> <j2> <j3> <grip>        # 关节角整帧（degree，浮点，保留 1 位小数）
JR 0 0.85 112.62 50             # HOME 位（= RESET，四舵机恰好全 90°）
JR 0 30 120 50                  # 肩前倾 30°、小臂绝对角 120°
```

回执（同时回出换算后的舵机角，便于上位机核对标定）：

```
OK JR S9=90.00 S7=90.00 S8=90.00 S6=90.00      # ← JR 0 0.85 112.62 50
OK JR S9=90.00 S7=131.98 S8=72.33 S6=90.00     # ← JR 0 30 120 50
ERR JOINT elbow 95.00 (limit 108.44..141.86)
```

注意第二条回执里 **S7 与 S8 都在动**：肩转 30° 时小臂的**绝对角**保持 120°，
但因为小臂相对大臂的夹角变了，两个舵机都要重新算。这正是"绝对角语义"在协议层的体现 ——
固件若按"相对角"实现，`S8` 会算出完全不同的值。

- 固件侧**内置同一张标定表**（由 `robot.yaml` 生成，或编译期常量 + 上电可写入）
- `JR` 与既有 `SET` / `JOY` 并存：`SET` 仍是舵机级直控（调试/回归用），`JR` 是关节级
- **`RESET` 语义保持"全部舵机 90°"**，即 HOME 位，不要改成"关节全 0"

### 状态回读

```
STATE <j1> <j2> <j3> <grip>          # 关节空间状态（由固件从舵机角反算）
# 或（有真实位置反馈时）
FB <j1> <j2> <j3> <grip>             # 实际关节角反馈，用于 Actual / Error 显示
```

无位置反馈的开环舵机下，`FB` 的值等于命令值；一旦换用带反馈的方案
（电位器 / 磁编码器 / 舵机回读），**上位机侧 UI 与误差计算无需改动**（见 D9）。

## 5. 上位机侧协议（Phase 8：Browser ↔ Go Server）—— **已实现**

实现落点：`MeArm-3D/backend/`（自包含 Go module `armpilot/backend`），
前端 `frontend/src/robot/transport/WebSocketTransport.ts` + `wsProtocol.ts`。

| 项 | 值 |
|----|-----|
| 默认监听 | `0.0.0.0:8090`，端点 `/ws/joint`，健康检查 `/healthz` |
| 为什么不是 8080 | `MeArm-RemoteControl` 已占用 8080（舵机级摇杆通道），两者**可同时运行** |
| 链路末端 | `device.mode = sim`（内置假固件，Phase 8）\| `serial`（真串口，Phase 9） |
| 应用层心跳 | 客户端发 `ping` / 服务端回 `pong`；间隔 10s（前端）/ 15s（服务端传输层 ping） |
| 传输层心跳 | 服务端发 RFC6455 `ping`，浏览器自动回 `pong`，40s 静默判死 |

### 5.1 消息全集（`version: 1`）

```jsonc
// ── 下行（浏览器 → 服务端）─────────────────────────────────────────────
{ "version": 1, "type": "joint_command", "timestamp": 1757654321000, "seq": 42,
  "joints": { "base": 0, "shoulder": 20, "elbow": 120, "gripper": 50 } }
{ "version": 1, "type": "ping",            "timestamp": 1757654321000 }
{ "version": 1, "type": "status_request",  "timestamp": 1757654321000 }

// ── 上行（服务端 → 浏览器）─────────────────────────────────────────────
// 接入即推 + status_request 响应：模型真值（"标定表只有一份"的在线校验源）
{ "version": 1, "type": "hello", "timestamp": 1757654321000,
  "model": { "id": "marm", "name": "mARM", "source": "…/config/robot.yaml",
             "jointOrder": ["base","shoulder","elbow","gripper"],
             "limits": [ { "id": "elbow", "role": "elbow", "min": 108.4414852068, "max": 141.8582211436 }, … ],
             "calibration": [ { "jointId": "shoulder", "servoId": "S7", "channel": 7,
                                "offset": 88.776, "scale": 1.44018, "reverse": false,
                                "servoLo": 80, "servoHi": 160 }, … ],
             "homePose": { "base": 0, "shoulder": 0.8498937633, "elbow": 112.6185771989, "gripper": 50 } },
  "device": "sim", "connected": true }

// 状态：关节角由链路末端从**舵机实际角反算**（不是命令回显）
{ "version": 1, "type": "joint_state", "timestamp": 1757654321100,
  "joints": { "base": 0, "shoulder": 19.8, "elbow": 120.1, "gripper": 50 } }

{ "version": 1, "type": "pong",          "timestamp": 1757654321010, "seq": 42 }
{ "version": 1, "type": "device_status", "timestamp": 1757654321000,
  "device": "sim", "connected": true }

// 事件 / 错误（**不是**连接终态：回一行 ERR，链路仍然活着）
{ "version": 1, "type": "error", "timestamp": 1757654321200,
  "code": "JOINT_LIMIT", "message": "elbow 95.00 超出 108.44..141.86" }
```

错误码（`protocol.Code*` / `wsProtocol.CODE_*`，两侧必须一致）：

| code | 含义 |
|------|------|
| `BAD_MESSAGE` | JSON 非法 / 缺 `type` / 关节值非数字 |
| `VERSION_MISMATCH` | `version` 非 0 且 ≠ 1（字段语义会静默错解，必须拒） |
| `JOINT_LIMIT` | 关节或舵机越界（前端计入 `rejected`，**不**断开连接） |
| `DEVICE_UNAVAILABLE` | 链路末端不可用（串口打开失败等） |
| `ACK_TIMEOUT` | 单条指令在 `ack_timeout_ms` 内没等到回执 |
| `INTERNAL` | 其它内部错误 |

### 5.2 Go 侧职责分层（不许 WebSocket 层直接操作串口）

```
WebSocket Client → Protocol(编解码) → Robot Controller → Device(sim | serial) → AVR
```

| 层 | 包 | 只负责 | **绝不**负责 |
|----|----|--------|--------------|
| 编解码 | `internal/protocol` | JSON / `JR` / `OK JR` / `STATE` / `ERR` 的字符串↔结构体 | 不认识关节、不认识标定、不认识机械结构 |
| 标定与限位 | `internal/robot` | 读 `config/robot.yaml`；关节↔舵机换算；限位校验 | 不认识 socket、不认识串口 |
| 控制器 | `internal/controller` | **唯一"懂机械臂"处**：ACK 门控、latest-wins、标定回执核对、状态发布 | 不认识 HTTP / WebSocket 帧 |
| 链路末端 | `internal/device` | `sim`（假固件）/ `serial`（Phase 9） | 不认识关节语义（只收发字节行） |
| 对外 | `internal/wsserver` | RFC6455 帧、路由、广播、心跳 | 不认识关节语义（只转发） |

### 5.3 四条**必须做对**的语义（都有测试锁死）

1. **`OK JR` 携带的是目标舵机角，不是实际位置。**
   它只用于**核对标定**（与本地标定算出的值比对，偏差 > 0.1° 报警告）。
   ❌ 拿它当 Actual 回推 ⇒ 误差恒为 0，整条误差链路形同虚设。
   ✅ 状态只能来自 `STATE` 行（由舵机**实际角**反算）。

2. **ACK 门控 + latest-wins。**
   同一时刻**只有 1 条 `JR` 在途**（Phase 9 串口 115200 + ACK 门控下必须如此）。
   在途期间新的 `joint_command` **只覆盖"待发槽"，不排队** —— 排队会让拖动时命令堆积，
   回执永远在追历史的某个中间位置。

3. **`min_send_interval_ms > 0` 时不许用 `time.Sleep`。**
   睡眠会卡住读循环，而回执正需要读循环来消费 ⇒ 自锁。正确做法是撤下在途、放回待发槽、
   用定时器补发。

4. **心跳是两层的。**
   浏览器 `WebSocket` API 不暴露传输层 `ping`，所以应用层必须自己发 `{"type":"ping"}` 等 `pong`；
   同时服务端发 RFC6455 `ping`（浏览器自动回 `pong`）由服务端看门狗判死。
   ⚠️ 上一轮 ping 仍在途时**不得**重复发，否则超时判定会被自己不断推后。

### 5.4 ⚠️ 链路精度 = 跟踪误差的可分辨下限

`JR` 只保留 **1 位小数**（0.1°），因此：

- 前端内部的**全精度**命令（如 HOME 位 `elbow = 112.6185771989`）与线上值（`112.6`）
  天然相差 `0.0186°`。**跟踪误差必须在线上精度上定义**，否则会得到一个
  **永远收敛不到 0 的量化残差**（实测表现为面板恒显 `0.02°`、`moving` 永不归零）。
  前端的处理：`wsProtocol.quantizeForWire()` —— 比对前把命令归整到 0.1°（`WIRE_JOINT_STEP_DEG`）。
- 0.1° 关节 ≙ 舵机 `0.1 × scale` 度：肩 `0.14°`、肘 `0.24°`。
  舵机自身（8 位定时器 / 1µs tick）分辨率约 `0.18°`，**同量级**，
  故 1 位小数没有浪费带宽也没有损失真实精度。


## 6. 待办（Phase 9 起）

- [ ] 由 `config/robot.yaml` **生成**固件标定表（避免手抄导致双份真值）
- [ ] 固件 `core/cmd.c` 增加 `JR` / `STATE <关节>` 解析与回执
- [x] Go `internal/protocol` 增加关节级编解码 + 单测（Phase 8：`EncodeJR` / `EncodeOKJR` /
      `EncodeState` / `ParseReply` / `ServoAnglesToJoints`，含"`ERR` 判定必须先于 `OK JR`"）
- [ ] `internal/device/serial.go` 落地真实串口（已留桩：显式返回"未实现"，不静默降级）
- [ ] `MeArm-RemoteControl` 现有摇杆通道与关节通道并存，注意 ACK 门控饥饿问题
      （见 skill `arm-robot-serial` 关键坑 6：周期查询会饿死遥控流）

### 6.1 Phase 9 接串口时的实测坑（已写入 `device/serial.go` 注释）

1. **ATmega328P 开机静默窗口**：打开串口拉低 DTR 会复位 MCU，bootloader 交权期内
   （约 1–2s）指令被吞且无回执。`SimTuning.BootMs` 就是为复现这条而存在的参数。
2. **Windows 非重叠 I/O 的读会立即返回 0 字节**（不是阻塞），照 `bufio.Scanner` 写会
   得到"读循环空转 100% CPU"或"半个包就当一行"。
3. **不要用 `bufio`**：它会把"还没收到换行的一行"留在内部缓冲，超时判定的时间基准就错了。
4. **状态应由固件主动上报**（周期 `STATE`），上位机轮询 `STATUS` 会与 ACK 门控互相饿死。
