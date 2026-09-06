# meArm 裸机 AVR 机械臂固件

基于 Arduino Uno（**ATmega328P**）的机械臂控制固件。**不使用 Arduino 框架**，
纯 C/C++ + avr-libc + 直接寄存器编程，由 PlatformIO 管理工具链、由脚本完成
编译与下载。

> 参考工程：`arm-document/09.通用代码/vscode/meArm`（Arduino 版，含摇杆录制/回放
> 与红外遥控）。本工程在其机械结构与舵机映射基础上，改写为裸机实现并扩展串口指令控制。

## 硬件
- MCU：ATmega328P（Arduino Uno），16 MHz
- 串口：USART0，PC 端 **COM4**，9600 8N1（既打印状态，也接收控制指令）
- 舵机（PWM，50 Hz）：

  | 逻辑名 | 舵机 | 引脚 | 强制范围 |
  |--------|------|------|----------|
  | 底座旋转 | servo_9 | D9 (PB1) | 30° ~ 150° |
  | 左舵机   | servo_8 | D8 (PB0) | 20° ~ 100° |
  | 右舵机   | servo_7 | D7 (PD7) | 80° ~ 160° |
  | 夹取舵机 | servo_6 | D6 (PD6) | 40° ~ 125° |

- 开机全部复位到 **90°**
- 状态指示：**板载 LED（D13/PB5）默认每 500 ms 翻转一次**，作为固件心跳（工作指示）
- 摇杆（真实硬件）：4 路模拟量经 **ADC（A0..A3 = PC0..PC3）** 读取，阈值与逻辑同原 Arduino
  `handleJoystickControl`（raw >800 → -1，raw <200 → +1，左舵机 8 反向），每主循环经
  `joystick_scan()` 直接 `arm_nudge` 微调。由 `JOYHW ON|OFF` 开关。
- 红外（真实硬件）：**PD2（INT0）** 接收 NEC 协议。两个用途：
  - **8 键手动微调**：复刻原 Arduino 8 键映射（各 ±2），每主循环经 `ir_ctrl_poll()`
    解码并 `arm_nudge`。
  - **动作序列（新增）**：红外按钮 **1 / 3 / 7 / 9** 各触发一套约 15 s 的动作（中间 5 s 暂停、
    循环执行）；按钮 **5** 停止当前循环。详见下文物件 `core/ir_seq.c`。
  由 `IRHW ON|OFF` 开关。

## 目录结构
```
arm-device/
├── bsp/       硬件驱动 (uart 串口, servo 舵机 PWM, led 心跳, adc 摇杆采样, ir NEC 解码)
├── core/      应用代码 (arm_control 运动控制, cmd 指令解析, joystick 硬件摇杆, ir_ctrl 红外控制, main)
├── third_party/  第三方库 (本版暂无)
├── library/   芯片官方库 (ATmega328P 设备头由 avr-libc/工具链提供，见其 README)
├── scripts/   编译/下载脚本 (固定路径指向 D:\tools\agent-tools)
├── tools/     host_verify.py 上位机自动验证脚本 + test_ir_decode.py NEC 解码单测
└── platformio.ini
```

## 构建与下载
工具链（avr-gcc / avrdude）由 PlatformIO 安装到 `D:\tools\agent-tools\platformio-core\packages`，
脚本内已固定该根路径并自动解析版本子目录。

```bat
scripts\build.bat          # 编译 + 链接 + 生成 firmware.hex + 打印 FLASH/RAM 占用
scripts\build_upload.bat   # 一键编译并烧录到 COM4 (先 build 再 avrdude arduino/115200)
scripts\build_upload.bat COM3   # 可指定端口
scripts\upload.bat  [COM]  # 仅烧录 (firmware.hex 已存在时)
scripts\monitor.bat        # 打开 COM4 9600 串口监视 (pio device monitor)
scripts\clean.bat          # 清理 .build
```
> 所有脚本为**纯英文 `.bat`**（无中文，无 UTF-8 BOM），在 **cmd 与 PowerShell 中均可直接运行**
> （PowerShell 下 `.\scripts\build.bat` 即可，不受执行策略限制；未使用 `.ps1`）。
> 工具链根路径固定为 `D:\tools\agent-tools\platformio-core\packages`，脚本自动解析版本子目录。

## 串口指令协议（COM4，换行结束）
- `SET <id> <角度>`：单舵机到角度（id ∈ 6/7/8/9），如 `SET 9 120`
- `SET <id> <角度> [id 角度]...`：组合控制，**最多 3 个**舵机；**左(8)与右(7)允许同条命令**
- `S<id>=<角度>`：单控简写，如 `S7=90`
- `STOP <id>`：停止自变化，冻结当前角度
- `AUTO <id>`：摇杆式自变化（在范围内三角波往返）
- `JOY <raw9> <raw8> <raw6> <raw7>`：摇杆整帧（4 路模拟量 raw 0~1023，**方向阈值同原 Arduino** >800/<200；步进按偏移比例，约 2~10 度/次，硬推更快）
- `JOY <id> <raw>`：单轴摇杆微调（同上比例步进）
- `IR <hexcode>`：红外遥控（NEC 32-bit 码）。8 键手动微调（各 ±2）+ **动作序列键**：
  按钮 **1/3/7/9** 启动对应动作集、**按钮 5** 停止循环（见下）
- `SEQ 1|3|7|9`：启动对应红外动作集（运行中再发即**切换**到该套从头执行）
- `SEQ STOP`：停止当前动作循环（等价于红外按钮 5）
- `SEQ ?`：查询当前运行状态（`SEQ running N` / `SEQ idle`）
- `JOYHW ON|OFF`：开启/关闭**真实硬件摇杆**扫描（`joystick_scan()` 读 ADC A0..A3）
- `IRHW ON|OFF`：开启/关闭**真实硬件红外**接收（`ir_ctrl_poll()` 解码 PD2 NEC）
- `ADC`：打印 4 路摇杆轴原始值 `ADC A0=.. A1=.. A2=.. A3=..`（调试/接线用）
- `RESET`：全部回到 90°
- `STATUS` 或 `?`：打印 `S6=..(H/A) S7=.. S8=.. S9=..`
- `HELP`：打印帮助

规则：**单条 `SET` 最多控制 3 个舵机；左舵机(8)与右舵机(7)允许同时控制**（用户要求：左右舵允许同时工作）。
`JOY` / `IR` 既可作为**串口手动指令**测试，其逻辑也与**真实硬件**摇杆（ADC）/红外（NEC）完全一致：硬件路径
经 `joystick_scan()` / `ir_ctrl_poll()` 每主循环调用，阈值与按键映射同上。

回显示例：`OK SET S9=120` / `ERR TOO_MANY` / `ERR BAD_ID S5`
`OK JOY S6=89 S7=89 S8=89 S9=89` / `OK IR 左 S9=92` / `ERR IR UNKNOWN DEADBEEF`
`OK JOYHW ON` / `OK IRHW OFF` / `ADC A0=507 A1=527 A2=517 A3=528`
`# IR RAW=0xF708FF00` / `OK IRLRN slot 1 = A1B2C3D4 (saved)` / `# IR ? (unbound code)`
`OK IRSEQ 1 start (7 steps)` / `SEQ running 1` / `OK IRSEQ 1 stop` / `OK IRSEQ idle (not running)` / `SEQ idle`

> **上位机命令-应答（ACK）契约（适配 arm-web 门控）**
> - 每一条以 `\r\n` 结尾的串口指令，**都必须且仅有一次应答行**：合法指令回 `OK ...`，非法/错误回 `ERR ...`。
>   其中 `SEQ STOP` 在序列未运行时也会回 `OK IRSEQ idle (not running)`，保证上位机门控不会因“永远等不到应答”而误判通讯失败。
> - **异步事件**（非命令应答）以 `# ` 开头，不会被上位机误判为命令应答：
>   `# IR RAW=0x...`（硬件红外帧回显）、`# IR ? (unbound code)`（硬件红外未绑定码）、
>   `# IRSEQ stop: joystick`（摇杆指令在动作序列运行中触发自动停止）。
> - 上位机（arm-web）约定：**任何非 `# ` 开头的回显行即视为应答**；`# ` 行仅转发到日志，不清除“等待应答”状态。

### 红外动作序列（按钮 1/3/7/9，新增）
每套动作约 **15 s**：前 3 个关键帧 → **中间 5 s 暂停** → 后 3 个关键帧 → 循环。
各关键帧"移动到位 + 保持间隔"后才进入下一帧（确保机械臂确实到达姿态）。规则：

- **循环执行**：一套动作打完自动从头再打，直到被停止。
- **停止循环**：收到 **红外按钮 5**（需先用 `IRLEARN 5` 学习）或**任何摇杆指令**（硬件摇杆移动 / 串口 `JOY`）。
- **切换任务**：一套运行中收到**另一套按钮（1/3/7/9）**，立即停掉当前、从另一套第一帧开始。
- 4 套动作（在强制范围内安全姿态）：
  - **1 抓取放置**：到位张爪 → 闭爪 → 抬起 → 暂停 → 旋转 → 下放 → 张开
  - **3 左右摇摆**：底座 30→90→150 → 暂停 → 俯仰上下 → 回中
  - **7 俯仰**：上 → 中 → 下 → 暂停 → 下 → 中 → 上
  - **9 开合旋转**：闭 → 开 → 闭 → 暂停 → 左转 → 右转 → 回中
- 可用串口 `SEQ 1|3|7|9` / `SEQ STOP` / `SEQ ?` 在无遥控器时驱动/验证同一套逻辑。

### 硬件控制接线（真实摇杆 / 红外）
- 摇杆 4 轴 → Arduino **A0..A3（PC0..PC3）**，AVCC 参考、预分频 /128（~125 kHz），10-bit。
  映射：A0=底座(9) A1=左舵(8) A2=夹取(6) A3=右舵(7)；中位置（200~800）不动作。
- 红外接收头 → Arduino **D2（PD2 / INT0）**， demod 输出空闲高、有载波拉低；Timer0 预分频 64
  （4 µs/tick）配合 INT0 双边沿测距解码 NEC 32-bit（地址/~地址/命令/~命令 校验）。
  8 键映射（与原 Arduino 一致）：左/右→底座(9) +2/-2，数字2/8→左舵(8) +2/-2，
  上/下→右舵(7) +2/-2，数字4/6→夹取(6) +2/-2。
- **动作序列键（1/3/7/9 启动，5 停止）需要"学习"**：原 Arduino 工程只定义了 8 个微调键，
  遥控器数字键 1/3/5/7/9 的 NEC 码因遥控器而异、无法在编译期预设，因此改为**运行时学习**：
  - `IRLEARN 1` 然后按遥控器"1"键 → 该键的 NEC 码绑定到动作集 1（3/7/9 同理）。
  - `IRLEARN 5` 然后按遥控器"5"键 → 绑定为"停止"键。
  - 绑定结果写入 **EEPROM**，断电不丢失；`IRCODES` 查看当前绑定，`IRCLEAR` 清除全部。
  - 上电自动从 EEPROM 载入，无需每次重新学习。
- **每条硬件红外帧都会回显**：`IR RAW=0xXXXXXXXX`，方便用真实遥控器测试时读出每个按键的实际 NEC 码
  （未绑定的码回显 `IR ? (unbound code)`）。串口 `IR <hex>` 仍可注入任意 32-bit 码直接验证逻辑。
- NEC 码格式：`地址<<24 | ~地址<<16 | 命令<<8 | ~命令`（与原 Arduino IRremote `decodedRawData` 一致）。
- 上电默认 `JOYHW ON` 与 `IRHW ON`；若未接硬件，浮动 ADC / 无红外信号不会误动作
  （中位置/无有效帧被忽略），可用 `JOYHW OFF` / `IRHW OFF` 关闭以排除干扰。

## 上位机自动验证
```bat
pip install pyserial
python tools/host_verify.py            # 默认 COM4 / 9600
python tools/host_verify.py COM3 115200
```
脚本下发多套指令（单控/钳位/非法id/左右同时/超3个/组合/简写/自变化冻结/
摇杆单轴与整帧/红外8键±2/未知码），读取 COM4 回显做 pass/fail 计数并输出报告。
最新结果：全部 PASS（见每次运行输出）。

## 实现要点
- 舵机 PWM：Timer1 CTC（TOP=ICR1=39999，20 ms 帧，预分频 8），用 COMPA 中断状态机
  顺序调度 4 路脉宽（1.0~2.0 ms 对应 0~180°，再按各舵机强制范围钳位）。
- **主循环不再忙等**：原 `_delay_ms(20)` 改为 **Timer2 1 ms 时基**（`bsp/systick.c`，
  CTC 预分频 64、OCR2A=249）。主循环**自由运行、零阻塞**，`arm_tick()`（斜坡）与
  `joystick_scan()`（ADC）按 20 ms tick 节拍调用；LED 心跳按 500 ms tick 翻转；
  红外脉冲/序列引擎按 tick 手动判定时机。对红外/串口/序列事件的响应延迟降到最低。
- 平滑运动：应用层每 ~20 ms 将当前角向目标角按步长斜坡逼近（RAMP_STEP=3°/20ms）；
  AUTO 模式三角波往返。
- 摇杆（硬件）：`bsp/adc.c` 单 ADC（AVCC、/128、10-bit，阻塞读）；`core/joystick.c`
  `joystick_scan()` 读 A0..A3 → `joystick_delta(id, raw)`（阈值同原 Arduino，左舵反向）
  → `arm_nudge` 直接步进；并置 `g_moved` 边沿标志供序列引擎检测"摇杆指令"以停止循环。
  **步进为比例式**（偏移越大越快，约 2~10°/20ms）：原 Arduino 主循环每秒数千次、固定 ±1 尚快，
  本工程主循环按 20ms tick 节拍，固定 ±1 仅约 50°/s 会"卡顿"，故改为比例步进。
- 红外（硬件）：`bsp/ir.c` NEC 解码（PD2/INT0 双边沿 + Timer0/64 4µs 时基，32-bit 校验
  ~b0==b1 && ~b2==b3）；`core/ir_ctrl.c` `ir_ctrl_poll()` / `ir_ctrl_dispatch()` 每主循环
  匹配 13 键表（8 微调 + 4 动作集 + 1 停止）→ `arm_nudge` / `ir_seq_trigger` / `ir_seq_stop`。
- **动作序列引擎**：`core/ir_seq.c` 关键帧表存于 `PROGMEM`（4 套各 7 帧，含 1 个 5 s 暂停帧）。
  `ir_seq_tick()` 每循环调用：仅在"移动到位（`arm_all_reached()`）**且**保持间隔 `hold_ms` 已到"
  后才进下一帧（满足"到位后留间隔"），循环执行；检测到摇杆指令或按钮 5 即停止；收到另一套
  按钮（1/3/7/9）立即切换从头执行。运行态小结构体在 RAM，序列数据全在 flash。
- `JOY`/`IR` 串口指令与硬件路径**共用同一套映射**（`joystick_delta` 与 `ir_ctrl` 的键表），
  且串口 `IR <hex>` 与 `SEQ` 命令经 `ir_ctrl_dispatch` / `ir_seq_trigger` 复用硬件逻辑，保证一致。
- 串口：USART0 中断 RX/TX 环形缓冲，printf 基于 avr-libc `vsnprintf_P`。

### ⚠️ AVR 内存铁律（ATmega328P 仅 2 KB RAM）
avr-gcc **默认把所有字符串字面量拷贝到 `.data`（RAM）**——本工程曾因 help 文本 + 状态/
错误字符串约 **1348 字节**进入 `.data`，导致 `.data`+`.bss` 占满、栈仅剩 ~97 字节；
而 `uart_printf` 的 160 字节栈缓冲在命令处理时直接把栈顶砸进 `.bss`，表现为**下发指令即复位/乱码**。
修复方式（务必延续）：
- 所有字符串字面量用 `PSTR("...")` 放入 flash，打印走 `uart_puts()` / `uart_printf()`
  的 `_P`（flash）变体（`vsnprintf_P`）；
- 常量表（`IR_TAB` / `SEQ*` / `J_CH` / `J_ID` / `ids`）一律 `PROGMEM` + `pgm_read_*`。
修复后本版：`.data` ≈ 98 B、`.bss` = 344 B、RAM 占用 ≈ 442 B，**栈余量 ≈ 1606 B**，FLASH
≈ 11052 B（34%），零警告构建、稳定运行、动作序列/硬件控制全通过验证。
（注：`strcmp` 等短动词字面量仍在 `.data`，仅 ~48 B，不影响稳定。）
