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
- 红外：已实现红外遥控指令通道（见 `IR` 指令，逻辑同原 Arduino 遥控器映射）

## 目录结构
```
arm-device/
├── bsp/       硬件驱动 (uart 串口, servo 舵机 PWM, led 心跳指示)
├── core/      应用代码 (arm_control 运动控制, cmd 指令解析, main)
├── third_party/  第三方库 (本版暂无)
├── library/   芯片官方库 (ATmega328P 设备头由 avr-libc/工具链提供，见其 README)
├── scripts/   编译/下载/监视脚本 (固定路径指向 D:\tools\agent-tools)
├── tools/     host_verify.py 上位机自动验证脚本
└── platformio.ini
```

## 构建与下载
工具链（avr-gcc / avrdude）由 PlatformIO 安装到 `D:\tools\agent-tools\platformio-core\packages`，
脚本内已固定该根路径并自动解析版本子目录。

```bat
scripts\build.bat     # 编译 + 链接 + 生成 firmware.hex + 打印 FLASH/RAM 占用
scripts\upload.bat    # 烧录到 COM4 (avrdude, arduino/115200)
scripts\upload.bat COM3   # 可指定端口
scripts\monitor.bat   # 打开 COM4 9600 串口监视 (pio device monitor)
scripts\clean.bat     # 清理 .build
```

## 串口指令协议（COM4，换行结束）
- `SET <id> <角度>`：单舵机到角度（id ∈ 6/7/8/9），如 `SET 9 120`
- `SET <id> <角度> [id 角度]...`：组合控制，**最多 3 个**舵机；**左(8)与右(7)不可同条命令**
- `S<id>=<角度>`：单控简写，如 `S7=90`
- `STOP <id>`：停止自变化，冻结当前角度
- `AUTO <id>`：摇杆式自变化（在范围内三角波往返）
- `JOY <raw9> <raw8> <raw6> <raw7>`：摇杆整帧（4 路模拟量 raw 0~1023，逻辑同原 Arduino：>800 / <200 阈值各 ±1 微调）
- `JOY <id> <raw>`：单轴摇杆微调
- `IR <hexcode>`：红外遥控（NEC 32-bit 码，复刻原 Arduino 8 键映射，各 ±2 微调）
- `RESET`：全部回到 90°
- `STATUS` 或 `?`：打印 `S6=..(H/A) S7=.. S8=.. S9=..`
- `HELP`：打印帮助

规则：**单条 `SET` 最多控制 3 个舵机；左舵机(8)与右舵机(7)允许同时控制**（用户要求：左右舵允许同时工作）。
`JOY` / `IR` 复刻原 Arduino 行为，各轴/各键独立动作。

回显示例：`OK SET S9=120` / `ERR TOO_MANY` / `ERR BAD_ID S5`
`OK JOY S6=89 S7=89 S8=89 S9=89` / `OK IR 左 S9=92` / `ERR IR UNKNOWN DEADBEEF`

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
- 平滑运动：应用层每 ~20 ms 主循环将当前角向目标角按步长斜坡逼近；AUTO 模式三角波往返。
- 摇杆/红外：`JOY`/`IR` 指令以 `arm_nudge(id, delta)` 直接步进（不绕斜坡），复刻原 Arduino
  `handleJoystickControl`（阈值 >800/<200 → ±1）与 IR 遥控器 8 键（±2）的映射。
- LED 心跳：板载 PB5，主循环每 25×20ms = 500ms 翻转一次，作为工作指示。
- 串口：USART0 中断 RX/TX 环形缓冲，printf 基于 avr-libc `vsnprintf`。
