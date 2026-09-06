// package protocol 定义了与 arm-device 固件对接的串口指令集。
//
// 串口处理严格依赖 arm-device 的指令协议（详见 arm-device/README.md 与
// core/cmd.c）：所有下发给设备的内容都必须是此处 grammar 认可的命令，避免
// 非法字符串烧入固件造成未知行为。Web / TCP 收到的控制意图最终都归一化为
// 下面这些命令文本，再经串口写出。
package protocol

import (
	"fmt"
	"strconv"
	"strings"
)

// 设备支持的舵机 id 与固件一致：6=夹取 7=右 8=左 9=底座
var validIDs = map[int]bool{6: true, 7: true, 8: true, 9: true}

// AxisMap 描述双 3D 摇杆（遥控形式）到 4 路舵机的映射关系：
//   - 左摇杆 X -> SERVO_BASE(9)，Y -> SERVO_LEFT(8)
//   - 右摇杆 X -> SERVO_GRIP(6)，Y -> SERVO_RIGHT(7)
// 每个轴可独立反转方向（invert_*），无需改代码即可适配实际安装。
type AxisMap struct {
	LXServo int // 左摇杆 X 轴 -> 舵机 id（默认 9=底座）
	LYServo int // 左摇杆 Y 轴 -> 舵机 id（默认 8=左舵）
	RXServo int // 右摇杆 X 轴 -> 舵机 id（默认 6=夹取）
	RYServo int // 右摇杆 Y 轴 -> 舵机 id（默认 7=右舵）
	InvLX   bool
	InvLY   bool
	InvRX   bool
	InvRY   bool
}

// DefaultAxisMap 返回推荐映射（遥控形式：左=底座/左舵，右=夹取/右舵）。
func DefaultAxisMap() AxisMap {
	return AxisMap{
		LXServo: 9, LYServo: 8, RXServo: 6, RYServo: 7,
		InvLX: false, InvLY: false, InvRX: false, InvRY: false,
	}
}

// Validate 校验一条原始命令是否符合 arm-device 语法。
// 返回 (ok, normalized, error)。normalized 是规整后的下发文本（去除多余空白）。
// 由于固件对所有字面量用 PSTR 且容忍大小写，这里做宽松但安全的校验：
// 拒绝明显越界/非法 id，避免把垃圾写进串口。
func Validate(raw string) (bool, string, error) {
	line := strings.TrimSpace(raw)
	if line == "" {
		return false, "", fmt.Errorf("空命令")
	}
	// 替换逗号分隔为空格，便于统一 tokenize
	norm := strings.ReplaceAll(line, ",", " ")
	fields := strings.Fields(norm)
	if len(fields) == 0 {
		return false, "", fmt.Errorf("空命令")
	}
	verb := strings.ToUpper(fields[0])

	// 简写 S<id>=<angle> 单独处理：verb 形如 "S7=90"，与下面的 switch verb 不匹配
	if len(verb) >= 3 && verb[0] == 'S' && verb[1] >= '6' && verb[1] <= '9' && verb[2] == '=' {
		id, _ := strconv.Atoi(verb[1:2])
		parts := strings.SplitN(verb, "=", 2)
		ang, err := strconv.Atoi(parts[1])
		if err != nil || !validIDs[id] || ang < 0 || ang > 180 {
			return false, "", fmt.Errorf("S 简写非法: %s", verb)
		}
		return true, fmt.Sprintf("S%d=%d", id, ang), nil
	}

	switch verb {
	case "HELP", "STATUS", "?", "RESET", "ADC":
		if len(fields) != 1 {
			return false, "", fmt.Errorf("%s 不接受参数", verb)
		}
		return true, verb, nil

	case "JOYHW", "IRHW":
		if len(fields) != 2 || !isOnOff(fields[1]) {
			return false, "", fmt.Errorf("%s 需要 ON|OFF", verb)
		}
		return true, fmt.Sprintf("%s %s", verb, strings.ToUpper(fields[1])), nil

	case "SEQ":
		if len(fields) != 2 {
			return false, "", fmt.Errorf("SEQ 需要 1|3|7|9|STOP|?")
		}
		arg := strings.ToUpper(fields[1])
		if arg == "STOP" || arg == "?" {
			return true, fmt.Sprintf("SEQ %s", arg), nil
		}
		n, err := strconv.Atoi(fields[1])
		if err != nil || (n != 1 && n != 3 && n != 7 && n != 9) {
			return false, "", fmt.Errorf("SEQ 参数非法: %s", fields[1])
		}
		return true, fmt.Sprintf("SEQ %d", n), nil

	case "STOP", "AUTO":
		if len(fields) != 2 {
			return false, "", fmt.Errorf("%s 需要 <id>", verb)
		}
		id, err := strconv.Atoi(fields[1])
		if err != nil || !validIDs[id] {
			return false, "", fmt.Errorf("%s 非法 id: %s", verb, fields[1])
		}
		return true, fmt.Sprintf("%s %d", verb, id), nil

	case "JOY":
		// JOY <id> <raw> 或 JOY <r9> <r8> <r6> <r7>
		if len(fields) == 3 {
			id, err := strconv.Atoi(fields[1])
			if err != nil || !validIDs[id] {
				return false, "", fmt.Errorf("JOY 非法 id: %s", fields[1])
			}
			raw, err := strconv.Atoi(fields[2])
			if err != nil || raw < 0 || raw > 1023 {
				return false, "", fmt.Errorf("JOY 非法 raw: %s (0..1023)", fields[2])
			}
			return true, fmt.Sprintf("JOY %d %d", id, raw), nil
		}
		if len(fields) == 5 {
			raws := make([]int, 4)
			for i := 0; i < 4; i++ {
				v, err := strconv.Atoi(fields[1+i])
				if err != nil || v < 0 || v > 1023 {
					return false, "", fmt.Errorf("JOY 非法 raw: %s (0..1023)", fields[1+i])
				}
				raws[i] = v
			}
			return true, fmt.Sprintf("JOY %d %d %d %d", raws[0], raws[1], raws[2], raws[3]), nil
		}
		return false, "", fmt.Errorf("JOY 语法: JOY <id> <raw> | JOY <r9> <r8> <r6> <r7>")

	case "IR":
		if len(fields) != 2 {
			return false, "", fmt.Errorf("IR 需要 <hexcode>")
		}
		if !isHex32(fields[1]) {
			return false, "", fmt.Errorf("IR 非法 hex: %s", fields[1])
		}
		return true, fmt.Sprintf("IR %s", fields[1]), nil

	case "IRLEARN", "IRCODES", "IRCLEAR":
		// IRLEARN 需要 1|3|5|7|9，其余无参
		if verb == "IRLEARN" {
			if len(fields) != 2 {
				return false, "", fmt.Errorf("IRLEARN 需要 <1|3|5|7|9>")
			}
			n, err := strconv.Atoi(fields[1])
			if err != nil || (n != 1 && n != 3 && n != 5 && n != 7 && n != 9) {
				return false, "", fmt.Errorf("IRLEARN 非法槽位: %s", fields[1])
			}
			return true, fmt.Sprintf("IRLEARN %d", n), nil
		}
		if len(fields) != 1 {
			return false, "", fmt.Errorf("%s 不接受参数", verb)
		}
		return true, verb, nil

	case "SET":
		if len(fields) < 3 || len(fields)%2 != 1 {
			return false, "", fmt.Errorf("SET 语法: SET <id> <ang> [id ang]..")
		}
		pairs := (len(fields) - 1) / 2
		if pairs > 3 {
			return false, "", fmt.Errorf("SET 最多 3 个舵机")
		}
		out := strings.Builder{}
		out.WriteString("SET")
		for i := 1; i+1 < len(fields); i += 2 {
			id, err := strconv.Atoi(fields[i])
			if err != nil || !validIDs[id] {
				return false, "", fmt.Errorf("SET 非法 id: %s", fields[i])
			}
			ang, err := strconv.Atoi(fields[i+1])
			if err != nil || ang < 0 || ang > 180 {
				return false, "", fmt.Errorf("SET 非法角度: %s (0..180)", fields[i+1])
			}
			out.WriteString(fmt.Sprintf(" %d %d", id, ang))
		}
		return true, out.String(), nil
	}

	return false, "", fmt.Errorf("未知命令: %s", verb)
}

func isOnOff(s string) bool {
	u := strings.ToUpper(s)
	return u == "ON" || u == "OFF"
}

func isHex32(s string) bool {
	t := strings.TrimPrefix(strings.ToUpper(s), "0X")
	if t == "" || len(t) > 8 {
		return false
	}
	for _, c := range t {
		if !((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F')) {
			return false
		}
	}
	return true
}

// axisRaw 把归一化坐标 v∈[-1,1] 转换为设备 raw 0..1023（512=中位/死区），
// inv=true 时左右镜像（适配实际安装方向）。
func axisRaw(v float64, inv bool) int {
	r := clampRaw(512 + int(v*512))
	if inv {
		r = 1023 - r
	}
	return r
}

// JoystickToJOYDual 把左右两个 3D 摇杆的归一化坐标 (∈[-1,1]) 合并为一条设备
// JOY 四轴帧：JOY <raw9> <raw8> <raw6> <raw7>（顺序与固件一致，ids={9,8,6,7}）。
//   - 左摇杆 X -> 底座(9)，Y -> 左舵(8)
//   - 右摇杆 X -> 夹取(6)，Y -> 右舵(7)
// 中位(0) -> raw 512 = 设备死区阈值，松手回中即停。
func JoystickToJOYDual(lx, ly, rx, ry float64, m AxisMap) string {
	r9 := axisRaw(lx, m.InvLX) // 底座
	r8 := axisRaw(ly, m.InvLY) // 左舵
	r6 := axisRaw(rx, m.InvRX) // 夹取
	r7 := axisRaw(ry, m.InvRY) // 右舵
	return fmt.Sprintf("JOY %d %d %d %d", r9, r8, r6, r7)
}

// JoystickToJOY 兼容旧的单摇杆调用：仅驱动左摇杆，右摇杆保持中位。
func JoystickToJOY(x, y float64, m AxisMap) string {
	return JoystickToJOYDual(x, y, 0, 0, m)
}

func clampRaw(v int) int {
	if v < 0 {
		return 0
	}
	if v > 1023 {
		return 1023
	}
	return v
}
