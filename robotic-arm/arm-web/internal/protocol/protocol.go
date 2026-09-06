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

// AxisMap 描述 2 轴网页摇杆到 4 路舵机的映射关系。
// 默认 X->底座(9)，Y->左舵(8)；其余两轴保持中位 512（设备死区，不动作）。
type AxisMap struct {
	XServo int // X 轴对应的舵机 id
	YServo int // Y 轴对应的舵机 id
	InvX   bool
	InvY   bool
}

// DefaultAxisMap 返回推荐映射（与硬件摇杆 A0=底座,A1=左舵 对齐）。
func DefaultAxisMap() AxisMap {
	return AxisMap{XServo: 9, YServo: 8, InvX: false, InvY: false}
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

// JoystickToJOY 把网页摇杆的归一化坐标 (x,y ∈ [-1,1]) 映射为设备 JOY 命令。
// 设备 JOY 命令格式：JOY <raw9> <raw8> <raw6> <raw7>（4 路 raw 0..1023，
// 阈值同硬件摇杆：<200 / >800 才动作，其余为死区）。
// 这样网页摇杆的"形状与逻辑"与真实硬件摇杆完全一致：
//   - 中位附近(约 |coord|<0.6)落入设备死区 -> 不动
//   - 偏离越大推动越快（设备按偏移比例步进 2..10°）
func JoystickToJOY(x, y float64, m AxisMap) string {
	rawX := clampRaw(512 + int(x*512))
	rawY := clampRaw(512 + int(y*512))
	if m.InvX {
		rawX = 1023 - rawX
	}
	if m.InvY {
		rawY = 1023 - rawY
	}
	// 构造 4 轴（顺序 r9 r8 r6 r7），未映射的轴保持 512（设备死区，不动作）
	r := []int{512, 512, 512, 512}
	setRaw(r, 9, rawX, m.XServo)
	setRaw(r, 8, rawY, m.YServo)
	return fmt.Sprintf("JOY %d %d %d %d", r[0], r[1], r[2], r[3])
}

// servoSlot 返回舵机 id 在 JOY 4 元组中的下标：9->0,8->1,6->2,7->3
func servoSlot(id int) int {
	switch id {
	case 9:
		return 0
	case 8:
		return 1
	case 6:
		return 2
	case 7:
		return 3
	}
	return -1
}

// setRaw 仅当 target id == servo 时才覆盖对应下标，保证 X/Y 各驱动自己的舵机，
// 未映射的轴保持 512（死区）。r 为切片（引用类型），修改对调用方可见。
func setRaw(r []int, servoID, raw, target int) {
	if servoID == target {
		slot := servoSlot(servoID)
		if slot >= 0 && slot < len(r) {
			r[slot] = raw
		}
	}
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
