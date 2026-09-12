// Package protocol 定义两个方向的编解码，本包**不认识机械结构**：
//
//	Browser ↔ Go      JSON（spec §二十一 / protocol/serial-v1.md §5）
//	Go ↔ Device       arm-device 文本协议（JR / STATE / OK / ERR，serial-v1.md §4）
//
// 分层铁律（serial-v1.md §1）：本包只负责把关节角编成字节 / 把字节解回关节角，
// 不做标定、不做限位判断 —— 那是 `internal/robot`（真值）与 `internal/controller`
// （策略）的职责。协议层一旦开始"懂关节"，双份真值就回来了。
package protocol

import (
	"fmt"
	"regexp"
	"sort"
	"strconv"
	"strings"

	"armpilot/backend/internal/robot"
)

// Version 是消息 schema 版本；前端与后端必须一致，不一致直接拒绝（避免静默错解）。
const Version = 1

// ---------------------------------------------------------------------------
// 消息类型
// ---------------------------------------------------------------------------

// 浏览器 → 服务器
const (
	TypeJointCommand  = "joint_command"  // 关节角整帧命令
	TypePing          = "ping"           // 心跳
	TypeStatusRequest = "status_request" // 请求重发 hello + 设备状态
)

// 服务器 → 浏览器
const (
	TypeHello        = "hello"         // 握手元数据（模型 / 限位 / 标定真值）
	TypeJointState   = "joint_state"   // 关节角状态（由设备回执反算）
	TypeError        = "error"         // 错误（含固定错误码）
	TypePong         = "pong"          // 心跳应答
	TypeDeviceStatus = "device_status" // 设备（串口/sim）连接状态变更
)

// 错误码（前端按码分支，不要匹配 message 文本）
const (
	CodeBadMessage = "BAD_MESSAGE"
	CodeVersion    = "VERSION_MISMATCH"
	CodeJointLimit = "JOINT_LIMIT"
	CodeDeviceDown = "DEVICE_UNAVAILABLE"
	CodeInternal   = "INTERNAL"
	CodeAckTimeout = "ACK_TIMEOUT"
)

// ClientMessage 是浏览器下行消息。
//
// `seq` 是本项目在 spec 字段之外加的**可选项**：拖动时命令高频变化，
// 前端用单调递增序号做去重与乱序丢弃；旧客户端不发 seq 也能工作（默认 0）。
type ClientMessage struct {
	Version   int                `json:"version"`
	Type      string             `json:"type"`
	Timestamp int64              `json:"timestamp,omitempty"`
	Seq       uint64             `json:"seq,omitempty"`
	Joints    map[string]float64 `json:"joints,omitempty"`
}

// ServerMessage 是服务器上行消息（一个结构覆盖全部上行类型，未用字段省略）。
type ServerMessage struct {
	Version   int                `json:"version"`
	Type      string             `json:"type"`
	Timestamp int64              `json:"timestamp"`
	Seq       uint64             `json:"seq,omitempty"`
	Joints    map[string]float64 `json:"joints,omitempty"`
	Code      string             `json:"code,omitempty"`
	Message   string             `json:"message,omitempty"`
	Model     *ModelInfo         `json:"model,omitempty"`
	Device    string             `json:"device,omitempty"`
	Connected *bool              `json:"connected,omitempty"`
}

// ModelInfo 是握手时下发的模型真值快照。前端拿它与本地 RobotModel 比对：
// 若限位/标定对不上，说明两侧读的不是同一份 robot.yaml —— 这正是
// "标定表只有一份"这条铁律的**在线校验手段**。
type ModelInfo struct {
	ID          string                 `json:"id"`
	Name        string                 `json:"name"`
	Source      string                 `json:"source"`
	JointOrder  []string               `json:"jointOrder"`
	Limits      []robot.LimitRow       `json:"limits"`
	Calibration []robot.CalibrationRow `json:"calibration"`
	HomePose    map[string]float64     `json:"homePose"`
}

// BuildModelInfo 从模型真值构造元数据。
func BuildModelInfo(m *robot.Model) *ModelInfo {
	return &ModelInfo{
		ID:          m.ID,
		Name:        m.Name,
		Source:      "config/robot.yaml",
		JointOrder:  m.JointOrder(),
		Limits:      m.LimitTable(),
		Calibration: m.CalibrationTable(),
		HomePose:    m.HomePose,
	}
}

// ---------------------------------------------------------------------------
// 设备文本协议（arm-device）
// ---------------------------------------------------------------------------

// EncodeJR 把关节角编成 JR 整帧文本。
//
// 协议 §4 规定**保留 1 位小数**，这是文本协议的固有量化：
// 往返一次会带来 ≤0.05° 的偏差，属预期行为（不是 bug），验收断言按此设容差。
func EncodeJR(order []string, joints map[string]float64) string {
	parts := make([]string, 0, len(order)+1)
	parts = append(parts, "JR")
	for _, id := range order {
		parts = append(parts, strconv.FormatFloat(joints[id], 'f', 1, 64))
	}
	return strings.Join(parts, " ")
}

// EncodeState 把关节角编成 STATE 文本（sim 设备模拟固件回读用）。
func EncodeState(order []string, joints map[string]float64) string {
	parts := make([]string, 0, len(order)+1)
	parts = append(parts, "STATE")
	for _, id := range order {
		parts = append(parts, strconv.FormatFloat(joints[id], 'f', 2, 64))
	}
	return strings.Join(parts, " ")
}

// EncodeSetServo 舵机级直控（sim 设备在 JR 回执里同时暴露舵机角，便于核对标定）。
func EncodeOKJR(servoAngles map[int]float64) string {
	chans := make([]int, 0, len(servoAngles))
	for ch := range servoAngles {
		chans = append(chans, ch)
	}
	sort.Sort(sort.Reverse(sort.IntSlice(chans)))
	parts := make([]string, 0, len(chans)+2)
	parts = append(parts, "OK", "JR")
	for _, ch := range chans {
		parts = append(parts, fmt.Sprintf("S%d=%.2f", ch, servoAngles[ch]))
	}
	return strings.Join(parts, " ")
}

var (
	reServoKV  = regexp.MustCompile(`S(\d+)=(-?\d+(?:\.\d+)?)`)
	reState    = regexp.MustCompile(`(?i)^\s*STATE\b(.*)$`)
	reErrJoint = regexp.MustCompile(`(?i)^\s*ERR\s+JOINT\s+(\w+)\s+(-?\d+(?:\.\d+)?)\s+\(limit\s+(-?\d+(?:\.\d+)?)\.\.(-?\d+(?:\.\d+)?)\)\s*$`)
)

// ReplyKind 区分设备回执的类别。
type ReplyKind int

const (
	ReplyOther ReplyKind = iota // 无法识别的行（异步事件 / 未实现命令）
	ReplyOKJR                   // OK JR S9=.. S7=.. S8=.. S6=..
	ReplyState                  // STATE <j1> <j2> <j3> <grip>
	ReplyError                  // ERR ...
)

func (k ReplyKind) String() string {
	switch k {
	case ReplyOKJR:
		return "OK_JR"
	case ReplyState:
		return "STATE"
	case ReplyError:
		return "ERR"
	default:
		return "OTHER"
	}
}

// Reply 是解析后的一条设备回执。
type Reply struct {
	Kind        ReplyKind
	Raw         string
	ServoAngles map[int]float64 // ReplyOKJR
	Joints      []float64       // ReplyState（按 JointOrder 位次）
	ErrText     string          // ReplyError
}

// ParseReply 解析一行设备回执。
//
// ⚠️ 判定顺序要紧：`OK JR S9=..` 里含 `S9=`，若先按"含 S<n>= 就是舵机回显"抢判，
// 会把 `ERR JOINT elbow ...` 也误吞（它不含 S=，此处安全）；
// 但 `STATE` 行不含 S=，故先判 ERR / STATE，再判 OK JR。
func ParseReply(line string) Reply {
	trimmed := strings.TrimSpace(line)
	r := Reply{Raw: trimmed}

	if strings.HasPrefix(strings.ToUpper(trimmed), "ERR") {
		r.Kind = ReplyError
		r.ErrText = trimmed
		return r
	}
	if m := reState.FindStringSubmatch(trimmed); m != nil {
		fields := strings.Fields(m[1])
		vals := make([]float64, 0, len(fields))
		for _, f := range fields {
			v, err := strconv.ParseFloat(f, 64)
			if err != nil {
				return r // 有一个不是数字就整体不认
			}
			vals = append(vals, v)
		}
		if len(vals) > 0 {
			r.Kind = ReplyState
			r.Joints = vals
		}
		return r
	}
	if m := reErrJoint.FindStringSubmatch(trimmed); m != nil {
		r.Kind = ReplyError
		r.ErrText = trimmed
		return r
	}
	if strings.HasPrefix(strings.ToUpper(trimmed), "OK JR") {
		if kv := reServoKV.FindAllStringSubmatch(trimmed, -1); len(kv) > 0 {
			angles := make(map[int]float64, len(kv))
			for _, pair := range kv {
				ch, err1 := strconv.Atoi(pair[1])
				ang, err2 := strconv.ParseFloat(pair[2], 64)
				if err1 != nil || err2 != nil {
					continue
				}
				angles[ch] = ang
			}
			r.Kind = ReplyOKJR
			r.ServoAngles = angles
		}
		return r
	}
	return r
}

// ParseErrorCode 把设备 ERR 文本归类成前端可分支的错误码。
func ParseErrorCode(errText string) (code, message string) {
	if m := reErrJoint.FindStringSubmatch(strings.TrimSpace(errText)); m != nil {
		return CodeJointLimit, errText
	}
	return CodeInternal, errText
}

// ServoAnglesToJoints 用模型标定表把舵机角反算回关节角。
//
// 这是闭环的关键一步：设备只回舵机角（它不认识关节），后端必须自己换回来。
// 走这一步而不是"把命令原样当状态回推"，才能让标定表的**可逆性**被真实检验 ——
// 若 offset/scale/reverse 写错，Actual 会立刻偏离 Command，而不是永远相等。
func ServoAnglesToJoints(m *robot.Model, servoAngles map[int]float64) map[string]float64 {
	out := make(map[string]float64, len(m.JointOrder()))
	for _, id := range m.JointOrder() {
		acts := m.ActuatorsForJoint(id)
		if len(acts) == 0 {
			continue
		}
		// 多舵机关节取平均（spec §二十五：上层无感）
		sum, n := 0.0, 0
		for _, a := range acts {
			servo, ok := servoAngles[a.Channel]
			if !ok {
				continue
			}
			sum += robot.ServoToJoint(a, servo)
			n++
		}
		if n > 0 {
			out[id] = sum / float64(n)
		}
	}
	return out
}
