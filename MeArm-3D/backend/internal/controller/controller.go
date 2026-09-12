// Package controller 是 Go 侧的唯一"懂机械臂"的地方（spec §二十二 的 Robot Controller）。
//
// 职责边界（对齐 protocol/serial-v1.md §5 的分层图）：
//
//	WebSocket Client → Protocol(编解码) → **Robot Controller** → Device → AVR
//
//	- 限位校验：以 config/robot.yaml 为唯一真值，越界直接拒（不写设备）
//	- 标定换算：关节角 → 舵机角（编码前）/ 舵机角 → 关节角（回执后）
//	- ACK 门控：同一时刻只允许 1 条 JR 在途，避免把串口打爆
//	- latest-wins：在途期间新命令只覆盖"待发槽"，绝不排队
//	               （否则拖动时命令堆积，回执永远追赶历史 —— skill arm-robot-serial 关键坑 6）
//	- 断线安全态：设备不可用时拒收命令并明确报错，不静默丢弃
//
// ⚠️ WebSocket 层不许直接碰 Device。所有控制意图必须经过本包。
package controller

import (
	"fmt"
	"log"
	"sync"
	"time"

	"armpilot/backend/internal/device"
	"armpilot/backend/internal/protocol"
	"armpilot/backend/internal/robot"
)

// Config 控制器策略参数。
type Config struct {
	// AckTimeoutMs 单条指令的回执超时（ms）。超时判链路异常，转入安全态。
	AckTimeoutMs int
	// MinSendIntervalMs 两次下发之间的最小间隔（ms）。0 = 不限制。
	MinSendIntervalMs int
	// EchoJointState 是否把 STATE 回执行透传给浏览器（默认 true）。
	EchoJointState bool
}

// DefaultConfig 返回推荐策略。
func DefaultConfig() Config {
	return Config{AckTimeoutMs: 800, MinSendIntervalMs: 0, EchoJointState: true}
}

// JointStateHandler 收到设备状态（关节角）。
type JointStateHandler func(joints map[string]float64, at time.Time)

// ErrorHandler 收到错误（限位拒绝 / 回执超时 / 设备异常）。
type ErrorHandler func(code, message string)

// StatusHandler 设备可用性变更。
type StatusHandler func(connected bool, reason string)

// Controller 承接所有控制意图。
type Controller struct {
	model  *robot.Model
	dev    device.Device
	cfg    Config
	order  []string
	nowFn  func() time.Time
	inited bool

	mu          sync.Mutex
	inflight    *inflight
	pending     map[string]float64
	lastCommand map[string]float64
	lastState   map[string]float64
	lastSendAt  time.Time
	closed      bool

	stateH []JointStateHandler
	errH   []ErrorHandler
	statH  []StatusHandler
}

type inflight struct {
	joints map[string]float64
	timer  *time.Timer
	at     time.Time
}

// New 创建控制器。
func New(m *robot.Model, dev device.Device, cfg Config) *Controller {
	return &Controller{
		model:       m,
		dev:         dev,
		cfg:         cfg,
		order:       m.JointOrder(),
		nowFn:       time.Now,
		lastCommand: copyJoints(m.HomePose),
		lastState:   copyJoints(m.HomePose),
	}
}

func copyJoints(in map[string]float64) map[string]float64 {
	out := make(map[string]float64, len(in))
	for k, v := range in {
		out[k] = v
	}
	return out
}

// Start 启动回执读取循环（幂等）。
func (c *Controller) Start() {
	c.mu.Lock()
	if c.inited {
		c.mu.Unlock()
		return
	}
	c.inited = true
	c.mu.Unlock()

	go c.readLoop()
	c.dev.OnStatus(func(connected bool, reason string) {
		c.emitStatus(connected, reason)
	})
}

func (c *Controller) readLoop() {
	for line := range c.dev.Lines() {
		c.handleLine(line)
	}
}

func (c *Controller) handleLine(line device.Line) {
	reply := protocol.ParseReply(line.Text)

	// 任何一行回执（含 ERR）都意味着在途指令已了结 → 放行 ACK 门控
	if reply.Kind == protocol.ReplyOKJR || reply.Kind == protocol.ReplyError {
		c.finishInflight()
	}

	switch reply.Kind {
	case protocol.ReplyOKJR:
		// ⚠️ OK JR 携带的舵机角是**目标角**（"我要去哪"），不是实际位置。
		//    绝不能拿它当 Actual 回推 —— 否则状态永远等于命令、误差恒为 0，
		//    "Actual 滞后 → 收敛"的整条语义会被一个乐观 ACK 抹掉。
		//    它的用途是**核对标定**：把回执里的舵机角与本地标定算出的值比对。
		c.verifyCalibrationEcho(reply.ServoAngles)
		c.drainPending()

	case protocol.ReplyState:
		if !c.cfg.EchoJointState {
			return
		}
		joints := c.stateFromValues(reply.Joints)
		c.publishState(joints)

	case protocol.ReplyError:
		code, msg := protocol.ParseErrorCode(reply.ErrText)
		log.Printf("[ctl] 设备拒绝: %s", msg)
		c.emitError(code, msg)
		c.drainPending()

	default:
		// 异步事件（# ...）或未识别行：仅记录，不影响门控
		if line.Text != "" {
			log.Printf("[ctl] 设备事件: %s", line.Text)
		}
	}
}

// stateFromValues 把 STATE 的数值数组按关节顺序还原成 map；
// 个数不符时按最小值截断，缺的沿用上一次状态（不让 UI 看到 0）。
func (c *Controller) stateFromValues(vals []float64) map[string]float64 {
	out := copyJoints(c.lastState)
	n := len(vals)
	if n > len(c.order) {
		n = len(c.order)
	}
	for i := 0; i < n; i++ {
		out[c.order[i]] = vals[i]
	}
	return out
}

// Apply 受理一条关节级命令。返回的错误表示**未被受理**（可立即回报给浏览器）。
//
// 受理后的链路异常（回执超时 / 设备拒绝）通过 ErrorHandler 异步上报。
func (c *Controller) Apply(joints map[string]float64) error {
	c.mu.Lock()
	closed := c.closed
	c.mu.Unlock()
	if closed {
		return fmt.Errorf("控制器已关闭")
	}
	if !c.dev.Connected() {
		reason := c.dev.UnavailableReason()
		if reason == "" {
			reason = "链路不可用"
		}
		return &RejectError{Code: protocol.CodeDeviceDown, Message: reason}
	}

	// 归一化到完整关节帧：缺项沿用当前命令值（夹爪常单独更新）
	full := copyJoints(c.lastCommand)
	for k, v := range joints {
		full[k] = v
	}

	// 限位校验：唯一真值是 model（= config/robot.yaml）
	if v := c.model.Validate(full); v != nil {
		return &RejectError{Code: protocol.CodeJointLimit, Message: v.Error()}
	}

	return c.dispatch(full)
}

// RejectError 表示命令被**同步拒绝**（浏览器会立刻收到 error 消息）。
type RejectError struct {
	Code    string
	Message string
}

func (e *RejectError) Error() string { return e.Message }

// dispatch 走 ACK 门控 + latest-wins。
func (c *Controller) dispatch(joints map[string]float64) error {
	c.mu.Lock()
	c.lastCommand = copyJoints(joints)

	if c.inflight != nil {
		// 有指令在途 → 只覆盖待发槽（latest-wins），不排队
		c.pending = joints
		suppressed := true
		c.mu.Unlock()
		if suppressed {
			// 静默合并：这是高频拖动的**正常路径**，不该刷日志
		}
		return nil
	}
	c.mu.Unlock()
	return c.send(joints)
}

// send 真正写设备，并挂上回执超时。
func (c *Controller) send(joints map[string]float64) error {
	c.mu.Lock()
	if c.closed {
		c.mu.Unlock()
		return fmt.Errorf("控制器已关闭")
	}
	line := protocol.EncodeJR(c.order, joints)
	in := &inflight{joints: copyJoints(joints), at: c.nowFn()}
	c.inflight = in
	lastSend := c.lastSendAt
	now := c.nowFn()
	c.lastSendAt = now
	minInterval := c.cfg.MinSendIntervalMs
	ackTimeout := c.cfg.AckTimeoutMs
	c.mu.Unlock()

	if minInterval > 0 {
		if wait := time.Duration(minInterval)*time.Millisecond - now.Sub(lastSend); wait > 0 {
			time.Sleep(wait)
		}
	}

	if err := c.dev.WriteLine(line); err != nil {
		c.mu.Lock()
		c.inflight = nil
		c.mu.Unlock()
		return &RejectError{Code: protocol.CodeDeviceDown, Message: err.Error()}
	}

	if ackTimeout > 0 {
		c.mu.Lock()
		if c.inflight == in {
			in.timer = time.AfterFunc(time.Duration(ackTimeout)*time.Millisecond, func() {
				c.onAckTimeout(in)
			})
		}
		c.mu.Unlock()
	}
	return nil
}

func (c *Controller) onAckTimeout(in *inflight) {
	c.mu.Lock()
	if c.inflight != in || c.closed {
		c.mu.Unlock()
		return
	}
	c.inflight = nil
	timeout := c.cfg.AckTimeoutMs
	c.mu.Unlock()

	log.Printf("[ctl] 回执超时（%dms）：%v", timeout, in.joints)
	c.emitError(protocol.CodeAckTimeout,
		fmt.Sprintf("设备回执超时（%dms），链路可能异常", timeout))
	c.drainPending()
}

// finishInflight 结束在途指令的等待（取消超时定时器）。
func (c *Controller) finishInflight() {
	c.mu.Lock()
	in := c.inflight
	c.inflight = nil
	c.mu.Unlock()
	if in != nil && in.timer != nil {
		in.timer.Stop()
	}
}

// calibToleranceDeg 舵机角回执与本地标定值的允许偏差（度）。
// 文本协议 JR 保留 1 位小数（量化 0.05°），此处放到 0.1° 留余量。
const calibToleranceDeg = 0.1

// verifyCalibrationEcho 核对设备回执里的舵机角与本地标定算出的值。
//
// 这是"标定表只有一份"的**运行期校验**：若固件（或 sim）内置的标定表与
// config/robot.yaml 漂移，这里会立刻给出偏差量，而不是让机械臂默默走错。
// 不匹配只报警告、不阻断（真实链路里偶发的采样抖动不该中断控制）。
func (c *Controller) verifyCalibrationEcho(servoAngles map[int]float64) {
	c.mu.Lock()
	cmd := c.lastCommand
	c.mu.Unlock()

	var (
		haveWorst  bool
		worstDelta float64
		worstCh    int
		worstLocal float64
		worstEcho  float64
	)
	for _, id := range c.order {
		for _, a := range c.model.ActuatorsForJoint(id) {
			echo, ok := servoAngles[a.Channel]
			if !ok {
				continue
			}
			local := robot.JointToServo(a, cmd[id])
			d := abs(local - echo)
			if !haveWorst || d > worstDelta {
				haveWorst, worstDelta = true, d
				worstCh, worstLocal, worstEcho = a.Channel, local, echo
			}
		}
	}
	if haveWorst && worstDelta > calibToleranceDeg {
		log.Printf("[ctl] ⚠️ 标定回执偏差 S%d: 本地 %.3f° vs 设备 %.3f°（差 %.3f°，容差 %.2f°）—— 检查两侧标定表",
			worstCh, worstLocal, worstEcho, worstDelta, calibToleranceDeg)
	}
}

func abs(v float64) float64 {
	if v < 0 {
		return -v
	}
	return v
}

// drainPending 把待发槽里的最新命令补发出去（尾沿语义）。
func (c *Controller) drainPending() {
	c.mu.Lock()
	next := c.pending
	c.pending = nil
	closed := c.closed
	c.mu.Unlock()
	if next == nil || closed {
		return
	}
	if err := c.send(next); err != nil {
		c.emitError(protocol.CodeDeviceDown, err.Error())
	}
}

// ---------------------------------------------------------------------------
// 状态与订阅
// ---------------------------------------------------------------------------

func (c *Controller) publishState(joints map[string]float64) {
	c.mu.Lock()
	c.lastState = copyJoints(joints)
	hs := append([]JointStateHandler(nil), c.stateH...)
	c.mu.Unlock()
	at := c.nowFn()
	for _, h := range hs {
		if h != nil {
			h(joints, at)
		}
	}
}

func (c *Controller) emitError(code, msg string) {
	c.mu.Lock()
	hs := append([]ErrorHandler(nil), c.errH...)
	c.mu.Unlock()
	for _, h := range hs {
		if h != nil {
			h(code, msg)
		}
	}
}

func (c *Controller) emitStatus(connected bool, reason string) {
	c.mu.Lock()
	hs := append([]StatusHandler(nil), c.statH...)
	c.mu.Unlock()
	for _, h := range hs {
		if h != nil {
			h(connected, reason)
		}
	}
}

// OnJointState 订阅关节状态。
func (c *Controller) OnJointState(h JointStateHandler) {
	c.mu.Lock()
	c.stateH = append(c.stateH, h)
	c.mu.Unlock()
}

// OnError 订阅错误。
func (c *Controller) OnError(h ErrorHandler) {
	c.mu.Lock()
	c.errH = append(c.errH, h)
	c.mu.Unlock()
}

// OnStatus 订阅设备状态。
func (c *Controller) OnStatus(h StatusHandler) {
	c.mu.Lock()
	c.statH = append(c.statH, h)
	c.mu.Unlock()
}

// Snapshot 返回当前"命令 / 状态"快照（新客户端接入时同步用）。
func (c *Controller) Snapshot() (command, state map[string]float64) {
	c.mu.Lock()
	defer c.mu.Unlock()
	return copyJoints(c.lastCommand), copyJoints(c.lastState)
}

// DeviceKind 链路末端类型（sim / serial）。
func (c *Controller) DeviceKind() string { return c.dev.Kind() }

// DeviceConnected 链路末端是否可用。
func (c *Controller) DeviceConnected() bool { return c.dev.Connected() }

// DeviceReason 不可用原因。
func (c *Controller) DeviceReason() string { return c.dev.UnavailableReason() }

// Model 返回模型真值（wsserver 构造 hello 用）。
func (c *Controller) Model() *robot.Model { return c.model }

// Close 停止控制器并释放设备。
func (c *Controller) Close() error {
	c.mu.Lock()
	if c.closed {
		c.mu.Unlock()
		return nil
	}
	c.closed = true
	if c.inflight != nil && c.inflight.timer != nil {
		c.inflight.timer.Stop()
	}
	c.inflight = nil
	c.pending = nil
	c.mu.Unlock()
	return c.dev.Close()
}
