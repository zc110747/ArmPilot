package device

import (
	"fmt"
	"log"
	"regexp"
	"strconv"
	"strings"
	"sync"
	"time"

	"armpilot/backend/internal/protocol"
	"armpilot/backend/internal/robot"
)

// SimTuning 控制"假固件"的物理特性。默认值与前端 MockTransport 对齐（240°/s、
// 15ms 延迟、20ms tick），这样两条链路的观感一致，验收数字可以互相印证。
type SimTuning struct {
	// MaxServoSpeed 舵机最大角速度（度/秒）；0 或负 = 瞬时到位
	MaxServoSpeed float64
	// LatencyMs 指令送达延迟（ms）—— 延迟期内不回复执
	LatencyMs int
	// TickMs 位置推进周期（ms）
	TickMs int
	// EnforceLimits 是否做限位校验（模拟固件的 ERR JOINT）
	EnforceLimits bool
	// BootMs 开机静默窗口（ms）：模拟 Uno bootloader 交权期，
	// 窗口内下发的指令被吞掉且不回执（对齐 skill arm-robot-serial 关键坑 1/2）
	BootMs int
}

// DefaultSimTuning 返回推荐参数。
func DefaultSimTuning() SimTuning {
	return SimTuning{
		MaxServoSpeed: 240,
		LatencyMs:     15,
		TickMs:        20,
		EnforceLimits: true,
		BootMs:        0,
	}
}

var reJR = regexp.MustCompile(`(?i)^\s*JR\b(.*)$`)

// SimDevice 模拟 arm-device 固件。
//
// 它**不是**等值回显器：内部维护舵机空间的 target/actual，以有限角速度逼近，
// 并把 actual 反算回关节角主动上报 STATE。这样"标定可逆性 + 状态滞后 + 收敛"
// 三件事才会被真实检验，而不是被一个理想回显掩盖到 Phase 11。
type SimDevice struct {
	model   *robot.Model
	order   []string
	tuning  SimTuning
	startAt time.Time

	mu       sync.Mutex
	actualS  map[int]float64 // 舵机实际角（度）
	targetS  map[int]float64 // 舵机目标角（度）
	closed   bool
	ticking  bool
	bootDone bool

	lines   chan Line
	statusF []StatusHandler
}

// NewSim 创建 sim 设备，初始位置取模型 HOME 位（四个舵机恰好 90°）。
func NewSim(m *robot.Model, t SimTuning) (*SimDevice, error) {
	home := make(map[int]float64, len(m.Actuators))
	for i := range m.Actuators {
		a := &m.Actuators[i]
		j := m.Joint(a.JointID)
		if j == nil {
			return nil, fmt.Errorf("执行器 %s 指向未知关节 %s", a.ID, a.JointID)
		}
		home[a.Channel] = robot.JointToServo(a, m.HomePose[j.ID])
	}
	return &SimDevice{
		model:    m,
		order:    m.JointOrder(),
		tuning:   t,
		startAt:  time.Now(),
		actualS:  copyMap(home),
		targetS:  copyMap(home),
		lines:    make(chan Line, 256),
		bootDone: t.BootMs <= 0,
	}, nil
}

func copyMap(in map[int]float64) map[int]float64 {
	out := make(map[int]float64, len(in))
	for k, v := range in {
		out[k] = v
	}
	return out
}

func (d *SimDevice) Kind() string { return "sim" }

func (d *SimDevice) Connected() bool {
	d.mu.Lock()
	defer d.mu.Unlock()
	return !d.closed
}

// bootRemaining 必须在**持锁状态下**调用（它会写入 bootDone）。
func (d *SimDevice) bootRemaining() time.Duration {
	if d.bootDone {
		return 0
	}
	remain := time.Duration(d.tuning.BootMs)*time.Millisecond - time.Since(d.startAt)
	if remain <= 0 {
		d.bootDone = true
		return 0
	}
	return remain
}

func (d *SimDevice) UnavailableReason() string {
	d.mu.Lock()
	defer d.mu.Unlock()
	if d.closed {
		return "sim 设备已关闭"
	}
	return ""
}

func (d *SimDevice) Lines() <-chan Line { return d.lines }

func (d *SimDevice) OnStatus(fn StatusHandler) func() {
	d.mu.Lock()
	d.statusF = append(d.statusF, fn)
	idx := len(d.statusF) - 1
	d.mu.Unlock()
	return func() {
		d.mu.Lock()
		defer d.mu.Unlock()
		if idx < len(d.statusF) {
			d.statusF[idx] = nil
		}
	}
}

// WriteLine 模拟固件的指令受理。
func (d *SimDevice) WriteLine(line string) error {
	d.mu.Lock()
	if d.closed {
		d.mu.Unlock()
		return fmt.Errorf("sim 设备已关闭")
	}
	apiLatency := d.tuning.LatencyMs
	enforce := d.tuning.EnforceLimits
	bootRemain := d.bootRemaining()
	d.mu.Unlock()

	trimmed := strings.TrimSpace(line)
	upper := strings.ToUpper(trimmed)

	// 开机静默窗口：模拟 Uno bootloader，指令被吞掉且不回执
	if bootRemain > 0 {
		log.Printf("[sim] 开机静默窗口内丢弃指令（剩余 %v）: %q", bootRemain.Round(time.Millisecond), trimmed)
		return nil
	}

	switch {
	case strings.HasPrefix(upper, "JR"):
		joints, err := d.parseJR(trimmed)
		if err != nil {
			d.emit(fmt.Sprintf("ERR ARG %s", err.Error()))
			return nil
		}
		// ① 关节限位（固件内置同一张标定表 + 关节限位，见 serial-v1.md §4）
		if enforce {
			if v := d.model.Validate(joints); v != nil {
				d.emit(v.Error())
				return nil
			}
		}
		// ② 换算舵机角 + 舵机硬限位
		servo := make(map[int]float64, len(d.order))
		for _, id := range d.order {
			for _, a := range d.model.ActuatorsForJoint(id) {
				s := robot.JointToServo(a, joints[id])
				if enforce && (s < a.Limits.Min-1e-6 || s > a.Limits.Max+1e-6) {
					d.emit(fmt.Sprintf("ERR SERVO S%d %.2f (limit %.2f..%.2f)",
						a.Channel, s, a.Limits.Min, a.Limits.Max))
					return nil
				}
				servo[a.Channel] = s
			}
		}
		// ③ 延迟送达：延迟期内不回执（印证"命令送达前不回推"的传输语义）
		deliver := func() {
			d.mu.Lock()
			if d.closed {
				d.mu.Unlock()
				return
			}
			for ch, v := range servo {
				d.targetS[ch] = v
			}
			d.mu.Unlock()
			d.emit(protocol.EncodeOKJR(servo))
			d.tick() // 送达瞬间先走一步（与前端 Mock 的 applyTarget 行为一致）
			d.ensureTicking()
		}
		if apiLatency <= 0 {
			deliver()
		} else {
			time.AfterFunc(time.Duration(apiLatency)*time.Millisecond, deliver)
		}
		return nil

	case upper == "STATUS" || upper == "STATE?":
		// v0 固件的 STATUS：回舵机角（协议 §3）
		cur := d.snapshot()
		d.emit(fmt.Sprintf("STATUS S9=%d S7=%d S8=%d S6=%d",
			int(round(cur[9])), int(round(cur[7])), int(round(cur[8])), int(round(cur[6]))))
		return nil

	case upper == "RESET":
		// RESET 语义 = 全部舵机 90°（= HOME）。协议 §4 明确不许改成"关节全 0"：
		// 关节全 0 对肘（绝对角 108..142）是不可达位姿。
		target := make(map[int]float64, len(d.model.Actuators))
		for i := range d.model.Actuators {
			target[d.model.Actuators[i].Channel] = 90
		}
		d.mu.Lock()
		for ch, v := range target {
			d.targetS[ch] = v
		}
		d.mu.Unlock()
		d.emit("OK RESET")
		d.emit(protocol.EncodeOKJR(target))
		d.ensureTicking()
		return nil

	case upper == "PING":
		d.emit("OK PING")
		return nil

	default:
		d.emit(fmt.Sprintf("ERR UNKNOWN %s", trimmed))
		return nil
	}
}

// parseJR 解析 `JR <j1> <j2> <j3> <grip>`（位次 = model.JointOrder()）。
func (d *SimDevice) parseJR(line string) (map[string]float64, error) {
	m := reJR.FindStringSubmatch(line)
	if m == nil {
		return nil, fmt.Errorf("不是 JR 指令")
	}
	fields := strings.Fields(m[1])
	if len(fields) != len(d.order) {
		return nil, fmt.Errorf("JR 需要 %d 个关节角，收到 %d 个", len(d.order), len(fields))
	}
	joints := make(map[string]float64, len(fields))
	for i, f := range fields {
		v, err := strconv.ParseFloat(f, 64)
		if err != nil {
			return nil, fmt.Errorf("第 %d 个关节角 %q 不是数字", i+1, f)
		}
		joints[d.order[i]] = v
	}
	return joints, nil
}

func (d *SimDevice) snapshot() map[int]float64 {
	d.mu.Lock()
	defer d.mu.Unlock()
	return copyMap(d.actualS)
}

func (d *SimDevice) ensureTicking() {
	d.mu.Lock()
	if d.ticking || d.closed {
		d.mu.Unlock()
		return
	}
	d.ticking = true
	tickMs := d.tuning.TickMs
	d.mu.Unlock()

	if tickMs <= 0 {
		tickMs = 20
	}
	go func() {
		t := time.NewTicker(time.Duration(tickMs) * time.Millisecond)
		defer t.Stop()
		for range t.C {
			d.mu.Lock()
			closed := d.closed
			d.mu.Unlock()
			if closed {
				d.mu.Lock()
				d.ticking = false
				d.mu.Unlock()
				return
			}
			if !d.tick() {
				d.mu.Lock()
				d.ticking = false
				d.mu.Unlock()
				return
			}
		}
	}()
}

// tick 推进一个仿真步，返回是否仍在运动。
func (d *SimDevice) tick() bool {
	d.mu.Lock()
	if d.closed {
		d.mu.Unlock()
		return false
	}
	tickMs := d.tuning.TickMs
	if tickMs <= 0 {
		tickMs = 20
	}
	maxStep := d.tuning.MaxServoSpeed * (float64(tickMs) / 1000)
	instant := maxStep <= 0

	changed := false
	for ch, goal := range d.targetS {
		cur := d.actualS[ch]
		delta := goal - cur
		if delta == 0 {
			continue
		}
		if instant || abs(delta) <= maxStep {
			d.actualS[ch] = goal
		} else {
			d.actualS[ch] = cur + sign(delta)*maxStep
		}
		changed = true
	}
	moving := false
	for ch, goal := range d.targetS {
		if abs(d.actualS[ch]-goal) > 1e-6 {
			moving = true
			break
		}
	}
	actual := copyMap(d.actualS)
	d.mu.Unlock()

	if changed {
		// ⚠️ 用舵机**实际角反算**关节角上报 —— 走真实路径，不做等值回显。
		//    若 offset/scale/reverse 写错，这里立刻暴露为 Actual ≠ Command。
		joints := protocol.ServoAnglesToJoints(d.model, actual)
		d.emit(protocol.EncodeState(d.order, joints))
	}
	return moving
}

func (d *SimDevice) emit(text string) {
	d.mu.Lock()
	closed := d.closed
	d.mu.Unlock()
	if closed {
		return
	}
	line := Line{Text: text, At: time.Now()}
	select {
	case d.lines <- line:
	default:
		// 接收端落后时丢弃最旧的行，保证设备侧永不阻塞
		select {
		case <-d.lines:
		default:
		}
		select {
		case d.lines <- line:
		default:
		}
	}
}

func (d *SimDevice) Close() error {
	d.mu.Lock()
	if d.closed {
		d.mu.Unlock()
		return nil
	}
	d.closed = true
	fns := append([]StatusHandler(nil), d.statusF...)
	d.mu.Unlock()
	for _, fn := range fns {
		if fn != nil {
			fn(false, "sim 设备已关闭")
		}
	}
	return nil
}

func abs(v float64) float64 {
	if v < 0 {
		return -v
	}
	return v
}

func sign(v float64) float64 {
	if v < 0 {
		return -1
	}
	return 1
}

func round(v float64) float64 {
	return float64(int(v + 0.5))
}
