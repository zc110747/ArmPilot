package device

import (
	"bytes"
	"fmt"
	"io"
	"log"
	"math"
	"regexp"
	"strconv"
	"strings"
	"sync"
	"time"

	"armpilot/backend/internal/protocol"
	"armpilot/backend/internal/robot"
)

// 本文件是 Phase 9 的真串口设备：把**舵机级固件**包装成**关节级 Device**。
//
// # 为什么翻译层放在这里，而不是下沉进固件
//
// 固件（MeArm-Device/core/cmd.c）只讲舵机级指令：
//
//	SET <id> <angle> [<id> <angle>…]   最多 3 组（MAX_PAIRS = 3）
//	STATUS | ?                         回 `STATUS S6=90(H) S7=90(H) S8=90(H) S9=90(H)`
//	RESET                              全部 -> 90°
//
// 若把关节级 JR 下沉到固件，关节↔舵机的**标定真值就会同时存在于固件与
// config/robot.yaml 两处**，违反"标定表只有一份"这条铁律（除非再加一层 codegen）。
// 因此 JR → SET 的翻译留在本层：本层对外**仍是关节级设备**，
// controller / wsserver / 前端 / sim 都不需要知道固件讲的是舵机角。
//
// # 真机没有位置反馈（本阶段最重要的事实）
//
// 固件 arm_get_angle() 返回的是它**记着的目标值**，不是编码器读数 —— 这个机械臂
// 根本没有位置传感器。所以：
//   - 连 STATUS 也只是"固件认为它在哪"，属**开环**信息；
//   - 本层合成的 STATE 同样只表示"命令已被接受并钳位到 X"，
//     **不代表机械臂已物理到位**；
//   - 真机是否真的动了、动到哪，唯一的外部真值是**相机**
//     （tools/verify_pose.py + tests/e2e/ui-smoke.mjs 的 Phase 9 段落）。
//
// 这一点在 API 上无法掩盖，只能如实标注（见 docs/decisions.md 对应 ADR）。

// maxPairsPerSet 固件单条 SET 能接受的 (id,angle) 组数上限（cmd.c: MAX_PAIRS）。
// 四舵机因此必须拆成 2 条 —— 这是真机与 sim 的一处结构性差异。
const maxPairsPerSet = 3

// SerialConfig 真串口参数（只放运行参数，不含任何标定/限位数值）。
type SerialConfig struct {
	Port     string
	Baud     int
	DataBits int
	StopBits int
	Parity   string
	// ReconnectSec 打开失败/断链后的重试间隔（秒）
	ReconnectSec int
	// AckTimeoutMs 单条固件指令的等待上限（ms）
	AckTimeoutMs int
	// ConnectSettleMs 打开端口后的 bootloader 静默窗口（ms）。
	// Uno 被 DTR 复位后 optiboot 要等 ~2.5s 才交权，窗口内指令会被丢弃。
	ConnectSettleMs int
	// Warmup 是否做"暖机包 + STATUS 探测"（Uno 引导交接后首个数据包常被吞）
	Warmup bool
}

// dialSerial 打开串口；测试通过覆写它注入假连接。
var dialSerial = openPort

// servoKVRe 从固件回执里抓 `S<id>=<angle>`。
// 同时适配 `OK SET S9=90 S7=131` 与 `STATUS S6=90(H) S7=90(H)`（在 `(` 前停住）。
var servoKVRe = regexp.MustCompile(`S(\d+)=(-?\d+(?:\.\d+)?)`)

type serialDevice struct {
	cfg   SerialConfig
	model *robot.Model
	order []string

	done      chan struct{}
	closeOnce sync.Once
	wg        sync.WaitGroup

	lines chan Line
	cmdCh chan string // 容量 1：latest-wins
	wake  chan struct{}

	mu        sync.Mutex
	conn      io.ReadWriteCloser
	connected bool
	reason    string
	statusH   []StatusHandler
}

// NewSerial 创建真串口设备并启动（重）连接管理。
//
// 即使机械臂此刻没插，也返回可用的 Device（Connected() == false），
// 上层因此能给出"设备未连接"的准确提示，而不是让进程起不来。
func NewSerial(cfg SerialConfig, model *robot.Model) (Device, error) {
	if strings.TrimSpace(cfg.Port) == "" {
		return nil, fmt.Errorf("serial.port 为空（需要 COM 口，例如 COM16）")
	}
	if model == nil {
		return nil, fmt.Errorf("serial 设备需要 robot.Model 做关节↔舵机换算")
	}
	order := model.JointOrder()
	if len(order) == 0 {
		return nil, fmt.Errorf("robot.yaml 没有可动关节，无法生成 JR 命令")
	}
	if len(cfg.Port) > 3 && strings.EqualFold(cfg.Port[:3], "COM") {
		if _, err := strconv.Atoi(cfg.Port[3:]); err != nil {
			return nil, fmt.Errorf("serial.port 形如 COM16，当前为 %q", cfg.Port)
		}
	}
	if cfg.Baud <= 0 {
		cfg.Baud = 115200
	}
	if cfg.DataBits == 0 {
		cfg.DataBits = 8
	}
	if cfg.StopBits == 0 {
		cfg.StopBits = 1
	}
	if cfg.Parity == "" {
		cfg.Parity = "N"
	}
	if cfg.ReconnectSec <= 0 {
		cfg.ReconnectSec = 3
	}
	if cfg.AckTimeoutMs <= 0 {
		cfg.AckTimeoutMs = 600
	}
	if cfg.ConnectSettleMs < 0 {
		cfg.ConnectSettleMs = 0
	}

	d := &serialDevice{
		cfg:   cfg,
		model: model,
		order: order,
		done:  make(chan struct{}),
		lines: make(chan Line, 64),
		cmdCh: make(chan string, 1),
		wake:  make(chan struct{}, 1),
	}
	d.wg.Add(1)
	go d.manage()
	return d, nil
}

// ---------------------------------------------------------------------------
// Device 接口
// ---------------------------------------------------------------------------

func (d *serialDevice) Kind() string { return "serial" }

func (d *serialDevice) Connected() bool {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.connected
}

func (d *serialDevice) UnavailableReason() string {
	d.mu.Lock()
	defer d.mu.Unlock()
	if d.connected {
		return ""
	}
	if d.reason == "" {
		return "串口未就绪"
	}
	return d.reason
}

// WriteLine 下发一行关节级指令（不含行尾）。
//
// 未连接时**返回错误而不是静默丢弃** —— 静默丢弃会让上层以为命令已下发，
// 从而把"链路断了"显示成"机械臂不动"。
func (d *serialDevice) WriteLine(line string) error {
	select {
	case <-d.done:
		return fmt.Errorf("串口设备已关闭")
	default:
	}
	if !d.Connected() {
		return fmt.Errorf("串口 %s 未连接：%s", d.cfg.Port, d.UnavailableReason())
	}
	// latest-wins：容量 1，覆盖尚未被取走的旧命令。
	// controller 侧本就有 ACK 门控（同一时刻仅 1 条 JR 在途），
	// 所以覆盖只可能发生在"上一条已收到 OK JR、下一条刚排上队"的瞬间。
	select {
	case d.cmdCh <- line:
	default:
		select {
		case <-d.cmdCh:
		default:
		}
		select {
		case d.cmdCh <- line:
		default:
		}
	}
	select {
	case d.wake <- struct{}{}:
	default:
	}
	return nil
}

func (d *serialDevice) Lines() <-chan Line { return d.lines }

func (d *serialDevice) OnStatus(fn StatusHandler) func() {
	d.mu.Lock()
	d.statusH = append(d.statusH, fn)
	idx := len(d.statusH) - 1
	d.mu.Unlock()
	return func() {
		d.mu.Lock()
		if idx < len(d.statusH) {
			d.statusH[idx] = nil
		}
		d.mu.Unlock()
	}
}

func (d *serialDevice) Close() error {
	d.closeOnce.Do(func() {
		close(d.done)
		d.mu.Lock()
		conn := d.conn
		d.mu.Unlock()
		if conn != nil {
			_ = conn.Close() // 唤醒阻塞中的读
		}
		d.wg.Wait()    // 等 manage / pump 全部退出
		close(d.lines) // 之后才能安全关闭：pump 已不会向它发送
	})
	return nil
}

// ---------------------------------------------------------------------------
// 连接管理
// ---------------------------------------------------------------------------

func (d *serialDevice) setConnected(ok bool, reason string) {
	d.mu.Lock()
	if ok {
		reason = ""
	}
	changed := d.connected != ok || d.reason != reason
	d.connected = ok
	d.reason = reason
	hs := append([]StatusHandler(nil), d.statusH...)
	d.mu.Unlock()
	if !changed {
		return
	}
	for _, h := range hs {
		if h != nil {
			h(ok, reason)
		}
	}
}

func (d *serialDevice) sleepOrDone(dur time.Duration) bool {
	select {
	case <-d.done:
		return false
	case <-time.After(dur):
		return true
	}
}

func (d *serialDevice) manage() {
	defer d.wg.Done()
	reconnect := time.Duration(d.cfg.ReconnectSec) * time.Second
	for {
		select {
		case <-d.done:
			return
		default:
		}
		conn, err := dialSerial(d.cfg)
		if err != nil {
			d.setConnected(false, err.Error())
			log.Printf("[serial] 打开 %s 失败: %v（%v 后重试）", d.cfg.Port, err, reconnect)
			if !d.sleepOrDone(reconnect) {
				return
			}
			continue
		}

		d.mu.Lock()
		d.conn = conn
		d.mu.Unlock()

		// ── 坑 1：bootloader 静默窗口 ──────────────────────────────────
		// 打开端口会拉低 DTR → ATmega328P 复位 → optiboot 等 ~2.5s 才把控制权
		// 交给固件；窗口内下发的指令会被**静默丢弃**。
		// 这段窗口内 Connected() 保持 false，让 controller 直接拒收命令
		// （比"收下再丢"诚实：上层能立刻告诉用户"机械臂还在启动"）。
		if d.cfg.ConnectSettleMs > 0 {
			if !d.sleepOrDone(time.Duration(d.cfg.ConnectSettleMs) * time.Millisecond) {
				_ = conn.Close()
				return
			}
		}

		// ── 坑 2：引导交接后首个数据包被吞 ────────────────────────────
		// 先投一个无害的换行当"第 1 个包"，让真正的 STATUS 成为第 2 个包。
		if d.cfg.Warmup {
			if !d.warmup(conn) {
				d.setConnected(false, "下位机无应答（暖机失败）")
				_ = conn.Close()
				log.Printf("[serial] %s 暖机失败，%v 后重试", d.cfg.Port, reconnect)
				if !d.sleepOrDone(reconnect) {
					return
				}
				continue
			}
		}

		log.Printf("[serial] 已连接 %s @ %d %d%s%d（ack 超时 %dms）",
			d.cfg.Port, d.cfg.Baud, d.cfg.DataBits, strings.ToUpper(d.cfg.Parity), d.cfg.StopBits, d.cfg.AckTimeoutMs)
		d.setConnected(true, "")

		d.pump(conn)

		_ = conn.Close()
		d.mu.Lock()
		d.conn = nil
		d.mu.Unlock()
		d.setConnected(false, "串口已断开，重连中…")
		if !d.sleepOrDone(reconnect) {
			return
		}
	}
}

// warmup 处理坑 2，并在成功时把"固件开机位（全舵机 90°）"作为首个 STATE 上报。
//
// 为什么首个 STATE 很重要：Uno 每次被 DTR 复位后舵机都会弹回 90°，
// 而 controller 的 lastState 是内存里的旧值。不纠正的话 UI 会显示一个与现场
// 完全不符的 Actual；这里用固件自己的 STATUS 把它拉回真相。
func (d *serialDevice) warmup(conn io.ReadWriteCloser) bool {
	if _, err := io.WriteString(conn, "\n"); err != nil {
		return false
	}
	deadline := time.Now().Add(4 * time.Second)
	for time.Now().Before(deadline) {
		select {
		case <-d.done:
			return false
		default:
		}
		if _, err := io.WriteString(conn, "STATUS\r\n"); err != nil {
			return false
		}
		applied := map[int]float64{}
		if d.readServoStatus(conn, applied, 250*time.Millisecond) {
			d.emitStateFromServo(applied)
			return true
		}
	}
	return false
}

// readServoStatus 直接读连接（不经 rawCh），直到拿到一行含 `S<id>=` 的 STATUS 回执。
// 仅用于暖机阶段与 pump 之外的一次性探测。
func (d *serialDevice) readServoStatus(conn io.Reader, dst map[int]float64, wait time.Duration) bool {
	until := time.Now().Add(wait)
	buf := make([]byte, 256)
	var acc []byte
	for time.Now().Before(until) {
		n, err := conn.Read(buf)
		if n > 0 {
			acc = append(acc, buf[:n]...)
			for {
				idx := bytes.IndexByte(acc, '\n')
				if idx < 0 {
					break
				}
				line := strings.TrimRight(string(acc[:idx]), "\r")
				acc = acc[idx+1:]
				if servoKVRe.MatchString(line) {
					collectServoKV(line, dst)
					return true
				}
			}
			if len(acc) > 4096 {
				acc = acc[:0]
			}
		}
		if err != nil {
			if ne, ok := err.(interface{ Timeout() bool }); ok && ne.Timeout() {
				time.Sleep(2 * time.Millisecond)
				continue
			}
			if err == io.EOF {
				return false
			}
		}
		if n == 0 {
			time.Sleep(2 * time.Millisecond)
		}
	}
	return false
}

// ---------------------------------------------------------------------------
// 单 goroutine 泵：读回执 + 执行待发命令
// ---------------------------------------------------------------------------
//
// 为什么读与写放在同一个 goroutine：命令-应答门控需要"发一条 SET，然后一直读
// 直到它的 OK SET 回来"。若读在另一个 goroutine，就得把回执再转发回来并处理
// 归属判定（哪一行属于哪条指令）。放在一个 goroutine 里，顺序天然确定，零锁。
//
// ⚠️ 这里**不能用 bufio.Reader**：Windows 串口读超时会返回 (0, nil)，
// bufio 连续 100 次空读会抛 io.ErrNoProgress，空闲约 20s 后必然假性断连，
// 而重连会拉 DTR 使 Uno 复位（舵机全部弹回 90°）。所以手动做行缓冲。
func (d *serialDevice) pump(conn io.ReadWriteCloser) {
	rawCh := make(chan string, 64)
	stop := make(chan struct{})
	var rwg sync.WaitGroup
	rwg.Add(1)
	go func() {
		defer rwg.Done()
		defer close(rawCh)
		buf := make([]byte, 256)
		var acc []byte
		for {
			n, err := conn.Read(buf)
			if n > 0 {
				acc = append(acc, buf[:n]...)
				for {
					idx := bytes.IndexByte(acc, '\n')
					if idx < 0 {
						break
					}
					line := strings.TrimRight(string(acc[:idx]), "\r")
					acc = acc[idx+1:]
					if strings.TrimSpace(line) == "" {
						continue
					}
					select {
					case rawCh <- line:
					default:
						log.Printf("[serial] 回执缓冲满，丢弃: %q", line)
					}
				}
				if len(acc) > 4096 { // 防御：长期无换行的异常流
					acc = acc[:0]
				}
			}
			if err != nil {
				if ne, ok := err.(interface{ Timeout() bool }); ok && ne.Timeout() {
					time.Sleep(2 * time.Millisecond)
					continue
				}
				if err != io.EOF {
					log.Printf("[serial] 读错误: %v", err)
				}
				return
			}
			time.Sleep(2 * time.Millisecond) // n==0：暂无数据，节流避免空转
		}
	}()

	pumpDone := make(chan struct{})
	go func() {
		defer close(pumpDone)
		for {
			// 优先执行待发命令（外层的非阻塞探测保证命令不会被回执流饿死）
			select {
			case cmd := <-d.cmdCh:
				d.execute(conn, rawCh, cmd)
				continue
			default:
			}
			select {
			case <-d.done:
				return
			case cmd := <-d.cmdCh:
				d.execute(conn, rawCh, cmd)
			case line, ok := <-rawCh:
				if !ok {
					return // 读侧退出（断链）
				}
				d.forward(line)
			case <-d.wake:
				// 唤醒信号：下一轮循环会从 cmdCh 取
			}
		}
	}()

	select {
	case <-d.done:
	case <-pumpDone:
	}
	close(stop)
	_ = conn.Close() // 唤醒阻塞中的读
	rwg.Wait()
	<-pumpDone
}

// forward 把非指令回执的行（开机横幅 / 异步事件）透传给上层。
// controller 会把无法识别的行记进日志，不影响 ACK 门控。
func (d *serialDevice) forward(line string) {
	log.Printf("[serial] 设备: %s", line)
	select {
	case d.lines <- Line{Text: line, At: time.Now()}:
	default:
	}
}

func (d *serialDevice) emit(text string) {
	select {
	case d.lines <- Line{Text: text, At: time.Now()}:
	default:
		log.Printf("[serial] 回执行满，丢弃: %q", text)
	}
}

// ---------------------------------------------------------------------------
// 关节级门面：JR / STATUS / RESET / PING
// ---------------------------------------------------------------------------

func (d *serialDevice) execute(conn io.ReadWriteCloser, rawCh <-chan string, cmd string) {
	verb, rest := splitVerb(cmd)
	switch strings.ToUpper(verb) {
	case "JR":
		d.execJR(conn, rawCh, rest)
	case "STATUS", "?":
		d.execStatus(conn, rawCh)
	case "RESET":
		d.execReset(conn, rawCh)
	case "PING":
		d.emit("OK PING")
	default:
		// 未识别的动词：透传给固件（便于调试期直接下发 SET 等），
		// 回执原样转发但**不合成**任何关节级回执。
		if _, err := io.WriteString(conn, cmd+"\r\n"); err != nil {
			log.Printf("[serial] 写入失败: %v", err)
		}
	}
}

func splitVerb(cmd string) (verb, rest string) {
	cmd = strings.TrimSpace(cmd)
	if i := strings.IndexAny(cmd, " \t"); i >= 0 {
		return cmd[:i], cmd[i+1:]
	}
	return cmd, ""
}

// execJR 把关节级整帧命令翻译成固件能懂的 SET 序列。
//
// ⚠️ 固件的 SET 语法是 `SET <id> <angle> <id> <angle>…`（空格分隔、**无 `=`**），
// 而回执是 `OK SET S<id>=<angle>` 形式 —— 两者格式不同，别照着回执拼请求。
//
// 四舵机 + MAX_PAIRS=3 ⇒ 恰好拆成 2 条。
func (d *serialDevice) execJR(conn io.ReadWriteCloser, rawCh <-chan string, rest string) {
	vals, err := parseFloats(rest)
	if err != nil || len(vals) < len(d.order) {
		d.emit(fmt.Sprintf("ERR JR 参数个数/格式错误: %q", rest))
		return
	}
	joints := make(map[string]float64, len(d.order))
	for i, id := range d.order {
		joints[id] = vals[i]
	}

	type pair struct {
		ch    int
		angle int
	}
	// 按关节顺序展开执行器；舵机角取整（固件只吃整数并会按硬限位钳位）
	pairs := make([]pair, 0, len(d.order))
	for _, id := range d.order {
		for _, a := range d.model.ActuatorsForJoint(id) {
			pairs = append(pairs, pair{ch: a.Channel, angle: int(math.Round(robot.JointToServo(a, joints[id])))})
		}
	}
	if len(pairs) == 0 {
		d.emit("ERR JR 没有可下发的执行器")
		return
	}

	applied := make(map[int]float64, len(pairs))
	for start := 0; start < len(pairs); start += maxPairsPerSet {
		end := start + maxPairsPerSet
		if end > len(pairs) {
			end = len(pairs)
		}
		line := "SET"
		for _, p := range pairs[start:end] {
			line += fmt.Sprintf(" %d %d", p.ch, p.angle)
		}
		if _, err := io.WriteString(conn, line+"\r\n"); err != nil {
			log.Printf("[serial] 写 SET 失败: %v", err)
			d.emit("ERR 串口写入失败")
			return
		}
		if err := d.awaitAck(rawCh, "SET", applied); err != nil {
			d.emit(fmt.Sprintf("ERR %s（%s）", err.Error(), line))
			return
		}
	}

	// OK JR 携带的是**已应用（钳位后）的舵机角**，供 controller 核对标定。
	// 注意它**不是实际位置** —— 固件不知道舵机真的转到哪了。
	d.emit(protocol.EncodeOKJR(applied))
	// STATE 同样只能表示"固件内部的目标值"（开环）。
	d.emitStateFromServo(applied)
}

// execStatus 查询固件内部状态并转成关节级 STATE。
func (d *serialDevice) execStatus(conn io.ReadWriteCloser, rawCh <-chan string) {
	if _, err := io.WriteString(conn, "STATUS\r\n"); err != nil {
		d.emit("ERR 串口写入失败")
		return
	}
	applied := map[int]float64{}
	if err := d.awaitLineWithServo(rawCh, applied); err != nil {
		d.emit(fmt.Sprintf("ERR %s（STATUS）", err.Error()))
		return
	}
	d.emitStateFromServo(applied)
}

// execReset 让固件把所有舵机驱到 90°（= 机械臂的 HOME 舵机位）。
func (d *serialDevice) execReset(conn io.ReadWriteCloser, rawCh <-chan string) {
	if _, err := io.WriteString(conn, "RESET\r\n"); err != nil {
		d.emit("ERR 串口写入失败")
		return
	}
	// 固件对 RESET 回两行：`OK RESET -> 90` 紧跟 `STATUS S6=..(H) …`。
	// 第一行只有 OK，没有数值；必须继续读第二行才拿得到舵机角。
	applied := map[int]float64{}
	if err := d.awaitAck(rawCh, "RESET", applied); err != nil {
		d.emit(fmt.Sprintf("ERR %s（RESET）", err.Error()))
		return
	}
	d.emit("OK RESET")
	// 这里**不合成 OK JR**：RESET 时没有 JR 在途，凭空发一个 OK JR
	// 会让 controller 的标定核对拿错基准去比对。
	if d.awaitLineWithServo(rawCh, applied) == nil {
		d.emitStateFromServo(applied)
	}
}

// awaitAck 读到本指令的回执行（OK/ERR 开头），顺带收集其中的 `S<id>=<angle>`。
//
// 其余行（开机横幅、`# ` 异步事件）记录并跳过，不参与判定。
func (d *serialDevice) awaitAck(rawCh <-chan string, what string, applied map[int]float64) error {
	return d.await(rawCh, applied, func(line string) bool {
		up := strings.ToUpper(line)
		return strings.HasPrefix(up, "OK") || strings.HasPrefix(up, "ERR")
	}, what)
}

// awaitLineWithServo 读到一行含 `S<id>=` 的回执（用于 STATUS 这类"回执本身就是数据"的情况）。
func (d *serialDevice) awaitLineWithServo(rawCh <-chan string, applied map[int]float64) error {
	return d.await(rawCh, applied, func(line string) bool {
		return servoKVRe.MatchString(line)
	}, "STATUS")
}

// await 在超时内读 rawCh 直到 hit(line) 为真。
func (d *serialDevice) await(rawCh <-chan string, applied map[int]float64, hit func(string) bool, what string) error {
	timeout := time.Duration(d.cfg.AckTimeoutMs) * time.Millisecond
	timer := time.NewTimer(timeout)
	defer timer.Stop()
	for {
		select {
		case <-d.done:
			return fmt.Errorf("设备已关闭")
		case <-timer.C:
			return fmt.Errorf("固件未应答 %s（>%dms）", what, d.cfg.AckTimeoutMs)
		case line, ok := <-rawCh:
			if !ok {
				return fmt.Errorf("串口已断开")
			}
			collectServoKV(line, applied)
			if hit(line) {
				if strings.HasPrefix(strings.ToUpper(line), "ERR") {
					return fmt.Errorf("%s", line)
				}
				return nil
			}
			d.forward(line)
		}
	}
}

// collectServoKV 把一行里的 `S<id>=<angle>` 全部收进 dst。
func collectServoKV(line string, dst map[int]float64) {
	for _, m := range servoKVRe.FindAllStringSubmatch(line, -1) {
		ch, err1 := strconv.Atoi(m[1])
		ang, err2 := strconv.ParseFloat(m[2], 64)
		if err1 == nil && err2 == nil {
			dst[ch] = ang
		}
	}
}

// emitStateFromServo 用舵机角反算关节角并上报 STATE。
//
// ⚠️ 这是**开环**状态：来源是固件记着的目标值，不是测量值。
func (d *serialDevice) emitStateFromServo(servoAngles map[int]float64) {
	if len(servoAngles) == 0 {
		return
	}
	d.emitState(protocol.ServoAnglesToJoints(d.model, servoAngles))
}

func (d *serialDevice) emitState(joints map[string]float64) {
	d.emit(protocol.EncodeState(d.order, joints))
}

// ---------------------------------------------------------------------------
// 小工具
// ---------------------------------------------------------------------------

func parseFloats(s string) ([]float64, error) {
	fields := strings.Fields(s)
	if len(fields) == 0 {
		return nil, fmt.Errorf("没有数值")
	}
	out := make([]float64, 0, len(fields))
	for _, f := range fields {
		v, err := strconv.ParseFloat(f, 64)
		if err != nil {
			return nil, fmt.Errorf("解析 %q 失败: %w", f, err)
		}
		out = append(out, v)
	}
	return out, nil
}
