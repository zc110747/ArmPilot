package device

import (
	"errors"
	"fmt"
	"io"
	"strconv"
	"strings"
	"sync"
	"testing"
	"time"
)

// ---------------------------------------------------------------------------
// 假固件：只做真固件真正会做的事（含它的怪癖）
// ---------------------------------------------------------------------------

// fakeHardLimits 取自 MeArm-Device/bsp/servo.h，与 config/robot.yaml 的
// actuator.limits 一致（这是"固件与真值同源"的一处可核对点）。
var fakeHardLimits = map[int][2]int{9: {30, 150}, 7: {80, 160}, 8: {20, 100}, 6: {40, 130}}

type fakeArm struct {
	mu      sync.Mutex
	state   map[int]int // 舵机 id -> 固件记着的角度
	written []string
	lines   chan string
	closed  chan struct{}
	once    sync.Once
	silent  bool // true = 只收不回（模拟 bootloader 窗口 / 死机）
}

func newFakeArm() *fakeArm {
	return &fakeArm{
		state:  map[int]int{9: 90, 8: 90, 7: 90, 6: 90},
		lines:  make(chan string, 64),
		closed: make(chan struct{}),
	}
}

// Read 复刻 Windows 的"立即返回"读模式：有数据就给，没有就返回 (0, nil)。
func (f *fakeArm) Read(p []byte) (int, error) {
	select {
	case <-f.closed:
		return 0, io.EOF
	case s := <-f.lines:
		return copy(p, s), nil
	case <-time.After(2 * time.Millisecond):
		return 0, nil
	}
}

func (f *fakeArm) Write(p []byte) (int, error) {
	f.mu.Lock()
	f.written = append(f.written, string(p))
	silent := f.silent
	f.mu.Unlock()
	if !silent {
		f.handle(strings.TrimSpace(string(p)))
	}
	return len(p), nil
}

func (f *fakeArm) Close() error {
	f.once.Do(func() { close(f.closed) })
	return nil
}

func (f *fakeArm) setSilent(v bool) {
	f.mu.Lock()
	f.silent = v
	f.mu.Unlock()
}

func (f *fakeArm) send(s string) {
	select {
	case f.lines <- s:
	default:
	}
}

func (f *fakeArm) writtenLines() []string {
	f.mu.Lock()
	defer f.mu.Unlock()
	out := make([]string, 0, len(f.written))
	for _, w := range f.written {
		out = append(out, strings.TrimSpace(w))
	}
	return out
}

func (f *fakeArm) angle(id int) int {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.state[id]
}

func (f *fakeArm) clamp(id, ang int) int {
	lim, ok := fakeHardLimits[id]
	if !ok {
		return ang
	}
	if ang < lim[0] {
		return lim[0]
	}
	if ang > lim[1] {
		return lim[1]
	}
	return ang
}

// statusLine 复刻固件 arm_status()：`STATUS S6=90(H) S7=90(H) S8=90(H) S9=90(H)`
// ⚠️ 注意是 `STATUS`（不是 `STATE`），且带模式字母 `(H)/(A)` —— 解析必须容忍。
func (f *fakeArm) statusLine() string {
	f.mu.Lock()
	defer f.mu.Unlock()
	return fmt.Sprintf("STATUS S6=%d(H) S7=%d(H) S8=%d(H) S9=%d(H)\r\n",
		f.state[6], f.state[7], f.state[8], f.state[9])
}

func (f *fakeArm) handle(cmd string) {
	fields := strings.Fields(cmd)
	if len(fields) == 0 {
		return
	}
	switch strings.ToUpper(fields[0]) {
	case "SET":
		if (len(fields)-1)%2 != 0 || len(fields) < 3 {
			f.send("ERR SYNTAX\r\n")
			return
		}
		pairs := fields[1:]
		out := "OK SET"
		for i := 0; i+1 < len(pairs); i += 2 {
			id, err1 := strconv.Atoi(pairs[i])
			ang, err2 := strconv.Atoi(pairs[i+1])
			if err1 != nil || err2 != nil {
				f.send("ERR BAD_ID\r\n")
				return
			}
			ang = f.clamp(id, ang)
			f.mu.Lock()
			f.state[id] = ang
			f.mu.Unlock()
			out += fmt.Sprintf(" S%d=%d", id, ang)
		}
		f.send(out + "\r\n")

	case "STATUS", "?":
		f.send(f.statusLine())

	case "RESET":
		f.mu.Lock()
		for id := range f.state {
			f.state[id] = 90
		}
		f.mu.Unlock()
		f.send("OK RESET -> 90\r\n")
		f.send(f.statusLine())

	default:
		f.send("ERR UNKNOWN\r\n")
	}
}

// ---------------------------------------------------------------------------
// 注入用的假串口工厂（每次 dial 返回一个新实例，便于测重连）
// ---------------------------------------------------------------------------

type armFactory struct {
	mu    sync.Mutex
	arms  []*fakeArm
	fails int
}

func (a *armFactory) dial(SerialConfig) (io.ReadWriteCloser, error) {
	a.mu.Lock()
	defer a.mu.Unlock()
	if a.fails > 0 {
		a.fails--
		return nil, errors.New("端口被其它程序占用")
	}
	f := newFakeArm()
	a.arms = append(a.arms, f)
	return f, nil
}

func (a *armFactory) last() *fakeArm {
	a.mu.Lock()
	defer a.mu.Unlock()
	if len(a.arms) == 0 {
		return nil
	}
	return a.arms[len(a.arms)-1]
}

func (a *armFactory) count() int {
	a.mu.Lock()
	defer a.mu.Unlock()
	return len(a.arms)
}

// ---------------------------------------------------------------------------
// 测试脚手架
// ---------------------------------------------------------------------------

func installFactory(t *testing.T) *armFactory {
	t.Helper()
	fac := &armFactory{}
	prev := dialSerial
	dialSerial = fac.dial
	t.Cleanup(func() { dialSerial = prev })
	return fac
}

func testSerialConfig() SerialConfig {
	return SerialConfig{
		Port: "COM99", Baud: 115200, DataBits: 8, StopBits: 1, Parity: "N",
		ReconnectSec: 1, AckTimeoutMs: 120, ConnectSettleMs: 0, Warmup: true,
	}
}

func openTestSerial(t *testing.T, cfg SerialConfig) Device {
	t.Helper()
	d, err := NewSerial(cfg, loadModel(t))
	if err != nil {
		t.Fatalf("NewSerial: %v", err)
	}
	t.Cleanup(func() { _ = d.Close() })
	return d
}

func waitConnected(t *testing.T, d Device, timeout time.Duration) {
	t.Helper()
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		if d.Connected() {
			return
		}
		time.Sleep(2 * time.Millisecond)
	}
	t.Fatalf("设备在 %v 内未进入 connected（原因: %s）", timeout, d.UnavailableReason())
}

// waitLine 从回执行里找第一个满足谓词的行。
func waitLine(t *testing.T, d Device, what string, match func(string) bool, timeout time.Duration) string {
	t.Helper()
	deadline := time.After(timeout)
	for {
		select {
		case l, ok := <-d.Lines():
			if !ok {
				t.Fatalf("等待 %s 时回执行被关闭", what)
			}
			if match(l.Text) {
				return l.Text
			}
		case <-deadline:
			t.Fatalf("超时未收到 %s", what)
		}
	}
}

// ---------------------------------------------------------------------------
// 用例
// ---------------------------------------------------------------------------

func TestSerialRejectsBadConfig(t *testing.T) {
	m := loadModel(t)
	if _, err := NewSerial(SerialConfig{Port: ""}, m); err == nil {
		t.Fatal("空 port 应当报错")
	}
	if _, err := NewSerial(SerialConfig{Port: "COM16"}, nil); err == nil {
		t.Fatal("缺 model 应当报错")
	}
	if _, err := NewSerial(SerialConfig{Port: "COMXX"}, m); err == nil {
		t.Fatal("COM 号非数字应当报错")
	}
}

// 坑 1：bootloader 静默窗口内不许假装可用。
// 若这里返回 true，controller 会把指令发进黑洞，UI 上表现为"机械臂不动"。
func TestSerialSilentDuringBootSettleWindow(t *testing.T) {
	fac := installFactory(t)
	cfg := testSerialConfig()
	cfg.ConnectSettleMs = 150
	d := openTestSerial(t, cfg)

	// dial 发生在静默窗口之前，先等它完成（否则 fac.last() 还是 nil）
	deadline := time.Now().Add(2 * time.Second)
	for fac.count() == 0 && time.Now().Before(deadline) {
		time.Sleep(2 * time.Millisecond)
	}
	if fac.count() != 1 {
		t.Fatalf("应在静默窗口前完成 1 次 dial，实际 %d", fac.count())
	}
	arm := fac.last()

	if d.Connected() {
		t.Fatal("静默窗口内 Connected() 必须为 false")
	}
	if err := d.WriteLine("JR 0 20 112.6 50"); err == nil {
		t.Fatal("静默窗口内 WriteLine 必须返回错误（不能静默丢弃）")
	}
	if got := arm.writtenLines(); len(got) != 0 {
		t.Fatalf("静默窗口内不该写出任何字节，实际=%v", got)
	}

	waitConnected(t, d, 3*time.Second)

	// 坑 2：暖机包必须是第一个写出的字节（它会被链路吞掉）
	writes := arm.writtenLines()
	if len(writes) == 0 || writes[0] != "" {
		t.Fatalf("首个写入应当是暖机换行（trim 后为空串），实际=%q", writes)
	}
	if len(writes) < 2 || !strings.HasPrefix(writes[1], "STATUS") {
		t.Fatalf("第二个写入应当是 STATUS 探测，实际=%q", writes)
	}
}

// 暖机成功时要把固件开机位（全舵机 90°）作为首个 STATE 上报，
// 否则 Uno 复位把舵机弹回 90° 后，UI 的 Actual 还是内存里的旧值。
func TestSerialWarmupPublishesPowerOnState(t *testing.T) {
	installFactory(t)
	d := openTestSerial(t, testSerialConfig())
	line := waitLine(t, d, "开机位 STATE", func(s string) bool {
		return strings.HasPrefix(s, "STATE")
	}, 3*time.Second)

	got, err := parseFloats(strings.TrimPrefix(line, "STATE"))
	if err != nil {
		t.Fatal(err)
	}
	m := loadModel(t)
	want := []float64{m.HomePose["base"], m.HomePose["shoulder"], m.HomePose["elbow"], m.HomePose["gripper"]}
	if len(got) != 4 {
		t.Fatalf("STATE 应有 4 个分量，实际 %v", got)
	}
	for i := range want {
		if diff := got[i] - want[i]; diff > 0.01 || diff < -0.01 {
			t.Errorf("STATE[%d] = %.4f，期望 HOME %.4f", i, got[i], want[i])
		}
	}
}

// 核心：关节级 JR 必须被翻译成固件能懂的舵机级 SET，且因 MAX_PAIRS=3 拆成 2 条。
func TestSerialJRSplitIntoTwoSets(t *testing.T) {
	fac := installFactory(t)
	d := openTestSerial(t, testSerialConfig())
	waitConnected(t, d, 3*time.Second)
	// 丢掉暖机产生的 STATUS 自身回执（开机位 STATE）
	drainLines(d, 200*time.Millisecond)

	if err := d.WriteLine("JR 0 20 112.6 50"); err != nil {
		t.Fatalf("WriteLine: %v", err)
	}
	okJR := waitLine(t, d, "OK JR", func(s string) bool {
		return strings.HasPrefix(s, "OK JR")
	}, 2*time.Second)
	state := waitLine(t, d, "STATE", func(s string) bool {
		return strings.HasPrefix(s, "STATE")
	}, 2*time.Second)

	// 4 舵机 = 3 + 1 ⇒ 恰好两条 SET，且语法是 `SET <id> <angle>`（空格分隔、无 `=`）
	writes := fac.last().writtenLines()
	var sets []string
	for _, w := range writes {
		if strings.HasPrefix(w, "SET") {
			sets = append(sets, w)
		}
	}
	if len(sets) != 2 {
		t.Fatalf("应当拆成 2 条 SET，实际 %d 条: %v", len(sets), sets)
	}
	if sets[0] != "SET 9 90 7 118 8 90" {
		t.Errorf("第一条 SET 不符: %q", sets[0])
	}
	if sets[1] != "SET 6 90" {
		t.Errorf("第二条 SET 不符: %q", sets[1])
	}
	// 每条最多 3 组
	for _, s := range sets {
		if n := (len(strings.Fields(s)) - 1) / 2; n > maxPairsPerSet {
			t.Errorf("SET 组数 %d 超过固件上限 %d: %q", n, maxPairsPerSet, s)
		}
	}

	// OK JR 携带的必须是**已应用的舵机角**（通道降序，2 位小数）
	if want := "OK JR S9=90.00 S8=90.00 S7=118.00 S6=90.00"; okJR != want {
		t.Errorf("OK JR = %q，期望 %q", okJR, want)
	}
	// STATE 由舵机角**反算**回关节角（这是标定可逆性的运行期检验）
	if want := "STATE 0.00 20.29 112.62 50.00"; state != want {
		t.Errorf("STATE = %q，期望 %q", state, want)
	}
}

// 固件会按硬限位钳位，回执里给的是钳位后的值 —— 必须原样带进 OK JR，
// 这样 controller 的标定核对才有可能发现两侧不一致。
func TestSerialClampedEchoSurvivesInOKJR(t *testing.T) {
	fac := installFactory(t)
	d := openTestSerial(t, testSerialConfig())
	waitConnected(t, d, 3*time.Second)
	drainLines(d, 200*time.Millisecond)

	// 夹爪 200° → 舵机 40+200 = 240 → 固件钳到硬限位 130
	if err := d.WriteLine("JR 0 0 112.6185771989 200"); err != nil {
		t.Fatal(err)
	}
	okJR := waitLine(t, d, "OK JR", func(s string) bool {
		return strings.HasPrefix(s, "OK JR")
	}, 2*time.Second)

	if !strings.Contains(okJR, "S6=130.00") {
		t.Errorf("OK JR 应携带固件钳位后的 S6=130.00，实际 %q", okJR)
	}
	if got := fac.last().angle(6); got != 130 {
		t.Errorf("固件状态 S6 应为 130，实际 %d", got)
	}
}

// JR 的数值参数必须能容忍「指数形式 / 多余空格」等文本差异
func TestSerialJRArgumentParsing(t *testing.T) {
	installFactory(t)
	d := openTestSerial(t, testSerialConfig())
	waitConnected(t, d, 3*time.Second)
	drainLines(d, 200*time.Millisecond)

	if err := d.WriteLine("JR   0   0   112.6185771989   50  "); err != nil {
		t.Fatal(err)
	}
	waitLine(t, d, "OK JR", func(s string) bool { return strings.HasPrefix(s, "OK JR") }, 2*time.Second)

	// 参数不足 → 明确报错，不许静默
	if err := d.WriteLine("JR 0 0"); err != nil {
		t.Fatal(err)
	}
	if got := waitLine(t, d, "ERR JR", func(s string) bool {
		return strings.HasPrefix(s, "ERR")
	}, 2*time.Second); !strings.Contains(got, "参数个数") {
		t.Errorf("参数不足应报 ERR ... 参数个数，实际 %q", got)
	}
}

// STATUS 的回执本身就是数据（`STATUS S6=..(H) …`），必须被解析成关节级 STATE。
func TestSerialStatusBecomesState(t *testing.T) {
	installFactory(t)
	d := openTestSerial(t, testSerialConfig())
	waitConnected(t, d, 3*time.Second)
	drainLines(d, 200*time.Millisecond)

	if err := d.WriteLine("STATUS"); err != nil {
		t.Fatal(err)
	}
	line := waitLine(t, d, "STATE", func(s string) bool { return strings.HasPrefix(s, "STATE") }, 2*time.Second)
	if want := "STATE 0.00 0.85 112.62 50.00"; line != want {
		t.Errorf("STATE = %q，期望 %q（开机全 90° 对应 HOME 位）", line, want)
	}
}

// RESET 后固件回两行（`OK RESET -> 90` + `STATUS …`）；
// 必须读到第二行才拿得到角度，且**不能**凭空合成 OK JR（会污染标定核对的基准）。
func TestSerialResetReadsSecondLineAndEmitsNoOKJR(t *testing.T) {
	fac := installFactory(t)
	d := openTestSerial(t, testSerialConfig())
	waitConnected(t, d, 3*time.Second)
	drainLines(d, 200*time.Millisecond)

	fac.last().mu.Lock()
	fac.last().state[9] = 30
	fac.last().state[7] = 140
	fac.last().mu.Unlock()

	if err := d.WriteLine("RESET"); err != nil {
		t.Fatal(err)
	}
	okReset := waitLine(t, d, "OK RESET", func(s string) bool { return strings.HasPrefix(s, "OK RESET") }, 2*time.Second)
	if okReset == "" {
		t.Fatal("缺 OK RESET")
	}
	line := waitLine(t, d, "RESET 后的 STATE", func(s string) bool {
		return strings.HasPrefix(s, "STATE")
	}, 2*time.Second)
	if want := "STATE 0.00 0.85 112.62 50.00"; line != want {
		t.Errorf("RESET 后 STATE = %q，期望 HOME %q", line, want)
	}
	// 确认没冒出 OK JR
	for _, l := range drainLines(d, 100*time.Millisecond) {
		if strings.HasPrefix(l, "OK JR") {
			t.Errorf("RESET 不该合成 OK JR，实际收到 %q", l)
		}
	}
}

func TestSerialPing(t *testing.T) {
	installFactory(t)
	d := openTestSerial(t, testSerialConfig())
	waitConnected(t, d, 3*time.Second)
	drainLines(d, 200*time.Millisecond)
	if err := d.WriteLine("PING"); err != nil {
		t.Fatal(err)
	}
	if got := waitLine(t, d, "OK PING", func(s string) bool { return s == "OK PING" }, 2*time.Second); got != "OK PING" {
		t.Errorf("PING 回执 = %q", got)
	}
}

// 固件的 ERR 必须原样透传：controller 依赖它归类错误码（JOINT_LIMIT / INTERNAL）。
func TestSerialErrPassthrough(t *testing.T) {
	installFactory(t)
	d := openTestSerial(t, testSerialConfig())
	waitConnected(t, d, 3*time.Second)
	drainLines(d, 200*time.Millisecond)

	if err := d.WriteLine("BOGUS"); err != nil {
		t.Fatal(err)
	}
	if got := waitLine(t, d, "ERR UNKNOWN", func(s string) bool {
		return strings.HasPrefix(s, "ERR")
	}, 2*time.Second); got != "ERR UNKNOWN" {
		t.Errorf("未知动词的 ERR 应原样透传，实际 %q", got)
	}
}

// 固件不回执时必须报 ERR 超时，而不是永久挂着让 controller 干等。
func TestSerialAckTimeoutEmitsErr(t *testing.T) {
	fac := installFactory(t)
	d := openTestSerial(t, testSerialConfig())
	waitConnected(t, d, 3*time.Second)
	drainLines(d, 200*time.Millisecond)

	fac.last().setSilent(true) // 固件"失联"
	if err := d.WriteLine("JR 0 0 112.6 50"); err != nil {
		t.Fatal(err)
	}
	got := waitLine(t, d, "超时 ERR", func(s string) bool {
		return strings.HasPrefix(s, "ERR")
	}, 2*time.Second)
	if !strings.Contains(got, "未应答") {
		t.Errorf("应报未应答，实际 %q", got)
	}
}

// 断链后必须自动重连（Uno 会因此复位到 HOME，故重连成功要重新上报开机位 STATE）。
func TestSerialReconnectsAfterDisconnect(t *testing.T) {
	fac := installFactory(t)
	d := openTestSerial(t, testSerialConfig())
	waitConnected(t, d, 3*time.Second)
	drainLines(d, 200*time.Millisecond)

	fac.last().Close() // 模拟拔线

	deadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(deadline) {
		if fac.count() >= 2 && d.Connected() {
			line := waitLine(t, d, "重连后的开机位 STATE", func(s string) bool {
				return strings.HasPrefix(s, "STATE")
			}, 2*time.Second)
			if !strings.HasPrefix(line, "STATE") {
				t.Errorf("重连后应重新上报 STATE，实际 %q", line)
			}
			return
		}
		time.Sleep(10 * time.Millisecond)
	}
	t.Fatal("断链后未能在 5s 内重连")
}

// 打开失败要如实上报原因（并持续重试），不能让 backend 起不来。
func TestSerialReportsOpenFailureButKeepsRetrying(t *testing.T) {
	fac := installFactory(t)
	fac.fails = 1
	d := openTestSerial(t, testSerialConfig())

	waitConnected(t, d, 5*time.Second)
	if fac.count() != 1 {
		t.Fatalf("重试后应有 1 个成功连接，实际 %d", fac.count())
	}
}

func TestSerialCloseIdempotentAndRejectsWrite(t *testing.T) {
	installFactory(t)
	d := openTestSerial(t, testSerialConfig())
	waitConnected(t, d, 3*time.Second)

	if err := d.Close(); err != nil {
		t.Fatalf("Close: %v", err)
	}
	if err := d.Close(); err != nil {
		t.Fatalf("Close 应幂等，第二次报: %v", err)
	}
	if err := d.WriteLine("JR 0 0 112.6 50"); err == nil {
		t.Fatal("关闭后 WriteLine 必须报错")
	}
}

// 固件原始回执里的 `STATUS S6=90(H)` 必须能被取数，且 `STATUS` 不会被
// 误判成关节级 `STATE`（两者前缀相近，是易错点）。
func TestServoKVRegexOnFirmwareStatusLine(t *testing.T) {
	got := map[int]float64{}
	collectServoKV("STATUS S6=90(H) S7=140(H) S8=20(A) S9=30(H)", got)
	want := map[int]float64{6: 90, 7: 140, 8: 20, 9: 30}
	if len(got) != len(want) {
		t.Fatalf("取到 %v，期望 %v", got, want)
	}
	for k, v := range want {
		if got[k] != v {
			t.Errorf("S%d = %v，期望 %v", k, got[k], v)
		}
	}
}

// ---------------------------------------------------------------------------

func drainLines(d Device, wait time.Duration) []string {
	var out []string
	deadline := time.After(wait)
	for {
		select {
		case l, ok := <-d.Lines():
			if !ok {
				return out
			}
			out = append(out, l.Text)
		case <-deadline:
			return out
		}
	}
}
