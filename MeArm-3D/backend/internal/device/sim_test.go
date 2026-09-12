package device

import (
	"math"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
	"time"

	"armpilot/backend/internal/robot"
)

func loadModel(t *testing.T) *robot.Model {
	t.Helper()
	p, err := filepath.Abs(filepath.Join("..", "..", "..", "config", "robot.yaml"))
	if err != nil {
		t.Fatalf("路径解析失败: %v", err)
	}
	m, err := robot.Load(p)
	if err != nil {
		t.Fatalf("Load 失败: %v", err)
	}
	return m
}

func newSim(t *testing.T, tune SimTuning) *SimDevice {
	t.Helper()
	d, err := NewSim(loadModel(t), tune)
	if err != nil {
		t.Fatalf("NewSim 失败: %v", err)
	}
	t.Cleanup(func() { d.Close() })
	return d
}

// next 取下一行；超时返回空串（避免测试挂死）。
func next(d *SimDevice, timeout time.Duration) string {
	select {
	case l := <-d.Lines():
		return l.Text
	case <-time.After(timeout):
		return ""
	}
}

// waitContains 跳过中间行，直到出现包含 want 的行。
func waitContains(d *SimDevice, want string, timeout time.Duration) string {
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		got := next(d, time.Until(deadline))
		if got == "" {
			return ""
		}
		if strings.Contains(got, want) {
			return got
		}
	}
	return ""
}

// 初始位置必须是 HOME 位对应的四个 90°（robot.yaml 的自洽性）。
func TestSimInitialPositionIsHomeServos(t *testing.T) {
	d := newSim(t, DefaultSimTuning())
	snap := d.snapshot()
	for _, ch := range []int{6, 7, 8, 9} {
		if math.Abs(snap[ch]-90) > 1e-6 {
			t.Errorf("S%d 初始 = %.6f, 期望 90（HOME 位四个舵机全 90）", ch, snap[ch])
		}
	}
}

func TestSimRejectsOutOfRangeJoint(t *testing.T) {
	d := newSim(t, DefaultSimTuning())
	if err := d.WriteLine("JR 0 0.85 95 50"); err != nil {
		t.Fatalf("WriteLine 报错: %v", err)
	}
	got := waitContains(d, "ERR JOINT", 300*time.Millisecond)
	want := "ERR JOINT elbow 95.00 (limit 108.44..141.86)"
	if got != want {
		t.Errorf("回执 = %q, 期望 %q", got, want)
	}
	// ⚠️ 被拒绝时**位置必须保持不动**：真实固件也不会执行非法指令
	snap := d.snapshot()
	for _, ch := range []int{6, 7, 8, 9} {
		if math.Abs(snap[ch]-90) > 1e-6 {
			t.Errorf("被拒绝后 S%d = %.6f, 期望仍为 90（位置不应变化）", ch, snap[ch])
		}
	}
}

func TestSimAcceptsHomeJR(t *testing.T) {
	d := newSim(t, DefaultSimTuning())
	if err := d.WriteLine("JR 0 0.85 112.62 50"); err != nil {
		t.Fatalf("WriteLine 报错: %v", err)
	}
	got := waitContains(d, "OK JR", 300*time.Millisecond)
	want := "OK JR S9=90.00 S8=90.00 S7=90.00 S6=90.00"
	if got != want {
		t.Errorf("回执 = %q, 期望 %q", got, want)
	}
}

// 延迟语义：延迟期内不得有任何回执（印证"命令送达前不回复"）。
func TestSimLatencyDelaysAck(t *testing.T) {
	tune := DefaultSimTuning()
	tune.LatencyMs = 120
	d := newSim(t, tune)

	if err := d.WriteLine("JR 0 0.85 112.62 50"); err != nil {
		t.Fatalf("WriteLine 报错: %v", err)
	}
	if got := next(d, 60*time.Millisecond); got != "" {
		t.Errorf("延迟期内不该有回执，却收到 %q", got)
	}
	if got := waitContains(d, "OK JR", 300*time.Millisecond); got == "" {
		t.Error("延迟结束后应收到 OK JR")
	}
}

// 有限角速度：舵机不可能瞬间到位，中途必须出现"未到位"的 STATE。
func TestSimFiniteServoSpeed(t *testing.T) {
	tune := DefaultSimTuning()
	tune.MaxServoSpeed = 60 // 度/秒
	tune.TickMs = 100       // 一步 = 6°
	tune.LatencyMs = 0      // 去掉延迟，专注速度
	d := newSim(t, tune)

	// 让 S7（肩，1.44018 关节度/舵机度）从 90° 转到 120°：差 30° 舵机角，需 5 步
	if err := d.WriteLine("JR 0 20.8 112.62 50"); err != nil {
		t.Fatalf("WriteLine 报错: %v", err)
	}
	if got := waitContains(d, "OK JR", 200*time.Millisecond); got == "" {
		t.Fatal("应收到 OK JR")
	}

	// 第一步之后肩关节应只走了一小段，而不是直接到 20.8°
	state := waitContains(d, "STATE", 300*time.Millisecond)
	if state == "" {
		t.Fatal("应收到 STATE")
	}
	early := parseStateShoulder(t, state)
	if early >= 20.8 {
		t.Errorf("第一步就到了 %.3f°（应远小于 20.8）—— 有限角速度未生效", early)
	}
	if early <= 0.85 {
		t.Errorf("第一步没有任何进展（%.3f°）—— 速度模型未推进", early)
	}

	// 最终必须收敛到命令值（容差取 JR 的 1 位小数量化）
	final := waitContains(d, "STATE 0.00 20.8", 2*time.Second)
	if final == "" {
		t.Errorf("未在 2s 内收敛到命令值，最后状态: %q", state)
	}
}

func parseStateShoulder(t *testing.T, state string) float64 {
	t.Helper()
	fields := strings.Fields(state)
	if len(fields) < 3 {
		t.Fatalf("STATE 行格式异常: %q", state)
	}
	v, err := strconv.ParseFloat(fields[2], 64)
	if err != nil {
		t.Fatalf("解析 STATE 的肩角失败: %q", state)
	}
	return v
}

// RESET 语义 = 全部舵机 90°（= HOME）。协议 §4 明确不许改成"关节全 0"。
func TestSimResetSemantics(t *testing.T) {
	d := newSim(t, DefaultSimTuning())
	// 先离开 HOME
	if err := d.WriteLine("JR 0 30 120 50"); err != nil {
		t.Fatalf("WriteLine 报错: %v", err)
	}
	waitContains(d, "OK JR", 300*time.Millisecond)

	if err := d.WriteLine("RESET"); err != nil {
		t.Fatalf("WriteLine 报错: %v", err)
	}
	got := waitContains(d, "OK JR", 500*time.Millisecond)
	want := "OK JR S9=90.00 S8=90.00 S7=90.00 S6=90.00"
	if got != want {
		t.Errorf("RESET 回执 = %q, 期望 %q", got, want)
	}
}

func TestSimStatusEchoesServoAngles(t *testing.T) {
	d := newSim(t, DefaultSimTuning())
	if err := d.WriteLine("STATUS"); err != nil {
		t.Fatalf("WriteLine 报错: %v", err)
	}
	got := waitContains(d, "STATUS", 300*time.Millisecond)
	if got != "STATUS S9=90 S7=90 S8=90 S6=90" {
		t.Errorf("STATUS = %q, 期望全 90", got)
	}
}

func TestSimUnknownCommand(t *testing.T) {
	d := newSim(t, DefaultSimTuning())
	if err := d.WriteLine("BOGUS 1 2 3"); err != nil {
		t.Fatalf("WriteLine 报错: %v", err)
	}
	if got := waitContains(d, "ERR UNKNOWN", 300*time.Millisecond); got == "" {
		t.Error("未知指令应回 ERR UNKNOWN")
	}
}

func TestSimJRArityMismatch(t *testing.T) {
	d := newSim(t, DefaultSimTuning())
	if err := d.WriteLine("JR 0 30"); err != nil {
		t.Fatalf("WriteLine 报错: %v", err)
	}
	got := waitContains(d, "ERR ARG", 300*time.Millisecond)
	if !strings.Contains(got, "需要 4 个关节角") {
		t.Errorf("参数个数不符应回 ERR ARG，实际 = %q", got)
	}
}

// 开机静默窗口：窗口内指令被吞掉且不回执（模拟 Uno bootloader）。
func TestSimBootSilenceWindow(t *testing.T) {
	tune := DefaultSimTuning()
	tune.BootMs = 150
	d := newSim(t, tune)

	if err := d.WriteLine("JR 0 0.85 112.62 50"); err != nil {
		t.Fatalf("WriteLine 报错: %v", err)
	}
	if got := next(d, 60*time.Millisecond); got != "" {
		t.Errorf("开机静默窗口内不该有回执，却收到 %q", got)
	}
	// 窗口结束后再发，应正常受理
	time.Sleep(120 * time.Millisecond)
	if err := d.WriteLine("JR 0 0.85 112.62 50"); err != nil {
		t.Fatalf("WriteLine 报错: %v", err)
	}
	if got := waitContains(d, "OK JR", 300*time.Millisecond); got == "" {
		t.Error("窗口结束后应能正常受理")
	}
}

// 越界到舵机硬限位也必须被拒（关节限位合法但标定后超舵机范围的极端情况）。
func TestSimInstantModeConvergesImmediately(t *testing.T) {
	tune := DefaultSimTuning()
	tune.MaxServoSpeed = 0 // 瞬时到位
	d := newSim(t, tune)

	if err := d.WriteLine("JR 0 20.8 112.62 50"); err != nil {
		t.Fatalf("WriteLine 报错: %v", err)
	}
	state := waitContains(d, "STATE", 300*time.Millisecond)
	if state == "" {
		t.Fatal("瞬时模式下送达即应回一次 STATE")
	}
	got := parseStateShoulder(t, state)
	if math.Abs(got-20.8) > 0.1 {
		t.Errorf("瞬时模式下肩角 = %.3f, 期望 ≈20.8", got)
	}
}

func TestSimCloseIsIdempotent(t *testing.T) {
	d := newSim(t, DefaultSimTuning())
	if err := d.Close(); err != nil {
		t.Fatalf("首次 Close 报错: %v", err)
	}
	if err := d.Close(); err != nil {
		t.Fatalf("重复 Close 报错: %v", err)
	}
	if d.Connected() {
		t.Error("Close 后 Connected 应为 false")
	}
	if err := d.WriteLine("JR 0 0.85 112.62 50"); err == nil {
		t.Error("Close 后 WriteLine 应报错，而不是静默丢弃")
	}
}
