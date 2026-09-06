package serial

import (
	"fmt"
	"io"
	"os"
	"strings"
	"sync"
	"testing"
	"time"
)

// fakeSerial 模拟一个会按需回送应答行的下位机，用于验证命令-应答门控。
type fakeSerial struct {
	mu      sync.Mutex
	written []string
	readCh  chan string // 设备应答行（需以 \r\n 结尾）
	doneCh  chan struct{}
	closed  bool
}

func newFakeSerial() *fakeSerial {
	return &fakeSerial{readCh: make(chan string, 16), doneCh: make(chan struct{})}
}

// pushACK 让“设备”回送一行应答（readLoop 会把它当作 ACK）。
func (f *fakeSerial) pushACK(line string) { f.readCh <- line }

func (f *fakeSerial) Write(p []byte) (int, error) {
	f.mu.Lock()
	f.written = append(f.written, string(p))
	f.mu.Unlock()
	return len(p), nil
}

func (f *fakeSerial) Read(p []byte) (int, error) {
	select {
	case <-f.doneCh:
		return 0, io.EOF
	case line := <-f.readCh:
		return copy(p, line), nil
	}
}

func (f *fakeSerial) Close() error {
	f.mu.Lock()
	defer f.mu.Unlock()
	if !f.closed {
		f.closed = true
		close(f.doneCh)
	}
	return nil
}

func (f *fakeSerial) getWritten() []string {
	f.mu.Lock()
	defer f.mu.Unlock()
	out := make([]string, len(f.written))
	copy(out, f.written)
	return out
}

func waitConnected(s *Serial, timeout time.Duration) bool {
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		if s.Connected() {
			return true
		}
		time.Sleep(10 * time.Millisecond)
	}
	return false
}

func TestAckGatingLatestWins(t *testing.T) {
	fake := newFakeSerial()
	dialSerial = func(Config) (io.ReadWriteCloser, error) { return fake, nil }
	defer func() { dialSerial = openPort }()

	var mu sync.Mutex
	var commErr bool
	s := Open(Config{Port: "FAKE", Baud: 9600, ReconnectSec: 1, MinIntervalMs: 0, AckTimeoutMs: 300})
	s.SetStatusHandler(func(_ bool, _ string, ce bool, _ string) {
		mu.Lock()
		commErr = ce
		mu.Unlock()
	})
	if !waitConnected(s, time.Second) {
		t.Fatalf("串口未连接")
	}

	// 1) 下发给首个位置，等 ACK 后再下发后续；验证“逐条下发 + 中间值合并”
	if err := s.WriteLine("JOY 100 512 512 512"); err != nil {
		t.Fatalf("WriteLine 失败: %v", err)
	}
	time.Sleep(30 * time.Millisecond)
	fake.pushACK("OK JOY S6=90 S7=90 S8=90 S9=90\r\n") // 应答首条

	// 2) 连续下发 4 个位置（无 ACK）：中间 3 个应被合并为最新
	for _, p := range []string{
		"JOY 200 512 512 512",
		"JOY 300 512 512 512",
		"JOY 400 512 512 512",
		"JOY 500 512 512 512",
	} {
		if err := s.WriteLine(p); err != nil {
			t.Fatalf("WriteLine 失败: %v", err)
		}
	}
	time.Sleep(30 * time.Millisecond)
	w := fake.getWritten()
	// 期望：首条(100) + 合并后的最新(500) = 2 条；中间 200/300/400 被丢弃
	if len(w) != 2 {
		t.Fatalf("期望下发 2 条(首条+最新)，实际 %d: %v", len(w), w)
	}
	if w[0] != "JOY 100 512 512 512\r\n" {
		t.Fatalf("首条应为 100，实际 %q", w[0])
	}
	if w[1] != "JOY 500 512 512 512\r\n" {
		t.Fatalf("最新应为 500（中间值已合并），实际 %q", w[1])
	}

	// 3) 回送 ACK，确认空闲态不再重复下发
	fake.pushACK("OK JOY S6=90 S7=90 S8=90 S9=90\r\n")
	time.Sleep(50 * time.Millisecond)
	w = fake.getWritten()
	if len(w) != 2 {
		t.Fatalf("空闲态不应再下发，实际 %d: %v", len(w), w)
	}

	mu.Lock()
	ce := commErr
	mu.Unlock()
	if ce {
		t.Fatalf("正常应答不应触发通讯失败")
	}

	s.Close()
}

// TestAsyncLineNotAck 验证：以 "# " 开头的异步事件行（如硬件 IR 回显、序列自动
// 停止）不会被当作命令应答，既不会误清“等待应答”、也不会误判通讯失败；只有真正的
// OK/ERR 应答行才能推进门控。这与下位机把异步事件标记为 "# " 前缀的约定一致。
func TestAsyncLineNotAck(t *testing.T) {
	fake := newFakeSerial()
	dialSerial = func(Config) (io.ReadWriteCloser, error) { return fake, nil }
	defer func() { dialSerial = openPort }()

	var mu sync.Mutex
	var commErr bool
	s := Open(Config{Port: "FAKE", Baud: 9600, ReconnectSec: 1, MinIntervalMs: 0, AckTimeoutMs: 200})
	s.SetStatusHandler(func(_ bool, _ string, ce bool, _ string) {
		mu.Lock()
		commErr = ce
		mu.Unlock()
	})
	if !waitConnected(s, time.Second) {
		t.Fatalf("串口未连接")
	}

	// 下发指令，进入“等待应答”
	if err := s.WriteLine("SET 6 90"); err != nil {
		t.Fatalf("WriteLine 失败: %v", err)
	}
	time.Sleep(20 * time.Millisecond)

	// 下位机先回送一个异步事件行（"# " 前缀），不应被当作应答
	fake.pushACK("# IR RAW=00FF45BA\r\n")
	time.Sleep(40 * time.Millisecond)

	w := fake.getWritten()
	if len(w) != 1 {
		t.Fatalf("异步事件行不应触发新下发，应仅 1 条，实际 %d: %v", len(w), w)
	}
	mu.Lock()
	ce := commErr
	mu.Unlock()
	if ce {
		t.Fatalf("异步事件行不应触发通讯失败，也不应误判应答")
	}

	// 真应答到达，才应清除等待（且因无 pending，不再下发新指令）
	fake.pushACK("OK SET S6=90\r\n")
	time.Sleep(40 * time.Millisecond)
	w = fake.getWritten()
	if len(w) != 1 {
		t.Fatalf("真应答后不应再下发新指令(无 pending)，实际 %d: %v", len(w), w)
	}

	s.Close()
}

// TestSeqStopIdleAcks 锁定下位机适配后的关键行为：此前 `SEQ STOP` 在序列未运行时会
// 直接返回、不输出任何应答行，导致上位机命令-应答门控永远等不到应答而误判通讯失败。
// 适配后固件对 `SEQ STOP` 恒回送 `OK IRSEQ idle ...`，本测试验证该应答被门控正确识别
// （仅下发 1 条、无通讯失败），即新固件契约在 web 侧端到端成立。
func TestSeqStopIdleAcks(t *testing.T) {
	fake := newFakeSerial()
	dialSerial = func(Config) (io.ReadWriteCloser, error) { return fake, nil }
	defer func() { dialSerial = openPort }()

	var mu sync.Mutex
	var commErr bool
	s := Open(Config{Port: "FAKE", Baud: 9600, ReconnectSec: 1, MinIntervalMs: 0, AckTimeoutMs: 200})
	s.SetStatusHandler(func(_ bool, _ string, ce bool, _ string) {
		mu.Lock()
		commErr = ce
		mu.Unlock()
	})
	if !waitConnected(s, time.Second) {
		t.Fatalf("串口未连接")
	}

	// 在序列未运行时下发 SEQ STOP
	if err := s.WriteLine("SEQ STOP"); err != nil {
		t.Fatalf("WriteLine 失败: %v", err)
	}
	time.Sleep(20 * time.Millisecond)

	// 下位机适配后恒回送 OK 应答（idle 情形）
	fake.pushACK("OK IRSEQ idle (not running)\r\n")
	time.Sleep(40 * time.Millisecond)

	w := fake.getWritten()
	if len(w) != 1 {
		t.Fatalf("应仅下发 1 条 SEQ STOP，实际 %d: %v", len(w), w)
	}
	if w[0] != "SEQ STOP\r\n" {
		t.Fatalf("下发内容应为 SEQ STOP，实际 %q", w[0])
	}
	mu.Lock()
	ce := commErr
	mu.Unlock()
	if ce {
		t.Fatalf("收到 OK IRSEQ idle 应答后不应判通讯失败")
	}

	s.Close()
}

// TestRealHardwareAck 在真实下位机（REAL_COM=COM4）上验证 web 侧命令-应答门控：
//   1) 离散指令能下发并被固件应答（收到 OK/ERR 行）；
//   2) `SEQ STOP` 在序列空闲时回送 `OK IRSEQ idle`，门控不误判通讯失败；
//   3) 连续下发大量 JOY 摇杆指令时，门控保持“一条在途、等待应答”，通讯状态正常；
//   4) 异步事件（"# " 前缀，如硬件 IR 回显）不会误清“等待应答”。
// 该测试仅在设置了 REAL_COM 环境变量时运行，避免 CI / 无硬件环境误触。
func TestRealHardwareAck(t *testing.T) {
	port := os.Getenv("REAL_COM")
	if port == "" {
		t.Skip("未设置 REAL_COM，跳过真实硬件测试（用法: REAL_COM=COM4 go test -run TestRealHardwareAck）")
	}
	var mu sync.Mutex
	var commErr bool
	var lines []string
	s := Open(Config{Port: port, Baud: 9600, ReconnectSec: 1, MinIntervalMs: 10, AckTimeoutMs: 800})
	s.SetStatusHandler(func(_ bool, _ string, ce bool, _ string) {
		mu.Lock()
		commErr = ce
		mu.Unlock()
	})
	s.SetLineHandler(func(line string) {
		mu.Lock()
		lines = append(lines, line)
		mu.Unlock()
	})
	if !waitConnected(s, 3*time.Second) {
		t.Fatalf("无法连接真实串口 %s", port)
	}
	// Uno 在打开串口时会因 DTR 边沿自动复位进入 optiboot 引导程序，
	// 需等待引导程序超时交出控制权、固件真正运行（与 host_verify.py 一致）。
	time.Sleep(2500 * time.Millisecond)

	got := func(sub string) bool {
		mu.Lock()
		defer mu.Unlock()
		for _, l := range lines {
			if strings.Contains(l, sub) {
				return true
			}
		}
		return false
	}
	commFailed := func() bool {
		mu.Lock()
		defer mu.Unlock()
		return commErr
	}

	mu.Lock()
	t.Logf("开机回显样例(前6): %v", lines[:min(6, len(lines))])
	mu.Unlock()

	// 1) 离散指令 RESET -> 固件应答 OK RESET（必要时重发一次，规避引导窗口吞首条）
	if err := s.WriteLine("RESET"); err != nil {
		t.Fatalf("WriteLine RESET 失败: %v", err)
	}
	time.Sleep(500 * time.Millisecond)
	if !got("OK RESET") {
		_ = s.WriteLine("RESET")
		time.Sleep(600 * time.Millisecond)
	}
	if !got("OK RESET") {
		mu.Lock()
		t.Fatalf("未收到固件对 RESET 的应答（门控可能未等待应答）；已捕获回显: %v", lines)
		mu.Unlock()
	}

	// 2) SEQ STOP 空闲 -> OK IRSEQ idle，不应判通讯失败
	if err := s.WriteLine("SEQ STOP"); err != nil {
		t.Fatalf("WriteLine SEQ STOP 失败: %v", err)
	}
	time.Sleep(400 * time.Millisecond)
	if !got("OK IRSEQ idle") {
		t.Fatalf("未收到固件对空闲 SEQ STOP 的应答 OK IRSEQ idle（适配失效）")
	}
	if commFailed() {
		t.Fatalf("空闲 SEQ STOP 误判为通讯失败")
	}

	// 3) 连续下发 20 条 JOY 摇杆指令（门控应串行化 + 合并中间值），通讯保持正常
	for i := 0; i < 20; i++ {
		_ = s.WriteLine(fmt.Sprintf("JOY %d 512 512 512", 100+i*20))
	}
	time.Sleep(800 * time.Millisecond)
	if !got("OK JOY") {
		t.Fatalf("未收到任何 JOY 应答，门控/固件链路异常")
	}
	if commFailed() {
		t.Fatalf("JOY 突发下发期间误判通讯失败")
	}

	mu.Lock()
	t.Logf("真实硬件回显样例(前8): %v", lines[:min(8, len(lines))])
	mu.Unlock()

	s.Close()
}

func TestAckTimeoutCommFailure(t *testing.T) {
	fake := newFakeSerial()
	dialSerial = func(Config) (io.ReadWriteCloser, error) { return fake, nil }
	defer func() { dialSerial = openPort }()

	var mu sync.Mutex
	var commErr bool
	var commMsg string
	s := Open(Config{Port: "FAKE", Baud: 9600, ReconnectSec: 1, MinIntervalMs: 0, AckTimeoutMs: 150})
	s.SetStatusHandler(func(_ bool, _ string, ce bool, msg string) {
		mu.Lock()
		commErr = ce
		commMsg = msg
		mu.Unlock()
	})
	if !waitConnected(s, time.Second) {
		t.Fatalf("串口未连接")
	}

	// 下发一条指令，但设备永远不回送 ACK
	if err := s.WriteLine("STATUS"); err != nil {
		t.Fatalf("WriteLine 失败: %v", err)
	}
	// 等待超过应答超时
	time.Sleep(400 * time.Millisecond)

	mu.Lock()
	ce, cm := commErr, commMsg
	mu.Unlock()
	t.Logf("commErr=%v msg=%q written=%v", ce, cm, fake.getWritten())
	if !ce {
		t.Fatalf("无应答应触发通讯失败")
	}
	if cm == "" {
		t.Fatalf("通讯失败应带有原因文本")
	}

	s.Close()
}
