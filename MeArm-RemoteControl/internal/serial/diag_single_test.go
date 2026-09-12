package serial

import (
	"os"
	"strings"
	"testing"
	"time"
)

// TestRealComSingleDelayProbe 在固定延迟后仅发送一次 RESET\r\n，验证“单条指令”在不同开机
// 延迟下的应答情况，定位 bootloader 静默窗口的真实长度与抖动。仅在 REAL_COM 设置时运行。
func TestRealComSingleDelayProbe(t *testing.T) {
	port := os.Getenv("REAL_COM")
	if port == "" {
		t.Skip("未设置 REAL_COM，跳过")
	}
	for _, delay := range []time.Duration{2 * time.Second, 3 * time.Second, 4 * time.Second, 5 * time.Second} {
		conn, err := openPort(Config{Port: port, Baud: 115200, DataBits: 8, StopBits: 1, Parity: "N"})
		if err != nil {
			t.Fatalf("openPort 失败: %v", err)
		}
		time.Sleep(delay)
		_, _ = conn.Write([]byte("RESET\r\n"))
		got := false
		deadline := time.Now().Add(1500 * time.Millisecond)
		var acc strings.Builder
		for time.Now().Before(deadline) {
			buf := make([]byte, 256)
			n, rerr := conn.Read(buf)
			if n > 0 {
				acc.Write(buf[:n])
				if strings.Contains(acc.String(), "OK RESET") {
					got = true
					break
				}
			}
			if rerr != nil {
				if te, ok := rerr.(interface{ Timeout() bool }); ok && te.Timeout() {
					continue
				}
				break
			}
		}
		conn.Close()
		t.Logf("delay=%4dms -> OK RESET=%v  (rx=%q)", delay.Milliseconds(), got, acc.String())
		// 给下次连接留足复位间隔
		time.Sleep(3500 * time.Millisecond)
	}
}
