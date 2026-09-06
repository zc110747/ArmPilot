package serial

import (
	"os"
	"strings"
	"testing"
	"time"
)

// TestRealComFirstCmdLost 验证“开机后首条指令被丢弃”的猜想：开机等待 3s 后连续发送两条
// RESET，观察第 1 条 vs 第 2 条 的应答差异，并捕获任何 ERR 回显。仅在 REAL_COM 设置时运行。
func TestRealComFirstCmdLost(t *testing.T) {
	port := os.Getenv("REAL_COM")
	if port == "" {
		t.Skip("未设置 REAL_COM，跳过")
	}
	conn, err := openPort(Config{Port: port, Baud: 115200, DataBits: 8, StopBits: 1, Parity: "N"})
	if err != nil {
		t.Fatalf("openPort 失败: %v", err)
	}
	defer conn.Close()
	time.Sleep(3000 * time.Millisecond)

	drain := func(label string) {
		deadline := time.Now().Add(1500 * time.Millisecond)
		var acc strings.Builder
		for time.Now().Before(deadline) {
			buf := make([]byte, 256)
			n, rerr := conn.Read(buf)
			if n > 0 {
				acc.Write(buf[:n])
			}
			if rerr != nil {
				if te, ok := rerr.(interface{ Timeout() bool }); ok && te.Timeout() {
					continue
				}
				break
			}
		}
		t.Logf("[%s] rx=%q", label, acc.String())
	}

	conn.Write([]byte("RESET\r\n"))
	drain("after-1st-RESET")
	conn.Write([]byte("RESET\r\n"))
	drain("after-2nd-RESET")
}
