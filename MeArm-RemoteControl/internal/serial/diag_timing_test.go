package serial

import (
	"os"
	"strings"
	"testing"
	"time"
)

// TestRealComTimingProbe 探测固件在打开串口后多久才开始响应指令：
// 每 500ms 发送一次 RESET\r\n，记录每次是否拿到 OK RESET，定位“开机静默窗口”。
// 仅在 REAL_COM 设置时运行。
func TestRealComTimingProbe(t *testing.T) {
	port := os.Getenv("REAL_COM")
	if port == "" {
		t.Skip("未设置 REAL_COM，跳过")
	}
	conn, err := openPort(Config{Port: port, Baud: 115200, DataBits: 8, StopBits: 1, Parity: "N"})
	if err != nil {
		t.Fatalf("openPort 失败: %v", err)
	}
	defer conn.Close()
	t.Logf("已打开 %s @ 115200，开始每 500ms 探测 RESET 应答...", port)

	start := time.Now()
	for i := 0; i < 12; i++ {
		// 发送 RESET\r\n
		_, _ = conn.Write([]byte("RESET\r\n"))
		// 在 500ms 内读取是否出现 OK RESET
		got := false
		deadline := time.Now().Add(500 * time.Millisecond)
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
		dt := time.Since(start)
		t.Logf("probe#%02d t=+%5.2fs -> OK RESET=%v", i, dt.Seconds(), got)
		if got {
			t.Logf(">>> 固件在 +%5.2fs 首次正确应答 RESET", dt.Seconds())
			return
		}
		// 若本次未拿到，等到下一个 500ms 边界再发
		remain := 500*time.Millisecond - time.Since(start)%500*time.Millisecond
		if remain < 0 {
			remain = 0
		}
		time.Sleep(remain)
	}
	t.Logf(">>> 6s 内固件始终未对 RESET 应答（疑似静默窗口过长或 TX 链路问题）")
}
