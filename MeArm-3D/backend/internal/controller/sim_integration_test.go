package controller

import (
	"testing"
	"time"

	"armpilot/backend/internal/device"
)

// 真 sim 设备 + 真控制器的集成测试（不经 WebSocket）。
// 覆盖"命令 → ACK 门控 → sim 受理 → 有限角速度推进 → STATE 反算 → 状态事件"全链。
func TestWithSimDevice(t *testing.T) {
	m := loadModel(t)
	dev, err := device.NewSim(m, device.DefaultSimTuning())
	if err != nil {
		t.Fatalf("NewSim: %v", err)
	}
	ctl := New(m, dev, Config{AckTimeoutMs: 800, EchoJointState: true})
	got := make(chan map[string]float64, 200)
	ctl.OnJointState(func(j map[string]float64, _ time.Time) { got <- j })
	ctl.OnError(func(code, msg string) { t.Logf("ERR %s: %s", code, msg) })
	ctl.Start()
	defer ctl.Close()

	if err := ctl.Apply(map[string]float64{"shoulder": 20.8}); err != nil {
		t.Fatalf("Apply: %v", err)
	}

	n := 0
	sawIntermediate := false
	last := 0.0
	for {
		select {
		case j := <-got:
			n++
			last = j["shoulder"]
			t.Logf("state#%d shoulder=%.4f elbow=%.4f", n, last, j["elbow"])
			if last > 5 && last < 20 {
				sawIntermediate = true
			}
			if last >= 20.7 && last <= 20.9 {
				if !sawIntermediate {
					t.Error("从未出现中间态 —— 说明不是有限角速度逼近（疑似等值回显）")
				}
				// 肘角应保持在 HOME 附近（命令里是 112.6，量化后 112.6）
				if e := j["elbow"]; e < 112.5 || e > 112.7 {
					t.Errorf("肘角 = %.4f, 期望 ≈112.6", e)
				}
				return
			}
		case <-time.After(3 * time.Second):
			t.Fatalf("3s 内未收敛，共 %d 个状态，最后 shoulder=%.4f", n, last)
		}
	}
}
