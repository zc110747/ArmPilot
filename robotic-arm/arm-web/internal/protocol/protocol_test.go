package protocol

import "testing"

func TestValidate(t *testing.T) {
	cases := []struct {
		in      string
		wantOK  bool
		wantOut string
	}{
		{"SET 9 120", true, "SET 9 120"},
		{"SET 9 120 8 90 7 100", true, "SET 9 120 8 90 7 100"},
		{"SET 9 120 8 90 7 100 6 50", false, ""}, // >3
		{"S7=90", true, "S7=90"},
		{"S5=90", false, ""}, // bad id
		{"STOP 8", true, "STOP 8"},
		{"AUTO 9", true, "AUTO 9"},
		{"JOY 900 200 512 800", true, "JOY 900 200 512 800"},
		{"JOY 8 50", true, "JOY 8 50"},
		{"JOY 8 2000", false, ""}, // raw>1023
		{"IR 0xF708FF00", true, "IR 0xF708FF00"},
		{"IR DEADBEEF", true, "IR DEADBEEF"},
		{"SEQ 1", true, "SEQ 1"},
		{"SEQ STOP", true, "SEQ STOP"},
		{"SEQ 2", false, ""},
		{"JOYHW ON", true, "JOYHW ON"},
		{"JOYHW maybe", false, ""},
		{"RESET", true, "RESET"},
		{"STATUS", true, "STATUS"},
		{"?", true, "?"},
		{"ADC", true, "ADC"},
		{"HELP", true, "HELP"},
		{"garbage", false, ""},
	}
	for _, c := range cases {
		ok, out, err := Validate(c.in)
		if ok != c.wantOK {
			t.Errorf("Validate(%q) ok=%v want %v (err=%v)", c.in, ok, c.wantOK, err)
			continue
		}
		if ok && out != c.wantOut {
			t.Errorf("Validate(%q) out=%q want %q", c.in, out, c.wantOut)
		}
	}
}

func TestJoystickToJOY(t *testing.T) {
	m := DefaultAxisMap()
	// 中位 -> 全 512（设备死区，不动作）
	if got := JoystickToJOY(0, 0, m); got != "JOY 512 512 512 512" {
		t.Errorf("center: %q", got)
	}
	// 右满 -> raw9 接近 1023
	if got := JoystickToJOY(1, 0, m); got != "JOY 1023 512 512 512" {
		t.Errorf("right: %q", got)
	}
	// 左满 -> raw9 接近 0
	if got := JoystickToJOY(-1, 0, m); got != "JOY 0 512 512 512" {
		t.Errorf("left: %q", got)
	}
	// 前满 -> raw8 = 1023
	if got := JoystickToJOY(0, 1, m); got != "JOY 512 1023 512 512" {
		t.Errorf("forward: %q", got)
	}
	// 反相 左X
	m2 := AxisMap{LXServo: 9, LYServo: 8, InvLX: true, InvLY: false}
	if got := JoystickToJOY(1, 0, m2); got != "JOY 0 512 512 512" {
		t.Errorf("invLX right: %q", got)
	}
}

func TestJoyCurve(t *testing.T) {
	// 死区内 -> 居中
	if got := joyCurve(0); got != 0 {
		t.Errorf("center: %v", got)
	}
	if got := joyCurve(0.05); got != 0 {
		t.Errorf("dead edge: %v", got)
	}
	// 满偏及以上 -> ±1
	if got := joyCurve(0.5); got != 1 {
		t.Errorf("full: %v", got)
	}
	if got := joyCurve(1); got != 1 {
		t.Errorf("max: %v", got)
	}
	if got := joyCurve(-0.8); got != -1 {
		t.Errorf("neg full: %v", got)
	}
	// 单调放大：0.3 行程应产生明显大于线性映射的等效偏移
	mid := joyCurve(0.3)
	if mid <= 0.3 {
		t.Errorf("curve should amplify: %v", mid)
	}
	// 反对称性
	if joyCurve(-0.3) != -mid {
		t.Errorf("odd symmetry broken: %v vs %v", joyCurve(-0.3), mid)
	}
}

func TestJoystickToJOYDualCurve(t *testing.T) {
	m := DefaultAxisMap()
	// 曲线下 0.5 行程即满偏 raw=1023（线性映射时只有 768，够不到 800 阈值）
	if got := JoystickToJOY(0.5, 0, m); got != "JOY 1023 512 512 512" {
		t.Errorf("curve mid travel: %q", got)
	}
	// invert 后推 + 方向应对应固件 +步长侧（raw<200）
	mi := AxisMap{LXServo: 9, LYServo: 8, RXServo: 6, RYServo: 7, InvLX: true, InvRX: true, InvRY: true}
	if got := JoystickToJOYDual(0.8, 0.8, 0.8, 0.8, mi); got != "JOY 0 1023 0 0" {
		t.Errorf("inverted dual: %q", got)
	}
}
