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
