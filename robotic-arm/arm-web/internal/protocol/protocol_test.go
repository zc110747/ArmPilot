package protocol

import (
	"strconv"
	"strings"
	"testing"
)

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

func TestDeadFracFromDeg(t *testing.T) {
	if got := DeadFracFromDeg(0); got != 0 {
		t.Errorf("0 deg should disable: %v", got)
	}
	if got := DeadFracFromDeg(-3); got != 0 {
		t.Errorf("negative should disable: %v", got)
	}
	if got := DeadFracFromDeg(10); got < 0.31 || got > 0.33 {
		t.Errorf("10 deg of 31.5 deg full tilt: %v", got)
	}
	if got := DeadFracFromDeg(100); got != 0.9 {
		t.Errorf("clamped to 0.9: %v", got)
	}
}

// joyRawOf 从 "JOY a b c d" 中取第 idx 个 raw。
func joyRawOf(t *testing.T, cmd string, idx int) int {
	t.Helper()
	fields := strings.Fields(cmd)
	if len(fields) != 5 {
		t.Fatalf("bad JOY cmd: %q", cmd)
	}
	v, err := strconv.Atoi(fields[1+idx])
	if err != nil {
		t.Fatalf("bad raw in %q: %v", cmd, err)
	}
	return v
}

func TestDeadband(t *testing.T) {
	m := DefaultAxisMap() // DeadFrac = 10°/31.5° ≈ 0.317

	// 死区内（≤10° 视觉倾角，≈32% 行程）-> raw 512（固件不动作）
	if got := JoystickToJOY(0.3, -0.2, m); got != "JOY 512 512 512 512" {
		t.Errorf("inside deadband should be centered: %q", got)
	}
	// 四轴全居中 -> 无命令，调用方应整帧跳过下发
	if JoystickHasCommand(0.3, 0.2, -0.1, 0, m) {
		t.Errorf("all axes inside deadband: expect no command")
	}
	// 任一轴超出死区 -> 有命令
	if !JoystickHasCommand(0.3, 0.2, -0.1, 0.4, m) {
		t.Errorf("axis beyond deadband: expect command")
	}

	// 刚出死区即进入固件命令区间：raw>800（正向）
	if r := joyRawOf(t, JoystickToJOY(0.35, 0, m), 0); r <= 800 {
		t.Errorf("just beyond deadband should command (raw>800): %d", r)
	}
	// 负向对称：raw<200
	if r := joyRawOf(t, JoystickToJOY(-0.35, 0, m), 0); r >= 200 {
		t.Errorf("just beyond deadband (neg) should command (raw<200): %d", r)
	}
	// 满偏 -> raw 1023 / 0（固件最大步长）
	if got := JoystickToJOY(1, -1, m); got != "JOY 1023 0 512 512" {
		t.Errorf("full travel: %q", got)
	}
	// 行程越大步长越大（单调）
	r35 := joyRawOf(t, JoystickToJOY(0.35, 0, m), 0)
	r80 := joyRawOf(t, JoystickToJOY(0.8, 0, m), 0)
	if !(r35 > 800 && r80 > r35 && r80 < 1023) {
		t.Errorf("monotonic response: at0.35=%d at0.8=%d", r35, r80)
	}
}

func TestDeadbandInverted(t *testing.T) {
	// 与 config.yaml 默认一致的镜像：9/6/7 轴 invert
	mi := AxisMap{LXServo: 9, LYServo: 8, RXServo: 6, RYServo: 7,
		InvLX: true, InvRX: true, InvRY: true, DeadFrac: DefaultDeadFrac()}
	// 推 + 满偏：invert 后 raw=0，对应固件 + 步长侧
	if got := JoystickToJOYDual(1, 1, 1, 1, mi); got != "JOY 0 1023 0 0" {
		t.Errorf("inverted full dual: %q", got)
	}
	// 死区内：invert 不影响居中值 512
	if got := JoystickToJOYDual(0.2, -0.2, 0.3, 0.1, mi); got != "JOY 512 512 512 512" {
		t.Errorf("inverted inside deadband: %q", got)
	}
}
