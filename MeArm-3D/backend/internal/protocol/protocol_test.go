package protocol

import (
	"math"
	"path/filepath"
	"testing"

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

func TestEncodeJR(t *testing.T) {
	order := []string{"base", "shoulder", "elbow", "gripper"}
	cases := []struct {
		name   string
		joints map[string]float64
		want   string
	}{
		{"HOME 位", map[string]float64{"base": 0, "shoulder": 0.8498937633, "elbow": 112.6185771989, "gripper": 50}, "JR 0.0 0.8 112.6 50.0"},
		{"整数", map[string]float64{"base": 0, "shoulder": 30, "elbow": 120, "gripper": 50}, "JR 0.0 30.0 120.0 50.0"},
		{"负数", map[string]float64{"base": -60, "shoulder": -6.0936827341, "elbow": 108.4414852068, "gripper": 0}, "JR -60.0 -6.1 108.4 0.0"},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			if got := EncodeJR(order, c.joints); got != c.want {
				t.Errorf("EncodeJR = %q, 期望 %q", got, c.want)
			}
		})
	}
}

func TestEncodeOKJR(t *testing.T) {
	got := EncodeOKJR(map[int]float64{9: 90, 7: 131.98, 8: 72.33, 6: 90})
	want := "OK JR S9=90.00 S8=72.33 S7=131.98 S6=90.00"
	if got != want {
		t.Errorf("EncodeOKJR = %q, 期望 %q", got, want)
	}
}

func TestParseReplyOKJR(t *testing.T) {
	r := ParseReply("OK JR S9=90.00 S7=131.98 S8=72.33 S6=90.00")
	if r.Kind != ReplyOKJR {
		t.Fatalf("Kind = %v, 期望 OK_JR", r.Kind)
	}
	if r.ServoAngles[9] != 90 || r.ServoAngles[7] != 131.98 || r.ServoAngles[8] != 72.33 || r.ServoAngles[6] != 90 {
		t.Errorf("舵机角解析错误: %v", r.ServoAngles)
	}
}

func TestParseReplyError(t *testing.T) {
	line := "ERR JOINT elbow 95.00 (limit 108.44..141.86)"
	r := ParseReply(line)
	if r.Kind != ReplyError {
		t.Fatalf("Kind = %v, 期望 ERR", r.Kind)
	}
	if r.ErrText != line {
		t.Errorf("ErrText = %q", r.ErrText)
	}
	code, msg := ParseErrorCode(r.ErrText)
	if code != CodeJointLimit {
		t.Errorf("错误码 = %q, 期望 %q", code, CodeJointLimit)
	}
	if msg != line {
		t.Errorf("message = %q", msg)
	}
}

// ERR 行里没有 S<n>=，但如果将来固件为了便于核对而带上舵机角，
// 也必须仍然判为 ERR 而不是 OK JR —— 这个用例锁住"先判 ERR"的顺序。
func TestParseReplyErrorWithServoEcho(t *testing.T) {
	r := ParseReply("ERR JOINT elbow 95.00 (limit 108.44..141.86) S9=90.00")
	if r.Kind != ReplyError {
		t.Fatalf("Kind = %v, 期望 ERR（ERR 判定必须先于 OK JR）", r.Kind)
	}
}

func TestParseReplyState(t *testing.T) {
	r := ParseReply("STATE 0 0.85 112.62 50.00")
	if r.Kind != ReplyState {
		t.Fatalf("Kind = %v, 期望 STATE", r.Kind)
	}
	want := []float64{0, 0.85, 112.62, 50}
	if len(r.Joints) != len(want) {
		t.Fatalf("数值个数 = %d, 期望 %d", len(r.Joints), len(want))
	}
	for i := range want {
		if math.Abs(r.Joints[i]-want[i]) > 1e-9 {
			t.Errorf("第 %d 个 = %v, 期望 %v", i, r.Joints[i], want[i])
		}
	}
}

func TestParseReplyOther(t *testing.T) {
	for _, line := range []string{"# IR RAW=...", "OK SET S9=120", "", "  ", "PONG", "OK JOY S6=89 S7=89"} {
		r := ParseReply(line)
		if r.Kind != ReplyOther {
			t.Errorf("行 %q 应判为 OTHER, 却得到 %v", line, r.Kind)
		}
	}
}

// 标定可逆性：舵机角 → 关节角 → 舵机角 必须回到 90°（HOME）。
func TestServoAnglesToJoints(t *testing.T) {
	m := loadModel(t)
	joints := ServoAnglesToJoints(m, map[int]float64{9: 90, 7: 90, 8: 90, 6: 90})
	for id, want := range m.HomePose {
		got, ok := joints[id]
		if !ok {
			t.Errorf("关节 %s 未出现在反算结果中", id)
			continue
		}
		if math.Abs(got-want) > 1e-9 {
			t.Errorf("%s = %.9f, 期望 %.9f", id, got, want)
		}
	}
}

// 若舵机角偏离 90°，反算出的关节角必须相应偏离 HOME —— 证明不是恒等回显。
func TestServoAnglesToJointsIsNotIdentity(t *testing.T) {
	m := loadModel(t)
	joints := ServoAnglesToJoints(m, map[int]float64{9: 90, 7: 120, 8: 90, 6: 90})
	got := joints["shoulder"]
	if math.Abs(got-m.HomePose["shoulder"]) < 1 {
		t.Errorf("S7 转了 30°，肩关节只动了 %.4f° —— 标定换算疑似退化为恒等", got)
	}
	// S7 标定：θ = (servo − 88.776) / 1.44018；舵机 120° → 21.6806°
	if want := (120 - 88.776) / 1.44018; math.Abs(got-want) > 1e-9 {
		t.Errorf("肩关节 = %.6f, 期望 %.6f", got, want)
	}
}

func TestBuildModelInfo(t *testing.T) {
	m := loadModel(t)
	info := BuildModelInfo(m)
	if info.ID != "mearm" {
		t.Errorf("ID = %q", info.ID)
	}
	if len(info.JointOrder) != 4 {
		t.Errorf("JointOrder = %v", info.JointOrder)
	}
	if len(info.Limits) != 4 || len(info.Calibration) != 4 {
		t.Errorf("Limits=%d Calibration=%d, 期望各 4", len(info.Limits), len(info.Calibration))
	}
	if _, ok := info.HomePose["elbow"]; !ok {
		t.Error("HomePose 缺少 elbow")
	}
}
