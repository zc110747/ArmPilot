package robot

import (
	"math"
	"path/filepath"
	"testing"
)

// robotYAML 取**选择器声明的** MeArm-V1 的 `robot.yaml` 绝对路径。
//
// 刻意不写相对路径常量：Phase 2 把真值搬进了 `robot-package/<id>/model/`，
// 而"散落在各处自己拼路径的读者"正是那次搬迁最容易漏改的地方 ——
// 漏改这一处**不会**报"文件缺失"，它会指向另一个**仍然存在**的同名文件。
// 从选择器出发 ⇒ 连"仓库根在第几层"都不用猜。
func robotYAML(t *testing.T) string {
	t.Helper()
	sel, err := LoadSelector(selectorYAML(t))
	if err != nil {
		t.Fatalf("LoadSelector 失败: %v", err)
	}
	entry, ok := sel.Robots["mearm-v1"]
	if !ok {
		t.Fatalf("选择器里没有 mearm-v1（Golden Baseline 必须可选）")
	}
	return entry.ConfigPath
}

func loadModel(t *testing.T) *Model {
	t.Helper()
	m, err := Load(robotYAML(t))
	if err != nil {
		t.Fatalf("Load 失败: %v", err)
	}
	return m
}

func TestLoadModel(t *testing.T) {
	m := loadModel(t)
	if m.ID != "mearm" {
		t.Errorf("robot.id = %q, 期望 mearm", m.ID)
	}
	if m.Name != "mARM" {
		t.Errorf("robot.name = %q, 期望 mARM", m.Name)
	}
}

// 关节顺序决定 JR 四元组的位次，必须与前端 movableJoints 一致。
func TestJointOrder(t *testing.T) {
	m := loadModel(t)
	got := m.JointOrder()
	want := []string{"base", "shoulder", "elbow", "gripper"}
	if len(got) != len(want) {
		t.Fatalf("关节数 = %d, 期望 %d（%v）", len(got), len(want), got)
	}
	for i := range want {
		if got[i] != want[i] {
			t.Errorf("第 %d 位 = %q, 期望 %q", i, got[i], want[i])
		}
	}
	// tool 是**被动腕**（passive）：角度由 coupling 从 elbow 派生，没有独立输入，
	// 必须被跳过。这条同时守住"判据不是 `!= fixed`"——
	// 若有人把它写成"非 fixed 即入列"，JR 会变成五元组而所有位次一起错位。
	if j := m.Joint("tool"); j == nil {
		t.Fatal("robot.yaml 里找不到 tool 关节 —— 机构已不是这个模型")
	} else if j.Type != "passive" {
		t.Errorf("tool.Type = %q，期望 passive（爪被平行四连杆锁成水平）", j.Type)
	}
	for _, id := range got {
		if id == "tool" {
			t.Error("被动关节 tool 不应出现在 JointOrder 中")
		}
	}
}

// 标定往返：关节角 → 舵机角 → 关节角 必须回到原值。
// 这里同时验证"舵机 90° 恰好对应 HOME 位"这一硬件自洽性。
func TestCalibrationRoundTrip(t *testing.T) {
	m := loadModel(t)
	probes := []struct {
		joint string
		theta float64
		servo float64
	}{
		{"base", 0, 90},
		{"shoulder", 0.8498937633, 90},
		{"elbow", 112.6185771989, 90},
		{"gripper", 50, 90},
	}
	for _, p := range probes {
		acts := m.ActuatorsForJoint(p.joint)
		if len(acts) != 1 {
			t.Fatalf("关节 %s 的执行器数 = %d, 期望 1", p.joint, len(acts))
		}
		a := acts[0]
		gotServo := JointToServo(a, p.theta)
		if math.Abs(gotServo-p.servo) > 1e-3 {
			t.Errorf("%s: 关节 %.6f° → 舵机 %.4f°, 期望 %.2f°", p.joint, p.theta, gotServo, p.servo)
		}
		gotTheta := ServoToJoint(a, p.servo)
		if math.Abs(gotTheta-p.theta) > 1e-9 {
			t.Errorf("%s: 舵机 %.2f° → 关节 %.6f°, 期望 %.6f°", p.joint, p.servo, gotTheta, p.theta)
		}
	}
}

// 限位反算出的舵机区间必须落在固件硬限位之内（超了说明标定表与固件不一致）。
func TestServoRangeInsideFirmwareLimits(t *testing.T) {
	m := loadModel(t)
	for i := range m.Actuators {
		a := &m.Actuators[i]
		j := m.Joint(a.JointID)
		if j == nil {
			t.Fatalf("执行器 %s 指向未知关节 %s", a.ID, a.JointID)
		}
		lo, hi := ServoRangeForLimits(a, j.Limit)
		if lo < a.Limits.Min-1.0 || hi > a.Limits.Max+1.0 {
			t.Errorf("%s: 关节限位映射的舵机区间 %.2f..%.2f 超出固件硬限位 %.2f..%.2f",
				a.ID, lo, hi, a.Limits.Min, a.Limits.Max)
		}
	}
}

func TestValidate(t *testing.T) {
	m := loadModel(t)

	// HOME 位必须合法
	if v := m.Validate(m.HomePose); v != nil {
		t.Errorf("HOME 位不应越界，却得到 %v", v)
	}

	cases := []struct {
		name   string
		joints map[string]float64
		want   string
	}{
		{"肘角取 95（真机不可达区间）", map[string]float64{"elbow": 95}, "ERR JOINT elbow 95.00 (limit 108.44..141.86)"},
		{"肩角超出上限", map[string]float64{"shoulder": 60}, "ERR JOINT shoulder 60.00 (limit -6.09..49.45)"},
		{"底座超出下限", map[string]float64{"base": -90}, "ERR JOINT base -90.00 (limit -60.00..60.00)"},
		{"夹爪超上限", map[string]float64{"gripper": 120}, "ERR JOINT gripper 120.00 (limit 0.00..90.00)"},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			v := m.Validate(c.joints)
			if v == nil {
				t.Fatalf("%s 期望越界，却通过了校验", c.name)
			}
			if v.Error() != c.want {
				t.Errorf("错误文案 = %q\n期望 = %q", v.Error(), c.want)
			}
		})
	}

	// 边界值必须放行（限位是闭区间）
	edge := map[string]float64{
		"base": 60, "shoulder": 49.454929245, "elbow": 108.4414852068, "gripper": 0,
	}
	if v := m.Validate(edge); v != nil {
		t.Errorf("边界值应放行，却得到 %v", v)
	}
}

// map 迭代顺序随机，但错误文案必须稳定 —— 按 JointOrder 取第一个越界项。
func TestValidateOrderStable(t *testing.T) {
	m := loadModel(t)
	all := map[string]float64{"base": 999, "shoulder": 999, "elbow": 999, "gripper": 999}
	for i := 0; i < 50; i++ {
		v := m.Validate(all)
		if v == nil {
			t.Fatal("期望越界")
		}
		if v.JointID != "base" {
			t.Fatalf("第 %d 次迭代首个越界关节 = %q, 期望 base（文案必须稳定）", i, v.JointID)
		}
	}
}

func TestCalibrationTableShape(t *testing.T) {
	m := loadModel(t)
	rows := m.CalibrationTable()
	if len(rows) != 4 {
		t.Fatalf("标定表行数 = %d, 期望 4", len(rows))
	}
	// 必须按关节顺序排列，且 channel 与 protocol/serial-v1.md §2 表一致
	wantCh := map[string]int{"base": 9, "shoulder": 7, "elbow": 8, "gripper": 6}
	for i, r := range rows {
		if r.Channel != wantCh[r.JointID] {
			t.Errorf("第 %d 行 %s 的 channel = %d, 期望 %d", i, r.JointID, r.Channel, wantCh[r.JointID])
		}
	}
	if rows[0].JointID != "base" || rows[3].JointID != "gripper" {
		t.Errorf("标定表顺序异常: %v", rows)
	}
}

func TestLoadRejectsMissingFile(t *testing.T) {
	if _, err := Load(filepath.Join("..", "..", "..", "config", "definitely-not-here.yaml")); err == nil {
		t.Error("读取不存在的文件应当报错")
	}
}
