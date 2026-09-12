// Package robot 解析 config/robot.yaml —— ArmPilot 的**唯一模型/标定真值**。
//
// 后端绝不自己存一份关节限位或标定表。原因见 protocol/serial-v1.md §2：
// 前端、固件、后端各存一份，三份必然漂移；实测已经证明"按固件命名推定角色"
// 会得出反着动的机械臂（S7=肩 / S8=肘 是靠相机实测纠正的）。
//
// 因此本包只做一件事：把 yaml 读进来，暴露
//
//	JointToServo / ServoToJoint   标定换算（与前端 calibration 层公式一致）
//	Validate                      关节限位校验（错误文案对齐固件 ERR JOINT ...）
//	JointOrder                    关节顺序（JR 四元组的位次）
//
// 若某天 yaml 改了，后端不需要改一行代码。
package robot

import (
	"fmt"
	"os"
	"sort"
	"strings"

	"gopkg.in/yaml.v3"
)

// Coupling 关节耦合：本关节的实际输出 = 自身角 + gain × 父关节角。
//
// 小臂（elbow）在真机上由**独立舵机经平行四连杆**驱动，存的是"离开天顶的绝对倾角"，
// 与肩角解耦；模型用 gain = -1 表达（串联网里的局部旋转 = θ_肘 + (-1)·θ_肩）。
// 注意：**JR 协议层传的是绝对角**，耦合只在 FK 里起作用，不在本包的标定里。
type Coupling struct {
	Joint string  `yaml:"joint"`
	Gain  float64 `yaml:"gain"`
}

// Joint 运动关节（含固定关节；固定关节不参与控制）。
type Joint struct {
	ID       string    `yaml:"id"`
	Name     string    `yaml:"name"`
	Role     string    `yaml:"role"`
	Type     string    `yaml:"type"`
	Limit    Limit     `yaml:"limit"`
	Coupling *Coupling `yaml:"coupling"`
}

// Limit 关节角软限位（degree）。
type Limit struct {
	Min float64 `yaml:"min"`
	Max float64 `yaml:"max"`
}

// Actuator 执行器：关节空间 → 舵机空间的映射，以及舵机（0..180°）硬件限位。
//
//	servo = reverse ? (-θ·scale + offset) : (θ·scale + offset)
type Actuator struct {
	ID      string  `yaml:"id"`
	Name    string  `yaml:"name"`
	JointID string  `yaml:"jointId"`
	Channel int     `yaml:"channel"`
	Offset  float64 `yaml:"offset"`
	Scale   float64 `yaml:"scale"`
	Reverse bool    `yaml:"reverse"`
	Limits  Limit   `yaml:"limits"`
}

// Model 从 robot.yaml 载入的模型视图（只保留控制所需字段）。
type Model struct {
	ID        string             `yaml:"-"`
	Name      string             `yaml:"-"`
	HomePose  map[string]float64 `yaml:"-"`
	Joints    []Joint            `yaml:"-"`
	Actuators []Actuator         `yaml:"-"`

	SourcePath string `yaml:"-"`

	byJoint map[string][]*Actuator
}

// yaml 文件结构（只取需要的部分）。
type yamlFile struct {
	Robot struct {
		ID       string             `yaml:"id"`
		Name     string             `yaml:"name"`
		HomePose map[string]float64 `yaml:"homePose"`
	} `yaml:"robot"`
	Joints    []Joint    `yaml:"joints"`
	Actuators []Actuator `yaml:"actuators"`
}

// Load 读取 robot.yaml 并建立关节 → 执行器索引。
func Load(path string) (*Model, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("读取 robot.yaml 失败: %w", err)
	}
	var f yamlFile
	if err := yaml.Unmarshal(raw, &f); err != nil {
		return nil, fmt.Errorf("解析 robot.yaml 失败: %w", err)
	}
	if len(f.Joints) == 0 {
		return nil, fmt.Errorf("robot.yaml 中没有 joints")
	}
	if len(f.Actuators) == 0 {
		return nil, fmt.Errorf("robot.yaml 中没有 actuators")
	}

	m := &Model{
		ID:         f.Robot.ID,
		Name:       f.Robot.Name,
		HomePose:   f.Robot.HomePose,
		Joints:     f.Joints,
		Actuators:  f.Actuators,
		SourcePath: path,
		byJoint:    make(map[string][]*Actuator),
	}
	for i := range m.Actuators {
		a := &m.Actuators[i]
		if a.Scale == 0 {
			return nil, fmt.Errorf("执行器 %s 的 scale 为 0（无法换算）", a.ID)
		}
		m.byJoint[a.JointID] = append(m.byJoint[a.JointID], a)
	}
	// 校验每个可动关节都有执行器：没有的话 JR 发不出去，属于配置错误
	for i := range m.Joints {
		j := &m.Joints[i]
		if j.Type == "fixed" {
			continue
		}
		if len(m.byJoint[j.ID]) == 0 {
			return nil, fmt.Errorf("关节 %s 没有对应的执行器（标定表不完整）", j.ID)
		}
	}
	return m, nil
}

// JointOrder 返回参与 JR 四元组的关节顺序（跳过 fixed 关节，如 tool）。
//
// ⚠️ 顺序必须与前端 `movableJoints(model)` 完全一致，否则 JR 的位次会错位。
// 前端按 robot.yaml 的 joints 数组顺序过滤 fixed，故此处同样按原始顺序过滤。
func (m *Model) JointOrder() []string {
	out := make([]string, 0, len(m.Joints))
	for i := range m.Joints {
		if m.Joints[i].Type == "fixed" {
			continue
		}
		out = append(out, m.Joints[i].ID)
	}
	return out
}

// Joint 按 id 取关节（返回副本指针，勿修改）。
func (m *Model) Joint(id string) *Joint {
	for i := range m.Joints {
		if m.Joints[i].ID == id {
			return &m.Joints[i]
		}
	}
	return nil
}

// ActuatorsForJoint 返回该关节的全部执行器（多舵机共同驱动时 > 1）。
func (m *Model) ActuatorsForJoint(jointID string) []*Actuator {
	return m.byJoint[jointID]
}

// JointToServo 关节角 → 舵机角。
func JointToServo(a *Actuator, thetaDeg float64) float64 {
	if a.Reverse {
		return -thetaDeg*a.Scale + a.Offset
	}
	return thetaDeg*a.Scale + a.Offset
}

// ServoToJoint 舵机角 → 关节角（上式的逆）。
func ServoToJoint(a *Actuator, servoDeg float64) float64 {
	if a.Reverse {
		return (a.Offset - servoDeg) / a.Scale
	}
	return (servoDeg - a.Offset) / a.Scale
}

// ServoRangeForLimits 关节限位映射后的舵机角区间（升序），用于展示与一致性核对。
func ServoRangeForLimits(a *Actuator, l Limit) (float64, float64) {
	p, q := JointToServo(a, l.Min), JointToServo(a, l.Max)
	if p > q {
		p, q = q, p
	}
	return p, q
}

// Violation 描述一次限位越界。
type Violation struct {
	JointID string
	Value   float64
	Min     float64
	Max     float64
}

// Error 生成与固件同格式的错误文案：`ERR JOINT elbow 95.00 (limit 108.44..141.86)`。
func (v Violation) Error() string {
	return fmt.Sprintf("ERR JOINT %s %.2f (limit %.2f..%.2f)", v.JointID, v.Value, v.Min, v.Max)
}

// Validate 按关节顺序逐个校验限位，返回第一个越界项；全部合法返回 nil。
//
// 顺序敏感：错误文案要稳定可预期（前端 UI 与 e2e 断言都依赖它），
// 因此按 JointOrder 而非 map 迭代（Go 的 map 迭代顺序是随机的）。
func (m *Model) Validate(joints map[string]float64) *Violation {
	for _, id := range m.JointOrder() {
		v, ok := joints[id]
		if !ok {
			continue
		}
		j := m.Joint(id)
		if j == nil {
			continue
		}
		if v < j.Limit.Min-1e-9 || v > j.Limit.Max+1e-9 {
			return &Violation{JointID: id, Value: v, Min: j.Limit.Min, Max: j.Limit.Max}
		}
	}
	return nil
}

// LimitTable 返回关节限位表（按 JointOrder），用于 hello 元数据与日志。
type LimitRow struct {
	ID   string  `json:"id"`
	Role string  `json:"role"`
	Min  float64 `json:"min"`
	Max  float64 `json:"max"`
}

func (m *Model) LimitTable() []LimitRow {
	rows := make([]LimitRow, 0, len(m.Joints))
	for _, id := range m.JointOrder() {
		j := m.Joint(id)
		if j == nil {
			continue
		}
		rows = append(rows, LimitRow{ID: j.ID, Role: j.Role, Min: j.Limit.Min, Max: j.Limit.Max})
	}
	return rows
}

// CalibrationRow 标定表的一行（供 hello 元数据）。
type CalibrationRow struct {
	JointID string  `json:"jointId"`
	ServoID string  `json:"servoId"`
	Channel int     `json:"channel"`
	Offset  float64 `json:"offset"`
	Scale   float64 `json:"scale"`
	Reverse bool    `json:"reverse"`
	ServoLo float64 `json:"servoLo"`
	ServoHi float64 `json:"servoHi"`
}

func (m *Model) CalibrationTable() []CalibrationRow {
	rows := make([]CalibrationRow, 0, len(m.Actuators))
	order := m.JointOrder()
	rank := make(map[string]int, len(order))
	for i, id := range order {
		rank[id] = i
	}
	sorted := make([]Actuator, len(m.Actuators))
	copy(sorted, m.Actuators)
	sort.SliceStable(sorted, func(i, j int) bool {
		return rank[sorted[i].JointID] < rank[sorted[j].JointID]
	})
	for i := range sorted {
		a := &sorted[i]
		j := m.Joint(a.JointID)
		if j == nil {
			continue
		}
		lo, hi := ServoRangeForLimits(a, j.Limit)
		rows = append(rows, CalibrationRow{
			JointID: a.JointID, ServoID: a.ID, Channel: a.Channel,
			Offset: a.Offset, Scale: a.Scale, Reverse: a.Reverse,
			ServoLo: lo, ServoHi: hi,
		})
	}
	return rows
}

// Describe 一行摘要（启动日志用）。
func (m *Model) Describe() string {
	order := m.JointOrder()
	parts := make([]string, 0, len(order))
	for _, id := range order {
		j := m.Joint(id)
		if j == nil {
			continue
		}
		acts := m.ActuatorsForJoint(id)
		ch := 0
		if len(acts) > 0 {
			ch = acts[0].Channel
		}
		parts = append(parts, fmt.Sprintf("%s(S%d %.2f..%.2f)", id, ch, j.Limit.Min, j.Limit.Max))
	}
	return strings.Join(parts, " ")
}
