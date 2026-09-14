// Package robot —— 模型**选择器**（`config/robots.yaml`）。
//
// # 它回答的问题
//
// 「当前该加载哪台机器人」。三端读的是**同一份文件**：
//
//	frontend → src/robot/model/robotConfigRegistry.ts（构建期静态登记 + 本文件选路径）
//	backend  → 本文件
//	python   → simulation/mujoco/robotcfg.py:load_robot_selector()
//
// # 铁律：选择器只放「指针」，不放「数值」
//
// 允许：文件路径、MJCF 里 site 的名字（= 哪一份文件属于这台机器人）。
// 禁止：尺寸 / 关节 / 限位 / 标定 / 物理量 / TCP 偏移。
//
// 把参数抄进选择器，就等于制造第二份真值 —— 与把限位抄进 `backend/config.yaml`
// 是同一类错误，而它的表现是"改了一处、另一处还是旧值，且没有任何报错"。
//
// # 未知 id 一律报错，绝不回退
//
// 回退会让"选择器里写错一个字母"表现为**静默加载了另一台机器人** ——
// 本包注释里反复出现的那类最贵的错误（模型不对，所有限位/标定判据全部失真却都能跑）。
package robot

import (
	"fmt"
	"os"
	"path/filepath"
	"sort"

	"gopkg.in/yaml.v3"
)

// Entry 选择器里的一条记录 —— 一台机器人涉及的全部文件。
//
// 路径在加载时已解析为**绝对路径**（相对选择器所在仓库的根），
// 这样调用方不必知道"仓库根在哪"这件事，也就不会有第三处去猜。
type Entry struct {
	// ID 机器人 id（选择器的 key）
	ID string
	// Name 人类可读名（仅日志 / UI）
	Name string
	// ConfigPath `robot.yaml` 的绝对路径（运动学 / 标定 / 限位真值）
	ConfigPath string
	// PhysicsPath `physics.yaml` 的绝对路径（物理量；真值来源可能是官方 MJCF）
	PhysicsPath string
	// MJCFPath MJCF 的绝对路径；空串 = 由 gen_model.py 从 robot.yaml 生成
	MJCFPath string
	// TCPSite MJCF 里代表 TCP 的 site 名
	TCPSite string
	// PhysicsKind `legacy`（MeArm：physics.yaml 顶层即驱动参数）
	// / `driver`（SO-101：顶层带 `driver:` 段，物理量真值在官方 MJCF）
	PhysicsKind string
}

// Selector 解析后的 `config/robots.yaml`。
type Selector struct {
	// Default 缺省机器人 id
	Default string
	// Robots 全部机器人（按 id 索引）
	Robots map[string]Entry
	// SourcePath 选择器文件自身的绝对路径
	SourcePath string
	// RepoRoot 仓库根（= 选择器所在目录的上一级）；全部路径按它解析
	RepoRoot string
}

// 选择器里允许出现的字段的诊断常量（拼错字段名时给得出提示）
const (
	physicsKindLegacy = "legacy"
	physicsKindDriver = "driver"
)

type selectorFile struct {
	Version int    `yaml:"version"`
	Default string `yaml:"default"`
	Robots  map[string]struct {
		Name       string `yaml:"name"`
		Config     string `yaml:"config"`
		Physics    string `yaml:"physics"`
		Simulation struct {
			MJCF    string `yaml:"mjcf"`
			TCPSite string `yaml:"tcpSite"`
		} `yaml:"simulation"`
	} `yaml:"robots"`
}

// LoadSelector 读取并校验模型选择器。
//
// 失败即返回错误（不回退到"上一次用过的机器人"那类兜底）：
// 选择器缺失或损坏时，最诚实的表现是**起不来**，而不是加载一台没人知道的机器人。
func LoadSelector(path string) (*Selector, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("读取模型选择器失败: %w", err)
	}
	var f selectorFile
	if err := yaml.Unmarshal(raw, &f); err != nil {
		return nil, fmt.Errorf("解析模型选择器失败: %w", err)
	}
	if len(f.Robots) == 0 {
		return nil, fmt.Errorf("%s: robots 不能为空", path)
	}

	abs, err := filepath.Abs(path)
	if err != nil {
		return nil, fmt.Errorf("解析选择器路径失败: %w", err)
	}
	// 选择器位于 <repo>/config/robots.yaml ⇒ 仓库根 = 它的上一级目录
	repoRoot := filepath.Dir(filepath.Dir(abs))

	sel := &Selector{
		Default:    f.Default,
		Robots:     make(map[string]Entry, len(f.Robots)),
		SourcePath: abs,
		RepoRoot:   repoRoot,
	}

	// 排序后遍历：错误信息稳定可预期（map 迭代顺序在 Go 里是随机的，
	// 同一个配置两次启动报文不同会让"配置有问题"变成"看起来像随机故障"）
	ids := make([]string, 0, len(f.Robots))
	for id := range f.Robots {
		ids = append(ids, id)
	}
	sort.Strings(ids)

	for _, id := range ids {
		item := f.Robots[id]
		if item.Name == "" {
			return nil, fmt.Errorf("%s: robots.%s.name 不能为空", path, id)
		}
		if item.Config == "" {
			return nil, fmt.Errorf("%s: robots.%s.config 不能为空", path, id)
		}
		configPath := filepath.Join(repoRoot, filepath.FromSlash(item.Config))
		if st, err := os.Stat(configPath); err != nil || st.IsDir() {
			return nil, fmt.Errorf("%s: robots.%s.config = %q 不存在（%s）",
				path, id, item.Config, configPath)
		}

		physics := item.Physics
		if physics == "" {
			// 约定：与 config 同目录的 physics.yaml
			physics = filepath.ToSlash(filepath.Join(filepath.Dir(filepath.FromSlash(item.Config)), "physics.yaml"))
		}
		physicsPath := filepath.Join(repoRoot, filepath.FromSlash(physics))
		if st, err := os.Stat(physicsPath); err != nil || st.IsDir() {
			return nil, fmt.Errorf("%s: robots.%s.physics = %q 不存在（%s）",
				path, id, physics, physicsPath)
		}

		mjcfPath := ""
		if item.Simulation.MJCF != "" {
			mjcfPath = filepath.Join(repoRoot, filepath.FromSlash(item.Simulation.MJCF))
			if st, err := os.Stat(mjcfPath); err != nil || st.IsDir() {
				return nil, fmt.Errorf("%s: robots.%s.simulation.mjcf = %q 不存在（%s）",
					path, id, item.Simulation.MJCF, mjcfPath)
			}
		}
		tcpSite := item.Simulation.TCPSite
		if tcpSite == "" {
			tcpSite = "tcp"
		}

		kind, err := physicsKind(physicsPath)
		if err != nil {
			return nil, err
		}

		sel.Robots[id] = Entry{
			ID:          id,
			Name:        item.Name,
			ConfigPath:  configPath,
			PhysicsPath: physicsPath,
			MJCFPath:    mjcfPath,
			TCPSite:     tcpSite,
			PhysicsKind: kind,
		}
	}

	if sel.Default == "" {
		return nil, fmt.Errorf("%s: default 不能为空", path)
	}
	if _, ok := sel.Robots[sel.Default]; !ok {
		return nil, fmt.Errorf("%s: default = %q 不在 robots 中，可选: %s",
			path, sel.Default, sel.IDList())
	}
	return sel, nil
}

// physicsKind 判据 = 顶层有没有 `driver:` 段。
//
// 刻意**不新增配置项**：判据一旦写进配置，就又变成两处要维护的东西，
// 而两边不一致时没人知道该信哪个。
func physicsKind(path string) (string, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return "", err
	}
	var probe map[string]any
	if err := yaml.Unmarshal(raw, &probe); err != nil {
		return "", fmt.Errorf("解析 %s 失败: %w", path, err)
	}
	if inner, ok := probe["driver"]; ok {
		if _, isMap := inner.(map[string]any); isMap {
			return physicsKindDriver, nil
		}
	}
	return physicsKindLegacy, nil
}

// IDList 全部 id（排好序，用于错误信息与日志）。
func (s *Selector) IDList() string {
	ids := make([]string, 0, len(s.Robots))
	for id := range s.Robots {
		ids = append(ids, id)
	}
	sort.Strings(ids)
	out := ""
	for i, id := range ids {
		if i > 0 {
			out += " / "
		}
		out += id
	}
	return out
}

// Entry 按 id 取记录。`id == ""` ⇒ 取 `default`。
//
// 未知 id **返回错误**（不回退到 default）：见包注释。
func (s *Selector) Entry(id string) (Entry, error) {
	if id == "" {
		id = s.Default
	}
	e, ok := s.Robots[id]
	if !ok {
		return Entry{}, fmt.Errorf("未知机器人 id %q；可选: %s", id, s.IDList())
	}
	return e, nil
}

// LoadByID 一步到位：选择器 → 记录 → `robot.Model`。
//
// 后端启动路径用它，避免 main 里出现三段各自去拼路径的代码
// （那种写法在"换了仓库布局"时会三个地方各错一部分）。
func LoadByID(selectorPath, id string) (*Model, *Selector, Entry, error) {
	sel, err := LoadSelector(selectorPath)
	if err != nil {
		return nil, nil, Entry{}, err
	}
	entry, err := sel.Entry(id)
	if err != nil {
		return nil, nil, Entry{}, err
	}
	m, err := Load(entry.ConfigPath)
	if err != nil {
		return nil, nil, Entry{}, err
	}
	return m, sel, entry, nil
}
