package robot

import (
	"os"
	"path/filepath"
	"strings"
	"testing"

	"gopkg.in/yaml.v3"
)

// selectorYAML 从包目录回溯到 MeArm-3D/config/robots.yaml —— 全程相对路径。
func selectorYAML(t *testing.T) string {
	t.Helper()
	p, err := filepath.Abs(filepath.Join("..", "..", "..", "config", "robots.yaml"))
	if err != nil {
		t.Fatalf("解析 robots.yaml 路径失败: %v", err)
	}
	return p
}

func loadSelector(t *testing.T) *Selector {
	t.Helper()
	sel, err := LoadSelector(selectorYAML(t))
	if err != nil {
		t.Fatalf("LoadSelector 失败: %v", err)
	}
	return sel
}

// 选择器只放"指针"。Default 必须真的存在于 Robots 里（否则启动期就会炸，
// 而且是"配置写错一个字"这类最难当场看出来的错误）。
func TestSelectorDefaultIsPresent(t *testing.T) {
	sel := loadSelector(t)
	if sel.Default == "" {
		t.Fatal("default 为空")
	}
	if _, ok := sel.Robots[sel.Default]; !ok {
		t.Fatalf("default = %q 不在 robots 中", sel.Default)
	}
}

// IDList 必须**排序稳定**：同一个配置两次启动的日志/报错要一致，
// 否则"配置有问题"会表现成"看起来像随机故障"。
func TestSelectorIDListIsSorted(t *testing.T) {
	sel := loadSelector(t)
	got := sel.IDList()
	if got != "mearm-v1" {
		t.Errorf("IDList() = %q", got)
	}
	// 再解析一次，确认稳定
	if again := loadSelector(t).IDList(); again != got {
		t.Errorf("两次解析结果不同: %q vs %q", got, again)
	}
}

// 选择器声明的每一台机器人都必须能被加载（数据驱动，不写死型号名）。
func TestSelectorDeclaresRobots(t *testing.T) {
	sel := loadSelector(t)
	if len(sel.Robots) == 0 {
		t.Fatal("选择器未声明任何机器人")
	}
	for id := range sel.Robots {
		if _, err := sel.Entry(id); err != nil {
			t.Errorf("选择器声明的 %q 无法加载: %v", id, err)
		}
	}
}

// physicsKind 的判据是「physics.yaml 顶层有没有 driver: 段」——**刻意不新增配置项**。
func TestSelectorPhysicsKind(t *testing.T) {
	sel := loadSelector(t)
	cases := map[string]string{
		"mearm-v1": "legacy",
	}
	for id, want := range cases {
		entry, err := sel.Entry(id)
		if err != nil {
			t.Fatalf("Entry(%q) 失败: %v", id, err)
		}
		if entry.PhysicsKind != want {
			t.Errorf("%s: physicsKind = %q, 期望 %q", id, entry.PhysicsKind, want)
		}
	}
}

// ★ 注册表 id ≠ 模型 id。选择器 key 是 `mearm-v1`，而 robot.yaml 里
// `robot.id` 是 `mearm` —— 两者不同名，所以**不能用 `Model.ID` 反查选择器**。
// 这条断言把这个坑固定下来（Python 侧靠 `resolve_robot_entry_by_config` 按路径反查）。
func TestSelectorIDDiffersFromModelID(t *testing.T) {
	m, _, entry, err := LoadByID(selectorYAML(t), "mearm-v1")
	if err != nil {
		t.Fatalf("LoadByID 失败: %v", err)
	}
	if entry.ID != "mearm-v1" {
		t.Errorf("entry.ID = %q", entry.ID)
	}
	if m.ID != "mearm" {
		t.Errorf("model.ID = %q，期望 mearm", m.ID)
	}
	if entry.ID == m.ID {
		t.Error("选择器 id 与模型 id 同名了 —— 那么本用例已失去意义，请更新它并检查反查逻辑")
	}
}

// 未知 id **必须报错**，绝不回退到 default。
// 回退会把"选择器里写错一个字母"变成"静默加载了另一台机器人"。
func TestUnknownIDDoesNotFallBack(t *testing.T) {
	sel := loadSelector(t)
	if _, err := sel.Entry("so-arm100"); err == nil {
		t.Fatal("未知 id 竟然没有报错")
	} else if !strings.Contains(err.Error(), "mearm-v1") {
		t.Errorf("错误信息里应列出可选项，实际: %v", err)
	}

	if _, _, _, err := LoadByID(selectorYAML(t), "nope"); err == nil {
		t.Fatal("LoadByID 对未知 id 竟然没有报错")
	}
}

// 空 id = 取 default（不是"报错"）—— 后端 `-robot` 留空就走这条路。
func TestEmptyIDMeansDefault(t *testing.T) {
	sel := loadSelector(t)
	byEmpty, err := sel.Entry("")
	if err != nil {
		t.Fatalf("Entry(\"\") 失败: %v", err)
	}
	byDefault, err := sel.Entry(sel.Default)
	if err != nil {
		t.Fatalf("Entry(default) 失败: %v", err)
	}
	if byEmpty.ID != byDefault.ID {
		t.Errorf("空 id 应等于 default: %q vs %q", byEmpty.ID, byDefault.ID)
	}
}

// 选择器路径解析：全部路径都要落在**仓库根**之下，且都真实存在。
func TestSelectorPathsResolveUnderRepoRoot(t *testing.T) {
	sel := loadSelector(t)
	for id, entry := range sel.Robots {
		for name, p := range map[string]string{
			"config":  entry.ConfigPath,
			"physics": entry.PhysicsPath,
			"mjcf":    entry.MJCFPath,
		} {
			if p == "" {
				continue // mjcf 允许为空（= 由 gen_model.py 从 robot.yaml 生成）
			}
			if !strings.HasPrefix(p, sel.RepoRoot) {
				t.Errorf("%s.%s = %q 不在仓库根 %q 之下", id, name, p, sel.RepoRoot)
			}
		}
	}
}

// 选择器里**禁止出现数值**（尺寸 / 限位 / 标定 / 物理量），只允许「指针」。
//
// 做法是直接读原文并检查**允许的键名集合**，而不是扫描坐标数：
// 结构化的白名单才能拦住"以后有人顺手加个 limit: …"，那正是"第二份真值"的开端。
func TestSelectorSchemaAllowsPointersOnly(t *testing.T) {
	sel := loadSelector(t)
	raw, err := os.ReadFile(sel.SourcePath)
	if err != nil {
		t.Fatalf("读取选择器失败: %v", err)
	}
	var doc map[string]any
	if err := yaml.Unmarshal(raw, &doc); err != nil {
		t.Fatalf("解析选择器失败: %v", err)
	}

	allowedRobotKeys := map[string]bool{"name": true, "config": true, "physics": true, "simulation": true}
	allowedSimKeys := map[string]bool{"mjcf": true, "tcpSite": true}

	robots, ok := doc["robots"].(map[string]any)
	if !ok {
		t.Fatal("robots 段不是映射")
	}
	for id, raw := range robots {
		entry, ok := raw.(map[string]any)
		if !ok {
			t.Fatalf("robots.%s 不是映射", id)
		}
		for key := range entry {
			if !allowedRobotKeys[key] {
				t.Errorf("robots.%s 出现了不允许的字段 %q —— 选择器只放指针（id/name/路径/site 名），"+
					"禁止放尺寸/限位/标定/物理量（那是第二份真值）", id, key)
			}
		}
		sim, hasSim := entry["simulation"]
		if !hasSim {
			continue
		}
		simMap, ok := sim.(map[string]any)
		if !ok {
			t.Fatalf("robots.%s.simulation 不是映射", id)
		}
		for key := range simMap {
			if !allowedSimKeys[key] {
				t.Errorf("robots.%s.simulation 出现了不允许的字段 %q（只允许 mjcf / tcpSite）", id, key)
			}
		}
	}
	// 顶层只允许 version / default / robots
	for key := range doc {
		switch key {
		case "version", "default", "robots":
		default:
			t.Errorf("选择器顶层出现了不允许的字段 %q", key)
		}
	}
}
