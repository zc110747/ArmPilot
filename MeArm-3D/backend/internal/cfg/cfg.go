// Package cfg 读取后端运行配置（backend/config.yaml）。
//
// 注意与 config/robot.yaml 的分工：
//
//	config/robot.yaml   —— **模型/标定/限位真值**（前后端共用，唯一来源）
//	backend/config.yaml —— 仅本服务的**运行参数**（端口、设备模式、模拟器参数）
//
// 运行配置里绝不允许出现关节限位或标定数值：那会立刻造成双份真值。
package cfg

import (
	"fmt"
	"os"
	"path/filepath"

	"gopkg.in/yaml.v3"
)

// Config 后端运行配置。
type Config struct {
	Web     WebConfig     `yaml:"web"`
	Robot   RobotConfig   `yaml:"robot"`
	Device  DeviceConfig  `yaml:"device"`
	Control ControlConfig `yaml:"control"`

	Path string `yaml:"-"` // 配置文件自身的路径（日志用）
}

// WebConfig HTTP / WebSocket 监听参数。
type WebConfig struct {
	Host string `yaml:"host"`
	Port int    `yaml:"port"`
	Path string `yaml:"path"`
	// PingIntervalMs 向浏览器发传输层 ping 的间隔
	PingIntervalMs int `yaml:"ping_interval_ms"`
	// ClientTimeoutMs 浏览器静默多久判死
	ClientTimeoutMs int `yaml:"client_timeout_ms"`
}

// RobotConfig 指向模型真值文件。
type RobotConfig struct {
	// ConfigPath robot.yaml 路径。支持相对路径；不存在时按候选列表回退
	// （见 ResolveRobotConfig），从而 backend/ 与 MeArm-3D/ 两种工作目录都能跑。
	ConfigPath string `yaml:"config_path"`
}

// DeviceConfig 链路末端。
type DeviceConfig struct {
	// Mode: sim（Phase 8，内置假固件）| serial（Phase 9）
	Mode   string       `yaml:"mode"`
	Sim    SimConfig    `yaml:"sim"`
	Serial SerialConfig `yaml:"serial"`
}

// SimConfig 模拟固件参数。
type SimConfig struct {
	MaxServoSpeed float64 `yaml:"max_servo_speed"`
	LatencyMs     int     `yaml:"latency_ms"`
	TickMs        int     `yaml:"tick_ms"`
	EnforceLimits bool    `yaml:"enforce_limits"`
	BootMs        int     `yaml:"boot_ms"`
}

// SerialConfig 真串口参数（Phase 9 使用）。
type SerialConfig struct {
	Port string `yaml:"port"`
	Baud int    `yaml:"baud"`
}

// ControlConfig 控制器策略。
type ControlConfig struct {
	AckTimeoutMs      int `yaml:"ack_timeout_ms"`
	MinSendIntervalMs int `yaml:"min_send_interval_ms"`
}

// Default 返回默认配置（文件缺失时使用）。
func Default() Config {
	return Config{
		Web: WebConfig{
			Host: "0.0.0.0", Port: 8090, Path: "/ws/joint",
			PingIntervalMs: 15000, ClientTimeoutMs: 40000,
		},
		Robot: RobotConfig{ConfigPath: "../config/robot.yaml"},
		Device: DeviceConfig{
			Mode: "sim",
			Sim: SimConfig{
				MaxServoSpeed: 240, LatencyMs: 15, TickMs: 20,
				EnforceLimits: true, BootMs: 0,
			},
			Serial: SerialConfig{Port: "COM4", Baud: 115200},
		},
		Control: ControlConfig{AckTimeoutMs: 800, MinSendIntervalMs: 0},
	}
}

// Load 读取配置文件；文件不存在时返回默认配置（不算错误，便于零配置启动）。
func Load(path string) (Config, error) {
	c := Default()
	c.Path = path
	raw, err := os.ReadFile(path)
	if err != nil {
		if os.IsNotExist(err) {
			return c, nil
		}
		return c, fmt.Errorf("读取配置失败: %w", err)
	}
	if err := yaml.Unmarshal(raw, &c); err != nil {
		return c, fmt.Errorf("解析配置失败: %w", err)
	}
	if c.Web.Port <= 0 {
		c.Web.Port = 8090
	}
	if c.Device.Mode == "" {
		c.Device.Mode = "sim"
	}
	return c, nil
}

// ResolveRobotConfig 定位 robot.yaml。
//
// 候选顺序（相对当前工作目录）：
//  1. 配置里写的路径（默认 ../config/robot.yaml —— 从 backend/ 启动）
//  2. config/robot.yaml（从 MeArm-3D/ 启动）
//  3. ../MeArm-3D/config/robot.yaml（从仓库根启动）
//
// 全部失败时返回错误并列出全部候选，避免"文件找不到"变成猜谜。
func ResolveRobotConfig(configured string) (string, error) {
	candidates := []string{configured, "../config/robot.yaml", "config/robot.yaml", "../MeArm-3D/config/robot.yaml"}
	seen := map[string]bool{}
	tried := make([]string, 0, len(candidates))
	for _, c := range candidates {
		if c == "" || seen[c] {
			continue
		}
		seen[c] = true
		abs, err := filepath.Abs(c)
		if err != nil {
			continue
		}
		tried = append(tried, abs)
		if st, err := os.Stat(abs); err == nil && !st.IsDir() {
			return abs, nil
		}
	}
	return "", fmt.Errorf("找不到 robot.yaml（模型真值），已尝试:\n  %s", joinLines(tried))
}

func joinLines(items []string) string {
	out := ""
	for i, s := range items {
		if i > 0 {
			out += "\n  "
		}
		out += s
	}
	return out
}
