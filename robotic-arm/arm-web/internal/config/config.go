package config

import (
	"fmt"
	"os"

	"gopkg.in/yaml.v3"
)

// Config 是 arm-web 的顶层配置，全部字段均可通过 YAML 修改。
type Config struct {
	Serial   SerialConfig   `yaml:"serial"`
	Web      WebConfig      `yaml:"web"`
	TCP      TCPConfig      `yaml:"tcp"`
	Joystick JoystickConfig `yaml:"joystick"`
	LogLevel string         `yaml:"log_level"`
}

// JoystickConfig 描述网页双 3D 摇杆（遥控形式）到 4 路舵机的映射，均可在 YAML 配置。
type JoystickConfig struct {
	LXServo int  `yaml:"lx_servo"` // 左摇杆 X 轴 -> 舵机 id（默认 9=底座）
	LYServo int  `yaml:"ly_servo"` // 左摇杆 Y 轴 -> 舵机 id（默认 8=左舵）
	RXServo int  `yaml:"rx_servo"` // 右摇杆 X 轴 -> 舵机 id（默认 6=夹取）
	RYServo int  `yaml:"ry_servo"` // 右摇杆 Y 轴 -> 舵机 id（默认 7=右舵）
	InvLX   bool   `yaml:"invert_lx"`
	InvLY   bool   `yaml:"invert_ly"`
	InvRX   bool   `yaml:"invert_rx"`
	InvRY   bool   `yaml:"invert_ry"`
	// DeadbandDeg 摇杆动作死区（视觉倾角，度）。偏移 ≤ 该值的轴视为居中、
	// 不产生任何下发；四轴全部在死区内时整帧 JOY 都不下发（串口零流量）。
	// 网页摇杆满偏 ≈ 31.5°，默认 10°（约 32% 行程）。
	DeadbandDeg float64 `yaml:"deadband_deg"`
}

type SerialConfig struct {
	Port          string `yaml:"port"`           // Windows: COMx ; Linux: /dev/ttyUSB0
	Baud          int    `yaml:"baud"`           // 波特率
	DataBits      int    `yaml:"databits"`       // 数据位
	StopBits      int    `yaml:"stopbits"`       // 停止位
	Parity        string `yaml:"parity"`         // N / E / O
	ReconnectSec  int    `yaml:"reconnect_sec"`  // 断线重连间隔(秒)
	MinIntervalMs int    `yaml:"min_interval_ms"` // 两条指令下发的最小间隔(毫秒)，防止高频冲刷设备
	AckTimeoutMs  int    `yaml:"ack_timeout_ms"`  // 等待下位机应答的超时(毫秒)；超时即判定通讯失败
	ConnectSettleMs int  `yaml:"connect_settle_ms"` // 连接建立后等待下位机 bootloader 交出的静默窗口(毫秒)；Arduino Uno 打开串口会触发自动复位，bootloader 约 2.5s 后才交权，窗口内指令会被丢弃
}

type WebConfig struct {
	Enabled bool   `yaml:"enabled"`
	Host    string `yaml:"host"`    // 绑定 IP，0.0.0.0 = 所有网卡
	Port    int    `yaml:"port"`
	WSPath  string `yaml:"ws_path"` // WebSocket 路径
}

type TCPConfig struct {
	Enabled bool   `yaml:"enabled"`
	Host    string `yaml:"host"` // 绑定 IP，0.0.0.0 = 局域网可访问
	Port    int    `yaml:"port"`
}

// Addr 返回 "ip:port" 形式，供 net.Listen 使用。
func (w WebConfig) Addr() string { return fmt.Sprintf("%s:%d", w.Host, w.Port) }
func (t TCPConfig) Addr() string { return fmt.Sprintf("%s:%d", t.Host, t.Port) }

// Load 从指定路径读取 YAML 并填充默认值。
func Load(path string) (*Config, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("读取配置文件 %s 失败: %w", path, err)
	}
	var c Config
	if err := yaml.Unmarshal(data, &c); err != nil {
		return nil, fmt.Errorf("解析配置文件 %s 失败: %w", path, err)
	}
	c.applyDefaults()
	if err := c.validate(); err != nil {
		return nil, err
	}
	return &c, nil
}

func (c *Config) applyDefaults() {
	if c.Serial.Port == "" {
		c.Serial.Port = "COM4"
	}
	if c.Serial.Baud == 0 {
		c.Serial.Baud = 115200
	}
	if c.Serial.DataBits == 0 {
		c.Serial.DataBits = 8
	}
	if c.Serial.StopBits == 0 {
		c.Serial.StopBits = 1
	}
	if c.Serial.Parity == "" {
		c.Serial.Parity = "N"
	}
	if c.Serial.ReconnectSec <= 0 {
		c.Serial.ReconnectSec = 3
	}
	if c.Serial.MinIntervalMs < 0 {
		c.Serial.MinIntervalMs = 0
	}
	// 注意：命令-应答(ACK)门控 + 摇杆最新值合并已能防止高频冲刷设备，
	// 故默认 0（不额外节流）。如需兜底再按需调大。
	if c.Serial.AckTimeoutMs <= 0 {
		c.Serial.AckTimeoutMs = 800
	}
	// Arduino Uno(ATmega328P) 打开串口会触发 DTR 自动复位进入 optiboot，
	// bootloader 约 2.5s 后才把控制权交给固件；窗口内下发的首条指令会被丢弃。
	// 默认 2500ms 与 host_verify.py 的等待对齐，避免“首条指令无应答”。
	if c.Serial.ConnectSettleMs <= 0 {
		c.Serial.ConnectSettleMs = 2500
	}
	if c.Web.WSPath == "" {
		c.Web.WSPath = "/ws"
	}
	if c.Web.Port == 0 {
		c.Web.Port = 8080
	}
	if c.Web.Host == "" {
		c.Web.Host = "0.0.0.0"
	}
	if c.TCP.Port == 0 {
		c.TCP.Port = 9001
	}
	if c.TCP.Host == "" {
		c.TCP.Host = "0.0.0.0"
	}
	if c.LogLevel == "" {
		c.LogLevel = "info"
	}
	if c.Joystick.LXServo == 0 {
		c.Joystick.LXServo = 9
	}
	if c.Joystick.LYServo == 0 {
		c.Joystick.LYServo = 8
	}
	if c.Joystick.RXServo == 0 {
		c.Joystick.RXServo = 6
	}
	if c.Joystick.RYServo == 0 {
		c.Joystick.RYServo = 7
	}
}

func (c *Config) validate() error {
	switch c.Serial.Parity {
	case "N", "E", "O", "n", "e", "o":
	default:
		return fmt.Errorf("serial.parity 非法: %q (应为 N/E/O)", c.Serial.Parity)
	}
	if c.Serial.DataBits < 5 || c.Serial.DataBits > 8 {
		return fmt.Errorf("serial.databits 非法: %d (5..8)", c.Serial.DataBits)
	}
	if c.Serial.StopBits != 1 && c.Serial.StopBits != 2 {
		return fmt.Errorf("serial.stopbits 非法: %d (1/2)", c.Serial.StopBits)
	}
	return nil
}
