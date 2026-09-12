// Package device 抽象链路末端：一台"能收 JR、回 OK/ERR/STATE"的机械臂。
//
// 它存在的意义是让 Phase 8 在**没有真机**的前提下也能跑通字节级闭环
// （spec §四十七明确要求 Phase 7~8 不接真实机械臂）：
//
//	SimDevice     —— 模拟固件（内置标定表 + 有限角速度 + 延迟 + 限位拒绝）
//	SerialDevice  —— 真串口（Phase 9；本阶段只留接口与配置位）
//
// 上层 `controller` 只认本接口，因此 Phase 9 换真串口时零改动。
package device

import (
	"time"
)

// Line 是从设备收到的一行回执（已去掉行尾）。
type Line struct {
	Text string
	At   time.Time
}

// StatusHandler 接收连接状态翻转。
type StatusHandler func(connected bool, reason string)

// Device 是链路末端的最小抽象。
//
// 契约（与 arm-device 一致）：
//   - 每条指令**恰好回一行**（OK/ERR），异步上报以 STATE 等主动帧出现
//   - WriteLine 可被并发调用（内部串行化）
//   - 不可用时 WriteLine 返回错误，而不是静默丢弃
type Device interface {
	// Kind 传输类型标识（sim / serial）
	Kind() string
	// Connected 当前是否可用
	Connected() bool
	// UnavailableReason 不可用原因（可用时为空串）
	UnavailableReason() string
	// WriteLine 下发一行指令（不含行尾）
	WriteLine(line string) error
	// Lines 回执行通道（只读）
	Lines() <-chan Line
	// OnStatus 订阅连接状态变化，返回退订函数
	OnStatus(fn StatusHandler) func()
	// Close 释放资源（幂等）
	Close() error
}
