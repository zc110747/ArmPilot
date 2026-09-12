package device

import "fmt"

// NewSerial 创建真串口设备。
//
// Phase 8 阶段尚未实现 —— 这里**显式报错**而不是悄悄退回 sim：
// 静默降级会让人以为"连上真机了"，而实际上所有指令都进了模拟器，
// 这是比崩溃更危险的失败模式。
//
// Phase 9 落地要点（来自 skill `arm-robot-serial` 的实测经验，勿重复踩）：
//   - Uno 开串口后 bootloader 有 ~2.5s 交权期，窗口内指令被吞 → 需要静默窗口
//   - 连接后首包常丢 → 需要先发一个暖机包
//   - Windows 非重叠 I/O 读写互斥（单条命令 200~900ms）→ 必须用「立即返回」读超时
//   - 手动行缓冲，且不能用 bufio（空闲 20s 会因空读抛错而断连）
//   - 命令-应答门控下，周期性 STATUS 查询会饿死遥控/控制流 → 状态改由固件主动上报
type SerialConfig struct {
	Port string
	Baud int
}

// NewSerial 返回未实现错误（接口已就位，Phase 9 填实现）。
func NewSerial(_ SerialConfig) (Device, error) {
	return nil, fmt.Errorf("device.mode=serial 尚未实现（Phase 9）：请先用 device.mode=sim 跑通链路")
}
