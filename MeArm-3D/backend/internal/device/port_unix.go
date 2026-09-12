//go:build !windows

package device

import (
	"fmt"
	"io"
	"os"
	"os/exec"
	"strconv"
	"strings"
)

// openPort 在类 Unix 系统上通过 stty 配置串口，然后以文件方式读写（纯标准库）。
//
// 与 Windows 侧的一处**有意差异**：这里用 `min 0 time 1`（0.1s 读超时）而不是
// 原实现的 `min 1 time 0`（无限阻塞）。原因是阻塞读在 tty 上无法被 Close 可靠中断，
// 会让 readLoop 永久挂住；0.1s 轮询等价于 Windows 的"立即返回"读模式，
// 由 readLoop 的 2ms sleep 进一步节流。
//
// 本项目目标平台是 Windows（CH340 + Uno），此路径仅为可移植性保留，未经真机验证。
func openPort(cfg SerialConfig) (io.ReadWriteCloser, error) {
	parityFlag := "-parenb"
	switch strings.ToUpper(cfg.Parity) {
	case "E":
		parityFlag = "parenb -parodd"
	case "O":
		parityFlag = "parenb parodd"
	}
	stopFlag := "-cstopb"
	if cfg.StopBits == 2 {
		stopFlag = "cstopb"
	}
	dataBits := cfg.DataBits
	if dataBits < 5 || dataBits > 8 {
		dataBits = 8
	}
	baud := cfg.Baud
	if baud <= 0 {
		baud = 115200
	}

	args := []string{
		"-F", cfg.Port,
		strconv.Itoa(baud),
		"cs" + strconv.Itoa(dataBits),
		"raw", "-icanon",
		"min", "0", "time", "1",
		parityFlag, stopFlag,
	}
	if err := exec.Command("stty", args...).Run(); err != nil {
		return nil, fmt.Errorf("stty 配置 %s 失败: %w", cfg.Port, err)
	}

	f, err := os.OpenFile(cfg.Port, os.O_RDWR|os.O_NOCTTY, 0)
	if err != nil {
		return nil, fmt.Errorf("打开 %s 失败: %w", cfg.Port, err)
	}
	return f, nil
}
