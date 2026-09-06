//go:build !windows

package serial

import (
	"fmt"
	"io"
	"os"
	"os/exec"
	"strconv"
	"strings"
)

// openPort 在类 Unix 系统上通过 stty 配置串口，然后以文件方式读写。
// 这是纯标准库实现：stty 设置波特率/数据位/停止位/校验，os.OpenFile 打开设备。
func openPort(cfg Config) (io.ReadWriteCloser, error) {
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

	args := []string{
		"-F", cfg.Port,
		strconv.Itoa(cfg.Baud),
		"cs" + strconv.Itoa(cfg.DataBits),
		"raw", "-icanon",
		"min", "1", "time", "0",
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
