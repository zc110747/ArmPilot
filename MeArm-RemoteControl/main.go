// arm-web：基于 Go 的串口转 Web / TCP 服务器。
//
// 架构：
//
//	arm-device (串口 COM4/9600)
//	   ▲      ▼
//	[ Serial 管理 + 自动重连 ]
//	   ▲      ▼
//	[   Hub 广播总线（设备回显->多端）  ]
//	   ▲                    ▲
//	[ TCP 局域网透传 ]     [ Web + WebSocket 3D 摇杆 ]
//
// 串口指令语法严格依赖 arm-device（见 internal/protocol），所有控制意图最终
// 都归一化为 arm-device 指令文本再下发，保证与固件一致。
package main

import (
	"embed"
	"flag"
	"io/fs"
	"log"
	"os"
	"os/signal"
	"syscall"

	"arm-web/internal/config"
	"arm-web/internal/hub"
	"arm-web/internal/serial"
	"arm-web/internal/tcp"
	"arm-web/internal/web"
)

// webFS 在编译期把 web/static 全部嵌入二进制，使产物成为自包含单文件，
// 运行时不再依赖磁盘上的 web/static 目录（“一键编译 web 文件”即指此步）。
//
//go:embed all:web/static
var webFS embed.FS

func main() {
	cfgPath := flag.String("c", "config.yaml", "配置文件路径 (YAML)")
	flag.Parse()

	cfg, err := config.Load(*cfgPath)
	if err != nil {
		log.Fatalf("[fatal] %v", err)
	}
	setupLog(cfg.LogLevel)

	log.Printf("========== arm-web 启动 ==========")
	log.Printf("串口: %s @ %d %dN%d (%c)", cfg.Serial.Port, cfg.Serial.Baud,
		cfg.Serial.DataBits, cfg.Serial.StopBits, upcase(cfg.Serial.Parity))
	log.Printf("摇杆映射: 左 X->S%d 左 Y->S%d | 右 X->S%d 右 Y->S%d",
		cfg.Joystick.LXServo, cfg.Joystick.LYServo, cfg.Joystick.RXServo, cfg.Joystick.RYServo)

	// 串口
	if cfg.LogLevel == "debug" {
		serial.SetDebug(true)
	}
	ser := serial.Open(serial.Config{
		Port:          cfg.Serial.Port,
		Baud:          cfg.Serial.Baud,
		DataBits:      cfg.Serial.DataBits,
		StopBits:      cfg.Serial.StopBits,
		Parity:        cfg.Serial.Parity,
		ReconnectSec:  cfg.Serial.ReconnectSec,
		MinIntervalMs: cfg.Serial.MinIntervalMs,
		AckTimeoutMs:  cfg.Serial.AckTimeoutMs,
		ConnectSettleMs: cfg.Serial.ConnectSettleMs,
	})
	defer ser.Close()

	// 广播总线
	h := hub.New()

	// 把设备回显广播给所有订阅者（Web/TCP）
	ser.SetLineHandler(func(line string) { h.Broadcast(line) })

	// TCP 局域网转发
	if cfg.TCP.Enabled {
		ts := tcp.New(tcp.Config{Host: cfg.TCP.Host, Port: cfg.TCP.Port}, ser, h)
		go func() {
			if err := ts.Listen(); err != nil {
				log.Printf("[tcp] 监听失败: %v", err)
			}
		}()
	}

	// Web + WebSocket
	if cfg.Web.Enabled {
		staticFS, err := fs.Sub(webFS, "web/static")
		if err != nil {
			log.Fatalf("[fatal] 内嵌 web 资源取出失败: %v", err)
		}
		ws := web.New(cfg.Web, cfg.Joystick, ser, h, staticFS)
		// 串口连接状态 / 通讯失败 -> 广播给所有 Web 客户端
		ser.SetStatusHandler(func(connected bool, serialErr string, commErr bool, commErrMsg string) {
			ws.BroadcastStatus(connected, serialErr, commErr, commErrMsg)
		})
		go func() {
			if err := ws.ListenAndServe(); err != nil {
				log.Printf("[web] 服务失败: %v", err)
			}
		}()
	}

	log.Printf("提示：浏览器打开 http://<本机IP>:%d 进行控制", cfg.Web.Port)
	log.Printf("提示：局域网设备可 telnet <本机IP> %d 下发 arm-device 指令", cfg.TCP.Port)

	// 等待退出信号
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, syscall.SIGINT, syscall.SIGTERM)
	<-sig
	log.Printf("========== arm-web 关闭 ==========")
}

func setupLog(level string) {
	// 简单级别过滤：低于设定级别的日志不打印（此处全部输出，级别存入结构化前缀）
	_ = level
	log.SetFlags(log.LstdFlags | log.Lmicroseconds)
}

func upcase(s string) byte {
	if len(s) == 0 {
		return 'N'
	}
	return s[0]
}
