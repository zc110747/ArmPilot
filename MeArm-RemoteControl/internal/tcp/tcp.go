// package tcp 实现“串口 <-> TCP”的局域网透传桥。
//
// 其它设备（或后期远程控制客户端）以 raw TCP 连接本服务器端口，直接下发
// arm-device 指令文本（换行结束），服务器原样转发给串口；设备回显经 hub 原样
// 写回该 TCP 连接。这样局域网内任意 telnet/自研客户端都能控制机械臂，并为远程
// 控制预留统一接口（见文件末尾 RemoteEnvelope 说明）。
package tcp

import (
	"bufio"
	"fmt"
	"log"
	"net"
	"sync"

	"arm-web/internal/hub"
	"arm-web/internal/serial"
)

// Client 是单个 TCP 连接的订阅者，实现 hub.Client。
type Client struct {
	conn net.Conn
	out  chan string
	mu   sync.Mutex
}

func (c *Client) Send(line string) {
	select {
	case c.out <- line:
	default:
		// 慢消费者：丢弃，避免阻塞广播
	}
}

func (c *Client) writeLoop(done chan struct{}) {
	for {
		select {
		case <-done:
			return
		case line := <-c.out:
			c.mu.Lock()
			_, err := fmt.Fprintf(c.conn, "%s\r\n", line)
			c.mu.Unlock()
			if err != nil {
				return
			}
		}
	}
}

// Server 监听 TCP 端口并桥接到串口。
type Server struct {
	cfg    Config
	serial *serial.Serial
	hub    *hub.Hub
}

type Config struct {
	Host string
	Port int
}

func New(cfg Config, s *serial.Serial, h *hub.Hub) *Server {
	return &Server{cfg: cfg, serial: s, hub: h}
}

func (s *Server) Addr() string { return fmt.Sprintf("%s:%d", s.cfg.Host, s.cfg.Port) }

// Listen 启动 TCP 监听（阻塞，直到监听失败或被关闭）。
func (s *Server) Listen() error {
	ln, err := net.Listen("tcp", s.Addr())
	if err != nil {
		return err
	}
	log.Printf("[tcp] 局域网转发已启动: %s（设备指令可经此端口下发）", s.Addr())
	for {
		conn, err := ln.Accept()
		if err != nil {
			return err
		}
		go s.handle(conn)
	}
}

func (s *Server) handle(conn net.Conn) {
	defer conn.Close()
	c := &Client{conn: conn, out: make(chan string, 64)}
	s.hub.Register(c)
	defer s.hub.Unregister(c)

	done := make(chan struct{})
	defer close(done)
	go c.writeLoop(done)

	remote := conn.RemoteAddr().String()
	log.Printf("[tcp] 客户端接入: %s", remote)

	// 发送一次欢迎/能力说明（可选，客户端可忽略）
	fmt.Fprintf(conn, "arm-web tcp bridge ready. send arm-device commands, one per line.\r\n")

	scanner := bufio.NewScanner(conn)
	scanner.Buffer(make([]byte, 0, 4096), 4096)
	for scanner.Scan() {
		line := scanner.Text()
		if line == "" {
			continue
		}
		// 透传：直接作为 arm-device 指令下发串口。
		// 此日志表示“服务器已收到该指令并入待发缓冲”；实际写向设备受命令-应答门控
		// 限速（同一时刻仅 1 条在途、最新摇杆位置合并），debug 日志的 [serial] TX 才是
		// 真正写串口的字节。
		// （此处保留“远程控制接口”扩展点：若行首为 '@' 可解析为结构化 JSON 指令，
		//  目前不做，保持与 Web/固件一致的纯文本协议。）
		log.Printf("[tcp] 收到指令 %q（入待发缓冲）", line)
		if err := s.serial.WriteLine(line); err != nil {
			log.Printf("[tcp] 下发失败(客户端 %s): %v", remote, err)
		}
	}
	log.Printf("[tcp] 客户端断开: %s", remote)
}
