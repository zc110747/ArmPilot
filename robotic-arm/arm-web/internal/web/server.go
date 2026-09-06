// package web 提供本机控制用的 HTTP 服务与 WebSocket。
//
//   - HTTP 服务 web/static 下的静态页面（Three.js 3D 摇杆）。
//   - /ws 提供 WebSocket：网页把摇杆坐标 / 指令发上来，服务器归一化为
//     arm-device 指令写串口；设备回显经 hub 以 JSON 推回网页。
package web

import (
	"encoding/json"
	"io/fs"
	"log"
	"net/http"
	"regexp"
	"strconv"
	"strings"
	"sync"

	"arm-web/internal/config"
	"arm-web/internal/hub"
	"arm-web/internal/protocol"
	"arm-web/internal/serial"
)

// Server 聚合 HTTP/WS 与依赖。
type Server struct {
	cfg      config.WebConfig
	joyCfg   config.JoystickConfig
	serial   *serial.Serial
	hub      *hub.Hub
	axisMap  protocol.AxisMap
	staticFS fs.FS

	mu      sync.Mutex
	clients map[*wsClient]struct{} // 仅 WebSocket 客户端，用于下发连接状态变更
}

func New(cfg config.WebConfig, joyCfg config.JoystickConfig, s *serial.Serial, h *hub.Hub, staticFS fs.FS) *Server {
	return &Server{
		cfg:       cfg,
		joyCfg:    joyCfg,
		serial:    s,
		hub:       h,
		axisMap:   protocol.AxisMap{XServo: joyCfg.XServo, YServo: joyCfg.YServo, InvX: joyCfg.InvX, InvY: joyCfg.InvY},
		staticFS:  staticFS,
		clients:   make(map[*wsClient]struct{}),
	}
}

// ListenAndServe 启动 HTTP 服务（阻塞）。
func (s *Server) ListenAndServe() error {
	mux := http.NewServeMux()
	wsPath := s.cfg.WSPath
	if !strings.HasPrefix(wsPath, "/") {
		wsPath = "/" + wsPath
	}
	mux.HandleFunc(wsPath, s.handleWS)
	// 静态资源（编译期内嵌，无需磁盘目录）
	fileServer := http.FileServer(http.FS(s.staticFS))
	mux.Handle("/", fileServer)

	addr := s.cfg.Addr()
	log.Printf("[web] 本机控制页面已启动: http://%s%s  (WebSocket: %s)", addr, "/", wsPath)
	return http.ListenAndServe(addr, mux)
}

// wsClient 实现 hub.Client：把设备回显以 JSON 形式发往浏览器。
type wsClient struct {
	conn *Conn
	out  chan string
	mu   sync.Mutex
}

func (c *wsClient) Send(line string) {
	select {
	case c.out <- line:
	default:
	}
}

// 客户端 -> 服务器 的消息
type clientMsg struct {
	T string  `json:"t"` // joy | cmd | ping
	X float64 `json:"x"` // 摇杆 X ∈ [-1,1]
	Y float64 `json:"y"` // 摇杆 Y ∈ [-1,1]
	C string  `json:"c"` // 原始指令文本（cmd 类型）
}

// 服务器 -> 客户端 的消息
type serverMsg struct {
	T           string      `json:"t"` // serial | serial_status | err | pong
	Line        string      `json:"line,omitempty"`
	Angles      *armAngles  `json:"angles,omitempty"`
	Msg         string      `json:"msg,omitempty"`
	Connected   *bool       `json:"connected,omitempty"`   // serial_status: 串口是否已连接
	SerialErr   string      `json:"serial_err,omitempty"` // serial_status: 未连接原因
	CommErr     *bool       `json:"comm_err,omitempty"`   // serial_status: 通讯是否失败（已连接但应答超时）
	CommErrMsg  string      `json:"comm_err_msg,omitempty"` // serial_status: 通讯失败原因
}

type armAngles struct {
	S6 int `json:"s6"`
	S7 int `json:"s7"`
	S8 int `json:"s8"`
	S9 int `json:"s9"`
	OK bool `json:"ok"`
}

var reStatus = regexp.MustCompile(`S6=(\d+) S7=(\d+) S8=(\d+) S9=(\d+)`)

func (s *Server) handleWS(w http.ResponseWriter, r *http.Request) {
	conn, err := Upgrade(w, r)
	if err != nil {
		log.Printf("[web] WS 握手失败: %v", err)
		return
	}
	defer conn.Close()

	c := &wsClient{conn: conn, out: make(chan string, 64)}
	s.hub.Register(c)
	s.addClient(c)
	defer s.hub.Unregister(c)
	defer s.removeClient(c)

	done := make(chan struct{})
	defer close(done)
	go s.wsWriter(c, done)

	// 接入即同步当前串口状态（连接 + 通讯），避免界面停留在“未知”
	connected, lerr, commErr, commMsg := s.serial.Status()
	s.sendStatusTo(c, connected, lerr, commErr, commMsg)

	log.Printf("[web] WebSocket 客户端接入: %s", r.RemoteAddr)

	for {
		raw, err := conn.ReadMessage()
		if err != nil {
			break
		}
		var m clientMsg
		if err := json.Unmarshal([]byte(raw), &m); err != nil {
			s.sendErr(c, "JSON 解析失败")
			continue
		}
		switch m.T {
		case "ping":
			s.wsSend(c, serverMsg{T: "pong"})
		case "joy":
			// 摇杆坐标 -> JOY 指令，由 protocol 依赖 arm-device 语法生成。
			// 串口层做命令-应答门控 + 摇杆最新值合并（中间位置不重复下发）。
			cmd := protocol.JoystickToJOY(clampF(m.X), clampF(m.Y), s.axisMap)
			if err := s.serial.WriteLine(cmd + "\n"); err != nil {
				s.sendErr(c, "串口未连接，无法下发")
			}
		case "cmd":
			ok, normalized, verr := protocol.Validate(m.C)
			if !ok {
				s.sendErr(c, verr.Error())
				continue
			}
			log.Printf("[web] 指令 %q -> 下发(归一化) %q", m.C, normalized)
			if err := s.serial.WriteLine(normalized + "\n"); err != nil {
				s.sendErr(c, "串口未连接，无法下发")
			}
		default:
			s.sendErr(c, "未知消息类型: "+m.T)
		}
	}
	log.Printf("[web] WebSocket 客户端断开: %s", r.RemoteAddr)
}

// wsWriter 把设备回显（经 hub）封装成 JSON 推送给浏览器，
// 同时解析角度用于 3D 机械臂可视化。
func (s *Server) wsWriter(c *wsClient, done chan struct{}) {
	for {
		select {
		case <-done:
			return
		case line := <-c.out:
			msg := serverMsg{T: "serial", Line: line}
			if ang, ok := parseAngles(line); ok {
				msg.Angles = ang
			}
			if data, err := json.Marshal(msg); err == nil {
				c.conn.WriteMessage(string(data))
			}
		}
	}
}

func (s *Server) wsSend(c *wsClient, m serverMsg) {
	if data, err := json.Marshal(m); err == nil {
		c.conn.WriteMessage(string(data))
	}
}

func (s *Server) sendErr(c *wsClient, msg string) {
	s.wsSend(c, serverMsg{T: "err", Msg: msg})
}

// addClient / removeClient 维护 WebSocket 客户端集合（仅用于串口状态广播）。
func (s *Server) addClient(c *wsClient) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.clients[c] = struct{}{}
}

func (s *Server) removeClient(c *wsClient) {
	s.mu.Lock()
	defer s.mu.Unlock()
	delete(s.clients, c)
}

// sendStatusTo 向单个客户端推送串口连接/通讯状态。
func (s *Server) sendStatusTo(c *wsClient, connected bool, serialErr string, commErr bool, commErrMsg string) {
	s.wsSend(c, serverMsg{
		T:          "serial_status",
		Connected:  &connected,
		SerialErr:  serialErr,
		CommErr:    &commErr,
		CommErrMsg: commErrMsg,
	})
}

// BroadcastStatus 把串口连接/通讯状态变更广播给所有 WebSocket 客户端。
// 由 serial.SetStatusHandler 在连接状态或通讯状态翻转时调用。
func (s *Server) BroadcastStatus(connected bool, serialErr string, commErr bool, commErrMsg string) {
	data, err := json.Marshal(serverMsg{
		T:          "serial_status",
		Connected:  &connected,
		SerialErr:  serialErr,
		CommErr:    &commErr,
		CommErrMsg: commErrMsg,
	})
	if err != nil {
		return
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	for c := range s.clients {
		c.conn.WriteMessage(string(data))
	}
}

// parseAngles 从形如 "OK JOY S6=89 S7=89 S8=89 S9=89" / "OK SET S9=120" /
// "S6=.. S7=.. S8=.. S9=.." 的回显中提取角度，供前端 3D 姿态刷新。
func parseAngles(line string) (*armAngles, bool) {
	a := &armAngles{}
	found := false
	for _, kv := range reStatus.FindAllStringSubmatch(line, -1) {
		s6, _ := strconv.Atoi(kv[1])
		s7, _ := strconv.Atoi(kv[2])
		s8, _ := strconv.Atoi(kv[3])
		s9, _ := strconv.Atoi(kv[4])
		a.S6, a.S7, a.S8, a.S9 = s6, s7, s8, s9
		a.OK = true
		found = true
	}
	if found {
		return a, true
	}
	// 兼容单控 "OK SET S9=120"
	reSingle := regexp.MustCompile(`S([6789])=(\d+)`)
	for _, kv := range reSingle.FindAllStringSubmatch(line, -1) {
		id, _ := strconv.Atoi(kv[1])
		v, _ := strconv.Atoi(kv[2])
		switch id {
		case 6:
			a.S6 = v
		case 7:
			a.S7 = v
		case 8:
			a.S8 = v
		case 9:
			a.S9 = v
		}
		a.OK = true
		found = true
	}
	if found {
		return a, true
	}
	return nil, false
}

func clampF(v float64) float64 {
	if v < -1 {
		return -1
	}
	if v > 1 {
		return 1
	}
	return v
}
