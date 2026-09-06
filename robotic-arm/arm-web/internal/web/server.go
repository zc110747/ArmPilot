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
	"time"

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

	// 双摇杆合并状态：左右摇杆各自最后上报的坐标，下发时合并为一条 JOY 四轴帧。
	joyMu sync.Mutex
	joyL  [2]float64 // {x, y} 左摇杆
	joyR  [2]float64 // {x, y} 右摇杆

	// 最近已知舵机角度（S6..S9）。设备仅在 STATUS / SET / JOY 应答里携带角度
	// （可能只带部分轴），这里做合并，保证单轴回显不会把其它轴清零。
	anglesMu   sync.Mutex
	lastAngles [4]int  // 下标 0..3 对应 S6..S9
	haveAngles [4]bool // 各轴是否已有过回显
}

func New(cfg config.WebConfig, joyCfg config.JoystickConfig, s *serial.Serial, h *hub.Hub, staticFS fs.FS) *Server {
	return &Server{
		cfg:      cfg,
		joyCfg:   joyCfg,
		serial:   s,
		hub:      h,
		axisMap: protocol.AxisMap{
			LXServo: joyCfg.LXServo, LYServo: joyCfg.LYServo,
			RXServo: joyCfg.RXServo, RYServo: joyCfg.RYServo,
			InvLX: joyCfg.InvLX, InvLY: joyCfg.InvLY, InvRX: joyCfg.InvRX, InvRY: joyCfg.InvRY,
			DeadFrac: protocol.DeadFracFromDeg(joyCfg.DeadbandDeg),
		},
		staticFS: staticFS,
		clients:  make(map[*wsClient]struct{}),
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
	// 周期 STATUS 轮询：保证网页角度显示常开且始终最新
	go s.statusPoller()
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
	T    string  `json:"t"`    // joy | cmd | ping
	Side string  `json:"side"` // joy 专用：L=左摇杆 / R=右摇杆
	X    float64 `json:"x"`    // 摇杆 X ∈ [-1,1]
	Y    float64 `json:"y"`    // 摇杆 Y ∈ [-1,1]
	C    string  `json:"c"`    // 原始指令文本（cmd 类型）
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

// reStatus 兼容固件 STATUS 应答的模式后缀：S6=90(H) / S8=100(L)
var reStatus = regexp.MustCompile(`S6=(\d+)(?:\([HL]\))? S7=(\d+)(?:\([HL]\))? S8=(\d+)(?:\([HL]\))? S9=(\d+)(?:\([HL]\))?`)

// reSingle 单舵机角度（带可选模式后缀）
var reSingle = regexp.MustCompile(`S([6789])=(\d+)(?:\([HL]\))?`)

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
	// 接入即推送当前角度快照（若有），新开的页面立刻显示既有角度
	if ang := s.anglesSnapshot(); ang != nil {
		s.wsSend(c, serverMsg{T: "serial", Angles: ang})
	}

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
			// 双摇杆遥控：左右摇杆各自上报坐标，合并为一条 JOY 四轴帧下发。
			// 串口层做命令-应答门控 + 摇杆最新值合并（中间位置不重复下发）。
			side := m.Side
			if side != "L" && side != "R" {
				side = "L"
			}
			s.joyMu.Lock()
			if side == "L" {
				s.joyL[0], s.joyL[1] = clampF(m.X), clampF(m.Y)
			} else {
				s.joyR[0], s.joyR[1] = clampF(m.X), clampF(m.Y)
			}
			lx, ly, rx, ry := s.joyL[0], s.joyL[1], s.joyR[0], s.joyR[1]
			s.joyMu.Unlock()
			// 四轴全部在动作死区内（≤10°）时整帧跳过：不下发任何指令
			if !protocol.JoystickHasCommand(lx, ly, rx, ry, s.axisMap) {
				continue
			}
			cmd := protocol.JoystickToJOYDual(lx, ly, rx, ry, s.axisMap)
			if err := s.serial.WriteLine(cmd); err != nil {
				s.sendErr(c, "串口未连接，无法下发")
			}
		case "cmd":
			ok, normalized, verr := protocol.Validate(m.C)
			if !ok {
				s.sendErr(c, verr.Error())
				continue
			}
			log.Printf("[web] 指令 %q -> 下发(归一化) %q", m.C, normalized)
			if err := s.serial.WriteLine(normalized); err != nil {
				s.sendErr(c, "串口未连接，无法下发")
			}
		default:
			s.sendErr(c, "未知消息类型: "+m.T)
		}
	}
	log.Printf("[web] WebSocket 客户端断开: %s", r.RemoteAddr)
}

// wsWriter 把设备回显（经 hub）封装成 JSON 推送给浏览器，
// 同时解析角度（合并进最近已知值）供网页舵机角度显示。
func (s *Server) wsWriter(c *wsClient, done chan struct{}) {
	for {
		select {
		case <-done:
			return
		case line := <-c.out:
			msg := serverMsg{T: "serial", Line: line}
			if m := parseAngles(line); len(m) > 0 {
				msg.Angles = s.mergeAngles(m)
			}
			if data, err := json.Marshal(msg); err == nil {
				c.conn.WriteMessage(string(data))
			}
		}
	}
}

// statusPollInterval STATUS 轮询周期。2Hz 足以让角度显示"实时"，
// 且经 ACK 门控与 JOY 帧串行化，几乎不占用链路带宽。
const statusPollInterval = 500 * time.Millisecond

// statusPoller 周期下发 STATUS，让网页舵机角度常显且始终最新。
// 仅在有 WS 客户端观看且串口在线时发送，无人观看零串口流量。
func (s *Server) statusPoller() {
	t := time.NewTicker(statusPollInterval)
	defer t.Stop()
	for range t.C {
		s.mu.Lock()
		n := len(s.clients)
		s.mu.Unlock()
		if n == 0 {
			continue
		}
		if connected, _, _, _ := s.serial.Status(); !connected {
			continue
		}
		_ = s.serial.WriteLine("STATUS") // 队列满时静默跳过本轮
	}
}

// mergeAngles 把本条回显里出现的舵机角度合并进最近已知值。
// 单轴回显（如 "OK SET S9=120"）只更新对应轴，不会把其它轴清零。
// 返回完整快照；尚有轴从未见过回显时返回 nil（避免 UI 把未知轴显示成 0）。
func (s *Server) mergeAngles(m map[int]int) *armAngles {
	s.anglesMu.Lock()
	defer s.anglesMu.Unlock()
	for id, v := range m {
		if id < 6 || id > 9 {
			continue
		}
		s.lastAngles[id-6] = v
		s.haveAngles[id-6] = true
	}
	return s.snapshotLocked()
}

// anglesSnapshot 返回当前完整角度快照（未凑齐四轴时返回 nil）。
func (s *Server) anglesSnapshot() *armAngles {
	s.anglesMu.Lock()
	defer s.anglesMu.Unlock()
	return s.snapshotLocked()
}

func (s *Server) snapshotLocked() *armAngles {
	for i := range s.haveAngles {
		if !s.haveAngles[i] {
			return nil
		}
	}
	return &armAngles{S6: s.lastAngles[0], S7: s.lastAngles[1], S8: s.lastAngles[2], S9: s.lastAngles[3], OK: true}
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

// parseAngles 从回显行中提取出现的舵机角度（可能只有部分轴）。
// 支持：
//   - "OK JOY S6=89 S7=89 S8=89 S9=89"
//   - "STATUS S6=90(H) S7=90(H) S8=100(H) S9=76(H)"（带 H/L 模式后缀）
//   - 单舵机 "OK SET S9=120" / "S8=75"
//
// 返回 id->角度 映射；调用方（mergeAngles）负责与最近已知值合并。
func parseAngles(line string) map[int]int {
	m := map[int]int{}
	for _, kv := range reStatus.FindAllStringSubmatch(line, -1) {
		s6, _ := strconv.Atoi(kv[1])
		s7, _ := strconv.Atoi(kv[2])
		s8, _ := strconv.Atoi(kv[3])
		s9, _ := strconv.Atoi(kv[4])
		m[6], m[7], m[8], m[9] = s6, s7, s8, s9
	}
	for _, kv := range reSingle.FindAllStringSubmatch(line, -1) {
		id, _ := strconv.Atoi(kv[1])
		v, _ := strconv.Atoi(kv[2])
		m[id] = v
	}
	return m
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
