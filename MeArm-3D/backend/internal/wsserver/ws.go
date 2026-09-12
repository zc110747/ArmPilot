// Package wsserver 提供关节级 HTTP + WebSocket 端点。
//
// 本文件是极简 RFC6455 服务端实现（仅标准库）。与 MeArm-RemoteControl 的版本同源，
// 但此处额外需要 **读超时**：服务端要能发现"半死连接"（浏览器崩溃、网线拔掉），
// 否则控制器会一直以为有人连着，且无法区分"客户端静默"与"链路死了"。
//
// 支持：文本帧、ping/pong、close、读超时。
// 不处理消息分片（浏览器对小块 JSON 不会分片，本场景足够）。
package wsserver

import (
	"bufio"
	"crypto/sha1"
	"encoding/base64"
	"encoding/binary"
	"errors"
	"io"
	"net"
	"net/http"
	"sync"
	"time"
)

const wsMagic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

// Conn 是一条已升级的 WebSocket 连接。
type Conn struct {
	conn net.Conn
	r    *bufio.Reader
	wmu  sync.Mutex

	actMu  sync.Mutex
	lastAt time.Time
}

// touch 记录一次有效的读活动（任何帧，含传输层 pong）。
//
// 为什么需要它：`ReadMessage` 内部会把 pong 吞掉并继续循环，外部观察不到；
// 若只看"业务消息"判断存活，一个只回 pong 的健康客户端会被误判为死连接。
// 因此在帧层记录活动时间，心跳看门狗据此判定。
func (c *Conn) touch() {
	c.actMu.Lock()
	c.lastAt = time.Now()
	c.actMu.Unlock()
}

// LastActivity 返回最后一次收到任何帧的时间。
func (c *Conn) LastActivity() time.Time {
	c.actMu.Lock()
	defer c.actMu.Unlock()
	return c.lastAt
}

// Upgrade 完成 WebSocket 握手。
func Upgrade(w http.ResponseWriter, r *http.Request) (*Conn, error) {
	if !headerContainsToken(r.Header.Get("Upgrade"), "websocket") {
		return nil, errors.New("非 WebSocket 请求")
	}
	key := r.Header.Get("Sec-WebSocket-Key")
	if key == "" {
		return nil, errors.New("缺少 Sec-WebSocket-Key")
	}
	hj, ok := w.(http.Hijacker)
	if !ok {
		return nil, errors.New("服务器不支持 Hijack")
	}
	netConn, _, err := hj.Hijack()
	if err != nil {
		return nil, err
	}
	sum := sha1.Sum([]byte(key + wsMagic))
	accept := base64.StdEncoding.EncodeToString(sum[:])
	handshake := "HTTP/1.1 101 Switching Protocols\r\n" +
		"Upgrade: websocket\r\n" +
		"Connection: Upgrade\r\n" +
		"Sec-WebSocket-Accept: " + accept + "\r\n\r\n"
	if _, err := netConn.Write([]byte(handshake)); err != nil {
		netConn.Close()
		return nil, err
	}
	return &Conn{conn: netConn, r: bufio.NewReader(netConn), lastAt: time.Now()}, nil
}

func headerContainsToken(header, token string) bool {
	lower := toLower(header)
	return indexOf(lower, toLower(token)) >= 0
}

// SetReadDeadline 设置下一次读取的超时（零值 = 不超时）。
func (c *Conn) SetReadDeadline(d time.Duration) error {
	if d <= 0 {
		return c.conn.SetReadDeadline(time.Time{})
	}
	return c.conn.SetReadDeadline(time.Now().Add(d))
}

// ReadMessage 读取一条文本消息（阻塞）。ping 自动回 pong；close 返回错误。
func (c *Conn) ReadMessage() (string, error) {
	for {
		opcode, payload, err := c.readFrame()
		if err != nil {
			return "", err
		}
		switch opcode {
		case 0x1, 0x2: // text / binary（本场景 binary 也按文本处理）
			return string(payload), nil
		case 0x9: // ping → pong
			_ = c.writeFrame(0xA, payload)
		case 0xA: // pong → 忽略（读超时在 SetReadDeadline 层面刷新）
		case 0x8: // close
			_ = c.writeFrame(0x8, nil)
			return "", errors.New("websocket closed by peer")
		}
	}
}

// WriteMessage 发送一条文本消息。
func (c *Conn) WriteMessage(s string) error { return c.writeFrame(0x1, []byte(s)) }

// Ping 发送一个 ping 控制帧。
func (c *Conn) Ping() error { return c.writeFrame(0x9, nil) }

// Close 关闭底层连接。
func (c *Conn) Close() error { return c.conn.Close() }

func (c *Conn) readFrame() (byte, []byte, error) {
	var b [2]byte
	if _, err := io.ReadFull(c.r, b[:]); err != nil {
		return 0, nil, err
	}
	c.touch()
	opcode := b[0] & 0x0f
	masked := b[1]&0x80 != 0
	length := int(b[1] & 0x7f)
	switch length {
	case 126:
		var ext [2]byte
		if _, err := io.ReadFull(c.r, ext[:]); err != nil {
			return 0, nil, err
		}
		length = int(binary.BigEndian.Uint16(ext[:]))
	case 127:
		var ext [8]byte
		if _, err := io.ReadFull(c.r, ext[:]); err != nil {
			return 0, nil, err
		}
		length = int(binary.BigEndian.Uint64(ext[:]))
	}
	// 上限保护：本项目消息均为小 JSON，超过 1MB 视为异常，直接断开
	if length > 1<<20 {
		return 0, nil, errors.New("帧过大（>1MB）")
	}
	var mask [4]byte
	if masked {
		if _, err := io.ReadFull(c.r, mask[:]); err != nil {
			return 0, nil, err
		}
	}
	payload := make([]byte, length)
	if length > 0 {
		if _, err := io.ReadFull(c.r, payload); err != nil {
			return 0, nil, err
		}
		if masked {
			for i := range payload {
				payload[i] ^= mask[i%4]
			}
		}
	}
	return opcode, payload, nil
}

func (c *Conn) writeFrame(opcode byte, data []byte) error {
	c.wmu.Lock()
	defer c.wmu.Unlock()
	length := len(data)
	var hdr []byte
	switch {
	case length < 126:
		hdr = []byte{0x80 | opcode, byte(length)}
	case length < 65536:
		hdr = []byte{0x80 | opcode, 126, byte(length >> 8), byte(length)}
	default:
		hdr = []byte{0x80 | opcode, 127}
		buf := make([]byte, 8)
		binary.BigEndian.PutUint64(buf, uint64(length))
		hdr = append(hdr, buf...)
	}
	if _, err := c.conn.Write(hdr); err != nil {
		return err
	}
	if length > 0 {
		if _, err := c.conn.Write(data); err != nil {
			return err
		}
	}
	return nil
}

// ---- 小工具（避免为两处字符串比较引入 strings 包的各种语义） ----

func toLower(s string) string {
	b := []byte(s)
	for i := range b {
		if b[i] >= 'A' && b[i] <= 'Z' {
			b[i] += 'a' - 'A'
		}
	}
	return string(b)
}

func indexOf(s, sub string) int {
	if len(sub) == 0 {
		return 0
	}
	if len(sub) > len(s) {
		return -1
	}
	for i := 0; i+len(sub) <= len(s); i++ {
		if s[i:i+len(sub)] == sub {
			return i
		}
	}
	return -1
}
