package web

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
)

const wsMagic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

// Conn 是极简 RFC6455 WebSocket 服务端连接（仅标准库实现）。
// 支持：文本帧、ping/pong、close；客户端帧按规范要求做掩码还原。
// 不处理消息分片（浏览器对小块 JSON 不会分片，足够本场景使用）。
type Conn struct {
	conn net.Conn
	r    *bufio.Reader
	wmu  sync.Mutex
}

// Upgrade 完成 WebSocket 握手并返回 *Conn。调用方需自行处理路由/鉴权。
func Upgrade(w http.ResponseWriter, r *http.Request) (*Conn, error) {
	if r.Header.Get("Upgrade") != "websocket" {
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
	return &Conn{conn: netConn, r: bufio.NewReader(netConn)}, nil
}

// ReadMessage 读取一条文本消息（阻塞）。遇到 ping 自动回 pong，close 返回错误。
func (c *Conn) ReadMessage() (string, error) {
	for {
		opcode, payload, err := c.readFrame()
		if err != nil {
			return "", err
		}
		switch opcode {
		case 0x1: // text
			return string(payload), nil
		case 0x2: // binary（本场景不使用，按文本处理）
			return string(payload), nil
		case 0x9: // ping -> pong
			_ = c.writeFrame(0xA, payload)
		case 0xA: // pong -> 忽略
		case 0x8: // close
			return "", errors.New("websocket closed by peer")
		}
	}
}

// WriteMessage 发送一条文本消息。
func (c *Conn) WriteMessage(s string) error {
	return c.writeFrame(0x1, []byte(s))
}

// Close 关闭底层连接。
func (c *Conn) Close() error {
	return c.conn.Close()
}

func (c *Conn) readFrame() (byte, []byte, error) {
	var b [2]byte
	if _, err := io.ReadFull(c.r, b[:]); err != nil {
		return 0, nil, err
	}
	opcode := b[0] & 0x0f
	masked := b[1]&0x80 != 0
	length := int(b[1] & 0x7f)
	if length == 126 {
		var ext [2]byte
		if _, err := io.ReadFull(c.r, ext[:]); err != nil {
			return 0, nil, err
		}
		length = int(binary.BigEndian.Uint16(ext[:]))
	} else if length == 127 {
		var ext [8]byte
		if _, err := io.ReadFull(c.r, ext[:]); err != nil {
			return 0, nil, err
		}
		length = int(binary.BigEndian.Uint64(ext[:]))
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
	if length < 126 {
		hdr = []byte{0x80 | opcode, byte(length)}
	} else if length < 65536 {
		hdr = []byte{0x80 | opcode, 126, byte(length >> 8), byte(length)}
	} else {
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
