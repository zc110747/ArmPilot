// package hub 是串口回显的广播总线：所有订阅者（WebSocket 客户端、TCP 客户端）
// 注册后都会收到设备回显。命令下发不经由 hub（直接写串口），hub 只负责“设备->多端”。
package hub

import (
	"sync"
)

// Client 是一个能接收一条串口回显的订阅者。
type Client interface {
	// Send 推送一条设备回显（已不含换行）。实现需自身保证非阻塞/串行化。
	Send(line string)
}

type Hub struct {
	mu      sync.Mutex
	clients map[Client]struct{}
}

func New() *Hub {
	return &Hub{clients: make(map[Client]struct{})}
}

func (h *Hub) Register(c Client) {
	h.mu.Lock()
	defer h.mu.Unlock()
	h.clients[c] = struct{}{}
}

func (h *Hub) Unregister(c Client) {
	h.mu.Lock()
	defer h.mu.Unlock()
	delete(h.clients, c)
}

// Broadcast 把一行设备回显广播给所有订阅者。
func (h *Hub) Broadcast(line string) {
	h.mu.Lock()
	defer h.mu.Unlock()
	for c := range h.clients {
		c.Send(line)
	}
}
