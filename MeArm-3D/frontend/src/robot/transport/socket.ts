/**
 * SocketLike —— WebSocket 的最小抽象（与 `timer.ts` 的 `TimerLike` 同一思路）。
 *
 * 为什么不让 `WebSocketTransport` 直接用全局 `WebSocket`：
 *   - 单元测试无法在 node 环境里造出真实握手、更无法精确制造"连上 40ms 后
 *     毫无征兆地断开""心跳发出去但永远不回 pong"这类时序场景；
 *   - 注入一个假 socket 后，**重连退避、心跳超时、乱序/坏消息丢弃**这些
 *     真正容易出错的逻辑才能被确定性地验证（配合 `FakeTimer` 零真实等待）。
 *
 * 生产实现 `createWebSocketSocket` 只是把 DOM 事件转成这四个回调字段。
 */

/** WebSocket.readyState 常量（避免依赖 DOM 全局常量在 node 下不存在） */
export const SOCKET_CONNECTING = 0;
export const SOCKET_OPEN = 1;
export const SOCKET_CLOSING = 2;
export const SOCKET_CLOSED = 3;

export interface SocketLike {
  readonly readyState: number;
  send(data: string): void;
  close(code?: number, reason?: string): void;
  onopen: (() => void) | null;
  onmessage: ((data: string) => void) | null;
  onclose: ((code: number, reason: string) => void) | null;
  onerror: ((err: unknown) => void) | null;
}

export type SocketFactory = (url: string) => SocketLike;

/** 生产用：包一层浏览器原生 WebSocket */
export function createWebSocketSocket(url: string): SocketLike {
  const ws = new WebSocket(url);
  const sock: SocketLike = {
    get readyState() {
      return ws.readyState;
    },
    send: (data) => ws.send(data),
    close: (code, reason) => ws.close(code, reason),
    onopen: null,
    onmessage: null,
    onclose: null,
    onerror: null,
  };
  ws.onopen = () => sock.onopen?.();
  ws.onmessage = (event) => {
    const data = event.data;
    sock.onmessage?.(typeof data === 'string' ? data : String(data));
  };
  ws.onclose = (event) => sock.onclose?.(event.code, event.reason ?? '');
  // 浏览器出于安全考虑不暴露具体错误内容，故只给一个可读的占位错误
  ws.onerror = () => sock.onerror?.(new Error('WebSocket 传输错误'));
  return sock;
}

/** 生产工厂 */
export const webSocketFactory: SocketFactory = (url) => createWebSocketSocket(url);
