/**
 * FakeSocket —— `SocketLike` 的测试实现。
 *
 * 真实浏览器 WebSocket 无法在 node 里制造这些场景：连上 40ms 后毫无征兆地断开、
 * 心跳发出去永远不回 pong、重连时前一个 socket 的回调才姗姗来迟。
 * 用本类可以**精确驱动**每一步，配合 `FakeTimer` 让时序断言完全确定。
 */
import {
  SOCKET_CLOSED,
  SOCKET_CONNECTING,
  SOCKET_OPEN,
  type SocketFactory,
  type SocketLike,
} from '@robot/index';

export class FakeSocket implements SocketLike {
  readyState: number = SOCKET_CONNECTING;
  onopen: (() => void) | null = null;
  onmessage: ((data: string) => void) | null = null;
  onclose: ((code: number, reason: string) => void) | null = null;
  onerror: ((err: unknown) => void) | null = null;

  /** 已发出的所有原始帧 */
  readonly sent: string[] = [];
  /** close() 的调用记录（null = 未调用） */
  closeCall: { code?: number; reason?: string } | null = null;

  send(data: string): void {
    if (this.readyState !== SOCKET_OPEN) {
      throw new Error(`[FakeSocket] 未打开就 send（readyState=${this.readyState}）`);
    }
    this.sent.push(data);
  }

  close(code?: number, reason?: string): void {
    this.closeCall = { code, reason };
    if (this.readyState === SOCKET_CLOSED) return;
    this.readyState = SOCKET_CLOSED;
    // 真实 WebSocket 的 close 会异步触发 onclose；同步触发让测试更好写，
    // 而 WebSocketTransport 已用 `this.socket !== socket` 守卫挡住重复回调。
    this.onclose?.(code ?? 1000, reason ?? '');
  }

  // -------------------------------------------------------------------------
  // 测试驱动接口
  // -------------------------------------------------------------------------

  /** 完成握手 */
  open(): void {
    this.readyState = SOCKET_OPEN;
    this.onopen?.();
  }

  /** 投递一条服务端消息（对象或原始字符串） */
  deliver(msg: unknown): void {
    if (this.readyState !== SOCKET_OPEN) {
      throw new Error('[FakeSocket] 连接未打开，无法投递消息');
    }
    this.onmessage?.(typeof msg === 'string' ? msg : JSON.stringify(msg));
  }

  /** 模拟连接被对端/网络掐断（不经过 close()） */
  drop(code = 1006, reason = ''): void {
    this.readyState = SOCKET_CLOSED;
    this.onclose?.(code, reason);
  }

  /** 解析出已发送的 JSON 帧 */
  frames(): Array<Record<string, unknown>> {
    return this.sent.map((raw) => JSON.parse(raw) as Record<string, unknown>);
  }

  /** 已发送帧里指定 type 的那些 */
  framesOfType(type: string): Array<Record<string, unknown>> {
    return this.frames().filter((f) => f.type === type);
  }
}

/** 工厂：记录每次创建的 socket，便于测试逐个驱动 */
export class FakeSocketFactory {
  readonly sockets: FakeSocket[] = [];
  readonly urls: string[] = [];

  readonly create: SocketFactory = (url: string): SocketLike => {
    this.urls.push(url);
    const socket = new FakeSocket();
    this.sockets.push(socket);
    return socket;
  };

  /** 最近创建的 socket */
  get last(): FakeSocket {
    const socket = this.sockets[this.sockets.length - 1];
    if (!socket) throw new Error('[FakeSocketFactory] 尚未创建任何 socket');
    return socket;
  }

  get count(): number {
    return this.sockets.length;
  }
}
