/**
 * transportBridge —— 把 Zustand store 与 `RobotTransport` 接起来（Phase 7 起）。
 *
 * Phase 8 起它**不再绑定 MockTransport**：任何实现 `RobotTransport` 的传输
 * （Mock / WebSocket / Phase 9 的 Serial）都能挂上来。它承担四件事，
 * 每件都有一个**必须做对**的细节：
 *
 * 1. **命令下发 + 尾沿节流**
 *    所有改 `commandJoints` 的路径（滑杆 / HOME / ZERO / 拖动 moveTo）统一在这里
 *    汇流。拖动时每帧都会变，若逐帧下发，60fps 就是每秒 60 条命令 ——
 *    Phase 9 串口（115200、ACK 门控）下必然打爆。故取 **尾沿合并（trailing edge）**：
 *    间隔内只保留最新一帧，松手时由 trailing 定时器补发最后一帧，不会停在半路。
 *
 * 2. **回推落地（回环打破）**
 *    `onState` 回调**只写 `actualJoints`**，绝不回写 `commandJoints`。
 *    若两边互相写，就形成 `命令 → 状态 → 命令` 的无限回环（spec §十九）。
 *
 * 3. **状态登记 + 统计刷新**
 *    `onStatus` 映射到 store 的 connection；`poll()` 每 `STATS_POLL_MS` 拉一次统计，
 *    **脏检查后**才写 store（避免收敛后每 200ms 无谓触发 React 重渲染）。
 *    发送/接收日志按 `LOG_FLUSH_MS` 合并成一条，避免拖动时刷屏。
 *
 * 4. **重连后补齐命令**（Phase 8）
 *    断线期间用户可能已经拖到别处。重连成功时把当前 `commandJoints` 补发一次，
 *    否则虚拟臂与实际臂会永久错开 —— 而 UI 上看不出任何异常。
 */
import {
  MockTransport,
  WebSocketTransport,
  realTimer,
  type JointState,
  type MockTransportStats,
  type MockTransportTuning,
  type RobotState,
  type RobotTransport,
  type TransportStats,
  type TransportStatusDetail,
  type TimerLike,
  type WebSocketTransportOptions,
} from '@robot/index';
import { useRobotStore } from './robotStore';

/** 命令最小发送间隔（ms）：约 30Hz，兼顾跟手与链路压力 */
export const MIN_SEND_INTERVAL_MS = 33;
/** 统计刷新周期（ms） */
export const STATS_POLL_MS = 200;
/** 发送/接收日志的合并周期（ms） */
export const LOG_FLUSH_MS = 500;
/** 未接入传输时的统一文案 */
export const NO_TRANSPORT_LABEL = '未接入（可在 Connection 面板连接）';

export interface TransportBridgeOptions {
  /**
   * 显式注入传输实现。缺省时建立 `MockTransport`（保持 Phase 7 行为不变）。
   * 测试可借此注入假传输。
   */
  transport?: RobotTransport;
  /** 可注入时钟；与传输实现共用同一个，测试才能确定性推进 */
  timer?: TimerLike;
  /** Mock 调参初始值 */
  tuning?: Partial<MockTransportTuning>;
  /** 可复现的随机源（丢帧） */
  random?: () => number;
  /** 仿真 tick 周期（ms） */
  tickMs?: number;
  /** 命令最小发送间隔覆盖值 */
  minimumSendIntervalMs?: number;
}

/** 状态面板上的一行描述（按传输类型给出最有信息量的那几项） */
export function describeTransportStats(stats: TransportStats): string {
  switch (stats.kind) {
    case 'mock': {
      const speed =
        Number.isFinite(stats.maxSpeedDegPerSec ?? 0) && (stats.maxSpeedDegPerSec ?? 0) > 0
          ? `${stats.maxSpeedDegPerSec}°/s`
          : '瞬时到位';
      const drop = ((stats.dropRate ?? 0) * 100).toFixed(0);
      return `MockTransport · ${speed} · ${stats.latencyMs ?? 0}ms 延迟 · 丢帧 ${drop}%`;
    }
    case 'websocket': {
      const rtt = stats.rttMs === null || stats.rttMs === undefined ? '—' : `${stats.rttMs}ms`;
      return `WebSocket · 心跳 RTT ${rtt} · 重连 ${stats.reconnects ?? 0} 次`;
    }
    default:
      return stats.kind;
  }
}

export class TransportBridge {
  private readonly timer: TimerLike;
  private readonly transport: RobotTransport;
  /** 非 null 表示当前挂的是 MockTransport（调参面板据此启用） */
  private readonly mock: MockTransport | null;
  private readonly minSendIntervalMs: number;

  /** 节流状态 */
  private pendingJoints: JointState | null = null;
  private lastSendAt = Number.NEGATIVE_INFINITY;
  private trailingHandle: number | null = null;

  private pollHandle: number | null = null;
  private unsubscribeStore: (() => void) | null = null;
  private unsubscribeState: (() => void) | null = null;
  private unsubscribeStatus: (() => void) | null = null;

  /** 待合并进日志的计数 */
  private txSince = 0;
  private rxSince = 0;
  private lastTxLogAt = Number.NEGATIVE_INFINITY;
  private lastRxLogAt = Number.NEGATIVE_INFINITY;

  private lastStatsKey = '';
  private disposed = false;
  /** 是否已经成功连接过一次（用于区分"首次连接"与"重连后补发命令"） */
  private hasConnectedOnce = false;

  constructor(options: TransportBridgeOptions = {}) {
    this.timer = options.timer ?? realTimer;
    this.minSendIntervalMs = options.minimumSendIntervalMs ?? MIN_SEND_INTERVAL_MS;

    if (options.transport) {
      this.transport = options.transport;
      this.mock = options.transport instanceof MockTransport ? options.transport : null;
    } else {
      const state = useRobotStore.getState();
      const mock = new MockTransport({
        model: state.model,
        timer: this.timer,
        // 初始实际位置与当前命令一致：连接不应让机械臂"跳"一下
        initialJoints: state.commandJoints,
        ...(options.tickMs !== undefined ? { tickMs: options.tickMs } : {}),
        ...(options.random !== undefined ? { random: options.random } : {}),
        ...(options.tuning ?? {}),
      });
      this.mock = mock;
      this.transport = mock;
    }
  }

  /** 当前传输类型（mock / websocket…） */
  kind(): string {
    return this.transport.kind;
  }

  async connect(): Promise<void> {
    // 先挂监听再 connect：否则 connect 期间发出的 status 事件会丢
    this.unsubscribeStatus = this.transport.onStatus((detail) => this.handleStatus(detail));
    this.unsubscribeState = this.transport.onState((state) => this.handleState(state));

    await this.transport.connect();

    // 命令变化 → 节流下发。比较的是 `commandJoints` 的对象引用，
    // 而 store 每次改命令都会生成新对象，所以不会漏事件。
    this.unsubscribeStore = useRobotStore.subscribe((state, prev) => {
      if (state.commandJoints !== prev.commandJoints) this.enqueue(state.commandJoints);
    });

    this.pollHandle = this.timer.setInterval(() => this.poll(), STATS_POLL_MS);
  }

  async dispose(): Promise<void> {
    if (this.disposed) return;
    this.disposed = true;

    if (this.trailingHandle !== null) {
      this.timer.clearTimeout(this.trailingHandle);
      this.trailingHandle = null;
    }
    if (this.pollHandle !== null) {
      this.timer.clearInterval(this.pollHandle);
      this.pollHandle = null;
    }
    this.unsubscribeStore?.();
    this.unsubscribeStore = null;
    this.unsubscribeState?.();
    this.unsubscribeState = null;
    this.unsubscribeStatus?.();
    this.unsubscribeStatus = null;

    await this.transport.disconnect();
    this.pendingJoints = null;
    this.hasConnectedOnce = false;

    // ⚠️ 这里必须**显式**复位，不能指望 transport.disconnect() 发出的 disconnected 事件：
    //    上面已经退订了 status 监听，事件根本没人接。早先的写法就是踩了这个坑 ——
    //    断开后状态灯仍显示 Connected、Actual 还挂着最后那个滞后值。
    this.store().setConnection(null, 'disconnected', NO_TRANSPORT_LABEL);
  }

  /** 运行期调参（面板滑杆）；非 Mock 传输下是 no-op */
  tune(patch: Partial<MockTransportTuning>): void {
    if (this.mock === null) return;
    this.mock.tune(patch);
    this.poll();
  }

  /** Mock 专有统计；非 Mock 返回 null（供调参面板回填滑杆） */
  mockStats(): MockTransportStats | null {
    return this.mock?.stats() ?? null;
  }

  /** 通用统计 */
  stats(): TransportStats | null {
    return this.transport.stats?.() ?? null;
  }

  // -------------------------------------------------------------------------
  // 内部
  // -------------------------------------------------------------------------

  private store() {
    return useRobotStore.getState();
  }

  /** 尾沿合并：间隔内只保留最新一帧 */
  private enqueue(joints: JointState): void {
    this.pendingJoints = joints;
    const now = this.timer.now();
    const elapsed = now - this.lastSendAt;

    if (elapsed >= this.minSendIntervalMs) {
      this.flush(now);
      return;
    }
    if (this.trailingHandle === null) {
      this.trailingHandle = this.timer.setTimeout(() => {
        this.trailingHandle = null;
        this.flush(this.timer.now());
      }, this.minSendIntervalMs - elapsed);
    }
  }

  private flush(now: number): void {
    const joints = this.pendingJoints;
    if (joints === null || this.disposed) return;
    this.pendingJoints = null;
    this.lastSendAt = now;
    this.txSince += 1;
    void this.transport.sendJointState(joints);
  }

  /** ⚠️ 回推只写 actualJoints —— 回写 command 即无限回环 */
  private handleState(state: RobotState): void {
    this.rxSince += 1;
    this.store().setActualJoints(state.joints);
  }

  private handleStatus(detail: TransportStatusDetail): void {
    const store = this.store();
    switch (detail.status) {
      case 'connecting':
        store.setConnection(
          this.transport.kind,
          'disconnected',
          `连接中…（${this.transport.kind}）`,
        );
        break;
      case 'connected': {
        store.setConnection(this.transport.kind, 'connected', this.describeCurrent());
        // ⚠️ 只在**重连**时补发（首次连接不补）：断线期间用户可能已改过命令，
        //    不补发会让虚拟臂与实际臂永久错开而 UI 看不出异常。
        //    首次连接若也补发，会无端占用一次节流窗口，改变 Phase 7 的时序契约。
        if (this.hasConnectedOnce) {
          this.enqueue(store.commandJoints);
        }
        this.hasConnectedOnce = true;
        break;
      }
      case 'disconnected':
        store.setConnection(null, 'disconnected', NO_TRANSPORT_LABEL);
        break;
      case 'error':
        // error 是**事件**而非连接终态：固件回 ERR 一行，链路仍然活着
        store.pushLog('in', detail.reason ?? '传输错误');
        break;
      default:
        break;
    }
  }

  private describeCurrent(): string {
    const stats = this.transport.stats?.();
    if (!stats) return `已连接（${this.transport.kind}）`;
    return describeTransportStats(stats);
  }

  /** 周期刷新统计与合并日志（脏检查 → 收敛后不再重渲染） */
  private poll(): void {
    if (this.disposed) return;
    const stats = this.transport.stats?.();
    if (!stats) {
      this.flushLogs();
      return;
    }
    const key = [
      stats.sent,
      stats.received,
      stats.dropped,
      stats.rejected,
      stats.moving ? 1 : 0,
      stats.lagDeg.toFixed(4),
      stats.maxSpeedDegPerSec ?? '',
      stats.latencyMs ?? '',
      stats.dropRate ?? '',
      stats.enforceLimits === undefined ? '' : stats.enforceLimits ? 1 : 0,
      stats.reconnects ?? '',
      stats.rttMs ?? '',
    ].join('|');

    if (key !== this.lastStatsKey) {
      this.lastStatsKey = key;
      const store = this.store();
      store.setTransportStats(stats);
      if (store.connection === 'connected') {
        store.setConnection(this.transport.kind, 'connected', describeTransportStats(stats));
      }
    }

    this.flushLogs();
  }

  private flushLogs(): void {
    const now = this.timer.now();
    const store = this.store();

    if (this.txSince > 0 && now - this.lastTxLogAt >= LOG_FLUSH_MS) {
      this.lastTxLogAt = now;
      store.pushLog('out', `joint_command ×${this.txSince} → ${this.transport.kind}`);
      this.txSince = 0;
    }
    if (this.rxSince > 0 && now - this.lastRxLogAt >= LOG_FLUSH_MS) {
      this.lastRxLogAt = now;
      store.pushLog('in', `joint_state ×${this.rxSince} ← ${this.transport.kind}`);
      this.rxSince = 0;
    }
  }
}

// ---------------------------------------------------------------------------
// 模块级单例 —— UI 只跟这几个函数打交道
// ---------------------------------------------------------------------------

let activeBridge: TransportBridge | null = null;

/** 连接 MockTransport（幂等：先释放旧的） */
export async function connectMockTransport(
  options: TransportBridgeOptions = {},
): Promise<TransportBridge> {
  await disconnectTransport();
  const bridge = new TransportBridge(options);
  activeBridge = bridge;
  await bridge.connect();
  return bridge;
}

/**
 * 连接后端关节级 WebSocket。
 *
 * `model` 由 store 提供（与前端渲染同一个 `RobotModel`），用于 FK 与 hello 一致性校验。
 */
export async function connectWebSocketTransport(
  options: Omit<WebSocketTransportOptions, 'model'> & { model?: WebSocketTransportOptions['model'] },
): Promise<TransportBridge> {
  await disconnectTransport();
  const state = useRobotStore.getState();
  const transport = new WebSocketTransport({ ...options, model: options.model ?? state.model });
  const bridge = new TransportBridge({ transport });
  activeBridge = bridge;
  await bridge.connect();
  return bridge;
}

/** 断开并释放当前传输 */
export async function disconnectTransport(): Promise<void> {
  const bridge = activeBridge;
  if (bridge === null) return;
  activeBridge = null;
  await bridge.dispose();
}

export function activeTransportBridge(): TransportBridge | null {
  return activeBridge;
}

/** 运行期调参（未连接或非 Mock 时是 no-op） */
export function tuneTransport(patch: Partial<MockTransportTuning>): void {
  activeBridge?.tune(patch);
}
