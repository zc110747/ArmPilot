/**
 * 连接面板（Phase 7 起，Phase 8 扩展）。
 *
 * 面板存在的意义不是"开关"，而是让**传输的真实约束可见**：
 *
 * - **MockTransport**：拖动时 Actual 会滞后、松手后收敛；把丢帧拉满能看到
 *   "机械臂照常运动但上位机收不到状态"。这些是 Phase 11 真机误差链路的前置演练。
 * - **WebSocketTransport**：连上后端的关节级服务（`armpilot-backend`），
 *   命令经 Go → 链路末端（sim / 串口）→ 反算关节角 → 回推 Actual。
 *   面板显示心跳 RTT、重连次数、后端模型一致性告警。
 *
 * 两种传输共用同一块 UI 与同一套节流/回环逻辑（`transportBridge`），
 * 这正是"接口不变、实现可替换"的体现。
 */
import { useState } from 'react';
import {
  DEFAULT_MOCK_TUNING,
  type MockTransportTuning,
  type WebSocketTransportStats,
} from '@robot/index';
import { useRobotStore } from '@/store/robotStore';
import {
  connectMockTransport,
  connectWebSocketTransport,
  disconnectTransport,
  tuneTransport,
} from '@/store/transportBridge';
import { getDefaultWsUrl } from './wsUrl';

type TransportChoice = 'mock' | 'websocket';

const DEFAULT_WS_URL = getDefaultWsUrl();

export function ConnectionControl() {
  const connection = useRobotStore((s) => s.connection);
  const connectionLabel = useRobotStore((s) => s.connectionLabel);
  const transportKind = useRobotStore((s) => s.transportKind);
  const stats = useRobotStore((s) => s.transportStats);
  const pushLog = useRobotStore((s) => s.pushLog);

  const [mode, setMode] = useState<TransportChoice>('mock');
  const [wsUrl, setWsUrl] = useState<string>(DEFAULT_WS_URL);
  const [tuning, setTuning] = useState<MockTransportTuning>({ ...DEFAULT_MOCK_TUNING });
  const [busy, setBusy] = useState(false);

  const connected = connection === 'connected';
  const showMockTuning = mode === 'mock' && transportKind !== 'websocket';
  const wsStats: WebSocketTransportStats | null =
    stats !== null && stats.kind === 'websocket' ? (stats as WebSocketTransportStats) : null;

  // 调参即刻生效（未连接时 `tuneTransport` 是 no-op，值留到连接时带上）
  const update = (patch: Partial<MockTransportTuning>) => {
    setTuning((prev) => ({ ...prev, ...patch }));
    tuneTransport(patch);
  };

  const handleConnect = async () => {
    setBusy(true);
    try {
      if (mode === 'mock') {
        await connectMockTransport({ tuning });
        pushLog(
          'sys',
          `已连接 MockTransport（${tuning.maxSpeedDegPerSec}°/s · ${tuning.latencyMs}ms）`,
        );
      } else {
        await connectWebSocketTransport({ url: wsUrl });
        pushLog('sys', `已发起 WebSocket 连接：${wsUrl}`);
      }
    } catch (err) {
      pushLog('sys', `连接失败：${err instanceof Error ? err.message : String(err)}`);
    } finally {
      setBusy(false);
    }
  };

  const handleDisconnect = async () => {
    setBusy(true);
    try {
      await disconnectTransport();
      pushLog('sys', '已断开传输，回到纯仿真');
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="card">
      <h2>连接 · Transport</h2>

      <div className="kv">
        <span>Connection</span>
        <span>
          <span className={`badge ${connected ? 'ok' : 'bad'}`}>
            <i className="dot" />
            {connected ? 'Connected' : 'Disconnected'}
          </span>
        </span>
      </div>
      <div className="kv">
        <span />
        <span className="dim" style={{ fontSize: 11 }}>
          {connectionLabel}
        </span>
      </div>

      <div className="kv">
        <span>传输方式</span>
        <span>
          <label style={{ marginRight: 10 }}>
            <input
              type="radio"
              name="transport-mode"
              value="mock"
              checked={mode === 'mock'}
              disabled={connected}
              onChange={() => setMode('mock')}
            />{' '}
            <span className="dim">Mock</span>
          </label>
          <label>
            <input
              type="radio"
              name="transport-mode"
              value="websocket"
              checked={mode === 'websocket'}
              disabled={connected}
              onChange={() => setMode('websocket')}
            />{' '}
            <span className="dim">WebSocket</span>
          </label>
        </span>
      </div>

      {mode === 'websocket' ? (
        <div className="kv">
          <span>后端地址</span>
          <span>
            <input
              type="text"
              className="text-input"
              data-testid="ws-url"
              value={wsUrl}
              disabled={connected}
              spellCheck={false}
              onChange={(event) => setWsUrl(event.target.value)}
              style={{ width: '100%', fontSize: 11 }}
            />
          </span>
        </div>
      ) : null}

      <div className="btn-row">
        <button type="button" disabled={busy || connected} onClick={() => void handleConnect()}>
          {mode === 'mock' ? 'Connect Mock' : 'Connect WS'}
        </button>
        <button
          type="button"
          disabled={busy || !connected}
          onClick={() => void handleDisconnect()}
        >
          Disconnect
        </button>
      </div>

      {showMockTuning ? (
        <>
          <div className="slider-row">
            <div className="label">
              <span className="name">舵机角速度</span>
              <span className="val">
                {tuning.maxSpeedDegPerSec > 0 ? tuning.maxSpeedDegPerSec : 0}
                <span className="dim"> °/s</span>
              </span>
            </div>
            <input
              type="range"
              min={0}
              max={600}
              step={30}
              value={tuning.maxSpeedDegPerSec}
              onChange={(event) => update({ maxSpeedDegPerSec: Number(event.target.value) })}
            />
            <div className="limits">0 = 瞬时到位 · 真实 MG90S 带载约 240°/s</div>
          </div>

          <div className="slider-row">
            <div className="label">
              <span className="name">传输延迟</span>
              <span className="val">
                {tuning.latencyMs}
                <span className="dim"> ms</span>
              </span>
            </div>
            <input
              type="range"
              min={0}
              max={120}
              step={5}
              value={tuning.latencyMs}
              onChange={(event) => update({ latencyMs: Number(event.target.value) })}
            />
            <div className="limits">命令送达前 Actual 不动</div>
          </div>

          <div className="slider-row">
            <div className="label">
              <span className="name">回推丢帧</span>
              <span className="val">
                {(tuning.dropRate * 100).toFixed(0)}
                <span className="dim"> %</span>
              </span>
            </div>
            <input
              type="range"
              min={0}
              max={0.8}
              step={0.05}
              value={tuning.dropRate}
              onChange={(event) => update({ dropRate: Number(event.target.value) })}
            />
            <div className="limits">丢帧只影响上报，机械臂照常运动</div>
          </div>

          <div className="kv">
            <span>限位校验</span>
            <span>
              <input
                type="checkbox"
                checked={tuning.enforceLimits}
                onChange={(event) => update({ enforceLimits: event.target.checked })}
              />{' '}
              <span className="dim">越界回 ERR JOINT 且不动</span>
            </span>
          </div>
        </>
      ) : null}

      {stats === null ? (
        <div className="dim" style={{ marginTop: 8 }}>
          未连接：命令不经过传输层，Actual 恒等于 Command。
        </div>
      ) : (
        <>
          <table className="grid" style={{ marginTop: 8 }}>
            <thead>
              <tr>
                <th>指标</th>
                <th>值</th>
              </tr>
            </thead>
            <tbody>
              <tr>
                <td>已发送 / 已接收</td>
                <td data-testid="stat-frames">
                  {stats.sent} / {stats.received}
                </td>
              </tr>
              <tr>
                <td>丢帧 / 拒绝</td>
                <td>
                  {stats.dropped} / {stats.rejected}
                </td>
              </tr>
              <tr>
                <td>跟踪误差</td>
                <td data-testid="stat-lag">{stats.lagDeg.toFixed(2)}°</td>
              </tr>
              <tr>
                <td>运动状态</td>
                <td>{stats.moving ? '正在逼近目标' : '已到位'}</td>
              </tr>
              {wsStats !== null ? (
                <>
                  <tr>
                    <td>心跳 RTT</td>
                    <td data-testid="stat-rtt">
                      {wsStats.rttMs === null ? '—' : `${wsStats.rttMs} ms`}
                    </td>
                  </tr>
                  <tr>
                    <td>重连次数</td>
                    <td>{wsStats.reconnects}</td>
                  </tr>
                  <tr>
                    <td>链路末端</td>
                    <td>{wsStats.device ?? '—'}</td>
                  </tr>
                </>
              ) : null}
            </tbody>
          </table>

          {wsStats?.modelMismatch ? (
            <div className="notes" style={{ marginTop: 6 }}>
              ⚠️ 模型/标定不一致：{wsStats.modelMismatch}
            </div>
          ) : null}

          <div className="notes" style={{ marginTop: 6 }}>
            {wsStats !== null
              ? 'Actual 由后端从舵机实际角**反算**后回推：走真实路径，标定表写错会立刻表现为误差不收敛。'
              : '拖动关节时 Actual 落后于 Command 是**真实行为**：舵机不可能瞬间到位。松手后误差收敛到 0。'}
          </div>
        </>
      )}
    </div>
  );
}
