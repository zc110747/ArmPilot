/**
 * 链路误差语义（Phase 11 · 误差反馈面板的纯逻辑层）。
 *
 * 为什么单独成模块
 * ----------------
 * 面板要回答的是「Actual 到底追上 Command 没有」，而"追上没有"**不能只看瞬时值**：
 *   - 拖动过程中误差非零是**正常**的（舵机有限角速度：命令先到、物理后到）
 *   - 命令停下后误差**仍不收敛**才是异常（卡死 / 失步 / 限位截断）
 * 区分这两者需要「趋势 + 命令已静止多久」，是纯逻辑、与 React 无关 ——
 * 单独放这里是为了能在 `environment: 'node'` 下直接单测（本项目无 jsdom）。
 *
 * ⚠️ 为什么不看 `TransportStats.moving`
 * ------------------------------------
 * 实测 `WebSocketTransport.stats()` 里 `moving = lagDeg > arrivedEps()` ——
 * 它**由 lag 推导**，与 lag 同义。拿它判"是否在追"等于把 `lag > tol` 重写一遍，
 * 永远得不到"卡死"这个结论。判据只能自己从**时间序列**里得出。
 */

/** 视为"已到位"的关节误差阈值（deg）。0.5° ≈ 舵机一个整数度的量级 */
export const ERROR_TOL_DEG = 0.5;

/** 命令静止后仍超差多久，才**允许**判"卡死"（ms）：留出物理到位与回推时延 */
export const STALL_HOLD_MS = 700;

/** 趋势判定里"基本没变"的死区（deg），吸收回推抖动量级 */
export const TREND_EPS_DEG = 0.05;

/** 趋势至少需要多少个采样点才下结论（poll 200ms ⇒ 约 800ms） */
export const MIN_TREND_SAMPLES = 4;

/** 误差历史窗口上限（poll 200ms ⇒ 约 12s） */
export const HISTORY_CAP = 60;

export type ErrorTrend = 'unknown' | 'shrinking' | 'flat' | 'growing';
export type LinkHealth = 'idle' | 'settled' | 'tracking' | 'stalled';

/** 中位数（会复制后排序，不改动入参）。偶数个取中间两个的均值。 */
function median(xs: readonly number[]): number {
  if (xs.length === 0) return 0;
  const s = xs.slice().sort((a, b) => a - b);
  const mid = s.length >> 1;
  return s.length % 2 === 1 ? s[mid] : (s[mid - 1] + s[mid]) / 2;
}

/**
 * 误差趋势：比较窗口**前 1/4** 与**后 1/4** 的**中位数**。
 *
 * 为什么不用「末值 − 首值」或均值：回推序列里混有单帧噪声（掉帧、量化台阶），
 * 只要尖峰落在端点就会把结论整个翻面 —— 而"误报卡死"会让人去查不存在的问题。
 * 取四分位窗口 + 中位数等于一次稳健平滑，个别异常帧支配不了结论。
 */
export function classifyTrend(history: readonly number[], eps = TREND_EPS_DEG): ErrorTrend {
  if (history.length < MIN_TREND_SAMPLES) return 'unknown';
  const q = Math.max(1, Math.floor(history.length / 4));
  const delta = median(history.slice(-q)) - median(history.slice(0, q));
  if (Math.abs(delta) <= eps) return 'flat';
  return delta < 0 ? 'shrinking' : 'growing';
}

/**
 * 追加一个采样点。
 *
 * `commandChanged === true` 时**清空历史**：跨命令的趋势没有意义 —— 命令一改，
 * 误差会从 0 跳到很大，`growing` 是必然的，会被误判成卡死。
 */
export function nextHistory(
  prev: readonly number[],
  value: number,
  commandChanged: boolean,
  cap = HISTORY_CAP,
): number[] {
  if (commandChanged) return [value];
  const start = prev.length >= cap ? prev.length - cap + 1 : 0;
  const next = prev.slice(start);
  next.push(value);
  return next;
}

export interface LinkHealthInput {
  /** `actualJoints` 是否由传输回推驱动。false ⇒ 误差恒为 0，无链路可谈 */
  transportDriven: boolean;
  /** 当前最大关节误差（deg）；null = 无数据 */
  lagDeg: number | null;
  /** 命令最后一次变化距今（ms）；null = 未知 */
  msSinceCommandChange: number | null;
}

/**
 * 链路健康判定。
 *
 * 关键纪律：**没有正面证据就不下"卡死"断言**。宁可停在 `tracking`（提示"还在追"），
 * 也不要误报异常 —— 误报会让人去查并不存在的问题。因此只有
 * 「命令已静止超过 `stallHoldMs`」**且**「趋势不再下降（flat / growing）」才判 `stalled`。
 */
export function classifyLinkHealth(
  history: readonly number[],
  input: LinkHealthInput,
  tol = ERROR_TOL_DEG,
  stallHoldMs = STALL_HOLD_MS,
): LinkHealth {
  if (!input.transportDriven) return 'idle';

  const lag = input.lagDeg ?? history[history.length - 1] ?? 0;
  if (lag <= tol) return 'settled';

  // 命令刚变：误差必然大，且还没到"该到位"的时间 —— 不算异常
  if (input.msSinceCommandChange !== null && input.msSinceCommandChange < stallHoldMs) {
    return 'tracking';
  }

  const trend = classifyTrend(history);
  if (trend === 'shrinking') return 'tracking'; // 还在收敛
  if (trend === 'unknown') return 'tracking'; // 样本不足，不下断言
  return 'stalled'; // flat（命令静止却纹丝不动）或 growing（越追越远）
}

export interface LinkHealthInfo {
  kind: LinkHealth;
  label: string;
  tone: 'dim' | 'live' | 'warn';
}

export function describeLinkHealth(kind: LinkHealth, lagDeg: number | null): LinkHealthInfo {
  const lag = (lagDeg ?? 0).toFixed(2);
  switch (kind) {
    case 'idle':
      return { kind, label: '未接入传输 · Actual ≡ Command，误差恒为 0', tone: 'dim' };
    case 'settled':
      return { kind, label: `已到位（最大偏差 ${lag}°）`, tone: 'live' };
    case 'tracking':
      return { kind, label: `跟踪中（滞后 ${lag}°，命令到位前属正常）`, tone: 'dim' };
    case 'stalled':
      return {
        kind,
        label: `异常：命令已静止但误差停在 ${lag}° —— 查卡死 / 失步 / 限位截断`,
        tone: 'warn',
      };
  }
}

export function describeTrend(trend: ErrorTrend): string {
  switch (trend) {
    case 'shrinking':
      return '↓ 收敛中';
    case 'flat':
      return '→ 持平';
    case 'growing':
      return '↑ 发散';
    default:
      return '· 采样不足';
  }
}
