/**
 * 示教回放器（Phase 13）—— 把 `TeachTrack` 按时间轴重放成关节命令。
 *
 * 时钟为什么可注入
 * ----------------
 * 回放是**纯时间驱动**的：单测要断言"推进到 400ms 时命令等于第 5 帧"，
 * 若写死 `setInterval` 就只能真等 —— 慢且必然 flaky。因此复用项目既有的
 * `TimerLike`（生产 `realTimer` / 测试 `FakeTimer`），与本项目其余时间相关逻辑
 * 保持同一套确定性时钟约定。
 *
 * 为什么按 `timer.now()` 反推时刻，而不是每次 tick 累加 `tickMs`
 * ----------------------------------------------------------
 * 累加会被定时器抖动与丢拍直接吃掉时间（每拍差 2ms，10s 就偏 400ms），
 * 且暂停 / 续播的接续要额外维护。这里用**锚点式**：`play()` 记下
 * `anchor = now − tMs`，之后每一拍直接 `t = now − anchor` ——
 * 抖动不累积，暂停 / 拖动进度条只需重设锚点。
 *
 * 回放为什么必须走**既有命令路径**
 * ------------------------------
 * `onSample` 的落点由调用方决定，而 UI 里就是 `store.setCommandJoints()` ——
 * 与拖滑杆走的是同一条路。于是节流（尾沿合并 30Hz）、安全门（Simulation 模式
 * 不驱动真机）全都自动生效。**若为了"回放"另开一条直发通道，就等于绕过了
 * 所有安全约束** —— 那正是本项目反复要避免的静默危险。
 */
import type { JointState } from '../model/Pose';
import { realTimer, type TimerLike } from '../transport/timer';
import { sampleAt, trackDurationMs, type TeachTrack } from './teachTrack';

/** 回放步进周期（ms）。与录制采样同量级（20Hz），对舵机足够连续 */
export const TEACH_TICK_MS = 50;

export type TeachPlaybackStatus = 'idle' | 'playing' | 'paused' | 'ended';

export interface TeachPlaybackState {
  status: TeachPlaybackStatus;
  /** 已回放到的时刻（ms）；暂停时即暂停位置 */
  tMs: number;
  durationMs: number;
  /** 0..1 */
  progress: number;
}

export interface TeachPlayerOptions {
  timer?: TimerLike;
  tickMs?: number;
  /** 每个采样点的落点。UI 传入 `(j) => store.setCommandJoints(j)` */
  onSample: (joints: JointState, tMs: number) => void;
  /** 状态变化（供 UI 刷新进度条 / 按钮可用性） */
  onState?: (state: TeachPlaybackState) => void;
}

export class TeachPlayer {
  private readonly timer: TimerLike;
  private readonly tickMs: number;
  private readonly onSample: (joints: JointState, tMs: number) => void;
  private readonly onState: ((state: TeachPlaybackState) => void) | undefined;

  private track: TeachTrack | null = null;
  private handle: number | null = null;
  private status: TeachPlaybackStatus = 'idle';
  private tMs = 0;
  /** 时间原点：`play()` 时记为 `now − tMs`，之后 `t = now − anchor` */
  private anchor = 0;

  constructor(options: TeachPlayerOptions) {
    this.timer = options.timer ?? realTimer;
    this.tickMs = options.tickMs ?? TEACH_TICK_MS;
    this.onSample = options.onSample;
    this.onState = options.onState;
  }

  hasTrack(): boolean {
    return this.track !== null && this.track.frames.length > 0;
  }

  isPlaying(): boolean {
    return this.status === 'playing';
  }

  state(): TeachPlaybackState {
    const durationMs = this.duration();
    return {
      status: this.status,
      tMs: this.tMs,
      durationMs,
      progress:
        durationMs <= 0
          ? this.status === 'ended'
            ? 1
            : 0
          : Math.min(1, this.tMs / durationMs),
    };
  }

  /** 载入轨迹（回到起点、停止播放） */
  load(track: TeachTrack): void {
    this.clearTimer();
    this.track = track;
    this.tMs = 0;
    this.status = 'idle';
    this.emitState();
  }

  play(): void {
    if (!this.hasTrack() || this.status === 'playing') return;
    // 播完之后再点播放 = 从头再来（否则 anchored 在终点，一拍就结束）
    if (this.status === 'ended') this.tMs = 0;

    // 先立刻吐第一拍：等一个 tickMs 才动会有明显"点了没反应"的停滞感
    this.emitSample();

    if (this.duration() <= 0) {
      // 单帧轨迹：没有可播的时间轴，直接判已完成
      this.status = 'ended';
      this.emitState();
      return;
    }

    this.status = 'playing';
    this.anchor = this.timer.now() - this.tMs;
    this.emitState();
    this.handle = this.timer.setInterval(() => this.tick(), this.tickMs);
  }

  pause(): void {
    if (this.status !== 'playing') return;
    this.clearTimer();
    // 用时钟对齐，而不是信任上一拍写的 tMs —— 上一拍通常已在若干 ms 之前
    this.tMs = Math.min(this.timer.now() - this.anchor, this.duration());
    this.status = 'paused';
    this.emitState();
  }

  /**
   * 停止并回到起点。
   *
   * ⚠️ 刻意**不**下发任何命令：臂停在当前位置。复位到起点是"下一个动作"，
   * 不该是"停止"的副作用 —— 突然回零在真机上就是一次不必要的急停。
   */
  stop(): void {
    this.clearTimer();
    this.tMs = 0;
    this.status = 'idle';
    this.emitState();
  }

  /** 跳到指定时刻（预览式：立即下发该时刻的姿态） */
  seek(tMs: number): void {
    if (this.track === null) return;
    const durationMs = this.duration();
    this.tMs = Math.min(Math.max(tMs, 0), durationMs);

    if (this.status === 'playing') this.anchor = this.timer.now() - this.tMs;
    else if (this.status === 'ended' && this.tMs < durationMs) this.status = 'paused';

    this.emitSample();
    this.emitState();
  }

  dispose(): void {
    this.clearTimer();
    this.track = null;
    this.tMs = 0;
    this.status = 'idle';
  }

  // -------------------------------------------------------------------------

  private duration(): number {
    return this.track === null ? 0 : trackDurationMs(this.track);
  }

  private tick(): void {
    const track = this.track;
    if (track === null) {
      this.finish();
      return;
    }
    const durationMs = this.duration();
    this.tMs = this.timer.now() - this.anchor;

    if (this.tMs >= durationMs) {
      // 终态**精确**落在末帧：插值到 duration 与直接取末帧同值，
      // 这样"回放终点 == 录制终点"是逐值成立的不变量，而不是"差一点点"。
      this.tMs = durationMs;
      this.emitSample();
      this.finish();
      return;
    }

    this.emitSample();
    this.emitState();
  }

  private finish(): void {
    this.clearTimer();
    this.status = 'ended';
    this.emitState();
  }

  private clearTimer(): void {
    if (this.handle === null) return;
    this.timer.clearInterval(this.handle);
    this.handle = null;
  }

  private emitSample(): void {
    const track = this.track;
    if (track === null) return;
    const joints = sampleAt(track, this.tMs);
    if (joints !== null) this.onSample(joints, this.tMs);
  }

  private emitState(): void {
    this.onState?.(this.state());
  }
}
