/**
 * 示教轨迹（Phase 13）—— 录制 / 回放的**纯逻辑层**。
 *
 * 为什么单独成模块
 * ----------------
 * 「录制」与「回放」里真正需要想清楚的部分（采样节流、去抖、插值、时间轴、
 * 序列化校验）全是纯函数，与 React 无关。放这里是为了能在 `environment: 'node'`
 * 下直接单测（本项目无 jsdom），也为了让 `TeachPlayer` 只负责"什么时候调用它们"。
 *
 * 录什么：只录 `commandJoints`，不录 `actualJoints`
 * ----------------------------------------------
 * `command` 是**我们要求机械臂去的角度**，`actual` 是链路回推的滞后值
 * （MG90S 无回读，甚至就是由目标值反算的，见 ADR D34）。
 * 录 actual 等于把链路时延一起录进去，回放时再叠加一次时延 —— 越放越偏。
 *
 * 为什么是「等距采样帧序列」而不是「关键点 + 插补指令」
 * ------------------------------------------------
 * 人手拖动给出的是连续轨迹，其中没有"关键点"概念（也没有 G 代码式的直线/圆弧
 * 插补语义可填）。因此录制就是**按时间采样**，回放就是**按时间重放**。
 * 采样率由 `TEACH_MIN_FRAME_INTERVAL_MS` 决定：50ms ⇒ 20Hz，
 * 对 0.2°/ms 量级的舵机足够还原人手轨迹，一个 10s 动作约 200 帧 / 约 6KB JSON。
 *
 * 回放为什么要插值（`sampleAt`）
 * ----------------------------
 * 把采样点逐帧原样抛出，两个采样点之间命令仍是**阶跃**的 —— 真机会一跳一跳。
 * 关节空间线性插值让回放周期的每一拍都给出连续值，配合既有尾沿节流（30Hz）
 * 输出，真机运动才平滑。这也是回放时长不被采样率量化的原因。
 */
import type { JointState } from '../model/Pose';
import { jointStatesEqual } from '../model/RobotState';

/** 采样最小间隔（ms）：50 ⇒ 20Hz */
export const TEACH_MIN_FRAME_INTERVAL_MS = 50;

/**
 * 轨迹帧数上限。2000 帧 × 50ms ≈ 100s 连续动作。
 * 达到上限后 `appendFrame` **拒绝新帧**（不是丢弃旧帧）——
 * 悄悄丢掉开头会让回放出来的动作缺少起手，比"录不进去"更难发现。
 */
export const TEACH_MAX_FRAMES = 2000;

/** 与上一帧各轴差都 ≤ 该值即视为"静止"，不产生新帧（去抖，显著减小体积） */
export const TEACH_STILL_EPS_DEG = 0.5;

/** 导出文件的格式标记。`parseTrack` 只认这一个 —— 防止把别的 JSON 误当轨迹载入 */
export const TEACH_FORMAT = 'armpilot.teach/1';

export interface TeachFrame {
  /** 相对录制起点的毫秒偏移，**严格递增**（`sampleAt` 的区间定位依赖它） */
  t: number;
  joints: JointState;
}

export interface TeachTrack {
  name: string;
  /** 录制起点的 Unix 毫秒；null = 尚未开始（空轨迹） */
  createdAt: number | null;
  frames: TeachFrame[];
}

export function emptyTrack(name = 'untitled'): TeachTrack {
  return { name, createdAt: null, frames: [] };
}

export interface AppendFrameOptions {
  minIntervalMs?: number;
  maxFrames?: number;
  stillEpsDeg?: number;
  /**
   * 强制记录（跳过节流与去抖）。
   *
   * 停止录制时必须用一次 —— 否则"最后一帧"可能停在去抖吃掉的那个值上，
   * 于是轨迹终点 ≠ 臂当前所在位置，回放结束时臂会莫名回退一点。
   */
  force?: boolean;
}

/**
 * 追加一帧，返回**新对象**（不改动入参）。
 *
 * 三类跳过：① 已达帧数上限；② 距上一帧不足 `minIntervalMs`（拖动时每帧都在变，
 * 逐帧记录会把轨迹撑爆）；③ 与上一帧各轴差都在 `stillEpsDeg` 内（静止数据）。
 */
export function appendFrame(
  track: TeachTrack,
  joints: JointState,
  nowMs: number,
  options: AppendFrameOptions = {},
): TeachTrack {
  const minIntervalMs = options.minIntervalMs ?? TEACH_MIN_FRAME_INTERVAL_MS;
  const maxFrames = options.maxFrames ?? TEACH_MAX_FRAMES;
  const eps = options.stillEpsDeg ?? TEACH_STILL_EPS_DEG;

  if (track.frames.length >= maxFrames) return track;

  const last = track.frames[track.frames.length - 1];
  if (last === undefined) {
    // 首帧无条件记：没有起点的话，回放时长与终点姿态都是错的
    return { ...track, createdAt: nowMs, frames: [{ t: 0, joints: { ...joints } }] };
  }

  const base = track.createdAt ?? nowMs;

  if (options.force) {
    // 已经与末帧同姿态 ⇒ 无补的必要（`sampleAt` 会插值到同一个值）
    if (jointStatesEqual(last.joints, joints, 1e-9)) return track;
    // +1ms 保证严格递增：同一毫秒内停录也要能补出一帧
    const t = Math.max(last.t + 1, Math.max(0, nowMs - base));
    return { ...track, frames: [...track.frames, { t, joints: { ...joints } }] };
  }

  const t = Math.max(0, nowMs - base);
  if (t - last.t < minIntervalMs) return track;
  if (jointStatesEqual(last.joints, joints, eps)) return track;

  return { ...track, frames: [...track.frames, { t, joints: { ...joints } }] };
}

/** 轨迹时长（ms）= 末帧时刻；空轨迹为 0 */
export function trackDurationMs(track: TeachTrack): number {
  const last = track.frames[track.frames.length - 1];
  return last === undefined ? 0 : last.t;
}

/**
 * 取 `tMs` 时刻的关节角（关节空间线性插值）。
 *
 * 越界**钳位**到首/末帧（不返回 null）—— 回放循环里每次都要一个可下发的值，
 * 让调用方处理 null 只会把"钳位"这件事散落到各处。空轨迹返回 null。
 */
export function sampleAt(track: TeachTrack, tMs: number): JointState | null {
  const frames = track.frames;
  const first = frames[0];
  if (first === undefined) return null;

  const last = frames[frames.length - 1] as TeachFrame;
  if (frames.length === 1) return { ...first.joints };

  const t = Math.min(Math.max(tMs, first.t), last.t);
  if (t <= first.t) return { ...first.joints };
  if (t >= last.t) return { ...last.joints };

  let i = 0;
  while (i + 1 < frames.length && (frames[i + 1] as TeachFrame).t <= t) i += 1;
  const a = frames[i] as TeachFrame;
  const b = frames[i + 1] as TeachFrame;
  const span = b.t - a.t;
  const u = span <= 0 ? 0 : (t - a.t) / span;

  const out: JointState = {};
  for (const key of new Set([...Object.keys(a.joints), ...Object.keys(b.joints)])) {
    const va = a.joints[key] ?? 0;
    const vb = b.joints[key] ?? va;
    out[key] = va + (vb - va) * u;
  }
  return out;
}

export interface TeachTrackStats {
  frames: number;
  durationMs: number;
  /** 出现过的关节 id（按首次出现顺序去重） */
  joints: string[];
}

export function trackStats(track: TeachTrack): TeachTrackStats {
  const joints: string[] = [];
  for (const frame of track.frames) {
    for (const key of Object.keys(frame.joints)) {
      if (!joints.includes(key)) joints.push(key);
    }
  }
  return { frames: track.frames.length, durationMs: trackDurationMs(track), joints };
}

/** 导出用的 JSON 文本。时刻取整到 ms、角度取整到 0.01°（远细于舵机分辨率） */
export function serializeTrack(track: TeachTrack): string {
  return JSON.stringify({
    format: TEACH_FORMAT,
    name: track.name,
    createdAt: track.createdAt,
    frames: track.frames.map((f) => ({
      t: Math.round(f.t),
      joints: roundJoints(f.joints, 2),
    })),
  });
}

function roundJoints(joints: JointState, digits: number): JointState {
  const factor = 10 ** digits;
  const out: JointState = {};
  for (const [key, value] of Object.entries(joints)) {
    out[key] = Math.round(value * factor) / factor;
  }
  return out;
}

/** 只接受"至少含一个有限数值"的对象 —— 挡掉 `{joints: null}` / `{joints: "x"}` */
function sanitizeJoints(raw: unknown): JointState | null {
  if (typeof raw !== 'object' || raw === null || Array.isArray(raw)) return null;
  const out: JointState = {};
  let count = 0;
  for (const [key, value] of Object.entries(raw as Record<string, unknown>)) {
    if (typeof value !== 'number' || !Number.isFinite(value)) continue;
    out[key] = value;
    count += 1;
  }
  return count > 0 ? out : null;
}

/**
 * 从导出文本还原轨迹；任何不可信输入一律返回 null（不抛异常、不半信半疑地截取）。
 *
 * 三类清洗：① 逐帧丢非法帧（t 非有限数 / joints 无有效数值）；
 * ② 丢弃 `t` 非严格递增的帧（保住 `sampleAt` 的区间定位前提）；
 * ③ 空结果 / 格式标记不符 ⇒ null。
 */
export function parseTrack(json: string): TeachTrack | null {
  let raw: unknown;
  try {
    raw = JSON.parse(json);
  } catch {
    return null;
  }
  if (typeof raw !== 'object' || raw === null) return null;

  const obj = raw as Record<string, unknown>;
  if (obj.format !== TEACH_FORMAT) return null;
  if (!Array.isArray(obj.frames)) return null;

  const frames: TeachFrame[] = [];
  for (const item of obj.frames) {
    if (typeof item !== 'object' || item === null) continue;
    const f = item as Record<string, unknown>;
    if (typeof f.t !== 'number' || !Number.isFinite(f.t) || f.t < 0) continue;
    const joints = sanitizeJoints(f.joints);
    if (joints === null) continue;
    const prev = frames[frames.length - 1];
    if (prev !== undefined && f.t <= prev.t) continue;
    frames.push({ t: f.t, joints });
  }
  if (frames.length === 0) return null;

  const name =
    typeof obj.name === 'string' && obj.name.trim() !== '' ? obj.name.trim() : 'imported';
  const createdAt =
    typeof obj.createdAt === 'number' && Number.isFinite(obj.createdAt) ? obj.createdAt : null;

  return { name, createdAt, frames };
}
