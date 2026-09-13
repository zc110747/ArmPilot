/**
 * 示教轨迹纯逻辑回归（Phase 13）。
 *
 * 这里钉死的是**四条会被误当成"能用"的错误行为**：
 *   1. 录制把静止段 / 60fps 拖动逐帧记下 ⇒ 轨迹体积爆炸（节流 + 去抖）；
 *   2. 原样逐帧回放 ⇒ 命令阶跃、真机一跳一跳（`sampleAt` 必须插值）；
 *   3. 回放终点停在倒数第二帧 ⇒ 臂在动作结束时莫名回退（终态必须精确落在末帧）；
 *   4. 载入任意 JSON 当作轨迹 ⇒ 崩在渲染里（`parseTrack` 必须返回 null 而不是抛）。
 */
import { describe, expect, it } from 'vitest';
import {
  TEACH_FORMAT,
  TEACH_MAX_FRAMES,
  TEACH_MIN_FRAME_INTERVAL_MS,
  TEACH_STILL_EPS_DEG,
  appendFrame,
  emptyTrack,
  parseTrack,
  sampleAt,
  serializeTrack,
  trackDurationMs,
  trackStats,
  type JointState,
  type TeachFrame,
  type TeachTrack,
} from '@robot/index';

function track(frames: TeachFrame[]): TeachTrack {
  return { name: 'unit', createdAt: 0, frames };
}

const J = (base: number, shoulder: number, elbow: number): JointState => ({
  base,
  shoulder,
  elbow,
  gripper: 50,
});

describe('Phase 13 · appendFrame 采样策略', () => {
  it('首帧无条件记录，并把 nowMs 记为轨迹起点（t 从 0 开始）', () => {
    const t0 = appendFrame(emptyTrack(), J(0, 0, 110), 1_000_000);
    expect(t0.frames).toHaveLength(1);
    expect(t0.frames[0]?.t).toBe(0);
    expect(t0.createdAt).toBe(1_000_000);
    expect(t0.frames[0]?.joints).toEqual(J(0, 0, 110));
  });

  it(`距上一帧不足 ${TEACH_MIN_FRAME_INTERVAL_MS}ms 不记（拖动 60fps 会被压到 20Hz）`, () => {
    const a = appendFrame(emptyTrack(), J(0, 0, 110), 0);
    const b = appendFrame(a, J(5, 0, 110), 30);
    expect(b.frames).toHaveLength(1);
    // 跳过时必须返回**同一引用** —— store 里 setState 后 React 才能 bail out
    expect(b).toBe(a);
  });

  it(`各轴差都在 ${TEACH_STILL_EPS_DEG}° 内视为静止，不产生新帧`, () => {
    const a = appendFrame(emptyTrack(), J(0, 0, 110), 0);
    const b = appendFrame(a, J(0.4, 0.3, 110.2), 500);
    expect(b.frames).toHaveLength(1);
    expect(b).toBe(a);
  });

  it('越过静止死区即记录（死区是为了滤抖，不是为了削掉小动作）', () => {
    const a = appendFrame(emptyTrack(), J(0, 0, 110), 0);
    const b = appendFrame(a, J(0.6, 0, 110), 500);
    expect(b.frames).toHaveLength(2);
    expect(b.frames[1]?.t).toBe(500);
  });

  it('不修改入参（appendFrame 必须是纯函数）', () => {
    const a = appendFrame(emptyTrack(), J(0, 0, 110), 0);
    const snapshot = JSON.stringify(a);
    appendFrame(a, J(20, 0, 110), 500);
    expect(JSON.stringify(a)).toBe(snapshot);
  });

  it(`达到 ${TEACH_MAX_FRAMES} 帧上限后拒绝新帧（宁可不录，也不悄悄丢掉开头）`, () => {
    let t = emptyTrack();
    t = appendFrame(t, J(0, 0, 110), 0, { maxFrames: 3 });
    t = appendFrame(t, J(10, 0, 110), 100, { maxFrames: 3 });
    t = appendFrame(t, J(20, 0, 110), 200, { maxFrames: 3 });
    expect(t.frames).toHaveLength(3);
    const full = appendFrame(t, J(30, 0, 110), 300, { maxFrames: 3 });
    expect(full.frames).toHaveLength(3);
    expect(full).toBe(t);
    // 保留的是**最早**三帧：起手动作不能被吞掉
    expect(full.frames[0]?.joints.shoulder).toBe(0);
  });

  describe('force（停止录制时补末帧）', () => {
    it('跳过节流与去抖 —— 否则轨迹终点 ≠ 臂当前所在位置', () => {
      const a = appendFrame(emptyTrack(), J(0, 0, 110), 0);
      // 静止 + 同毫秒：常规路径必然跳过
      const normal = appendFrame(a, J(0.1, 0, 110), 0);
      expect(normal).toBe(a);
      const forced = appendFrame(a, J(0.1, 0, 110), 0, { force: true });
      expect(forced.frames).toHaveLength(2);
      // t 必须**严格递增**：同一毫秒补帧也要 +1ms，否则 sampleAt 的区间定位会退化
      expect(forced.frames[1]?.t).toBeGreaterThan(a.frames[0]!.t);
    });

    it('与末帧完全同姿态时无事可做（不产生 1ms 的重复帧）', () => {
      const a = appendFrame(emptyTrack(), J(0, 0, 110), 0);
      expect(appendFrame(a, J(0, 0, 110), 999, { force: true })).toBe(a);
    });
  });
});

describe('Phase 13 · sampleAt 时间轴插值', () => {
  const t = track([
    { t: 0, joints: J(0, 0, 100) },
    { t: 100, joints: J(10, 0, 100) },
    { t: 400, joints: J(0, 90, 100) },
  ]);

  it('空轨迹返回 null（回放循环据此判"没东西可放"）', () => {
    expect(sampleAt(emptyTrack(), 0)).toBeNull();
  });

  it('单帧轨迹：任何时刻都返回该帧', () => {
    const one = track([{ t: 0, joints: J(1, 2, 3) }]);
    expect(sampleAt(one, 0)).toEqual(J(1, 2, 3));
    expect(sampleAt(one, 5000)).toEqual(J(1, 2, 3));
  });

  it('两端钳位而非返回 null（调用方每拍都要一个可下发的值）', () => {
    expect(sampleAt(t, -50)?.base).toBe(0);
    expect(sampleAt(t, 99_999)?.shoulder).toBe(90);
  });

  it('区间内线性插值：100ms 处 base 走到 5°（不是阶跃到 10°）', () => {
    const mid = sampleAt(t, 50);
    expect(mid?.base).toBeCloseTo(5, 9);
  });

  it('区间定位正确：跨过第二帧后按第二段插值', () => {
    const q = sampleAt(t, 250); // 400−100=300 段的中点 ⇒ 50%
    expect(q?.base).toBeCloseTo(5, 9);
    expect(q?.shoulder).toBeCloseTo(45, 9);
  });

  it('恰好落在采样点上时逐值等于该帧（不留插值残差）', () => {
    expect(sampleAt(t, 100)).toEqual(t.frames[1]?.joints);
    expect(sampleAt(t, 400)).toEqual(t.frames[2]?.joints);
  });

  it('关节键集不一致时按并集插值，缺失侧按 0 起算', () => {
    const mixed = track([
      { t: 0, joints: { base: 0 } },
      { t: 100, joints: { base: 10, shoulder: 20 } },
    ]);
    const mid = sampleAt(mixed, 50);
    expect(mid).toEqual({ base: 5, shoulder: 10 });
  });
});

describe('Phase 13 · 统计与序列化', () => {
  it('trackDurationMs = 末帧时刻；空轨迹为 0', () => {
    expect(trackDurationMs(emptyTrack())).toBe(0);
    expect(trackDurationMs(track([{ t: 0, joints: J(0, 0, 0) }]))).toBe(0);
    expect(
      trackDurationMs(
        track([
          { t: 0, joints: J(0, 0, 0) },
          { t: 1250, joints: J(1, 1, 1) },
        ]),
      ),
    ).toBe(1250);
  });

  it('trackStats 汇总帧数 / 时长 / 关节并集（去重且保持首次出现顺序）', () => {
    const s = trackStats(
      track([
        { t: 0, joints: { base: 0, shoulder: 0 } },
        { t: 200, joints: { shoulder: 5, elbow: 100 } },
      ]),
    );
    expect(s).toEqual({ frames: 2, durationMs: 200, joints: ['base', 'shoulder', 'elbow'] });
  });

  it('serialize → parse 往返一致（帧数 / 时刻 / 角度精确到 0.01°）', () => {
    const src = track([
      { t: 0, joints: { base: -18.25, shoulder: 8.5, elbow: 118, gripper: 38 } },
      { t: 420, joints: { base: 26.75, shoulder: 34.25, elbow: 132, gripper: 58 } },
    ]);
    const back = parseTrack(serializeTrack(src));
    expect(back).not.toBeNull();
    expect(back?.frames).toEqual(src.frames);
    expect(back?.name).toBe('unit');
    expect(back?.createdAt).toBe(0);
  });

  it('parseTrack 对不可信输入一律返回 null（不抛异常）', () => {
    expect(parseTrack('not json at all')).toBeNull();
    expect(parseTrack('null')).toBeNull();
    expect(parseTrack('[]')).toBeNull();
    // 缺 format 标记：哪怕结构长得像，也不认 —— 防止把别的 JSON 误当轨迹
    expect(parseTrack(JSON.stringify({ frames: [{ t: 0, joints: { base: 1 } }] }))).toBeNull();
    expect(parseTrack(JSON.stringify({ format: TEACH_FORMAT }))).toBeNull();
    expect(
      parseTrack(JSON.stringify({ format: TEACH_FORMAT, frames: [{ t: 0, joints: {} }] })),
    ).toBeNull();
  });

  it('parseTrack 清洗非法帧：丢非有限 t、丢空 joints、丢破坏递增的帧', () => {
    const json = JSON.stringify({
      format: TEACH_FORMAT,
      name: '  messy  ',
      createdAt: 123,
      frames: [
        { t: 0, joints: { base: 0 } },
        { t: -5, joints: { base: 99 } }, // t 为负 ⇒ 丢
        { t: 100, joints: { base: 10 } },
        { t: 50, joints: { base: 77 } }, // 不递增 ⇒ 丢（保住 sampleAt 的区间定位前提）
        { t: 200, joints: {} }, // 无有效数值 ⇒ 丢
        { t: 300, joints: { base: 30 } },
        { t: 400, joints: { base: Number.NaN } }, // NaN ⇒ 丢
      ],
    });
    const parsed = parseTrack(json);
    expect(parsed?.name).toBe('messy');
    expect(parsed?.createdAt).toBe(123);
    expect(parsed?.frames.map((f) => f.t)).toEqual([0, 100, 300]);
  });
});
