/**
 * 示教回放器回归（Phase 13）。
 *
 * 全部用 `FakeTimer` 精确推进虚拟时钟 —— 回放是纯时间驱动的，
 * 若靠真实等待，这些断言在 CI 上必然 flaky（本项目既有确定性时钟约定）。
 *
 * 重点是三条**只有时间轴才能证伪**的行为：
 *   ① `play()` 必须立刻吐第一拍（否则"点了没反应"）；
 *   ② 结束时刻的采样必须**精确取末帧**而非插值（否则臂在动作尾端回退一点点）；
 *   ③ 暂停 / 续播用时钟锚点对齐，抖动不累积、位置不跳回。
 */
import { beforeEach, describe, expect, it } from 'vitest';
import {
  TeachPlayer,
  emptyTrack,
  sampleAt,
  trackDurationMs,
  type JointState,
  type TeachPlaybackState,
  type TeachTrack,
} from '@robot/index';
import { FakeTimer } from '../helpers/fakeTimer';
import { useRobotStore } from '../../src/store/robotStore';

const T: TeachTrack = {
  name: 'p',
  createdAt: 0,
  frames: [
    { t: 0, joints: { base: 0, shoulder: 0, elbow: 100, gripper: 50 } },
    { t: 100, joints: { base: 10, shoulder: 0, elbow: 100, gripper: 50 } },
    { t: 200, joints: { base: 20, shoulder: 0, elbow: 100, gripper: 50 } },
    { t: 300, joints: { base: 30, shoulder: 0, elbow: 100, gripper: 50 } },
  ],
};

interface Harness {
  player: TeachPlayer;
  samples: { joints: JointState; tMs: number }[];
  states: TeachPlaybackState[];
}

function makePlayer(timer: FakeTimer): Harness {
  const samples: Harness['samples'] = [];
  const states: TeachPlaybackState[] = [];
  const player = new TeachPlayer({
    timer,
    onSample: (joints, tMs) => samples.push({ joints, tMs }),
    onState: (s) => states.push(s),
  });
  return { player, samples, states };
}

describe('Phase 13 · TeachPlayer 时间轴', () => {
  it('load 前没有任何轨迹，play 是 no-op', () => {
    const timer = new FakeTimer();
    const { player, samples } = makePlayer(timer);
    expect(player.hasTrack()).toBe(false);
    player.play();
    expect(samples).toHaveLength(0);
    expect(player.state().status).toBe('idle');
  });

  it('play() 立刻吐出起点那一拍 —— 不等一个 tick，否则有明显停滞感', () => {
    const timer = new FakeTimer();
    const { player, samples } = makePlayer(timer);
    player.load(T);
    player.play();
    expect(samples).toHaveLength(1);
    expect(samples[0]?.tMs).toBe(0);
    expect(samples[0]?.joints).toEqual(T.frames[0]?.joints);
    expect(player.state().status).toBe('playing');
  });

  it('按时间轴插值推进：50ms 处 base 走到 5°（命令连续，不是阶跃到 10°）', () => {
    const timer = new FakeTimer();
    const { player, samples } = makePlayer(timer);
    player.load(T);
    player.play();
    timer.advance(50);
    const last = samples[samples.length - 1];
    expect(last?.tMs).toBe(50);
    expect(last?.joints.base).toBeCloseTo(5, 9);
  });

  it('结束时刻的采样**精确取末帧**，且定时器被清干净（不空转）', () => {
    const timer = new FakeTimer();
    const { player, samples } = makePlayer(timer);
    player.load(T);
    player.play();
    timer.advance(trackDurationMs(T) + 500);

    expect(player.state().status).toBe('ended');
    expect(player.state().tMs).toBe(trackDurationMs(T));
    const last = samples[samples.length - 1];
    // 逐值相等而不是"接近"：终态若走插值就会差最后一段的一小截
    expect(last?.joints).toEqual(T.frames[3]?.joints);
    expect(timer.pending()).toBe(0);
  });

  it('结束只发生一次：末帧之后不再有新采样', () => {
    const timer = new FakeTimer();
    const { player, samples } = makePlayer(timer);
    player.load(T);
    player.play();
    timer.advance(300);
    const atEnd = samples.length;
    timer.advance(1000);
    expect(samples).toHaveLength(atEnd);
  });

  it('pause 保留位置，续播从该位置接上（不跳回 0）', () => {
    const timer = new FakeTimer();
    const { player } = makePlayer(timer);
    player.load(T);
    player.play();
    timer.advance(120); // 有效 tick 落在 50 / 100 ⇒ tMs = 100
    player.pause();
    expect(player.state().status).toBe('paused');
    expect(player.state().tMs).toBe(120); // 用时钟对齐到暂停瞬间，而非上一拍

    player.play();
    expect(player.state().status).toBe('playing');
    timer.advance(50);
    expect(player.state().tMs).toBe(170);
  });

  it('pause 在非播放态是 no-op（不会把 idle 弄成 paused）', () => {
    const timer = new FakeTimer();
    const { player } = makePlayer(timer);
    player.load(T);
    player.pause();
    expect(player.state().status).toBe('idle');
  });

  it('stop 回到起点且**不下发任何命令**（臂停在原地，不被动复位）', () => {
    const timer = new FakeTimer();
    const { player, samples } = makePlayer(timer);
    player.load(T);
    player.play();
    timer.advance(150);
    const before = samples.length;
    player.stop();
    expect(samples).toHaveLength(before);
    expect(player.state().status).toBe('idle');
    expect(player.state().tMs).toBe(0);
    expect(timer.pending()).toBe(0);
  });

  it('seek 钳位到 [0, duration] 并立即下发该时刻姿态', () => {
    const timer = new FakeTimer();
    const { player, samples } = makePlayer(timer);
    player.load(T);

    player.seek(-100);
    expect(player.state().tMs).toBe(0);
    player.seek(99_999);
    expect(player.state().tMs).toBe(trackDurationMs(T));
    expect(samples[samples.length - 1]?.joints).toEqual(T.frames[3]?.joints);

    player.seek(150);
    expect(player.state().tMs).toBe(150);
    expect(samples[samples.length - 1]?.joints).toEqual(sampleAt(T, 150));
  });

  it('播完后再点播放 → 从头开始（而不是停在终点一拍即结束）', () => {
    const timer = new FakeTimer();
    const { player, samples } = makePlayer(timer);
    player.load(T);
    player.play();
    timer.advance(300);
    expect(player.state().status).toBe('ended');

    player.play();
    expect(player.state().status).toBe('playing');
    expect(player.state().tMs).toBe(0);
    expect(samples[samples.length - 1]?.joints).toEqual(T.frames[0]?.joints);
    timer.advance(100);
    expect(player.state().tMs).toBe(100);
  });

  it('单帧轨迹：play 后立即判完成（没有可播的时间轴）', () => {
    const timer = new FakeTimer();
    const { player, samples } = makePlayer(timer);
    const one: TeachTrack = { name: 'one', createdAt: 0, frames: [{ t: 0, joints: { base: 7 } }] };
    player.load(one);
    player.play();
    expect(samples).toHaveLength(1);
    expect(player.state().status).toBe('ended');
    expect(player.state().progress).toBe(1);
    expect(timer.pending()).toBe(0);
  });

  it('load 重置为起点 idle（换轨迹不该延续上一条的播放位置）', () => {
    const timer = new FakeTimer();
    const { player } = makePlayer(timer);
    player.load(T);
    player.play();
    timer.advance(150);
    player.load(T);
    expect(player.state().status).toBe('idle');
    expect(player.state().tMs).toBe(0);
    expect(timer.pending()).toBe(0);
  });

  it('state().progress 随回放单调增长到 1', () => {
    const timer = new FakeTimer();
    const { player } = makePlayer(timer);
    player.load(T);
    expect(player.state().progress).toBe(0);
    player.play();
    timer.advance(150);
    expect(player.state().progress).toBeCloseTo(0.5, 9);
    timer.advance(300);
    expect(player.state().progress).toBe(1);
  });

  it('空轨迹的 state 是 idle / duration 0（面板据此禁用按钮）', () => {
    const timer = new FakeTimer();
    const { player } = makePlayer(timer);
    player.load(emptyTrack('e'));
    expect(player.state()).toEqual({ status: 'idle', tMs: 0, durationMs: 0, progress: 0 });
    expect(player.hasTrack()).toBe(false);
  });
});

describe('Phase 13 · 录制 → 回放闭环（store 集成）', () => {
  const store = () => useRobotStore.getState();

  beforeEach(() => {
    useRobotStore.setState({ teachTrack: emptyTrack('unit'), teachRecording: false });
  });

  it('回放终点逐值等于录制末帧，且回放确实经过 store 的命令路径', () => {
    const timer = new FakeTimer();

    // 模拟"手动摆三个姿态"：时刻隔开以越过 50ms 采样节流
    store().setCommandJoints({ base: -18, shoulder: 8, elbow: 118, gripper: 38 });
    store().appendTeachFrame(store().commandJoints, 0);
    store().setCommandJoints({ base: 6, shoulder: 26, elbow: 126, gripper: 46 });
    store().appendTeachFrame(store().commandJoints, 200);
    store().setCommandJoints({ base: 26, shoulder: 34, elbow: 132, gripper: 58 });
    // 停录时补末帧 —— 保证轨迹终点就是臂当时所在位置
    store().appendTeachFrame(store().commandJoints, 500, { force: true });

    const rec = store().teachTrack;
    expect(rec.frames).toHaveLength(3);
    expect(trackDurationMs(rec)).toBe(500);

    const player = new TeachPlayer({
      timer,
      onSample: (j) => useRobotStore.getState().setCommandJoints(j),
    });
    player.load(rec);
    player.play();
    timer.advance(trackDurationMs(rec) + 200);

    expect(player.state().status).toBe('ended');
    expect(store().commandJoints).toEqual(rec.frames[2]?.joints);
  });

  it('回放途中的命令是插值值（介于首末帧之间），而不是直接跳到末帧', () => {
    const timer = new FakeTimer();
    store().setCommandJoints({ base: 0, shoulder: 0, elbow: 110, gripper: 50 });
    store().appendTeachFrame(store().commandJoints, 0);
    store().setCommandJoints({ base: 40, shoulder: 0, elbow: 110, gripper: 50 });
    store().appendTeachFrame(store().commandJoints, 200, { force: true });

    const rec = store().teachTrack;
    const player = new TeachPlayer({
      timer,
      onSample: (j) => useRobotStore.getState().setCommandJoints(j),
    });
    player.load(rec);
    player.play();
    timer.advance(100); // 行程中点

    const base = store().commandJoints.base ?? 0;
    expect(base).toBeGreaterThan(0);
    expect(base).toBeLessThan(40);
  });

  it('appendTeachFrame 在跳过时保持同一引用（store 订阅者不会被无谓唤醒）', () => {
    store().appendTeachFrame({ base: 0, shoulder: 0, elbow: 110, gripper: 50 }, 0);
    const first = store().teachTrack;
    // 静止且未越节流窗口 ⇒ 必然跳过
    store().appendTeachFrame({ base: 0.2, shoulder: 0, elbow: 110, gripper: 50 }, 10);
    expect(store().teachTrack).toBe(first);
  });
});
