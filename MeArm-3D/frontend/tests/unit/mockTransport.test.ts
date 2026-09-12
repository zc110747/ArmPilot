/**
 * MockTransport 行为单测（Phase 7）。
 *
 * 全部用 `FakeTimer` 精确推进虚拟时间 —— 没有一处真实等待，
 * 所以"延迟 15ms 才动""每 tick 恰好 4.8°"这类断言是**确定性**的。
 */
import { describe, expect, it } from 'vitest';
import {
  MockTransport,
  createJointCommand,
  createSimpleCommand,
  homeJointState,
  jointByRole,
  loadRobotModel,
  type JointState,
  type MockTransportOptions,
  type TransportStatusDetail,
} from '@robot/index';
import { FakeTimer } from '../helpers/fakeTimer';

const model = loadRobotModel();
const home = homeJointState(model);

/** 默认调参：240°/s、15ms 延迟、tick 20ms ⇒ 每 tick 最多 4.8° */
function makeTransport(options: Partial<MockTransportOptions> = {}) {
  const timer = new FakeTimer();
  const statuses: TransportStatusDetail[] = [];
  const states: { joints: JointState; source: string }[] = [];

  const transport = new MockTransport({
    ...options,
    model,
    timer,
    tickMs: options.tickMs ?? 20,
    maxSpeedDegPerSec: options.maxSpeedDegPerSec ?? 240,
    latencyMs: options.latencyMs ?? 15,
  });
  transport.onStatus((detail) => statuses.push(detail));
  transport.onState((state) => states.push({ joints: state.joints, source: state.source }));

  return { timer, transport, statuses, states };
}

/** HOME 的 shoulder 加 `delta` 度（限位内） */
function shoulderShift(delta: number): JointState {
  return { ...home, shoulder: home.shoulder + delta };
}

describe('MockTransport · 连接与状态', () => {
  it('初始状态：actual 与 target 都等于 HOME', () => {
    const { transport } = makeTransport();
    const actual = transport.actualJoints();
    for (const [id, value] of Object.entries(home)) {
      expect(actual[id]).toBeCloseTo(value, 12);
    }
    expect(transport.stats().lagDeg).toBe(0);
  });

  it('connect / disconnect 的状态流转与通知', async () => {
    const { transport, statuses } = makeTransport();
    expect(transport.status()).toBe('disconnected');

    await transport.connect();
    expect(transport.status()).toBe('connected');
    expect(statuses.map((s) => s.status)).toEqual(['connecting', 'connected']);

    await transport.disconnect();
    expect(transport.status()).toBe('disconnected');
    expect(statuses.at(-1)?.status).toBe('disconnected');
  });

  it('未连接时的命令被丢弃，且不计数为 sent', async () => {
    const { transport, statuses } = makeTransport();
    await transport.sendJointState(shoulderShift(10));
    expect(transport.stats().sent).toBe(0);
    expect(statuses.at(-1)?.status).toBe('error');
    expect(statuses.at(-1)?.reason).toContain('未连接');
  });
});

describe('MockTransport · 传输延迟', () => {
  it('延迟未到之前不回推任何状态，到点后立刻回推', async () => {
    const { timer, transport, states } = makeTransport({ latencyMs: 15 });
    await transport.connect();
    await transport.sendJointState(shoulderShift(48));

    timer.advance(14);
    expect(states).toHaveLength(0);
    expect(transport.stats().sent).toBe(1);

    timer.advance(1);
    expect(states.length).toBeGreaterThan(0);
    expect(states[0].source).toBe('real');
  });

  it('latency=0 时命令同步生效', async () => {
    const { transport, states } = makeTransport({ latencyMs: 0 });
    await transport.connect();
    await transport.sendJointState(shoulderShift(48));
    expect(states.length).toBeGreaterThan(0);
  });
});

describe('MockTransport · 有限角速度', () => {
  it('每 tick 位移恰好等于 maxSpeed × dt，且不越过目标', async () => {
    const { timer, transport } = makeTransport({
      maxSpeedDegPerSec: 240,
      tickMs: 20,
      latencyMs: 15,
    });
    await transport.connect();
    await transport.sendJointState(shoulderShift(48));

    // 送达时立刻走一步
    timer.advance(15);
    expect(transport.actualJoints().shoulder).toBeCloseTo(home.shoulder + 4.8, 9);

    // 之后每 20ms 走一步
    timer.advance(20);
    expect(transport.actualJoints().shoulder).toBeCloseTo(home.shoulder + 9.6, 9);

    timer.advance(20);
    expect(transport.actualJoints().shoulder).toBeCloseTo(home.shoulder + 14.4, 9);
  });

  it('maxSpeed=0（或非有限）时视为瞬时到位', async () => {
    const { timer, transport } = makeTransport({ maxSpeedDegPerSec: 0, latencyMs: 15 });
    await transport.connect();
    await transport.sendJointState(shoulderShift(48));

    timer.advance(15);
    expect(transport.actualJoints().shoulder).toBeCloseTo(home.shoulder + 48, 9);
  });

  it('收敛后停止 tick，不再空转', async () => {
    const { timer, transport } = makeTransport();
    await transport.connect();
    await transport.sendJointState(shoulderShift(48));

    timer.advance(2000);
    expect(transport.actualJoints().shoulder).toBeCloseTo(home.shoulder + 48, 9);
    expect(transport.stats().moving).toBe(false);
    expect(transport.stats().lagDeg).toBeLessThan(1e-9);
    // 收敛后既没有 tick 定时器，也没有在途命令
    expect(timer.pending()).toBe(0);
  });
});

describe('MockTransport · 丢帧', () => {
  it('丢帧只影响上报：位置照常收敛，但 received 为 0', async () => {
    const { timer, transport } = makeTransport({ dropRate: 1, random: () => 0 });
    await transport.connect();
    await transport.sendJointState(shoulderShift(48));

    timer.advance(2000);
    expect(transport.stats().received).toBe(0);
    expect(transport.stats().dropped).toBeGreaterThan(0);
    // 机械臂并不知道上位机没收到 —— 它照样走到了目标
    expect(transport.actualJoints().shoulder).toBeCloseTo(home.shoulder + 48, 9);
  });
});

describe('MockTransport · 限位拒绝', () => {
  it('越界命令被拒绝（ERR JOINT），位置保持不动', async () => {
    const { timer, transport, statuses } = makeTransport({ enforceLimits: true });
    await transport.connect();

    const before = transport.actualJoints();
    // base 限位 ±60，这里给 90
    await transport.sendJointState({ ...home, base: 90 });
    timer.advance(500);

    expect(transport.stats().rejected).toBe(1);
    const error = statuses.find((s) => s.status === 'error');
    expect(error?.reason).toMatch(/^ERR JOINT base 90\.00 \(limit -60\.00\.\.60\.00\)$/);

    const after = transport.actualJoints();
    for (const id of Object.keys(before)) {
      expect(after[id]).toBeCloseTo(before[id], 12);
    }
  });

  it('enforceLimits=false 时越界命令照常执行（模拟不校验的固件）', async () => {
    const { timer, transport } = makeTransport({ enforceLimits: false, latencyMs: 0 });
    await transport.connect();
    await transport.sendJointState({ ...home, base: 90 });
    timer.advance(2000);
    expect(transport.stats().rejected).toBe(0);
    expect(transport.actualJoints().base).toBeCloseTo(90, 9);
  });
});

describe('MockTransport · sendCommand 语义', () => {
  it('home 命令回到 HOME', async () => {
    const { timer, transport } = makeTransport({ latencyMs: 0 });
    await transport.connect();
    await transport.sendJointState({ ...home, base: 40 });
    timer.advance(2000);
    expect(transport.actualJoints().base).toBeCloseTo(40, 9);

    await transport.sendCommand(createSimpleCommand('home'));
    timer.advance(2000);
    expect(transport.actualJoints().base).toBeCloseTo(home.base, 9);
  });

  it('stop 冻结在当前姿态', async () => {
    const { timer, transport } = makeTransport({ latencyMs: 0 });
    await transport.connect();
    await transport.sendJointState(shoulderShift(48));
    timer.advance(20);
    const mid = transport.actualJoints().shoulder;

    await transport.sendCommand(createSimpleCommand('stop'));
    timer.advance(2000);
    expect(transport.actualJoints().shoulder).toBeCloseTo(mid, 9);
  });

  it('emergency_stop 清空在途命令：延迟中的目标不会被应用', async () => {
    const { timer, transport } = makeTransport({ latencyMs: 15 });
    await transport.connect();
    await transport.sendJointState(shoulderShift(48));

    // 命令还在途中
    expect(transport.stats().lagDeg).toBe(0);
    await transport.sendCommand(createSimpleCommand('emergency_stop'));
    timer.advance(2000);

    expect(transport.actualJoints().shoulder).toBeCloseTo(home.shoulder, 9);
  });

  it('gripper 命令只改夹爪，其余关节沿用当前目标', async () => {
    const { timer, transport } = makeTransport({ latencyMs: 0 });
    const gripper = jointByRole(model, 'gripper');
    expect(gripper).toBeDefined();

    await transport.connect();
    await transport.sendJointState(shoulderShift(20));
    timer.advance(2000);
    const shoulderNow = transport.actualJoints().shoulder;

    await transport.sendCommand(createJointCommand({ gripper: 80 }));
    timer.advance(2000);

    expect(transport.actualJoints().gripper).toBeCloseTo(80, 9);
    expect(transport.actualJoints().shoulder).toBeCloseTo(shoulderNow, 9);
  });
});
