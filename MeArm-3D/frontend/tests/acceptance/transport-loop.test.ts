/**
 * 传输闭环验收（Phase 7）。
 *
 * 走**完整链路**：store.commandJoints → 节流 → MockTransport → 回推 → store.actualJoints。
 * 与 `mockTransport.test.ts`（只测 Mock 自身）的分工：
 *   本文件证明的是「接线正确」—— 尤其是 **回环打破** 与 **节流生效** 两件事。
 *
 * 全部用 `FakeTimer` 精确推进，无真实等待。
 */
import { afterEach, beforeEach, describe, expect, it } from 'vitest';
import {
  endEffectorPose,
  homeJointState,
  loadRobotModel,
  movableJoints,
  type JointState,
} from '@robot/index';
import { connectMockTransport, disconnectTransport } from '@/store/transportBridge';
import { useRobotStore } from '@/store/robotStore';
import { FakeTimer } from '../helpers/fakeTimer';

const model = loadRobotModel();
const home = homeJointState(model);

/** 仿真 tick 20ms / 每 tick 4.8° / 延迟 15ms —— 与单测保持同一套参数 */
const TICK_MS = 20;
const MAX_SPEED = 240;
const MAX_STEP_DEG = (MAX_SPEED * TICK_MS) / 1000;

function resetStore(): void {
  const pose = endEffectorPose(model, home);
  useRobotStore.setState({
    commandJoints: { ...home },
    actualJoints: { ...home },
    endEffector: pose,
    actualEndEffector: pose,
    target: [pose.position[0], pose.position[1], pose.position[2]],
    ikStatus: null,
    dragging: false,
    mode: 'simulation',
    controlSource: 'virtual',
    connection: 'disconnected',
    connectionLabel: '未接入',
    transportKind: null,
    transportDriven: false,
    transportStats: null,
    log: [],
  });
}

async function connect(timer: FakeTimer) {
  await connectMockTransport({
    timer,
    tickMs: TICK_MS,
    tuning: { maxSpeedDegPerSec: MAX_SPEED, latencyMs: 15, dropRate: 0, enforceLimits: true },
  });
  return useRobotStore.getState();
}

beforeEach(() => {
  resetStore();
});

afterEach(async () => {
  await disconnectTransport();
});

describe('连接登记', () => {
  it('连接后 store 进入 transportDriven，断开后复位', async () => {
    const timer = new FakeTimer();
    const s = await connect(timer);

    expect(s.connection).toBe('connected');
    expect(s.transportKind).toBe('mock');
    expect(s.transportDriven).toBe(true);
    expect(s.connectionLabel).toContain('MockTransport');

    await disconnectTransport();
    const after = useRobotStore.getState();
    expect(after.connection).toBe('disconnected');
    expect(after.transportDriven).toBe(false);
    expect(after.transportStats).toBeNull();
  });

  it('连接时不会让机械臂"跳"一下（initialJoints 对齐命令）', async () => {
    useRobotStore.getState().setJoint('shoulder', home.shoulder + 20);
    const timer = new FakeTimer();
    await connect(timer);
    const s = useRobotStore.getState();
    expect(s.actualJoints.shoulder).toBeCloseTo(s.commandJoints.shoulder, 9);
  });
});

describe('拖动闭环 · 400 点轨迹', () => {
  it('command 逐帧更新、回推不碰 command、节流生效、松手后收敛', async () => {
    const timer = new FakeTimer();
    await connect(timer);

    const store = () => useRobotStore.getState();
    const FRAMES = 400;
    const from = home.shoulder;
    const to = home.shoulder + 40; // 40.8499 < 限位 49.4549

    let maxLag = 0;
    let maxActualStep = 0;
    let previousActual = store().actualJoints.shoulder;

    for (let i = 0; i < FRAMES; i += 1) {
      const value = from + ((to - from) * i) / (FRAMES - 1);
      store().setJoint('shoulder', value);

      const commandBefore = store().commandJoints;
      expect(commandBefore.shoulder).toBeCloseTo(value, 9);

      // 一帧 4ms
      timer.advance(4);

      const current = store();
      // ★ 回环打破的硬证据：引用都没变 —— 回推完全没有触碰 commandJoints
      expect(current.commandJoints).toBe(commandBefore);

      const step = Math.abs(current.actualJoints.shoulder - previousActual);
      if (step > maxActualStep) maxActualStep = step;
      previousActual = current.actualJoints.shoulder;

      const lag = Math.abs(current.commandJoints.shoulder - current.actualJoints.shoulder);
      if (lag > maxLag) maxLag = lag;
    }

    // 无跳变：单帧位移不超过一个仿真 tick 的最大步长
    expect(maxActualStep).toBeLessThanOrEqual(MAX_STEP_DEG + 1e-9);
    // 12.5°/s 的拖动速度远低于 240°/s 的舵机速度，所以全程跟得上
    expect(maxLag).toBeLessThan(MAX_STEP_DEG + 1e-9);

    // 松手后收敛
    timer.advance(5000);
    const final = store();
    for (const joint of movableJoints(model)) {
      const gap = Math.abs(final.actualJoints[joint.id] - final.commandJoints[joint.id]);
      expect(gap, `关节 ${joint.id} 未收敛`).toBeLessThan(1e-6);
    }
    expect(final.transportStats?.moving).toBe(false);

    // ★ 节流生效：400 次命令变化，实际下发数远少于 400（约 1600ms / 33ms ≈ 49）
    const sent = final.transportStats?.sent ?? 0;
    expect(sent).toBeGreaterThan(10);
    expect(sent).toBeLessThan(80);
    expect(final.transportStats?.received ?? 0).toBeGreaterThan(0);
  });
});

describe('阶跃响应', () => {
  it('瞬间跳 40° 时 Actual 明显滞后，随后按有限速度收敛', async () => {
    const timer = new FakeTimer();
    await connect(timer);

    const store = () => useRobotStore.getState();
    store().setJoint('shoulder', home.shoulder + 40);

    // 延迟 15ms 内命令还没生效，Actual 一动没动
    timer.advance(14);
    expect(store().actualJoints.shoulder).toBeCloseTo(home.shoulder, 9);

    // 命令送达的那一刻立刻走一步（4.8°），此后每 20ms 一步
    timer.advance(1);
    expect(store().actualJoints.shoulder).toBeCloseTo(home.shoulder + MAX_STEP_DEG, 9);

    timer.advance(20);
    expect(store().actualJoints.shoulder).toBeCloseTo(home.shoulder + MAX_STEP_DEG * 2, 9);

    // 40° / 4.8° = 9 步；送达在 15ms，第 9 步在 15 + 8×20 = 175ms
    timer.advance(1000);
    expect(store().actualJoints.shoulder).toBeCloseTo(home.shoulder + 40, 9);
    expect(store().actualJoints.shoulder).toBeCloseTo(store().commandJoints.shoulder, 9);
  });
});

describe('断开语义', () => {
  it('断开后 actual 拉回 command，且不再有回推', async () => {
    const timer = new FakeTimer();
    await connect(timer);
    const store = () => useRobotStore.getState();

    store().setJoint('shoulder', home.shoulder + 30);
    timer.advance(20); // 只走了一步，仍有滞后
    expect(store().actualJoints.shoulder).not.toBeCloseTo(store().commandJoints.shoulder, 3);

    await disconnectTransport();
    const after = useRobotStore.getState();
    // 断开后回到「仿真立即跟随」，不留假误差
    expect(after.actualJoints.shoulder).toBeCloseTo(after.commandJoints.shoulder, 9);
    expect(after.transportDriven).toBe(false);

    // 再推时间也不会有任何回推把 actual 改回去
    const snapshot: JointState = { ...after.actualJoints };
    timer.advance(2000);
    for (const id of Object.keys(snapshot)) {
      expect(useRobotStore.getState().actualJoints[id]).toBeCloseTo(snapshot[id], 12);
    }
  });
});

describe('限位拒绝路径', () => {
  it('store 侧已钳位，因此正常路径不会触发 ERR JOINT', async () => {
    const timer = new FakeTimer();
    await connect(timer);
    const store = () => useRobotStore.getState();

    // 远超限位：store 会钳到 max，Mock 收到的仍是合法值
    store().setJoint('base', 999);
    timer.advance(2000);

    expect(store().transportStats?.rejected).toBe(0);
    expect(store().commandJoints.base).toBeCloseTo(
      model.joints.find((j) => j.id === 'base')?.limits.max ?? 0,
      9,
    );
  });
});
