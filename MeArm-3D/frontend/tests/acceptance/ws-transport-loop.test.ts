/**
 * WebSocket 传输闭环验收（Phase 8）。
 *
 * 与 `webSocketTransport.test.ts`（只测传输自身）的分工：
 *   本文件证明的是「**接线正确**」—— 把 WebSocketTransport 挂到 transportBridge 上之后：
 *     1. 命令节流生效（拖动不会把链路打爆）
 *     2. 回推只写 actualJoints（回环打破）
 *     3. **重连后自动补发当前命令**（否则虚拟臂与实际臂永久错开）
 *     4. 首次连接不补发（不改变 Phase 7 的时序契约）
 *
 * 用 FakeSocket + FakeTimer：无真实网络、无真实等待。
 */
import { afterEach, beforeEach, describe, expect, it } from 'vitest';
import {
  WebSocketTransport,
  endEffectorPose,
  homeJointState,
  loadRobotModel,
} from '@robot/index';
import { TransportBridge } from '@/store/transportBridge';
import { useRobotStore } from '@/store/robotStore';
import { FakeSocketFactory } from '../helpers/fakeSocket';
import { FakeTimer } from '../helpers/fakeTimer';
import { backendInfoFromLocal } from '../helpers/backendModel';

const model = loadRobotModel();
const home = homeJointState(model);
const URL = 'ws://test.local:8090/ws/joint';

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

interface Harness {
  timer: FakeTimer;
  factory: FakeSocketFactory;
  transport: WebSocketTransport;
  bridge: TransportBridge;
  frames(type: string): Array<Record<string, unknown>>;
}

let active: Harness | null = null;

async function connect(): Promise<Harness> {
  const timer = new FakeTimer(1000);
  const factory = new FakeSocketFactory();
  const transport = new WebSocketTransport({
    url: URL,
    model,
    socketFactory: factory.create,
    timer,
    heartbeatIntervalMs: 10_000,
    heartbeatTimeoutMs: 30_000,
    reconnectBaseMs: 500,
  });
  const bridge = new TransportBridge({ transport, timer });
  await bridge.connect();
  active = {
    timer,
    factory,
    transport,
    bridge,
    frames: (type) => factory.last.framesOfType(type),
  };
  return active;
}

/** 连接并完成握手 */
async function connected(): Promise<Harness> {
  const h = await connect();
  h.factory.last.open();
  return h;
}

beforeEach(() => {
  resetStore();
});

afterEach(async () => {
  if (active) {
    await active.bridge.dispose();
    active = null;
  }
});

describe('WebSocket 接线 · 连接登记', () => {
  it('连接后 store 记 websocket 且进入 transportDriven', async () => {
    await connected();
    const s = useRobotStore.getState();
    expect(s.connection).toBe('connected');
    expect(s.transportKind).toBe('websocket');
    expect(s.transportDriven).toBe(true);
  });

  it('断开后复位：actual 拉回 command，统计清空', async () => {
    const h = await connected();
    // 先制造一次滞后
    h.factory.last.deliver({
      version: 1,
      type: 'joint_state',
      joints: { ...home, shoulder: 30 },
    });
    expect(useRobotStore.getState().actualJoints.shoulder).toBeCloseTo(30, 9);

    await h.bridge.dispose();
    active = null;
    const s = useRobotStore.getState();
    expect(s.connection).toBe('disconnected');
    expect(s.transportDriven).toBe(false);
    expect(s.transportStats).toBeNull();
    expect(s.actualJoints.shoulder).toBeCloseTo(home.shoulder, 9);
  });

  it('首次连接**不**补发命令（保持 Phase 7 时序契约）', async () => {
    const h = await connected();
    h.timer.advance(200);
    expect(h.frames('joint_command')).toHaveLength(0);
  });
});

describe('WebSocket 接线 · 节流', () => {
  it('400 次命令变化只下发个位数~十几帧（尾沿合并 33ms）', async () => {
    const h = await connected();
    const store = useRobotStore.getState();
    for (let i = 0; i < 400; i += 1) {
      store.setJoint('shoulder', 10 + (i % 20));
      h.timer.advance(1); // 每 1ms 改一次，共 400ms
    }
    h.timer.advance(100); // 放掉 trailing

    const sent = h.frames('joint_command').length;
    expect(sent).toBeGreaterThan(0);
    expect(sent).toBeLessThan(60); // 400ms / 33ms ≈ 13 帧
    expect(sent).toBeLessThan(400);
  });

  it('最后一帧一定是"最新命令"（trailing 保证不会停在半路）', async () => {
    const h = await connected();
    const store = useRobotStore.getState();
    store.setJoint('shoulder', 20);
    h.timer.advance(1);
    store.setJoint('shoulder', 25);
    h.timer.advance(1);
    store.setJoint('shoulder', 33.3); // 最终值
    h.timer.advance(200);

    const cmds = h.frames('joint_command');
    const last = cmds[cmds.length - 1];
    expect((last.joints as Record<string, number>).shoulder).toBeCloseTo(33.3, 9);
  });
});

describe('WebSocket 接线 · 回环打破', () => {
  it('joint_state 回推只写 actualJoints，commandJoints **引用不变**', async () => {
    const h = await connected();
    const before = useRobotStore.getState().commandJoints;
    for (let i = 0; i < 50; i += 1) {
      h.factory.last.deliver({
        version: 1,
        type: 'joint_state',
        joints: { ...home, shoulder: 5 + i * 0.5 },
      });
    }
    const after = useRobotStore.getState();
    expect(after.commandJoints).toBe(before); // toBe：引用级断言
    expect(after.actualJoints.shoulder).toBeCloseTo(5 + 49 * 0.5, 9);
    expect(after.controlSource).toBe('real');
  });

  it('回推不会反过来触发新的下发（无 命令→状态→命令 回环）', async () => {
    const h = await connected();
    for (let i = 0; i < 30; i += 1) {
      h.factory.last.deliver({
        version: 1,
        type: 'joint_state',
        joints: { ...home, shoulder: 10 + i },
      });
    }
    h.timer.advance(500);
    expect(h.frames('joint_command')).toHaveLength(0);
  });
});

describe('WebSocket 接线 · 重连补发', () => {
  it('断线重连成功后自动补发当前命令', async () => {
    const h = await connected();
    const store = useRobotStore.getState();
    store.setJoint('shoulder', 30);
    h.timer.advance(100);
    const beforeDrop = h.frames('joint_command').length;
    expect(beforeDrop).toBeGreaterThan(0);

    // 网络掉线 → 退避 500ms → 重连 → 握手成功
    h.factory.last.drop(1006, 'network lost');
    h.timer.advance(500);
    expect(h.factory.count).toBe(2);
    h.factory.last.open();
    h.timer.advance(10);

    // 补发发生在新连接上，故按新 socket 计数（旧 socket 的帧不再增长）
    const after = h.factory.last.framesOfType('joint_command');
    expect(after).toHaveLength(1);
    expect((after[0].joints as Record<string, number>).shoulder).toBeCloseTo(30, 9);
    expect(h.transport.stats().reconnects).toBe(1);
  });

  it('断线期间的命令改动也会被补发（不是补发旧值）', async () => {
    const h = await connected();
    const store = useRobotStore.getState();
    store.setJoint('shoulder', 10);
    h.timer.advance(100);

    h.factory.last.drop(1006);
    // 断线期间用户继续拖动
    store.setJoint('shoulder', 42);
    h.timer.advance(500);
    h.factory.last.open();
    h.timer.advance(10);

    // 补发的必须是**当前**命令（42），而不是断线前那条（10）
    const cmds = h.factory.last.framesOfType('joint_command');
    expect(cmds).toHaveLength(1);
    expect((cmds[0].joints as Record<string, number>).shoulder).toBeCloseTo(42, 9);
  });
});

describe('WebSocket 接线 · 状态面板', () => {
  it('hello 后统计里带上 device 与模型一致性结论', async () => {
    const h = await connected();
    h.factory.last.deliver({
      version: 1,
      type: 'hello',
      model: backendInfoFromLocal(model),
      device: 'sim',
    });
    h.timer.advance(300); // 等一次 poll

    const stats = useRobotStore.getState().transportStats;
    expect(stats).not.toBeNull();
    expect(stats?.kind).toBe('websocket');
    expect(stats?.rttMs ?? null).toBeNull();
    const wsStats = stats as { device?: string | null; modelMismatch?: string | null };
    expect(wsStats.device).toBe('sim');
    expect(wsStats.modelMismatch).toBeNull();
  });

  it('限位拒绝回执计入 rejected，连接仍保持', async () => {
    const h = await connected();
    h.factory.last.deliver({
      version: 1,
      type: 'error',
      code: 'JOINT_LIMIT',
      message: 'ERR JOINT elbow 95.00 (limit 108.44..141.86)',
    });
    h.timer.advance(300);
    const stats = useRobotStore.getState().transportStats;
    expect(stats?.rejected).toBe(1);
    expect(useRobotStore.getState().connection).toBe('connected');
  });
});
