/**
 * mode ↔ transport 联动验收（Phase 9 修正 · ADR D41）。
 *
 * 背景：早期实现里 `mode`（Real / Simulation）是**纯 UI 状态** ——
 * `setMode('real')` 只改按钮样式，命令照样走当时连着的 transport。
 * 后果有两条，都很糟：
 *   A. 点了「Real Robot」但没连后端 ⇒ 真机纹丝不动，UI 却显示"真实机械臂"
 *   B. 切回「Simulation」后命令仍下发给真机 ⇒ 以为在仿真，实际在驱动硬件
 *
 * 本文件锁定两条修正后的契约：
 *   1. `setMode('real')` 在链路不具备时**必须拒绝切换**并明确告警（D43 收紧：
 *      v2 只告警不改状态，结果 UI 显示 Real 而末端是 sim —— 用户就报了这个障）
 *   2. `mode === 'simulation'` 时，**真机链路**（websocket + device=serial）不得收到命令；
 *      但仿真链路（mock / device=sim）**必须照常放行**（否则打死 Phase 7/8 闭环）
 *
 * 用 FakeSocket + FakeTimer：无真实网络、无真实等待。
 */
import { afterEach, beforeEach, describe, expect, it } from 'vitest';
import { WebSocketTransport, endEffectorPose, homeJointState, loadRobotModel } from '@robot/index';
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
  /** 最近一段日志文本（用于断言告警/拦截可见） */
  logs(): string;
  cmds(): Array<Record<string, unknown>>;
}

let active: Harness | null = null;

/** 挂上 WebSocket 传输并完成 open + hello（device 可指定 sim / serial） */
async function connect(device: 'sim' | 'serial' | null): Promise<Harness> {
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
  factory.last.open();

  if (device !== null) {
    factory.last.deliver({
      version: 1,
      type: 'hello',
      model: backendInfoFromLocal(model),
      device,
    });
    timer.advance(300); // 让 poll 把 stats（含 device）写进 store
  }

  active = {
    timer,
    factory,
    transport,
    bridge,
    logs: () => useRobotStore.getState().log.map((l) => l.text).join('\n'),
    cmds: () => factory.last.framesOfType('joint_command'),
  };
  return active;
}

/** 纯 mock 传输（用于验证仿真链路不被误拦） */
async function connectMock(): Promise<Harness> {
  const timer = new FakeTimer(1000);
  const bridge = new TransportBridge({ timer });
  await bridge.connect();
  active = {
    timer,
    factory: null as unknown as FakeSocketFactory,
    transport: null as unknown as WebSocketTransport,
    bridge,
    logs: () => useRobotStore.getState().log.map((l) => l.text).join('\n'),
    cmds: () => [],
  };
  return active;
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

// ---------------------------------------------------------------------------
// 一、setMode('real') 的准入校验：不具备条件时必须**拒绝切换**
//
// ⚠️ D43 收紧：早期（D41）这里断言的是"模式本身仍然切换（用户意图被记录），
//    但告警必须出现" —— 也就是 `set({ mode })` 留在校验之前。那等于校验只发
//    日志、不拦状态，UI 于是显示「Real」高亮而链路末端是 sim。用户报了障，
//    测试还把这个行为背书成了"设计"。现在改为：**拒绝 + 保持 Simulation**。
// ---------------------------------------------------------------------------

describe('mode 联动 · Real Robot 准入校验', () => {
  it('未连接任何传输时点 Real Robot → 拒绝切换，保持 Simulation，并告警', () => {
    useRobotStore.getState().setMode('real');
    const logs = useRobotStore.getState().log.map((l) => l.text).join('\n');
    expect(logs).toContain('未连接');
    // ★ 关键：不得切过去。所见即所是。
    expect(useRobotStore.getState().mode).toBe('simulation');
  });

  it('连着 MockTransport 时点 Real Robot → 拒绝切换，告警指明是浏览器内仿真', async () => {
    await connectMock();
    useRobotStore.getState().setMode('real');
    const logs = useRobotStore.getState().log.map((l) => l.text).join('\n');
    expect(logs).toContain('MockTransport');
    expect(useRobotStore.getState().mode).toBe('simulation');
  });

  it('连着后端但末端是 sim 时点 Real Robot → 拒绝切换，告警指明末端非 serial', async () => {
    await connect('sim');
    useRobotStore.getState().setMode('real');
    const logs = useRobotStore.getState().log.map((l) => l.text).join('\n');
    expect(logs).toMatch(/末端是「sim」|非真机|serial/);
    expect(useRobotStore.getState().mode).toBe('simulation');
  });

  it('已连 WebSocket 但 hello 未到（device 未知）→ 拒绝切换（不乐观放行）', async () => {
    const h = await connect(null); // 不发 hello ⇒ device 未知
    expect(h.transport?.kind).toBe('websocket');
    useRobotStore.getState().setMode('real');
    const logs = useRobotStore.getState().log.map((l) => l.text).join('\n');
    expect(logs).toContain('链路末端未知');
    // D43：早期这里会乐观放行 ⇒ 「Real」高亮而末端实为 sim。现在必须拒绝。
    expect(useRobotStore.getState().mode).toBe('simulation');
  });

  it('连着真机（device=serial）时点 Real Robot → 切换成功', async () => {
    await connect('serial');
    useRobotStore.getState().setMode('real');
    const logs = useRobotStore.getState().log.map((l) => l.text).join('\n');
    expect(logs).toContain('切换到 Real Robot');
    expect(useRobotStore.getState().mode).toBe('real');
  });

  it('device 从 sim 变为 serial 后再点 → 这次成功（拒绝不是永久黑名单）', async () => {
    const h = await connect('sim');
    useRobotStore.getState().setMode('real');
    expect(useRobotStore.getState().mode).toBe('simulation');

    // 后端换成真机（重连并上报 serial）
    h.factory.last.deliver({
      version: 1,
      type: 'hello',
      model: backendInfoFromLocal(model),
      device: 'serial',
    });
    h.timer.advance(300);

    useRobotStore.getState().setMode('real');
    expect(useRobotStore.getState().mode).toBe('real');
  });
});

// ---------------------------------------------------------------------------
// 二、安全门：simulation 模式下，**真机链路**不得收到命令
// ---------------------------------------------------------------------------

describe('mode 联动 · Safety gate（simulation 不下发给真机）', () => {
  it('真机链路 + simulation → 命令被拦截，且日志可见', async () => {
    const h = await connect('serial');
    expect(useRobotStore.getState().mode).toBe('simulation');

    useRobotStore.getState().setJoint('shoulder', 20);
    h.timer.advance(200);

    expect(h.cmds().length).toBe(0); // 一条都没出去
    expect(h.logs()).toContain('已拦截');
  });

  it('真机链路 + 切到 real → 命令正常下发', async () => {
    const h = await connect('serial');
    useRobotStore.getState().setMode('real');

    useRobotStore.getState().setJoint('shoulder', 20);
    h.timer.advance(200);

    const cmds = h.cmds();
    expect(cmds.length).toBeGreaterThan(0);
    expect((cmds[0].joints as Record<string, number>).shoulder).toBeCloseTo(20, 9);
  });

  it('切回 simulation → 再次被拦截（"以为在仿真其实在动真机"必须不可能）', async () => {
    const h = await connect('serial');
    useRobotStore.getState().setMode('real');
    useRobotStore.getState().setJoint('shoulder', 10);
    h.timer.advance(200);
    const afterReal = h.cmds().length;
    expect(afterReal).toBeGreaterThan(0);

    useRobotStore.getState().setMode('simulation');
    useRobotStore.getState().setJoint('shoulder', 25);
    h.timer.advance(200);

    // 仍然只有 real 模式那一批，simulation 下没有新增
    expect(h.cmds().length).toBe(afterReal);
    expect(h.logs()).toContain('已拦截');
  });

  it('sim 末端（device=sim）在 simulation 下**照常放行**（不能打死 Phase 8 仿真闭环）', async () => {
    const h = await connect('sim');
    useRobotStore.getState().setJoint('shoulder', 15);
    h.timer.advance(200);
    expect(h.cmds().length).toBeGreaterThan(0);
  });

  it('MockTransport 在 simulation 下**照常放行**（Phase 7 行为不变）', async () => {
    const h = await connectMock();
    useRobotStore.getState().setJoint('shoulder', 15);
    h.timer.advance(200);
    // mock 的发送不落 FakeSocket，看日志里的 out 即可
    expect(h.logs()).toContain('joint_command');
  });

  it('device 未知（尚未 hello）时**不拦**（宁可少拦，不要误判仿真）', async () => {
    const h = await connect(null); // 不发 hello ⇒ device 未知
    useRobotStore.getState().setJoint('shoulder', 12);
    h.timer.advance(200);
    expect(h.cmds().length).toBeGreaterThan(0);
  });
});
