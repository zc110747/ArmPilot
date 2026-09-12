/**
 * WebSocketTransport 单测（Phase 8）。
 *
 * 全部用 `FakeTimer` + `FakeSocket` 驱动 —— 没有一处真实等待，也没有一次真实网络：
 * 「心跳超时判死」「指数退避重连」「旧 socket 迟到回调不得把状态拽回去」
 * 这类最容易出错的时序逻辑因此是**确定性**的。
 */
import { describe, expect, it } from 'vitest';
import {
  SOCKET_CLOSING,
  WebSocketTransport,
  endEffectorPose,
  loadRobotModel,
  quantizeForWire,
  quantizeViaServo,
  type JointState,
  type RobotState,
  type TransportStatusDetail,
  type WebSocketTransportOptions,
} from '@robot/index';
import { FakeSocketFactory } from '../helpers/fakeSocket';
import { FakeTimer } from '../helpers/fakeTimer';
import { backendInfoFromLocal } from '../helpers/backendModel';

const model = loadRobotModel();
const URL = 'ws://test.local:8090/ws/joint';

interface Harness {
  timer: FakeTimer;
  factory: FakeSocketFactory;
  transport: WebSocketTransport;
  states: RobotState[];
  statuses: TransportStatusDetail[];
  /** 状态序列（只取 status 字段） */
  seq(): string[];
  errors(): string[];
}

function setup(options: Partial<WebSocketTransportOptions> = {}): Harness {
  const timer = new FakeTimer(1000);
  const factory = new FakeSocketFactory();
  const transport = new WebSocketTransport({
    url: URL,
    model,
    socketFactory: factory.create,
    timer,
    ...options,
  });
  const states: RobotState[] = [];
  const statuses: TransportStatusDetail[] = [];
  transport.onState((s) => states.push(s));
  transport.onStatus((d) => statuses.push(d));
  return {
    timer,
    factory,
    transport,
    states,
    statuses,
    seq: () => statuses.map((s) => s.status),
    errors: () => statuses.filter((s) => s.status === 'error').map((s) => s.reason ?? ''),
  };
}

const jointsAt = (shoulder: number): JointState => ({
  base: 0,
  shoulder,
  elbow: 112.6185771989,
  gripper: 50,
});

/** HOME 位的**全精度**关节角（前端内部值；线上会被量化成 `0 / 0.8 / 112.6 / 50`） */
const HOME_FULL: JointState = {
  base: 0,
  shoulder: 0.8498937633,
  elbow: 112.6185771989,
  gripper: 50,
};

/** 后端回推值只可能是线上值（JR 1 位小数 → 舵机角 → 反算 → STATE 2 位小数） */
const onWire = (joints: JointState): JointState => quantizeForWire(joints);

async function connected(opts: Partial<WebSocketTransportOptions> = {}): Promise<Harness> {
  const h = setup(opts);
  await h.transport.connect();
  h.factory.last.open();
  return h;
}

describe('WebSocketTransport · 连接', () => {
  it('connect → connecting → open → connected', async () => {
    const h = setup();
    await h.transport.connect();
    expect(h.transport.status()).toBe('connecting');
    expect(h.factory.count).toBe(1);
    expect(h.factory.urls).toEqual([URL]);

    h.factory.last.open();
    expect(h.transport.status()).toBe('connected');
    expect(h.seq()).toEqual(['connecting', 'connected']);
  });

  it('连上后主动请求一次 hello + 状态（不依赖服务端推送时序）', async () => {
    const h = await connected();
    expect(h.factory.last.framesOfType('status_request')).toHaveLength(1);
  });

  it('重复 connect 是幂等的（不会建出第二个 socket）', async () => {
    const h = await connected();
    await h.transport.connect();
    expect(h.factory.count).toBe(1);
  });
});

describe('WebSocketTransport · 模型一致性校验', () => {
  it('hello 与本地一致 → 无 error，记录 device', async () => {
    const h = await connected();
    h.factory.last.deliver({
      version: 1,
      type: 'hello',
      timestamp: 1,
      model: backendInfoFromLocal(model),
      device: 'sim',
    });
    expect(h.errors()).toEqual([]);
    expect(h.transport.stats().modelMismatch).toBeNull();
    expect(h.transport.stats().device).toBe('sim');
    expect(h.transport.status()).toBe('connected');
  });

  it('hello 里限位与本地不符 → 报警但**不断开**（链路仍可用）', async () => {
    const h = await connected();
    const info = backendInfoFromLocal(model);
    info.limits.find((l) => l.id === 'elbow')!.min = 0; // 典型的"套用 0..90 通例"错误
    h.factory.last.deliver({ version: 1, type: 'hello', model: info, device: 'sim' });

    const mismatch = h.transport.stats().modelMismatch;
    expect(mismatch).toContain('elbow');
    expect(h.errors().some((r) => r.includes('模型/标定不一致'))).toBe(true);
    expect(h.transport.status()).toBe('connected');
  });

  it('hello 里舵机通道颠倒 → 能测出（S7/S8 角色错的真实风险）', async () => {
    const h = await connected();
    const info = backendInfoFromLocal(model);
    const sh = info.calibration.find((c) => c.jointId === 'shoulder')!;
    const el = info.calibration.find((c) => c.jointId === 'elbow')!;
    [sh.channel, el.channel] = [el.channel, sh.channel];
    h.factory.last.deliver({ version: 1, type: 'hello', model: info });
    expect(h.transport.stats().modelMismatch).toContain('通道不一致');
  });
});

describe('WebSocketTransport · 接收状态', () => {
  it('joint_state → onState，且 endEffector 由 FK 得到（与场景同一份几何）', async () => {
    const h = await connected();
    const joints = jointsAt(20.8);
    h.factory.last.deliver({
      version: 1,
      type: 'joint_state',
      timestamp: 1234,
      joints,
    });
    expect(h.states).toHaveLength(1);
    const state = h.states[0];
    expect(state.source).toBe('real');
    expect(state.timestamp).toBe(1234);
    expect(state.joints.shoulder).toBeCloseTo(20.8, 12);
    const expected = endEffectorPose(model, joints);
    expect(state.endEffector.position[0]).toBeCloseTo(expected.position[0], 12);
    expect(state.endEffector.position[2]).toBeCloseTo(expected.position[2], 12);
  });

  it('坏 JSON → 丢弃并报错，绝不产生状态（否则 Actual 可能变 NaN）', async () => {
    const h = await connected();
    h.factory.last.deliver('{ 这不是 JSON');
    expect(h.states).toHaveLength(0);
    expect(h.errors().some((r) => r.includes('无法解析'))).toBe(true);
  });

  it('版本不匹配 → 丢弃（字段语义会静默错解）', async () => {
    const h = await connected();
    h.factory.last.deliver({ version: 99, type: 'joint_state', joints: jointsAt(10) });
    expect(h.states).toHaveLength(0);
  });

  it('joint_state 缺 joints 字段 → 忽略', async () => {
    const h = await connected();
    h.factory.last.deliver({ version: 1, type: 'joint_state', timestamp: 1 });
    expect(h.states).toHaveLength(0);
  });

  it('error 是事件而非终态：计入 rejected，链路保持 connected', async () => {
    const h = await connected();
    h.factory.last.deliver({
      version: 1,
      type: 'error',
      code: 'JOINT_LIMIT',
      message: 'ERR JOINT elbow 95.00 (limit 108.44..141.86)',
    });
    expect(h.transport.stats().rejected).toBe(1);
    expect(h.errors().some((r) => r.includes('ERR JOINT elbow'))).toBe(true);
    expect(h.transport.status()).toBe('connected');
  });

  it('device_status connected=false → 报错', async () => {
    const h = await connected();
    h.factory.last.deliver({
      version: 1,
      type: 'device_status',
      device: 'serial',
      connected: false,
      message: '串口未插入',
    });
    expect(h.errors().some((r) => r.includes('链路末端不可用'))).toBe(true);
    expect(h.transport.stats().device).toBe('serial');
  });

  it('未知 type → 静默忽略（向后兼容）', async () => {
    const h = await connected();
    h.factory.last.deliver({ version: 1, type: 'future_thing', payload: 1 });
    expect(h.states).toHaveLength(0);
    expect(h.errors()).toEqual([]);
  });
});

describe('WebSocketTransport · 发送命令', () => {
  it('sendJointState → joint_command 帧，seq 单调递增', async () => {
    const h = await connected();
    await h.transport.sendJointState(jointsAt(10));
    await h.transport.sendJointState(jointsAt(20));
    const cmds = h.factory.last.framesOfType('joint_command');
    expect(cmds).toHaveLength(2);
    expect(cmds.map((c) => c.seq)).toEqual([1, 2]);
    expect(h.transport.stats().sent).toBe(2);
    expect(cmds[1].joints).toEqual(jointsAt(20));
  });

  it('未连接时 sendJointState → 报错且一个字节都不发', async () => {
    const h = setup();
    await h.transport.sendJointState(jointsAt(10));
    expect(h.factory.count).toBe(0);
    expect(h.errors().some((r) => r.includes('未连接'))).toBe(true);
    expect(h.transport.stats().sent).toBe(0);
  });

  it('socket 非 OPEN 时发送计入 dropped（不抛异常打断上层）', async () => {
    const h = await connected();
    h.factory.last.readyState = SOCKET_CLOSING;
    await h.transport.sendJointState(jointsAt(10));
    expect(h.transport.stats().dropped).toBe(1);
    expect(h.factory.last.framesOfType('joint_command')).toHaveLength(0);
  });

  it('lagDeg = |最近状态 − 最近命令| 的最大值；追上后 moving 变 false', async () => {
    const h = await connected();
    await h.transport.sendJointState(jointsAt(30));
    h.factory.last.deliver({ version: 1, type: 'joint_state', joints: jointsAt(10) });
    expect(h.transport.stats().lagDeg).toBeCloseTo(20, 9);
    expect(h.transport.stats().moving).toBe(true);

    // ⚠️ 回推值只可能是**线上值**：JR 只到 0.1° → 舵机角 → 反算 → STATE 2 位小数。
    //    后端永远不可能回推 112.6185771989 这种全精度值。
    h.factory.last.deliver({ version: 1, type: 'joint_state', joints: onWire(jointsAt(30)) });
    expect(h.transport.stats().lagDeg).toBe(0);
    expect(h.transport.stats().moving).toBe(false);
  });

  it('命令侧按线上精度（0.1°）比对：全精度命令 vs 线上回显 ⇒ 误差恰为 0', async () => {
    // 实测踩到的**假误差**：内部 elbow = 112.6185771989，线上（及回推）= 112.6，
    // 直接相减恒为 0.0186° ⇒ 面板永久显示「跟踪误差 0.02°」、
    // 「正在逼近目标」永不熄灭（e2e 抓到的就是这条）。
    const h = await connected();
    await h.transport.sendJointState(HOME_FULL);
    h.factory.last.deliver({ version: 1, type: 'joint_state', joints: onWire(HOME_FULL) });
    expect(h.transport.stats().lagDeg).toBe(0);
    expect(h.transport.stats().moving).toBe(false);
  });

  it('线上精度之外的**真实**误差仍照实上报（不因量化被抹掉）', async () => {
    const h = await connected();
    await h.transport.sendJointState(jointsAt(30));
    h.factory.last.deliver({ version: 1, type: 'joint_state', joints: onWire(jointsAt(28)) });
    expect(h.transport.stats().lagDeg).toBeCloseTo(2, 9);
    expect(h.transport.stats().moving).toBe(true);
  });
});

describe('WebSocketTransport · 心跳', () => {
  it('按间隔发 ping，收到 pong 记录 RTT', async () => {
    const h = await connected({ heartbeatIntervalMs: 1000, heartbeatTimeoutMs: 3000 });
    expect(h.factory.last.framesOfType('ping')).toHaveLength(0);

    h.timer.advance(1000); // t=2000
    expect(h.factory.last.framesOfType('ping')).toHaveLength(1);
    expect(h.transport.stats().rttMs).toBeNull();

    h.timer.advance(50); // t=2050
    h.factory.last.deliver({ version: 1, type: 'pong' });
    expect(h.transport.stats().rttMs).toBe(50);
  });

  it('ping 在途时不重复发（否则超时判定会被自己不断推后）', async () => {
    const h = await connected({ heartbeatIntervalMs: 1000, heartbeatTimeoutMs: 100_000 });
    h.timer.advance(1000);
    h.timer.advance(1000);
    h.timer.advance(1000);
    expect(h.factory.last.framesOfType('ping')).toHaveLength(1);
  });

  it('超过心跳超时未收到 pong → 判死、主动断开并重连', async () => {
    const h = await connected({
      heartbeatIntervalMs: 1000,
      heartbeatTimeoutMs: 3000,
      reconnectBaseMs: 500,
    });
    h.timer.advance(1000); // t=2000 发 ping#1
    h.timer.advance(5000); // 期间无 pong → t=7000 超时

    expect(h.errors().some((r) => r.includes('心跳超时'))).toBe(true);
    expect(h.factory.count).toBe(2);

    // 旧 socket 是被主动 close(4000) 的
    expect(h.factory.sockets[0].closeCall?.code).toBe(4000);
  });
});

describe('WebSocketTransport · 重连', () => {
  it('意外断开 → 按指数退避重连：500 → 1000 → 2000', async () => {
    const h = await connected({ reconnectBaseMs: 500, reconnectMaxMs: 8000 });

    h.factory.last.drop(1006, 'network lost');
    expect(h.errors().some((r) => r.includes('连接断开'))).toBe(true);
    h.timer.advance(499);
    expect(h.factory.count).toBe(1);
    h.timer.advance(1);
    expect(h.factory.count).toBe(2);

    h.factory.last.drop(1006); // 第二个 socket 未 open 就断
    h.timer.advance(999);
    expect(h.factory.count).toBe(2);
    h.timer.advance(1);
    expect(h.factory.count).toBe(3);

    h.factory.last.drop(1006);
    h.timer.advance(1999);
    expect(h.factory.count).toBe(3);
    h.timer.advance(1);
    expect(h.factory.count).toBe(4);
    expect(h.transport.stats().reconnects).toBe(3);
  });

  it('退避封顶在 reconnectMaxMs', async () => {
    const h = await connected({ reconnectBaseMs: 1000, reconnectMaxMs: 2000 });
    for (let i = 0; i < 4; i += 1) {
      h.factory.last.drop(1006);
      h.timer.advance(2000);
    }
    expect(h.factory.count).toBe(5);
    // 最后一次等待仍是 2000（而不是 16000）
    const before = h.factory.count;
    h.factory.last.drop(1006);
    h.timer.advance(1999);
    expect(h.factory.count).toBe(before);
    h.timer.advance(1);
    expect(h.factory.count).toBe(before + 1);
  });

  it('重连成功后 attempt 归零（下一次断线仍从最短间隔开始）', async () => {
    const h = await connected({ reconnectBaseMs: 500 });
    h.factory.last.drop(1006);
    h.timer.advance(500); // socket#2
    h.factory.last.open(); // 连上 → attempt 归零
    h.factory.last.drop(1006);
    h.timer.advance(500); // 应仍是 500ms 后，而不是 1000
    expect(h.factory.count).toBe(3);
  });

  it('重连次数达上限 → 进入 error 且停止重连', async () => {
    const h = await connected({ reconnectBaseMs: 100, maxReconnectAttempts: 2 });
    h.factory.last.drop(1006);
    h.timer.advance(100); // socket#2
    h.factory.last.drop(1006);
    h.timer.advance(200); // socket#3
    h.factory.last.drop(1006);

    expect(h.transport.status()).toBe('error');
    expect(h.errors().some((r) => r.includes('重连次数已达上限'))).toBe(true);
    h.timer.advance(10_000);
    expect(h.factory.count).toBe(3);
  });

  it('显式 disconnect 不触发重连（用户点"断开"不该被连回来）', async () => {
    const h = await connected({ reconnectBaseMs: 500 });
    await h.transport.disconnect();
    expect(h.transport.status()).toBe('disconnected');
    expect(h.seq()[h.seq().length - 1]).toBe('disconnected');

    h.timer.advance(60_000);
    expect(h.factory.count).toBe(1);
    expect(h.transport.stats().reconnects).toBe(0);
  });

  it('旧 socket 的迟到 onopen 不得把状态拽回 connected', async () => {
    const h = await connected({ reconnectBaseMs: 500 });
    const stale = h.factory.last;
    stale.drop(1006);
    h.timer.advance(500); // socket#2（当前 socket）
    expect(h.factory.count).toBe(2);
    expect(h.transport.status()).toBe('connecting');

    // 过期的 socket#1 此时才把 open 回调送达
    stale.open();
    expect(h.transport.status()).toBe('connecting');
    expect(h.transport.status()).not.toBe('connected');
  });
});

describe('WebSocketTransport · 连接超时', () => {
  it('握手迟迟不完成 → 主动放弃并重连', async () => {
    const h = setup({ connectTimeoutMs: 5000, reconnectBaseMs: 500 });
    await h.transport.connect(); // socket#1，不 open
    h.timer.advance(5000);
    expect(h.errors().some((r) => r.includes('连接超时'))).toBe(true);
    expect(h.factory.sockets[0].closeCall?.code).toBe(4000);

    h.timer.advance(500);
    expect(h.factory.count).toBe(2);
  });

  it('超时前完成握手就不该被放弃', async () => {
    const h = setup({ connectTimeoutMs: 5000 });
    await h.transport.connect();
    h.factory.last.open();
    h.timer.advance(10_000);
    expect(h.transport.status()).toBe('connected');
    expect(h.factory.count).toBe(1);
  });
});

// ---------------------------------------------------------------------------
// Phase 9：真机（device === 'serial'）的到位判定
// ---------------------------------------------------------------------------

describe('WebSocketTransport · 真机到位判定', () => {
  /** 让 transport 进入 serial 模式（hello 里带 device:'serial'） */
  const asSerial = (h: Harness) => {
    h.factory.last.deliver({
      version: 1,
      type: 'hello',
      timestamp: 1,
      model: backendInfoFromLocal(model),
      device: 'serial',
    });
  };

  it('固件按整数舵机取整后回推 → 跟踪误差归零，moving 不永亮', async () => {
    const h = await connected();
    asSerial(h);

    await h.transport.sendJointState(jointsAt(30));
    // 后端把命令经整数舵机量化后再回推（真机固件只吃整数度）
    const echoed = quantizeViaServo(model, jointsAt(30));
    // 再叠加 EncodeState 的 %.2f 格式化
    const printed: JointState = {};
    for (const [id, v] of Object.entries(echoed)) printed[id] = Math.round(v * 100) / 100;
    h.factory.last.deliver({ version: 1, type: 'joint_state', joints: printed });

    expect(h.transport.stats().lagDeg).toBeLessThan(0.02);
    expect(h.transport.stats().moving).toBe(false);
  });

  it('⚠️ 反例：不按真机口径归整（用 sim 的 0.1° 格点）会留下 0.35° 假误差', async () => {
    const h = await connected();
    asSerial(h);

    await h.transport.sendJointState(jointsAt(30));
    const echoed = quantizeViaServo(model, jointsAt(30));
    const printed: JointState = {};
    for (const [id, v] of Object.entries(echoed)) printed[id] = Math.round(v * 100) / 100;
    h.factory.last.deliver({ version: 1, type: 'joint_state', joints: printed });

    // 若拿 sim 的口径（0.1° 文本格点）当比较基准，残差落在 0.02~0.4 之间
    const wrong = quantizeForWire(jointsAt(30));
    let worst = 0;
    for (const id of Object.keys(wrong)) {
      worst = Math.max(worst, Math.abs((printed[id] ?? 0) - (wrong[id] ?? 0)));
    }
    expect(worst).toBeGreaterThan(0.02);
  });

  it('真机模式下仍能识别"真的没到位"（容差不掩盖真实滞后）', async () => {
    const h = await connected();
    asSerial(h);

    await h.transport.sendJointState(jointsAt(30));
    // 固件报告它还在 10°（差 20°）—— 远超 0.02° 容差
    h.factory.last.deliver({ version: 1, type: 'joint_state', joints: jointsAt(10) });

    expect(h.transport.stats().lagDeg).toBeGreaterThan(15);
    expect(h.transport.stats().moving).toBe(true);
  });
});
