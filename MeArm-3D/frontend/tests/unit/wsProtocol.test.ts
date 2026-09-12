/**
 * wsProtocol 单测（Phase 8）—— 纯函数，无 socket / 无计时器。
 *
 * 重点在两件容易出错的事：
 *   1. **坏消息必须被拒**：一条 NaN 关节角会把 Actual 污染成 NaN，且界面上看不出来。
 *   2. **模型一致性校验要真的能测出差异**：这是"标定表只有一份"唯一的在线检验手段。
 */
import { describe, expect, it } from 'vitest';
import {
  CODE_JOINT_LIMIT,
  PROTOCOL_VERSION,
  SERVER_HELLO,
  SERVER_JOINT_STATE,
  SERVER_PONG,
  WIRE_JOINT_STEP_DEG,
  decodeServer,
  describeModelMismatch,
  encodeJointCommand,
  encodePing,
  encodeStatusRequest,
  isError,
  isHello,
  isJointState,
  isPong,
  jointToServo,
  loadRobotModel,
  quantizeForWire,
  quantizeViaServo,
  servoToJoint,
} from '@robot/index';
import { backendInfoFromLocal } from '../helpers/backendModel';

const model = loadRobotModel();

describe('wsProtocol · 编码', () => {
  it('joint_command 带 version / type / timestamp / joints', () => {
    const raw = encodeJointCommand(
      { base: 0, shoulder: 20.8, elbow: 112.6, gripper: 50 },
      { seq: 7, timestamp: 1234 },
    );
    const msg = JSON.parse(raw) as Record<string, unknown>;
    expect(msg.version).toBe(PROTOCOL_VERSION);
    expect(msg.type).toBe('joint_command');
    expect(msg.timestamp).toBe(1234);
    expect(msg.seq).toBe(7);
    expect(msg.joints).toEqual({ base: 0, shoulder: 20.8, elbow: 112.6, gripper: 50 });
  });

  it('seq 是可选扩展：不传时不出现该字段（老后端也吃得下）', () => {
    const msg = JSON.parse(encodeJointCommand({ base: 0 })) as Record<string, unknown>;
    expect('seq' in msg).toBe(false);
    expect(typeof msg.timestamp).toBe('number');
  });

  it('ping / status_request 形状正确', () => {
    const ping = JSON.parse(encodePing(500)) as Record<string, unknown>;
    expect(ping).toMatchObject({ version: PROTOCOL_VERSION, type: 'ping', timestamp: 500 });
    const status = JSON.parse(encodeStatusRequest(600)) as Record<string, unknown>;
    expect(status).toMatchObject({
      version: PROTOCOL_VERSION,
      type: 'status_request',
      timestamp: 600,
    });
  });
});

describe('wsProtocol · 解码（坏消息必须被拒）', () => {
  it('合法消息解出信封', () => {
    const env = decodeServer(
      JSON.stringify({ version: 1, type: SERVER_JOINT_STATE, joints: { base: 1 } }),
    );
    expect(env?.type).toBe(SERVER_JOINT_STATE);
    expect(env?.joints).toEqual({ base: 1 });
  });

  it('非法 JSON → null', () => {
    expect(decodeServer('{ 这不是 JSON')).toBeNull();
    expect(decodeServer('')).toBeNull();
  });

  it('非对象 → null', () => {
    expect(decodeServer('123')).toBeNull();
    expect(decodeServer('null')).toBeNull();
    expect(decodeServer('"str"')).toBeNull();
  });

  it('缺 type / type 为空 → null（不能让无名消息混进来）', () => {
    expect(decodeServer(JSON.stringify({ version: 1, joints: { base: 1 } }))).toBeNull();
    expect(decodeServer(JSON.stringify({ version: 1, type: '' }))).toBeNull();
  });

  it('版本不匹配 → null（字段语义会静默错解）', () => {
    expect(decodeServer(JSON.stringify({ version: 99, type: SERVER_PONG }))).toBeNull();
  });

  it('版本缺省视为兼容（老后端不带 version 也能用）', () => {
    const env = decodeServer(JSON.stringify({ type: SERVER_PONG }));
    expect(env?.type).toBe(SERVER_PONG);
  });

  it('类型守卫', () => {
    const ws = decodeServer(
      JSON.stringify({ version: 1, type: SERVER_JOINT_STATE, joints: { base: 0 } }),
    )!;
    expect(isJointState(ws)).toBe(true);
    expect(isError(ws)).toBe(false);
    // 有 type=joint_state 但没有 joints 时不算状态帧
    const broken = decodeServer(JSON.stringify({ version: 1, type: SERVER_JOINT_STATE }))!;
    expect(isJointState(broken)).toBe(false);
    // 有 type=hello 但没有 model 时不算 hello
    const helloNoModel = decodeServer(JSON.stringify({ version: 1, type: SERVER_HELLO }))!;
    expect(isHello(helloNoModel)).toBe(false);
    // pong 是心跳应答，与状态无关
    const pong = decodeServer(JSON.stringify({ version: 1, type: SERVER_PONG }))!;
    expect(isPong(pong)).toBe(true);
    expect(isJointState(pong)).toBe(false);
  });
});

describe('wsProtocol · 模型一致性校验', () => {
  it('与本地一致的元数据 → null', () => {
    expect(describeModelMismatch(model, backendInfoFromLocal(model))).toBeNull();
  });

  it('关节顺序不同 → 报错（JR 位次会错位，最危险）', () => {
    const info = backendInfoFromLocal(model);
    info.jointOrder = [...info.jointOrder].reverse();
    const msg = describeModelMismatch(model, info);
    expect(msg).toContain('顺序不一致');
  });

  it('关节数量不同 → 报错', () => {
    const info = backendInfoFromLocal(model);
    info.jointOrder = info.jointOrder.slice(0, 3);
    info.limits = info.limits.slice(0, 3);
    const msg = describeModelMismatch(model, info);
    expect(msg).toContain('关节数量不一致');
  });

  it('限位不同 → 报错（一侧放行另一侧拒绝）', () => {
    const info = backendInfoFromLocal(model);
    const elbow = info.limits.find((l) => l.id === 'elbow')!;
    elbow.min = 0; // 典型的"套用通例 0..90"错误
    const msg = describeModelMismatch(model, info);
    expect(msg).toContain('elbow');
    expect(msg).toContain('限位不一致');
  });

  it('舵机通道不同 → 报错（S7/S8 角色颠倒就是这种）', () => {
    const info = backendInfoFromLocal(model);
    const shoulder = info.calibration.find((c) => c.jointId === 'shoulder')!;
    const elbow = info.calibration.find((c) => c.jointId === 'elbow')!;
    [shoulder.channel, elbow.channel] = [elbow.channel, shoulder.channel];
    const msg = describeModelMismatch(model, info);
    expect(msg).toContain('通道不一致');
  });

  it('标定参数不同 → 报错', () => {
    const info = backendInfoFromLocal(model);
    const elbow = info.calibration.find((c) => c.jointId === 'elbow')!;
    elbow.scale *= 1.1;
    const msg = describeModelMismatch(model, info);
    expect(msg).toContain('标定参数不一致');
  });

  it('后端多出本地没有的关节 → 报错', () => {
    const info = backendInfoFromLocal(model);
    info.limits = [...info.limits, { id: 'wrist', role: 'wrist', min: 0, max: 90 }];
    const msg = describeModelMismatch(model, info);
    // 数量先被检出（limits 多一条但 jointOrder 没变 → 走到逐项比对才报未知关节）
    expect(msg === null || msg.length > 0).toBe(true);
    expect(describeModelMismatch(model, info)).not.toBeNull();
  });

  it('限位差异在 1e-6 以内视为一致（浮点容差）', () => {
    const info = backendInfoFromLocal(model);
    info.limits[0].min += 1e-9;
    expect(describeModelMismatch(model, info)).toBeNull();
  });
});

describe('wsProtocol · 错误码常量', () => {
  it('JOINT_LIMIT 常量与后端一致', () => {
    expect(CODE_JOINT_LIMIT).toBe('JOINT_LIMIT');
  });
});

describe('wsProtocol · 链路精度（quantizeForWire）', () => {
  it('线上步进是 0.1°（与 serial-v1.md §4 的 `JR` 精度一致）', () => {
    expect(WIRE_JOINT_STEP_DEG).toBe(0.1);
  });

  it('四舍五入到 1 位小数', () => {
    expect(quantizeForWire({ shoulder: 29.9063172659 })).toEqual({ shoulder: 29.9 });
    expect(quantizeForWire({ elbow: 112.6185771989 })).toEqual({ elbow: 112.6 });
    expect(quantizeForWire({ elbow: 141.8582211436 })).toEqual({ elbow: 141.9 });
    expect(quantizeForWire({ base: 0, gripper: 50 })).toEqual({ base: 0, gripper: 50 });
  });

  it('HOME 位全精度 → 线上值（0 / 0.8 / 112.6 / 50）', () => {
    expect(
      quantizeForWire({
        base: 0,
        shoulder: 0.8498937633,
        elbow: 112.6185771989,
        gripper: 50,
      }),
    ).toEqual({ base: 0, shoulder: 0.8, elbow: 112.6, gripper: 50 });
  });

  it('⚠️ 量化结果必须与 JSON 字面量**逐位相同**（否则"误差 0"会退化成 1e-14）', () => {
    // 这条守着实现细节：`Math.round(v / 0.1) * 0.1` 会因 0.1 不精确而差 1 ulp，
    // 让 quantizeForWire(cmd) 与 JSON.parse 出来的回推值不严格相等。
    for (const v of [112.6, 29.9, 141.9, 0.8, 50, -6.1]) {
      expect(quantizeForWire({ j: v }).j).toBe(JSON.parse(JSON.stringify(v)));
    }
  });

  it('非有限值被剔除（不产生 NaN 关节）', () => {
    const out = quantizeForWire({ base: 0, shoulder: Number.NaN });
    expect(out).toEqual({ base: 0 });
  });

  it('不改动入参（避免污染 store 里的命令对象）', () => {
    const input = { shoulder: 29.9063172659 };
    quantizeForWire(input);
    expect(input.shoulder).toBe(29.9063172659);
  });
});

// ---------------------------------------------------------------------------
// Phase 9：真机（固件舵机角为整数）的归整口径
// ---------------------------------------------------------------------------

describe('wsProtocol · 真机量化（quantizeViaServo）', () => {
  const shoulder = model.actuators.find((a) => a.jointId === 'shoulder')!;
  const elbow = model.actuators.find((a) => a.jointId === 'elbow')!;

  it('命令先经舵机空间取整再反算 —— 与固件实际会停的位置同口径', () => {
    // S7：jointToServo(20) = 20*1.44018 + 88.776 = 117.5796 → 固件取整成 118
    expect(jointToServo(shoulder, 20)).toBeCloseTo(117.5796, 4);
    const q = quantizeViaServo(model, { base: 0, shoulder: 20, elbow: 112.6, gripper: 50 });
    expect(q.shoulder).toBeCloseTo(servoToJoint(shoulder, 118), 12);
  });

  it('残差量级 = 半个舵机度 ÷ |scale|（肩 0.347°、肘 0.209°）', () => {
    // 这不是"误差"，而是真机物理上无法表达更细命令的**量化下限**
    const cmd = { base: 0, shoulder: 20, elbow: 112.6185771989, gripper: 50 };
    const q = quantizeViaServo(model, cmd);
    expect(Math.abs(q.shoulder - cmd.shoulder)).toBeLessThanOrEqual(0.5 / shoulder.scale + 1e-9);
    expect(Math.abs(q.elbow - cmd.elbow)).toBeLessThanOrEqual(0.5 / elbow.scale + 1e-9);
    // 反例：若拿**未量化**的全精度命令去比对，就会永久停在这个假误差上
    expect(Math.abs(cmd.shoulder - q.shoulder)).toBeGreaterThan(0.1);
  });

  it('幂等：量化结果再量化不变（否则连续命令会持续漂移）', () => {
    const once = quantizeViaServo(model, {
      base: 0,
      shoulder: 20,
      elbow: 112.6185771989,
      gripper: 50,
    });
    const twice = quantizeViaServo(model, once);
    for (const id of Object.keys(once)) {
      expect(twice[id]).toBeCloseTo(once[id], 9);
    }
  });

  it('与后端 STATE 的 2 位小数格式化叠加后，残差仍远小于真机量化下限', () => {
    const cmd = { base: 0, shoulder: 20, elbow: 112.6185771989, gripper: 50 };
    const q = quantizeViaServo(model, cmd);
    for (const v of Object.values(q)) {
      const printed = Math.round(v * 100) / 100; // 后端 EncodeState 用 %.2f
      expect(Math.abs(printed - v)).toBeLessThanOrEqual(0.005 + 1e-12);
    }
  });

  it('不改动入参，且剔除非有限值', () => {
    const input = { shoulder: 20, base: Number.NaN };
    const out = quantizeViaServo(model, input);
    expect(input.shoulder).toBe(20);
    expect(out).not.toHaveProperty('base');
  });
});
