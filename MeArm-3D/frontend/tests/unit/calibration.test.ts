/**
 * Phase 1 验收：Calibration（关节角 ↔ 舵机角 标定层）
 *
 * 关键约束（spec §二十四 / §二十五）：
 *   - 标定只做 `Joint Angle → Servo Angle`，绝不进入 IK
 *   - 关节限位映射到舵机空间必须与固件硬限位严丝合缝
 *   - 真实装反 / 零点不同只改 yaml，不改代码（reverse / offset）
 */
import { describe, expect, it } from 'vitest';
import {
  actuatorById,
  actuatorJointToServo,
  actuatorServoToJoint,
  buildCalibrationTable,
  checkCalibration,
  describeCalibration,
  jointStateToServoChannels,
  jointToServoChannels,
  loadRobotModel,
  servoChannelsToJointState,
} from '../../src/robot';

describe('Phase 1 · 标定层', () => {
  const model = loadRobotModel();

  it('关节限位端点标定后恰好落在固件舵机硬限位上', () => {
    const checks = checkCalibration(model);
    expect(checks).toHaveLength(4);
    for (const check of checks) {
      expect(check.ok, `${check.actuatorId} 标定越界: ${JSON.stringify(check)}`).toBe(true);
    }

    const base = checks.find((c) => c.jointId === 'base')!;
    expect(base.channel).toBe(9);
    expect(base.servoRange.min).toBeCloseTo(30, 9);
    expect(base.servoRange.max).toBeCloseTo(150, 9);

    // ⚠️ 角色以实拍为准：S7 = 肩，S8 = 肘（固件的 left/right 只是安装位称呼）
    const shoulder = checks.find((c) => c.jointId === 'shoulder')!;
    expect(shoulder.channel).toBe(7);
    // θ=-6.09 -> 80°，θ=49.45 -> 160°
    expect(shoulder.servoRange.min).toBeCloseTo(80, 6);
    expect(shoulder.servoRange.max).toBeCloseTo(160, 6);

    const elbow = checks.find((c) => c.jointId === 'elbow')!;
    expect(elbow.channel).toBe(8);
    // reverse = true：θ=108.44 -> 100°，θ=141.86 -> 20°
    expect(elbow.servoRange.min).toBeCloseTo(20, 6);
    expect(elbow.servoRange.max).toBeCloseTo(100, 6);

    const gripper = checks.find((c) => c.jointId === 'gripper')!;
    expect(gripper.servoRange.min).toBeCloseTo(40, 9);
    expect(gripper.servoRange.max).toBeCloseTo(130, 9);
  });

  it('reverse 语义：servo = -θ·scale + offset', () => {
    const elbow = actuatorById(model, 'servo_8')!;
    expect(elbow.reverse).toBe(true);
    expect(actuatorJointToServo(elbow, 108.4414852068)).toBeCloseTo(100, 6);
    expect(actuatorJointToServo(elbow, 125)).toBeCloseTo(60.36, 2);
    expect(actuatorJointToServo(elbow, 141.8582211436)).toBeCloseTo(20, 6);

    // 非 reverse 的一侧仍然用 +θ·scale + offset（S7 肩）
    const shoulder = actuatorById(model, 'servo_7')!;
    expect(shoulder.reverse).toBe(false);
    expect(actuatorJointToServo(shoulder, 0)).toBeCloseTo(88.776, 6);
    expect(actuatorJointToServo(shoulder, 10)).toBeCloseTo(103.1778, 4);
  });

  it('标定可逆：servoToJoint 与 jointToServo 互逆', () => {
    for (const actuator of model.actuators) {
      for (const angle of [-30, 0, 17.5, 45, 90]) {
        const servo = actuatorJointToServo(actuator, angle);
        expect(actuatorServoToJoint(actuator, servo)).toBeCloseTo(angle, 9);
      }
    }
  });

  it('jointToServoChannels / servoChannelsToJointState 往返一致（真实反馈同步用）', () => {
    const joints = { base: 12, shoulder: 20, elbow: 120, gripper: 70 };
    const servo = jointStateToServoChannels(model, joints);
    expect(servo[9]).toBeCloseTo(102, 9); // base     : 12 × 1 + 90
    expect(servo[7]).toBeCloseTo(117.5796, 4); // shoulder : 20 × 1.44018 + 88.776
    expect(servo[8]).toBeCloseTo(72.3288, 4); // elbow    : -120 × 2.39401 + 359.61
    expect(servo[6]).toBeCloseTo(110, 9); // gripper  : 70 × 1 + 40

    const back = servoChannelsToJointState(model, servo);
    for (const [id, value] of Object.entries(joints)) {
      expect(back[id], id).toBeCloseTo(value, 9);
    }
  });

  it('单关节查询：jointToServoChannels 只返回该关节的舵机通道', () => {
    expect(jointToServoChannels(model, 'base', 0)).toEqual({ 9: 90 });
    expect(jointToServoChannels(model, 'gripper', 0)).toEqual({ 6: 40 });
  });

  it('标定表只对可动关节建条目，固定关节（tool）被排除', () => {
    const table = buildCalibrationTable(model);
    expect([...table.byJoint.keys()]).toEqual(['base', 'shoulder', 'elbow', 'gripper']);
    expect(table.byJoint.get('tool')).toBeUndefined();
    expect(table.entries.every((entry) => entry.coupled === false)).toBe(true);
  });

  it('describeCalibration 输出人类可读的标定式', () => {
    const lines = describeCalibration(model);
    expect(lines).toHaveLength(4);
    expect(lines[1]).toContain('S7 (shoulder)');
    expect(lines[1]).toContain('+θ × 1.44018 + 88.776');
    expect(lines[2]).toContain('S8 (elbow)');
    expect(lines[2]).toContain('-θ × 2.39401 + 359.61');
  });
});
