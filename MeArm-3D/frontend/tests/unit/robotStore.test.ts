/**
 * Phase 4 状态层验收：滑杆改动 JointState → 末端位姿随之变化，且严格受限位约束。
 * 3D 场景跟随由 `applyJointState` 保证，其正确性由 Phase 3 的 FK↔Three.js 验收测试锁定。
 */
import { beforeEach, describe, expect, it } from 'vitest';
import {
  endEffectorPosition,
  homeJointState,
  jointById,
  loadRobotModel,
} from '../../src/robot';
import { useRobotStore } from '../../src/store/robotStore';

const model = loadRobotModel();
const store = () => useRobotStore.getState();

describe('Phase 4 · 关节控制 → RobotState', () => {
  beforeEach(() => {
    store().goHome();
    store().setMode('simulation');
  });

  it('初始状态 = HOME 位姿，默认模式为 Simulation', () => {
    expect(store().commandJoints).toEqual(homeJointState(model));
    expect(store().actualJoints).toEqual(homeJointState(model));
    expect(store().mode).toBe('simulation');
    expect(store().controlSource).toBe('virtual');
  });

  it('滑动单关节立即更新末端位姿（与 FK 一致）', () => {
    // 「大臂竖直 + 小臂最竖直」：小臂是**绝对角**，真机可达下限 108.44°（不是 0°），
    // 所以最竖直的姿态不是「全臂伸直」，而是小臂仍前倾 ~18.4° 的姿态。
    // ⚠️ 爪锁水平 ⇒ 高度只算到**腕枢轴**（60 + 80·cosθs + 80·cosθe）：
    //    腕→TCP 那 40mm 是**水平**的，完全不贡献高度。
    //    老模型把它当作小臂的延长线（120·cosθe），那会凭空多算出 12.6mm 的高度。
    const elbowMin = jointById(model, 'elbow')!.limits.min;
    store().setJoint('shoulder', 0);
    store().setJoint('elbow', elbowMin);
    store().setJoint('base', 0);
    const upright =
      60 + 80 * Math.cos((0 * Math.PI) / 180) + 80 * Math.cos((elbowMin * Math.PI) / 180);
    expect(store().endEffector.position[2]).toBeCloseTo(upright, 9);

    store().setJoint('shoulder', 40);
    const expected = endEffectorPosition(model, store().commandJoints);
    expect(store().endEffector.position).toEqual(expected);
    // 大臂前倾 → 末端必然下降
    expect(store().endEffector.position[2]).toBeLessThan(upright);
  });

  it('超出限位的输入被钳位，不会写出非法 JointState', () => {
    store().setJoint('elbow', 999);
    expect(store().commandJoints.elbow).toBe(jointById(model, 'elbow')!.limits.max);
    store().setJoint('base', -999);
    expect(store().commandJoints.base).toBe(-60);
    store().setJoint('shoulder', -50);
    expect(store().commandJoints.shoulder).toBe(jointById(model, 'shoulder')!.limits.min);
  });

  it('夹爪开合不改变末端位姿（IK 只解 J1/J2/J3 的前提）', () => {
    store().setJoint('shoulder', 25);
    store().setJoint('elbow', 120);
    const before = [...store().endEffector.position];
    for (const value of [0, 30, 60, 90]) {
      store().setJoint('gripper', value);
      expect(store().endEffector.position[2]).toBeCloseTo(before[2]!, 9);
    }
  });

  it('HOME / ZERO 动作切换位姿', () => {
    store().setJoint('shoulder', 40);
    store().goZero();
    // ZERO = 关节空间原点经限位钳位：小臂（绝对角，可达 108.44..141.86°）落到最竖直可达角
    expect(store().commandJoints).toEqual({
      base: 0,
      shoulder: 0,
      elbow: jointById(model, 'elbow')!.limits.min,
      gripper: 0,
    });
    expect(store().endEffector.position).toEqual(endEffectorPosition(model, store().commandJoints));

    store().goHome();
    expect(store().commandJoints).toEqual(homeJointState(model));
  });

  it('真实反馈写入时控制源变为 real（虚拟↔真实回环的断环依据）', () => {
    store().setActualJoints({ base: 12, shoulder: 30, elbow: 120, gripper: 70 });
    expect(store().controlSource).toBe('real');
    expect(store().actualJoints.base).toBe(12);
    // 命令值未被真实反馈覆盖
    expect(store().commandJoints).toEqual(homeJointState(model));
  });

  it('无连接时点 Real Robot 被拒绝：mode 保持 Simulation，但日志有拒绝原因', () => {
    const logBefore = store().log.length;
    store().setMode('real');
    expect(store().mode).toBe('simulation'); // ★ D43：拒绝，不切
    expect(store().log.length).toBeGreaterThan(logBefore);

    store().setMode('simulation');
    expect(store().mode).toBe('simulation');
  });
});
