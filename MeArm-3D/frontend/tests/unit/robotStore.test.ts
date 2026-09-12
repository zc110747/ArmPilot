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
    // 所以最竖直的姿态不是「全臂伸直 260mm」，而是小臂仍前倾 ~18.4° 的姿态。
    const elbowMin = jointById(model, 'elbow')!.limits.min;
    store().setJoint('shoulder', 0);
    store().setJoint('elbow', elbowMin);
    store().setJoint('base', 0);
    // z = 立柱 60 + 大臂 80·cos(0) + (小臂 80 + 手部 40)·cos(小臂绝对角)
    const upright =
      60 + 80 * Math.cos((0 * Math.PI) / 180) + 120 * Math.cos((elbowMin * Math.PI) / 180);
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
