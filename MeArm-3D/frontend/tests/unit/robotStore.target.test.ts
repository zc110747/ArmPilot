/**
 * Phase 6 单元测试：末端目标（`target`）与命令关节（`commandJoints`）的分离语义。
 *
 * 锁定三条已拍板的规则：
 *   ① 目标可达      → 关节跟随，TCP 落在目标上；
 *   ② 目标越界      → **关节逐位不变**，只记录 `ikStatus`（绝不静默钳位）；
 *   ③ 越界后回到可达 → 恢复跟随（拖动不会"粘死")。
 *
 * 另锁一条易被忽略的约定：滑杆 / HOME / ZERO 属**非目标驱动**，会让 `target`
 * 跟随新 TCP 并清空 `ikStatus` —— 否则幽灵标记会停在旧位置误报"未到位"。
 */
import { beforeEach, describe, expect, it } from 'vitest';
import {
  endEffectorPosition,
  endEffectorPose,
  homeJointState,
  jointByRole,
  loadRobotModel,
  movableJoints,
  type JointState,
  type Vec3,
} from '../../src/robot';
import { targetGapMm, useRobotStore } from '../../src/store/robotStore';

const model = loadRobotModel();
const home = homeJointState(model);
const store = () => useRobotStore.getState();

/** 由一个合法关节状态反推目标点，保证目标一定可达 */
function reachable(patch: Partial<JointState>): Vec3 {
  const p = endEffectorPosition(model, { ...home, ...patch } as JointState);
  return [p[0], p[1], p[2]];
}

describe('Phase 6 · 目标点驱动关节（可达）', () => {
  beforeEach(() => {
    store().goHome();
  });

  it('初始 target = HOME 的 TCP（幽灵标记与 TCP 重合，场景里不显眼）', () => {
    const tcp = endEffectorPose(model, home).position;
    expect(store().target).toEqual([tcp[0], tcp[1], tcp[2]]);
    expect(store().ikStatus).toBeNull();
    expect(targetGapMm(store().target, store().commandJoints)).toBeCloseTo(0, 9);
  });

  it('moveTo 可达目标：TCP 精确落在目标上，ikStatus 记录分支与残差', () => {
    const target = reachable({ base: 25, shoulder: 20 });
    const result = store().moveTo(target);

    expect(result.success).toBe(true);
    expect(store().ikStatus?.ok).toBe(true);
    expect(store().ikStatus?.branch).toBe('elbow-up');
    expect(store().ikStatus?.residual).toBeLessThan(1e-9);
    // 实际末端位置与目标之差（独立于 IK 自报的 residual 再算一遍）
    expect(targetGapMm(store().target, store().commandJoints)).toBeLessThan(1e-9);
  });

  it('moveTo 不改夹爪（未参与解算的关节原样透传）', () => {
    const gripperId = jointByRole(model, 'gripper')!.id;
    store().setJoint(gripperId, 88);
    expect(store().commandJoints[gripperId]).toBeCloseTo(88, 9);

    store().moveTo(reachable({ base: 12 }));
    expect(store().commandJoints[gripperId]).toBeCloseTo(88, 9);
  });

  it('getState 里 target 与 commandJoints 是两个独立量：target 可越界而关节不越界', () => {
    const far: Vec3 = [600, 0, 60];
    store().moveTo(far);
    expect(store().target).toEqual(far);
    for (const joint of movableJoints(model)) {
      const value = store().commandJoints[joint.id];
      expect(value).toBeGreaterThanOrEqual(joint.limits.min - 1e-9);
      expect(value).toBeLessThanOrEqual(joint.limits.max + 1e-9);
    }
  });
});

describe('Phase 6 · 目标越界：关节逐位不变', () => {
  beforeEach(() => {
    store().goHome();
  });

  it('几何不可达 → OUT_OF_WORKSPACE，关节与末端位姿逐位不变', () => {
    const before: JointState = { ...store().commandJoints };
    const beforeTcp = [...store().endEffector.position] as Vec3;

    const result = store().moveTo([600, 0, 60]);

    expect(result.success).toBe(false);
    // 类型收窄（同时是"断言失败时的兜底报错"，比 undefined 属性报错更可读）
    if (result.success) throw new Error('该目标应当求解失败');
    expect(result.reason).toBe('OUT_OF_WORKSPACE');
    expect(store().commandJoints).toEqual(before);
    expect(store().endEffector.position).toEqual(beforeTcp);
    // 目标照常保留，让用户看到"差多远"
    expect(store().target).toEqual([600, 0, 60]);
    expect(store().ikStatus?.ok).toBe(false);
    expect(store().ikStatus?.reason).toBe('OUT_OF_WORKSPACE');
  });

  it('几何可达但方位角超限 → JOINT_LIMIT @ base，关节不变', () => {
    const before: JointState = { ...store().commandJoints };
    // 方位角 atan2(30, 10) ≈ 71.6° > base 上限 60°；枢轴距离 ≈ 143.5mm ∈ [40, 200]（几何可达）
    const result = store().moveTo([10, 30, 200]);

    expect(result.success).toBe(false);
    // 类型收窄（同时是"断言失败时的兜底报错"，比 undefined 属性报错更可读）
    if (result.success) throw new Error('该目标应当求解失败');
    expect(result.reason).toBe('JOINT_LIMIT');
    expect(result.joint).toBe(jointByRole(model, 'base')!.id);
    expect(store().commandJoints).toEqual(before);
    expect(store().ikStatus?.reason).toBe('JOINT_LIMIT');
  });

  it('越界后回到可达区：立刻恢复跟随（拖动不会"粘死"）', () => {
    store().moveTo([600, 0, 60]);
    const frozen: JointState = { ...store().commandJoints };

    const good = reachable({ base: 30 });
    store().moveTo(good);

    expect(store().ikStatus?.ok).toBe(true);
    expect(targetGapMm(store().target, store().commandJoints)).toBeLessThan(1e-9);
    expect(store().commandJoints).not.toEqual(frozen);
  });

  it('连续越界多次不会累积任何关节漂移', () => {
    const before: JointState = { ...store().commandJoints };
    for (const bad of [[600, 0, 60], [0, 0, 900], [-400, 300, 20]] as Vec3[]) {
      const r = store().moveTo(bad);
      expect(r.success).toBe(false);
    }
    expect(store().commandJoints).toEqual(before);
  });
});

describe('Phase 6 · 非目标驱动（滑杆 / HOME / ZERO）同步 target', () => {
  beforeEach(() => {
    store().goHome();
  });

  it('滑杆改变关节后，target 跟随新 TCP 且 ikStatus 归零', () => {
    store().moveTo(reachable({ base: 30 }));
    expect(store().ikStatus).not.toBeNull();

    store().setJoint('shoulder', 20);

    expect(store().ikStatus).toBeNull();
    const tcp = endEffectorPosition(model, store().commandJoints);
    expect(store().target).toEqual([tcp[0], tcp[1], tcp[2]]);
    expect(targetGapMm(store().target, store().commandJoints)).toBeCloseTo(0, 9);
  });

  it('goHome / goZero 同样把 target 收回 TCP', () => {
    store().moveTo([600, 0, 60]);
    expect(store().ikStatus?.ok).toBe(false);

    store().goHome();
    expect(store().ikStatus).toBeNull();
    expect(targetGapMm(store().target, store().commandJoints)).toBeCloseTo(0, 9);

    store().goZero();
    expect(store().ikStatus).toBeNull();
    expect(targetGapMm(store().target, store().commandJoints)).toBeCloseTo(0, 9);
  });

  it('resetTarget 把目标收回当前 TCP（等价于"取消目标"）', () => {
    store().moveTo([600, 0, 60]);
    store().resetTarget();
    expect(store().ikStatus).toBeNull();
    expect(targetGapMm(store().target, store().commandJoints)).toBeCloseTo(0, 9);
  });
});

describe('Phase 6 · 拖动交互状态', () => {
  beforeEach(() => {
    store().goHome();
  });

  it('默认拖动平面为水平面 xy', () => {
    expect(store().dragPlane).toBe('xy');
  });

  it('setDragPlane / setDragging 可切换，且不影响关节', () => {
    const before: JointState = { ...store().commandJoints };
    store().setDragPlane('camera');
    store().setDragging(true);
    expect(store().dragPlane).toBe('camera');
    expect(store().dragging).toBe(true);
    expect(store().commandJoints).toEqual(before);

    store().setDragging(false);
    expect(store().dragging).toBe(false);
  });
});
