/**
 * Phase 5 单元测试：IK 求解器的几何求导、边界、错误码与多解策略。
 *
 * 重点验证「不写死常数」与「绝对角肘关节」两件事：
 *   - 几何量（L1 / L2 / 枢轴）全部从 RobotModel 求导 → 改 yaml 即改机构；
 *   - `elbow` 存**绝对角**，IK 解出的 θe 直接写入 JointState，不再叠加肩角。
 */
import { describe, expect, it } from 'vitest';
import {
  endEffectorPosition,
  homeJointState,
  ikGeometry,
  IkModelError,
  jointByRole,
  loadRobotModel,
  movableJoints,
  solveIk,
  solveIkAll,
  type JointState,
  type RobotModel,
  type Vec3,
} from '../../src/robot';

const model = loadRobotModel();
const geometry = ikGeometry(model);
const home = homeJointState(model);
const shoulder = jointByRole(model, 'shoulder')!;
const elbow = jointByRole(model, 'elbow')!;
const base = jointByRole(model, 'base')!;

/** 复制模型并改一个关节字段（用于验证"改配置后解析式必须报错而不是静默解错"） */
function withJoint(model: RobotModel, jointId: string, patch: Record<string, unknown>): RobotModel {
  return {
    ...model,
    joints: model.joints.map((j) => (j.id === jointId ? { ...j, ...patch } : j)),
  };
}

describe('Phase 5 · IK 几何求导（全部来自模型）', () => {
  it('大臂 / 小臂等效长与肩枢轴高度从 links 与 tcp.offset 求导，不写死常数', () => {
    // 大臂 = upper_arm_link.length；小臂 = forearm_link.length + tool_link.length + tcp.offset.z
    expect(geometry.l1).toBeCloseTo(80, 6);
    expect(geometry.l2).toBeCloseTo(80 + 40, 6);
    expect(geometry.pivotZ).toBeCloseTo(60, 6);
    expect(geometry.pivotR).toBeCloseTo(0, 9);
  });

  it('可达球壳 = [|L1−L2|, L1+L2]', () => {
    expect(geometry.reach[0]).toBeCloseTo(Math.abs(geometry.l1 - geometry.l2), 9);
    expect(geometry.reach[1]).toBeCloseTo(geometry.l1 + geometry.l2, 9);
    expect(geometry.reach[0]).toBeCloseTo(40, 6);
    expect(geometry.reach[1]).toBeCloseTo(200, 6);
  });

  it('几何量随模型变化：改 length 后 L1 随之改变（证明确非硬编码）', () => {
    const taller: RobotModel = {
      ...model,
      links: model.links.map((l) => (l.id === 'upper_arm_link' ? { ...l, length: 90 } : l)),
    };
    expect(ikGeometry(taller).l1).toBeCloseTo(90, 6);
  });

  it('改 base/shoulder/elbow 的 axis 后抛 IkModelError，而不是静默解出错解', () => {
    expect(() => ikGeometry(withJoint(model, 'shoulder', { axis: [0, 0, 1] }))).toThrow(IkModelError);
    expect(() => ikGeometry(withJoint(model, 'base', { axis: [0, 1, 0] }))).toThrow(IkModelError);
    const tilted = withJoint(model, 'elbow', { origin: { position: [0, 0, 0], rotation: [10, 0, 0] } });
    expect(() => ikGeometry(tilted)).toThrow(/origin\.rotation/);
  });
});

describe('Phase 5 · IK 基本反解', () => {
  it('HOME 末端位置能反解回 HOME 的定位三关节', () => {
    const target = endEffectorPosition(model, home);
    const result = solveIk(model, target);
    expect(result.success).toBe(true);
    if (!result.success) return;
    expect(result.joints.base).toBeCloseTo(home.base!, 9);
    expect(result.joints.shoulder).toBeCloseTo(home.shoulder!, 9);
    expect(result.joints.elbow).toBeCloseTo(home.elbow!, 9);
    expect(result.branch).toBe('elbow-up');
  });

  it('elbow 存绝对角：解出的 θe 直接可用，相对角 = θe − θs', () => {
    const target = endEffectorPosition(model, home);
    const result = solveIk(model, target);
    expect(result.success).toBe(true);
    if (!result.success) return;
    expect(result.relativeAngle).toBeCloseTo(result.joints.elbow! - result.joints.shoulder!, 9);
    // 绝对角远大于相对角之外的直觉值：小臂绝对倾角 ≈112.62°，而相对角 ≈111.77°
    expect(result.joints.elbow!).toBeGreaterThan(100);
    expect(result.relativeAngle).toBeLessThan(result.joints.elbow!);
  });

  it('解算残差在浮点噪声级（返回前已用 FK 自查）', () => {
    const target = endEffectorPosition(model, home);
    const result = solveIk(model, target);
    expect(result.success).toBe(true);
    if (!result.success) return;
    expect(result.residual).toBeLessThan(1e-9);
  });

  it('返回的 joints 是完整状态（含夹爪），可直接喂给 FK 闭环', () => {
    // 目标由「合法关节状态」经 FK 生成，保证一定可达（手挑 XYZ 容易踩到关节限位边界）
    const from: JointState = { base: 25, shoulder: 20, elbow: 125, gripper: 50 };
    const target = endEffectorPosition(model, from);
    const seed: JointState = { ...home, gripper: 77 };
    const result = solveIk(model, target, { seed });
    expect(result.success).toBe(true);
    if (!result.success) return;
    for (const joint of movableJoints(model)) {
      expect(result.joints[joint.id]).toBeDefined();
    }
    expect(result.joints.gripper).toBe(77);
    const achieved = endEffectorPosition(model, result.joints);
    expect(Math.hypot(achieved[0] - target[0], achieved[1] - target[1], achieved[2] - target[2])).toBeLessThan(1e-9);
  });
});

describe('Phase 5 · IK 错误码', () => {
  it('超出最大伸展 → OUT_OF_WORKSPACE', () => {
    const result = solveIk(model, [geometry.reach[1] + 0.5, 0, geometry.pivotZ]);
    expect(result.success).toBe(false);
    if (result.success) return;
    expect(result.reason).toBe('OUT_OF_WORKSPACE');
    expect(result.message).toMatch(/超出工作空间/);
  });

  it('小于最小伸展（连杆无法折叠到那么近）→ OUT_OF_WORKSPACE', () => {
    const result = solveIk(model, [geometry.reach[0] - 1, 0, geometry.pivotZ]);
    expect(result.success).toBe(false);
    if (result.success) return;
    expect(result.reason).toBe('OUT_OF_WORKSPACE');
  });

  it('恰好落在伸展边界上：不再是 OUT_OF_WORKSPACE，而是关节做不出来 → JOINT_LIMIT', () => {
    const result = solveIk(model, [geometry.reach[1], 0, geometry.pivotZ]);
    expect(result.success).toBe(false);
    if (result.success) return;
    expect(result.reason).toBe('JOINT_LIMIT');
  });

  it('几何可达但底座方位角超出 ±60° → JOINT_LIMIT 且指名 base', () => {
    // 方位角 80°，半径 100 ⇒ 枢轴距 D=hypot(100,40)≈107.7，属几何可达范围内
    const az = (80 * Math.PI) / 180;
    const target: Vec3 = [100 * Math.cos(az), 100 * Math.sin(az), 100];
    const result = solveIk(model, target);
    expect(result.success).toBe(false);
    if (result.success) return;
    expect(result.reason).toBe('JOINT_LIMIT');
    expect(result.joint).toBe(base.id);
    expect(result.message).toMatch(/limit/);
  });

  it('几何可达但俯仰角越界（目标在正上方 240mm）→ JOINT_LIMIT，并在信息里给出越界量', () => {
    const result = solveIk(model, [0, 0, 240]);
    expect(result.success).toBe(false);
    if (result.success) return;
    expect(result.reason).toBe('JOINT_LIMIT');
    expect(['shoulder', 'elbow']).toContain(result.joint);
    expect(result.message).toMatch(/两支解都越界/);
    // 候选仍应带回来，便于 UI 展示"差多少度"
    expect(result.candidates).toHaveLength(2);
  });
});

describe('Phase 5 · 退化与多解', () => {
  it('目标落在偏航轴上（r≈0）不报错，方位角取当前值并标注不定', () => {
    const probe = solveIkAll(model, [0, 0, 190]);
    expect(probe.azimuthIndeterminate).toBe(true);
    // 若该点俯仰不可达，至少不应抛异常、不应返回 OUT_OF_WORKSPACE
    if (!probe.result.success) expect(probe.result.reason).toBe('JOINT_LIMIT');
  });

  it('⚠️ 本机结构上只有一支可行解：相对肘角恒 > 0', () => {
    // 这是由实测反解出的限位直接推出的结构不变量：
    //   elbow 绝对角下限 108.44° − shoulder 绝对角上限 49.45° = 58.99° > 0
    // 因此「肘向下」支（相对角 < 0）在本机永远越界。
    expect(elbow.limits.min - shoulder.limits.max).toBeGreaterThan(0);

    const candidates = solveIkAll(model, endEffectorPosition(model, home)).candidates;
    expect(candidates).toHaveLength(2);
    const up = candidates.find((c) => c.branch === 'elbow-up')!;
    const down = candidates.find((c) => c.branch === 'elbow-down')!;
    expect(up.feasible).toBe(true);
    expect(up.relativeAngle).toBeGreaterThan(0);
    expect(down.relativeAngle).toBeLessThan(0);
    expect(down.feasible).toBe(false);
  });

  it('prefer: elbow-up / nearest 都取 elbow-up', () => {
    const target = endEffectorPosition(model, home);
    const up = solveIk(model, target, { prefer: 'elbow-up' });
    const nearest = solveIk(model, target, { prefer: 'nearest', near: home });
    expect(up.success && up.branch).toBe('elbow-up');
    expect(nearest.success && nearest.branch).toBe('elbow-up');
  });

  it('prefer: elbow-down 在本机不可达 → 回退到可行解，绝不返回越界解', () => {
    const target = endEffectorPosition(model, home);
    const result = solveIk(model, target, { prefer: 'elbow-down' });
    expect(result.success).toBe(true);
    if (!result.success) return;
    expect(result.branch).toBe('elbow-up');
    expect(result.joints.shoulder).toBeGreaterThanOrEqual(shoulder.limits.min);
    expect(result.joints.shoulder).toBeLessThanOrEqual(shoulder.limits.max);
    expect(result.joints.elbow).toBeGreaterThanOrEqual(elbow.limits.min);
    expect(result.joints.elbow).toBeLessThanOrEqual(elbow.limits.max);
  });

  it('prefer: nearest 缺 near 时退化为 elbow-up（不抛异常）', () => {
    const target = endEffectorPosition(model, home);
    const result = solveIk(model, target, { prefer: 'nearest' });
    expect(result.success).toBe(true);
    if (!result.success) return;
    expect(result.branch).toBe('elbow-up');
  });

  it('solveIkAll 同时返回两支候选与最终选取结果', () => {
    const probe = solveIkAll(model, endEffectorPosition(model, home));
    expect(probe.candidates).toHaveLength(2);
    expect(probe.result.success).toBe(true);
    expect(probe.candidates.map((c) => c.branch).sort()).toEqual(['elbow-down', 'elbow-up']);
  });
});
