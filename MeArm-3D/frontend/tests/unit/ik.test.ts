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
  jointByRole,
  loadRobotModel,
  movableJoints,
  type JointState,
  type RobotModel,
  type Vec3,
} from '../../src/robot';
// MeArm 的解析解住在**包内**（Phase 2 步④）—— 它不从 `@robot/index` 出口再导出。
import { ikGeometry, IkModelError, solveIk, solveIkAll } from '../../../robot-package/mearm-v1/kinematics/ik';

const model = loadRobotModel('mearm-v1');
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
  it('杆长与枢轴从 links 求导：L1 = 大臂，L2 = 肘枢轴→**腕枢轴**（不含腕→TCP）', () => {
    // 大臂 = upper_arm_link.length；小臂 = forearm_link.length（止于腕枢轴）
    expect(geometry.l1).toBeCloseTo(80, 6);
    expect(geometry.l2).toBeCloseTo(80, 6);
    expect(geometry.pivotZ).toBeCloseTo(60, 6);
    expect(geometry.pivotR).toBeCloseTo(0, 9);
    // TCP 参考关节 = 被动腕；它的坐标系原点就是 2R 子链的末端「腕枢轴」
    expect(geometry.wristId).toBe('tool');
  });

  it('腕枢轴→TCP 的**常量**偏移由模型求导：爪锁水平 ⇒ [40, 0]', () => {
    // 这条与上面分开写，是因为它们来自模型里两个不同的语义层：
    //   l2 由 links[].length 决定；toolOffset 由「腕被 coupling 锁在 90°」决定。
    expect(geometry.toolOffset[0]).toBeCloseTo(40, 6);
    expect(geometry.toolOffset[1]).toBeCloseTo(0, 9);
  });

  it('可达球壳 = [|L1−L2|, L1+L2]；本机 L1 === L2 ⇒ 内半径 0、无内锥空洞', () => {
    expect(geometry.reach[0]).toBeCloseTo(Math.abs(geometry.l1 - geometry.l2), 9);
    expect(geometry.reach[1]).toBeCloseTo(geometry.l1 + geometry.l2, 9);
    // 两根 80 的杆 ⇒ 几何上腕能折叠到枢轴，所以「目标太近」这一类越界**不再存在**；
    // 工作空间的内边界完全由**限位**决定（见错误码用例里的 JOINT_LIMIT 一条）。
    expect(geometry.reach[0]).toBeCloseTo(0, 6);
    expect(geometry.reach[1]).toBeCloseTo(160, 6);
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

  // -------------------------------------------------------------------------
  // ★ 被动腕的守卫：把「偏移是常量」这件事变成可被打破的断言
  // -------------------------------------------------------------------------
  it('★ 腕若被改回刚性固连（fixed），求导必须报错而不是解出偏差 40mm 的解', () => {
    // 这是本文件里最贵的一条：`tool` 从 passive 退回 fixed 后，「腕→TCP」会随 θe 转动，
    // 而 2R 仍会给出一个**看起来完全正常**的解 —— 只是系统性偏掉几十毫米，
    // 且残差自查同样是偏的（自证），整套测试会一起变绿。所以必须显式拦住。
    const rigid = withJoint(model, 'tool', { type: 'fixed', coupling: undefined });
    expect(() => ikGeometry(rigid)).toThrow(/固定关节/);
  });

  it('★ 腕失去"锁"之后就报错：revolute 且不带 coupling ⇒ 偏移随姿态变化', () => {
    // ⚠️ 这条用例第一版写错过，值得留个记号：只把 `type` 改成 revolute、**保留 coupling**
    // 并不会破坏锁 —— 那种模型的爪绝对倾角仍是常量（= limits.min = −90°），
    // 求导完全正确。真正的失效点是"**锁没了**"：没有 coupling 的折角随小臂一起转，
    // 三个采样姿态下偏移各不相同 ⇒ 守卫必须命中。
    const unlocked: RobotModel = {
      ...model,
      joints: model.joints.map((j) =>
        j.id === 'tool'
          ? { ...j, type: 'revolute' as const, coupling: undefined, limits: { min: -90, max: 90 } }
          : j,
      ),
    };
    expect(() => ikGeometry(unlocked)).toThrow(/偏移随姿态变化/);
  });

  it('★ 偏移必须落在矢状面内：tcp.offset 带横向分量 ⇒ 报错', () => {
    const skewed: RobotModel = { ...model, tcp: { ...model.tcp, offset: [0, 12, 40] } };
    expect(() => ikGeometry(skewed)).toThrow(/矢状面之外/);
  });

  it('tcp.joint 不存在时有明确报错（不是 undefined 解引用）', () => {
    const broken: RobotModel = { ...model, tcp: { ...model.tcp, joint: 'nope' } };
    expect(() => ikGeometry(broken)).toThrow(/nope/);
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
  /**
   * 构造「腕枢轴距肩枢轴恰好 d、方位角 0」的目标点。
   *
   * ⚠️ 必须**反着**把 `toolOffset` 加回去：爪锁水平 ⇒ TCP 恒比腕枢轴在径向上多 40mm。
   * 直接写 `[d, 0, pivotZ]` 实际表达的是"腕枢轴距枢轴 d − 40"，越界判据会整整差 40mm。
   */
  const targetAtWristDistance = (d: number): Vec3 => [
    d + geometry.pivotR + geometry.toolOffset[0],
    0,
    geometry.pivotZ + geometry.toolOffset[1],
  ];

  it('超出最大伸展 → OUT_OF_WORKSPACE', () => {
    const result = solveIk(model, targetAtWristDistance(geometry.reach[1] + 0.5));
    expect(result.success).toBe(false);
    if (result.success) return;
    expect(result.reason).toBe('OUT_OF_WORKSPACE');
    expect(result.message).toMatch(/超出工作空间/);
  });

  it('本机 L1 === L2 ⇒ 不存在"太近"的越界；贴近枢轴的目标由**限位**否决', () => {
    // 老模型（L2 = 120）有内锥空洞，那时才能构造出 `reach[0]` 以内的越界目标。
    // 现在两根杆等长（80 / 80），几何上腕能折叠到枢轴 ⇒ 这一类 OUT_OF_WORKSPACE
    // 根本不存在；同一个点在几何上可达，是被 `elbow` 限位拦下的 —— 拒绝理由本质不同。
    expect(geometry.reach[0]).toBeCloseTo(0, 9);
    const result = solveIk(model, [0, 0, geometry.pivotZ]);
    expect(result.success).toBe(false);
    if (result.success) return;
    expect(result.reason).toBe('JOINT_LIMIT');
  });

  it('恰好落在伸展边界上（两杆共线）：不再是 OUT_OF_WORKSPACE，而是关节做不出来 → JOINT_LIMIT', () => {
    const result = solveIk(model, targetAtWristDistance(geometry.reach[1]));
    expect(result.success).toBe(false);
    if (result.success) return;
    expect(result.reason).toBe('JOINT_LIMIT');
  });

  it('几何可达但底座方位角超出 ±60° → JOINT_LIMIT 且指名 base', () => {
    // 方位角 80°、半径 100 ⇒ 腕枢轴距枢轴 hypot(100−40, 100−60)=hypot(60,40)≈72.1，几何可达
    const az = (80 * Math.PI) / 180;
    const target: Vec3 = [100 * Math.cos(az), 100 * Math.sin(az), 100];
    const result = solveIk(model, target);
    expect(result.success).toBe(false);
    if (result.success) return;
    expect(result.reason).toBe('JOINT_LIMIT');
    expect(result.joint).toBe(base.id);
    expect(result.message).toMatch(/limit/);
  });

  it('几何可达但俯仰角越界（偏航轴上、高于枢轴 100mm）→ JOINT_LIMIT，并在信息里给出越界量', () => {
    // ⚠️ 目标点必须落在**腕枢轴**的可达壳内：[0,0,pivotZ+100] ⇒ 腕距 = hypot(40, 100) ≈ 107.7 < 160。
    // 若直接写"正上方 240mm"，腕距会到 184mm，那就成了 OUT_OF_WORKSPACE —— 测的就不是限位了。
    const result = solveIk(model, [0, 0, geometry.pivotZ + 100]);
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
