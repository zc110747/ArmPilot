/**
 * Phase 5 验收：**FK(IK(XYZ)) == XYZ**（闭环一致性）。
 *
 * 采样方式刻意反过来做，才能覆盖整个可达工作空间而不带上"手挑目标点"的偏置：
 *
 * ```
 *   随机关节状态 θ（在限位内）
 *        │
 *        ├─ FK ─→ XYZ            ← 由构造保证一定可达
 *        │
 *        └─ IK(XYZ) ─→ θ'
 *                        │
 *                        └─ FK ─→ XYZ'   ，断言 |XYZ' − XYZ| < 容差
 * ```
 *
 * 注意断言的是**位置**而非角度：平面 2R 存在多解，`θ'` 不必等于 `θ`，
 * 只有末端位置必须逐位吻合。
 */
import { describe, expect, it } from 'vitest';
import {
  endEffectorPosition,
  ikGeometry,
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

/** 可复现的伪随机数（LCG），保证验收结果可复盘 */
function makeRandom(seed: number): () => number {
  let state = seed >>> 0;
  return () => {
    state = (state * 1664525 + 1013904223) >>> 0;
    return state / 0x100000000;
  };
}

function randomJointState(model: RobotModel, random: () => number): JointState {
  const state: JointState = {};
  for (const joint of movableJoints(model)) {
    const { min, max } = joint.limits;
    state[joint.id] = min + (max - min) * random();
  }
  return state;
}

function distance(a: Vec3, b: Vec3): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

/** 容差：浮点噪声级（实测 FK(IK(·)) 残差 ~1e-13 mm） */
const TOLERANCE_MM = 1e-9;

describe('Phase 5 验收 · FK(IK(XYZ)) 闭环一致性', () => {
  it('2000 组随机关节状态生成的 XYZ，全部可反解且位置残差 < 1e-9 mm', () => {
    const random = makeRandom(0x5eed01);
    let maxResidual = 0;
    let maxReported = 0;
    let failures = 0;
    const branchCount = { 'elbow-up': 0, 'elbow-down': 0 };

    for (let i = 0; i < 2000; i += 1) {
      const state = randomJointState(model, random);
      const target = endEffectorPosition(model, state);
      const result = solveIk(model, target);
      if (!result.success) {
        failures += 1;
        continue;
      }
      branchCount[result.branch] += 1;
      const achieved = endEffectorPosition(model, result.joints);
      maxResidual = Math.max(maxResidual, distance(achieved, target));
      maxReported = Math.max(maxReported, result.residual);
    }

    // eslint-disable-next-line no-console
    console.log(
      `[Phase5] 2000 组随机位姿：实得末端最大误差 = ${maxResidual.toExponential(3)} mm ` +
        `（IK 自查上报最大残差 ${maxReported.toExponential(3)} mm），` +
        `失败 ${failures} 组，分支分布 ${JSON.stringify(branchCount)}`,
    );

    expect(failures).toBe(0);
    expect(maxResidual).toBeLessThan(TOLERANCE_MM);
  });

  it('多解模式下仍收敛：prefer=nearest 逐点跟随前一解（模拟拖动）', () => {
    const random = makeRandom(0x5eed02);
    let near: JointState | undefined;
    let maxResidual = 0;
    let branchFlips = 0;
    let previousBranch: string | undefined;

    for (let i = 0; i < 300; i += 1) {
      const state = randomJointState(model, random);
      const target = endEffectorPosition(model, state);
      const result = solveIk(model, target, near ? { prefer: 'nearest', near } : undefined);
      expect(result.success).toBe(true);
      if (!result.success) return;
      if (previousBranch && result.branch !== previousBranch) branchFlips += 1;
      previousBranch = result.branch;
      near = result.joints;
      maxResidual = Math.max(maxResidual, distance(endEffectorPosition(model, result.joints), target));
    }

    // eslint-disable-next-line no-console
    console.log(
      `[Phase5] 拖动连续性 300 点：最大误差 ${maxResidual.toExponential(3)} mm，支解切换 ${branchFlips} 次`,
    );

    expect(maxResidual).toBeLessThan(TOLERANCE_MM);
    // 本机相对肘角恒 > 0 ⇒ 只有一支可达，拖动不应发生翻肘
    expect(branchFlips).toBe(0);
  });

  it('全部随机目标都落在可达球壳内（交叉验证错误码判据不自相矛盾）', () => {
    const random = makeRandom(0x5eed03);
    let outside = 0;
    for (let i = 0; i < 500; i += 1) {
      const target = endEffectorPosition(model, randomJointState(model, random));
      const r = Math.hypot(target[0], target[1]);
      const d = Math.hypot(r - geometry.pivotR, target[2] - geometry.pivotZ);
      if (d > geometry.reach[1] + 1e-9 || d < geometry.reach[0] - 1e-9) outside += 1;
    }
    expect(outside).toBe(0);
  });

  it('随机目标里「肘向下」支从未可行（结构不变量）', () => {
    const random = makeRandom(0x5eed04);
    let downFeasible = 0;
    for (let i = 0; i < 300; i += 1) {
      const target = endEffectorPosition(model, randomJointState(model, random));
      const probe = solveIkAll(model, target);
      if (probe.candidates.find((c) => c.branch === 'elbow-down')?.feasible) downFeasible += 1;
    }
    expect(downFeasible).toBe(0);
  });
});
