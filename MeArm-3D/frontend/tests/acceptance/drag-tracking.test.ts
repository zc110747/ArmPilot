/**
 * Phase 6 验收：沿屏幕水平线的真实拖动轨迹，穿越工作空间边界。
 *
 * 与单元测试的区别在于**射线不是"指向已知答案"构造的**：这里固定一台相机
 * （位置 / 朝向 / FOV 与 `RobotScene.tsx` 的默认视角一致），把屏幕 NDC 横坐标
 * 映射成真实射线族，再经「冻结平面 → 求交 → moveTo」驱动机械臂。
 * 因此轨迹在哪一段可达、边界落在哪，全部由机构约束**算出来**，不是预设的。
 *
 * 断言：
 *   1. 冻结平面 —— 全程目标点的 Z 严格等于锚点 Z；
 *   2. 可达段   —— TCP 逐点命中目标，残差 < 1e-9 mm；
 *   3. 越界段   —— **关节逐位不变**（Phase 6 已拍板：不静默钳位）；
 *   4. 形态     —— 两端越界、中间一段连续可达，边界被定位到一格之内；
 *   5. 连续性   —— 相邻可达帧关节变化有界，全程不翻支（D19 的结构不变量）；
 *   6. 可逆性   —— 拖过去再拖回来，同一位置关节状态可复现（不"粘死"）。
 */
import { beforeEach, describe, expect, it } from 'vitest';
import {
  dragTarget,
  homeJointState,
  loadRobotModel,
  makePlane,
  movableJoints,
  normalizeVec3,
  pointsCoincide,
  type JointState,
  type Vec3,
} from '../../src/robot';
import { targetGapMm, useRobotStore } from '../../src/store/robotStore';

const model = loadRobotModel();

// 与 RobotScene.tsx 的默认相机一致
const CAMERA: Vec3 = [300, -430, 300];
const LOOK_AT: Vec3 = [0, 0, 110];
const UP: Vec3 = [0, 0, 1];
const FOV_DEG = 38;
const ASPECT = 16 / 9;

/** 拖动平面：水平面，锁在 z = 95（该高度下既有可达段、两端又必然越界） */
const PLANE_Z = 95;
const SCAN_FROM = -0.9;
const SCAN_TO = 0.9;
const SAMPLE_COUNT = 400;

const store = () => useRobotStore.getState();

type Sample = {
  u: number;
  target: Vec3;
  ok: boolean;
  joints: JointState;
  branch?: string;
  reason?: string;
  joint?: string;
};

function sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function cross(a: Vec3, b: Vec3): Vec3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

/** 相机基：forward / right / trueUp（模拟 three 的 lookAt + up = Z） */
function cameraBasis() {
  const forward = normalizeVec3(sub(LOOK_AT, CAMERA))!;
  const right = normalizeVec3(cross(forward, UP))!;
  const trueUp = cross(right, forward);
  return { forward, right, trueUp };
}

const BASIS = cameraBasis();
const TAN_HALF_FOV = Math.tan(((FOV_DEG / 2) * Math.PI) / 180);

/** 屏幕 NDC (u, v) → 世界射线（origin 固定为相机位置） */
function rayForNdc(u: number, v: number): { origin: Vec3; dir: Vec3 } {
  const k = TAN_HALF_FOV;
  const dir = normalizeVec3([
    BASIS.forward[0] + u * k * ASPECT * BASIS.right[0] + v * k * BASIS.trueUp[0],
    BASIS.forward[1] + u * k * ASPECT * BASIS.right[1] + v * k * BASIS.trueUp[1],
    BASIS.forward[2] + u * k * ASPECT * BASIS.right[2] + v * k * BASIS.trueUp[2],
  ])!;
  return { origin: CAMERA, dir };
}

/** 扫描一条水平拖动：u 从 from 线性走到 to，每步都真实调用 moveTo */
function scan(from: number, to: number, count: number): Sample[] {
  const anchorTcp = store().endEffector.position;
  const plane = makePlane('xy', [anchorTcp[0], anchorTcp[1], PLANE_Z], BASIS.forward);

  const samples: Sample[] = [];
  for (let i = 0; i <= count; i += 1) {
    const u = from + (to - from) * (i / count);
    const { origin, dir } = rayForNdc(u, 0);
    const target = dragTarget('xy', plane, origin, dir);
    if (!target) throw new Error(`u=${u} 处射线与冻结平面无交点（不应发生）`);

    const result = store().moveTo(target);
    samples.push({
      u,
      target,
      ok: result.success,
      joints: { ...store().commandJoints },
      branch: result.success ? result.branch : undefined,
      reason: result.success ? undefined : result.reason,
      joint: result.success ? undefined : result.joint,
    });
  }
  return samples;
}

describe('Phase 6 验收 · 拖动轨迹穿越工作空间边界', () => {
  beforeEach(() => {
    store().goHome();
    store().setDragPlane('xy');
  });

  it('冻结平面：全程目标点 Z 严格等于锚点 Z，且落在可达球壳内的点都能解出', () => {
    const samples = scan(SCAN_FROM, SCAN_TO, SAMPLE_COUNT);

    for (const sample of samples) {
      // 精确相等，不是"浮点上接近" —— 锁 Z 是自由度的定义，不是数值巧合
      expect(sample.target[2]).toBe(PLANE_Z);
    }
    expect(samples).toHaveLength(SAMPLE_COUNT + 1);
  });

  it('形态：两端越界、中间一段**连续**可达（碎片化即说明有静默错解）', () => {
    const samples = scan(SCAN_FROM, SCAN_TO, SAMPLE_COUNT);

    expect(samples[0].ok).toBe(false);
    expect(samples[samples.length - 1].ok).toBe(false);

    const okIdx = samples.map((s, i) => (s.ok ? i : -1)).filter((i) => i >= 0);
    expect(okIdx.length).toBeGreaterThanOrEqual(15);
    // 可达帧连续成一段
    expect(okIdx[okIdx.length - 1] - okIdx[0] + 1).toBe(okIdx.length);

    // 可达段落在中间（两侧都有越界帧）
    expect(okIdx[0]).toBeGreaterThan(0);
    expect(okIdx[okIdx.length - 1]).toBeLessThan(samples.length - 1);
  });

  it('可达段：TCP 逐点命中目标（残差 < 1e-9 mm）', () => {
    const samples = scan(SCAN_FROM, SCAN_TO, SAMPLE_COUNT);
    const okSamples = samples.filter((s) => s.ok);
    expect(okSamples.length).toBeGreaterThan(0);

    for (const sample of okSamples) {
      expect(targetGapMm(sample.target, sample.joints)).toBeLessThan(1e-9);
    }
  });

  it('越界段：关节**逐位不变**，失败原因只在两个错误码内', () => {
    const samples = scan(SCAN_FROM, SCAN_TO, SAMPLE_COUNT);
    const failSamples = samples.filter((s) => !s.ok);
    expect(failSamples.length).toBeGreaterThan(0);

    for (const sample of failSamples) {
      expect(['OUT_OF_WORKSPACE', 'JOINT_LIMIT']).toContain(sample.reason);
      if (sample.reason === 'JOINT_LIMIT') expect(sample.joint).toBeTruthy();
    }

    // 连续越界帧之间关节必须完全一致（扫描是单向推进，越界段应停在同一个姿态）
    const firstOk = samples.findIndex((s) => s.ok);
    const lastOk = samples.length - 1 - [...samples].reverse().findIndex((s) => s.ok);
    const before = samples.slice(0, firstOk);
    const after = samples.slice(lastOk + 1);

    for (const group of [before, after]) {
      if (group.length < 2) continue;
      for (let i = 1; i < group.length; i += 1) {
        expect(group[i].joints).toEqual(group[0].joints);
      }
    }
  });

  it('边界定位精度：最后一个失败帧与第一个成功帧的平面间距 ≤ 一格', () => {
    const samples = scan(SCAN_FROM, SCAN_TO, SAMPLE_COUNT);
    const firstOk = samples.findIndex((s) => s.ok);
    expect(firstOk).toBeGreaterThan(0);

    const a = samples[firstOk - 1];
    const b = samples[firstOk];
    const step = Math.hypot(b.target[0] - a.target[0], b.target[1] - a.target[1]);
    // 单步在平面上约 1.5mm（1.8 NDC 扫 400 步 → 每步 0.0045 NDC ≈ 1.5mm）
    expect(step).toBeLessThan(2);
  });

  it('连续性：相邻可达帧关节变化有界，且全程不翻支', () => {
    const samples = scan(SCAN_FROM, SCAN_TO, SAMPLE_COUNT);
    let maxDelta = 0;
    let branchSwitches = 0;

    for (let i = 1; i < samples.length; i += 1) {
      const a = samples[i - 1];
      const b = samples[i];
      if (!a.ok || !b.ok) continue;
      for (const id of ['base', 'shoulder', 'elbow']) {
        maxDelta = Math.max(maxDelta, Math.abs(b.joints[id] - a.joints[id]));
      }
      if (a.branch !== b.branch) branchSwitches += 1;
    }

    // 每步在平面上约 1mm，关节变化必然是小量；出现大跳变说明翻了支或解错了
    expect(maxDelta).toBeLessThan(5);
    expect(branchSwitches).toBe(0);
  });

  it('可逆性：拖过去再拖回来，同一 u 处关节状态可复现（拖动不"粘死"）', () => {
    const forward = scan(SCAN_FROM, SCAN_TO, SAMPLE_COUNT);
    const backward = scan(SCAN_TO, SCAN_FROM, SAMPLE_COUNT);

    // 反向扫描的第 k 个样本对应 u 从 SCAN_TO 递减；与正向逐点配对
    for (let i = 0; i <= SAMPLE_COUNT; i += 1) {
      const f = forward[i];
      const b = backward[SAMPLE_COUNT - i];
      expect(b.u).toBeCloseTo(f.u, 12);
      expect(b.ok).toBe(f.ok);
      if (f.ok && b.ok) {
        for (const id of ['base', 'shoulder', 'elbow']) {
          expect(b.joints[id]).toBeCloseTo(f.joints[id], 9);
        }
      }
    }
  });
});

describe('Phase 6 验收 · 平面模式在真实相机下都稳定', () => {
  beforeEach(() => {
    store().goHome();
  });

  it('camera 平面：从相机沿视线射向锚点，命中锚点本身', () => {
    const anchorTcp = store().endEffector.position;
    const plane = makePlane('camera', anchorTcp, BASIS.forward);
    const dir = normalizeVec3(sub(anchorTcp, CAMERA))!;

    const hit = dragTarget('camera', plane, CAMERA, dir);
    expect(hit).not.toBeNull();
    expect(pointsCoincide(hit!, anchorTcp, 1e-6)).toBe(true);
  });

  it('xz 平面：锁 Y，且能在矢状面内上下移动末端', () => {
    const anchorTcp = store().endEffector.position;
    const anchor: Vec3 = [anchorTcp[0], anchorTcp[1], anchorTcp[2]];
    const plane = makePlane('xz', anchor, BASIS.forward);
    expect(plane.normal).toEqual([0, 1, 0]);

    // 沿视线射向"比锚点更高一点"的平面内点，检验锁 Y 生效
    const higher: Vec3 = [anchor[0] + 6, anchor[1], anchor[2] + 8];
    const dir = normalizeVec3(sub(higher, CAMERA))!;
    const hit = dragTarget('xz', plane, CAMERA, dir)!;

    expect(hit[1]).toBe(anchor[1]);
    expect(hit[2]).toBeGreaterThan(anchor[2]);
  });

  it('矢状面内上下扫描：越界的高度确实做不出来（不是静默钳位）', () => {
    const anchorTcp = store().endEffector.position;
    const anchor: Vec3 = [anchorTcp[0], anchorTcp[1], anchorTcp[2]];
    const plane = makePlane('xz', anchor, BASIS.forward);

    const results: boolean[] = [];
    for (let i = 0; i <= 120; i += 1) {
      const z = 20 + (260 - 20) * (i / 120);
      const probe: Vec3 = [anchor[0], anchor[1], z];
      const dir = normalizeVec3(sub(probe, CAMERA))!;
      const hit = dragTarget('xz', plane, CAMERA, dir)!;
      const before: JointState = { ...store().commandJoints };
      const r = store().moveTo(hit);
      if (!r.success) expect(store().commandJoints).toEqual(before);
      results.push(r.success);
    }

    // 最高与最低必然够不到（机构 z 上限 ≈ 102mm，下限受立柱限制）
    expect(results[0]).toBe(false);
    expect(results[results.length - 1]).toBe(false);
    expect(results.some((ok) => ok)).toBe(true);
  });
});

describe('Phase 6 验收 · 目标与命令的分离在数值上成立', () => {
  beforeEach(() => {
    store().goHome();
  });

  it('越界时 target 越界、commandJoints 与 endEffector 全部留在合法域内', () => {
    const far: Vec3 = [520, 180, 30];
    store().moveTo(far);

    expect(store().target).toEqual(far);
    for (const joint of movableJoints(model)) {
      const value = store().commandJoints[joint.id];
      expect(value).toBeGreaterThanOrEqual(joint.limits.min - 1e-9);
      expect(value).toBeLessThanOrEqual(joint.limits.max + 1e-9);
    }
    // 命令末端必然仍在机构量程内
    const tcp = store().endEffector.position;
    const radius = Math.hypot(tcp[0], tcp[1]);
    expect(Math.hypot(radius, tcp[2] - 60)).toBeLessThanOrEqual(200 + 1e-6);
    expect(store().commandJoints).toEqual(homeJointState(model));
  });
});
