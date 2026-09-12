/**
 * Phase 6 单元测试：拖动平面与射线求交（纯数学层）。
 *
 * 这一层不 import three、不碰 DOM，因此可以把「平面冻结」「射线平行 / 背向」等
 * 最容易出**静默错误**的边界一次算清 —— 交互层出问题时，先怀疑这里。
 */
import { describe, expect, it } from 'vitest';
import {
  DRAG_PLANE_MODES,
  dragPlaneLabel,
  dragTarget,
  intersectRayPlane,
  makePlane,
  normalizeVec3,
  planeNormalFor,
  pointsCoincide,
  snapToMode,
  type Vec3,
} from '../../src/robot';

describe('Phase 6 · 拖动平面法线', () => {
  it('xy 模式法线恒为世界 Z，与相机方向无关', () => {
    expect(planeNormalFor('xy', [0.3, 0.4, 0.5])).toEqual([0, 0, 1]);
    expect(planeNormalFor('xy', [0, 0, 0])).toEqual([0, 0, 1]);
  });

  it('xz 模式法线恒为世界 Y', () => {
    expect(planeNormalFor('xz', [1, 2, 3])).toEqual([0, 1, 0]);
  });

  it('camera 模式法线 = 相机视线反向（指向相机）且已归一化', () => {
    const n = planeNormalFor('camera', [0, 0, -2]);
    expect(n[0]).toBeCloseTo(0, 12);
    expect(n[1]).toBeCloseTo(0, 12);
    expect(n[2]).toBeCloseTo(1, 12);
    expect(Math.hypot(n[0], n[1], n[2])).toBeCloseTo(1, 12);
  });

  it('相机方向退化（零向量）时回退到水平面法线，绝不产生 NaN', () => {
    const n = planeNormalFor('camera', [0, 0, 0]);
    expect(n).toEqual([0, 0, 1]);
    expect(n.every((v) => Number.isFinite(v))).toBe(true);
  });

  it('三种模式都有可读标签', () => {
    for (const mode of DRAG_PLANE_MODES) {
      expect(dragPlaneLabel(mode).length).toBeGreaterThan(2);
    }
  });
});

describe('Phase 6 · normalizeVec3', () => {
  it('零向量返回 null 而不是 NaN 向量', () => {
    expect(normalizeVec3([0, 0, 0])).toBeNull();
  });

  it('归一化后模长为 1', () => {
    const n = normalizeVec3([3, 4, 0])!;
    expect(n[0]).toBeCloseTo(0.6, 12);
    expect(n[1]).toBeCloseTo(0.8, 12);
    expect(Math.hypot(n[0], n[1], n[2])).toBeCloseTo(1, 12);
  });
});

describe('Phase 6 · 射线 ∩ 平面', () => {
  const plane = makePlane('xy', [0, 0, 100], [0, 0, -1]);

  it('正交命中：交点落在平面内', () => {
    const hit = intersectRayPlane([20, 30, 300], [0, 0, -1], plane)!;
    expect(hit).not.toBeNull();
    expect(hit[0]).toBeCloseTo(20, 9);
    expect(hit[1]).toBeCloseTo(30, 9);
    expect(hit[2]).toBeCloseTo(100, 9);
  });

  it('斜射命中：从 z=300 沿 (0.6, 0, -0.8) 应落在 x=150', () => {
    const hit = intersectRayPlane([0, 0, 300], [0.6, 0, -0.8], plane)!;
    expect(hit[0]).toBeCloseTo(150, 9);
    expect(hit[1]).toBeCloseTo(0, 9);
    expect(hit[2]).toBeCloseTo(100, 9);
  });

  it('方向向量长度不影响交点（t 的求解与 |d| 无关）', () => {
    const a = intersectRayPlane([0, 0, 300], [0, 0, -1], plane)!;
    const b = intersectRayPlane([0, 0, 300], [0, 0, -7.5], plane)!;
    expect(pointsCoincide(a, b, 1e-9)).toBe(true);
  });

  it('平行射线 → null（无交点，或射线整条躺在平面内）', () => {
    expect(intersectRayPlane([0, 0, 300], [1, 0, 0], plane)).toBeNull();
    expect(intersectRayPlane([5, 5, 100], [1, 1, 0], plane)).toBeNull();
  });

  it('背向射线（平面在相机身后）→ null，不产生"幽灵目标"', () => {
    expect(intersectRayPlane([0, 0, 50], [0, 0, -1], plane)).toBeNull();
  });

  it('射线起点恰在平面上 → t=0，命中起点自身', () => {
    const hit = intersectRayPlane([7, 8, 100], [0, 0, -1], plane)!;
    expect(hit[0]).toBeCloseTo(7, 9);
    expect(hit[2]).toBeCloseTo(100, 9);
  });
});

describe('Phase 6 · 自由度吸附（锁死分量取精确相等，而非"浮点上接近"）', () => {
  it('xy 模式把 Z 精确设回锚点 Z', () => {
    expect(snapToMode('xy', [12.5, -3.25, 999.125], [0, 0, 100])).toEqual([12.5, -3.25, 100]);
  });

  it('xz 模式把 Y 精确设回锚点 Y', () => {
    expect(snapToMode('xz', [12.5, 77, 40], [0, 5, 100])).toEqual([12.5, 5, 40]);
  });

  it('camera 模式三轴全保留', () => {
    const v: Vec3 = [1, 2, 3];
    expect(snapToMode('camera', v, [9, 9, 9])).toEqual(v);
  });
});

describe('Phase 6 · dragTarget 组合（射线 → 冻结平面 → 吸附）', () => {
  it('xy 平面拖动：只改 X / Y，Z 严格等于锚点 Z', () => {
    const anchor: Vec3 = [100, 0, 95];
    const frozen = makePlane('xy', anchor, [0, 0, -1]);

    expect(dragTarget('xy', frozen, [100, 0, 400], [0, 0, -1])).toEqual([100, 0, 95]);
    expect(dragTarget('xy', frozen, [260, 55, 400], [0, 0, -1])).toEqual([260, 55, 95]);
  });

  it('平面冻结后目标点恒在该平面上（换射线起点也成立）', () => {
    const anchor: Vec3 = [120, -30, 95];
    const frozen = makePlane('xy', anchor, [0, 0, -1]);
    for (const origin of [[300, -430, 300], [-200, 100, 500], [0, 0, 1000]] as Vec3[]) {
      const dir = normalizeVec3([
        anchor[0] - origin[0],
        anchor[1] - origin[1],
        anchor[2] - origin[2],
      ])!;
      const hit = dragTarget('xy', frozen, origin, dir)!;
      expect(hit[2]).toBe(anchor[2]);
      expect(pointsCoincide(hit, anchor, 1e-6)).toBe(true);
    }
  });

  it('平行 / 背向时返回 null，调用方据此保持上一目标不动', () => {
    const frozen = makePlane('xy', [0, 0, 100], [0, 0, -1]);
    expect(dragTarget('xy', frozen, [0, 0, 300], [1, 0, 0])).toBeNull();
    expect(dragTarget('xy', frozen, [0, 0, 50], [0, 0, -1])).toBeNull();
  });
});
