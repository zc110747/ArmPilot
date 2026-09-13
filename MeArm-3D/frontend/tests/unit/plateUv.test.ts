/**
 * `RoundedBoxGeometry` 的 UV 布局守卫（ADR D63 一 / 二）。
 *
 * 为什么值得钉住：这张表决定了**照片纹理贴在哪个面、朝哪个方向**。
 * 它不会"坏掉"，但会在 **three 升级**时**静默改变** —— 类型检查看不见，
 * 单元测试也不会失败，只会表现为"贴图突然反了/跑到侧面去了"。
 * 所以把实测值写死在这里当基线：一旦不符，测试会直接打印实际方向。
 *
 * ⚠️ 同一批实测的另一个事实：`RoundedBoxGeometry` 是**非索引几何**
 * （`g.index === null`）⇒ group 的 `start/count` 是**顶点范围**，不是索引范围。
 */
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { describe, expect, it } from 'vitest';

/** 大臂板的真实尺寸（与 robot.yaml 一致），用于确保测的是实际用到的那档几何 */
const SIZE: [number, number, number] = [22, 5, 74];

interface FaceAxis {
  /** 面法向（近似） */
  normal: [number, number, number];
  /** uv 的 u 增大方向（局部坐标，近似） */
  du: [number, number, number];
  /** uv 的 v 增大方向（局部坐标，近似） */
  dv: [number, number, number];
}

/** 实测基线（three 0.186；`.workbuddy/captures/uv_probe.mjs` 的产物） */
const EXPECTED: Record<number, FaceAxis> = {
  0: { normal: [1, 0, 0], du: [0, 0, -1], dv: [0, 1, 0] },
  1: { normal: [-1, 0, 0], du: [0, 0, 1], dv: [0, 1, 0] },
  2: { normal: [0, 1, 0], du: [1, 0, 0], dv: [0, 0, -1] },
  3: { normal: [0, -1, 0], du: [1, 0, 0], dv: [0, 0, 1] },
  4: { normal: [0, 0, 1], du: [1, 0, 0], dv: [0, 1, 0] },
  5: { normal: [0, 0, -1], du: [-1, 0, 0], dv: [0, 1, 0] },
};

const geometry = new RoundedBoxGeometry(SIZE[0], SIZE[1], SIZE[2], 2, 2);
const position = geometry.attributes.position!;
const normal = geometry.attributes.normal!;
const uv = geometry.attributes.uv!;

/** 在某个 group 的顶点里，找 uv 最接近给定角的顶点（倒角区不会覆盖到极值点，最近邻足够） */
function nearestVertex(vertices: number[], tu: number, tv: number): number {
  let best = vertices[0]!;
  let bestDist = Infinity;
  for (const k of vertices) {
    const d = (uv.getX(k) - tu) ** 2 + (uv.getY(k) - tv) ** 2;
    if (d < bestDist) {
      bestDist = d;
      best = k;
    }
  }
  return best;
}

function axisOf(vertices: number[]): FaceAxis {
  const avg = new THREE.Vector3();
  let umin = Infinity;
  let umax = -Infinity;
  let vmin = Infinity;
  let vmax = -Infinity;
  for (const k of vertices) {
    avg.add(new THREE.Vector3(normal.getX(k), normal.getY(k), normal.getZ(k)));
    umin = Math.min(umin, uv.getX(k));
    umax = Math.max(umax, uv.getX(k));
    vmin = Math.min(vmin, uv.getY(k));
    vmax = Math.max(vmax, uv.getY(k));
  }
  avg.normalize();

  const pick = (a: number, b: number) => {
    const k = nearestVertex(vertices, a, b);
    return new THREE.Vector3(position.getX(k), position.getY(k), position.getZ(k));
  };
  const o = pick(umin, vmin);
  const du = pick(umax, vmin).sub(o).normalize();
  const dv = pick(umin, vmax).sub(o).normalize();

  const t = (v: THREE.Vector3): [number, number, number] => {
    // `+ 0` 把 -0 归一成 +0：方向分量的负零没有意义，但会让 toEqual 判不等
    const r = (x: number) => Math.round(x * 100) / 100 + 0;
    return [r(v.x), r(v.y), r(v.z)];
  };
  return { normal: t(avg), du: t(du), dv: t(dv) };
}

describe('RoundedBoxGeometry 的 UV 布局（防 three 升级静默改行为）', () => {
  it('是非索引几何 —— 按索引遍历会 TypeError，必须按顶点范围读', () => {
    expect(geometry.index).toBeNull();
  });

  it('6 个 group 覆盖全部顶点，且每个面 UV 各自铺满 [0,1]', () => {
    expect(geometry.groups.length).toBe(6);

    const covered = new Set<number>();
    for (const group of geometry.groups) {
      for (let i = group.start; i < group.start + group.count; i++) covered.add(i);
      const verts = Array.from({ length: group.count }, (_, i) => group.start + i);
      let umin = Infinity;
      let umax = -Infinity;
      let vmin = Infinity;
      let vmax = -Infinity;
      for (const k of verts) {
        umin = Math.min(umin, uv.getX(k));
        umax = Math.max(umax, uv.getX(k));
        vmin = Math.min(vmin, uv.getY(k));
        vmax = Math.max(vmax, uv.getY(k));
      }
      expect(umin).toBeCloseTo(0, 5);
      expect(umax).toBeCloseTo(1, 5);
      expect(vmin).toBeCloseTo(0, 5);
      expect(vmax).toBeCloseTo(1, 5);
    }
    expect(covered.size).toBe(position.count);
  });

  it('★ 每个面的 Δu/Δv 方向与 D63 实测基线一致', () => {
    for (const group of geometry.groups) {
      const index = group.materialIndex!;
      const verts = Array.from({ length: group.count }, (_, i) => group.start + i);
      const actual = axisOf(verts);
      const expected = EXPECTED[index]!;

      for (const key of ['normal', 'du', 'dv'] as const) {
        expect(
          actual[key],
          `materialIndex=${index} 的 ${key} 实测 ${actual[key].join(',')} ` +
            `与基线 ${expected[key].join(',')} 不符 —— three 可能改了 UV 布局，` +
            `需同步 buildRobotObject3D 的 FACE_GROUPS 与 ADR D63`,
        ).toEqual(expected[key]);
      }
    }
  });

  it('两个大面（±Y）的 Δv 相反 —— 因此同一张图必有一面要镜像', () => {
    const yPlus = EXPECTED[2]!;
    const yMinus = EXPECTED[3]!;
    expect(yPlus.du).toEqual(yMinus.du);
    expect(yPlus.dv).not.toEqual(yMinus.dv);
  });
});
