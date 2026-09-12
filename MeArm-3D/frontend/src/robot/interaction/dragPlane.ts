/**
 * 拖动平面：把鼠标射线投影成 3D 目标点。
 *
 * **纯数学、零依赖** —— 不 import three、不碰 DOM，因此可在 node 中直接单测。
 * 调用方（R3F 组件）只需把 `event.ray.origin` / `event.ray.direction` 拆成 Vec3 传进来。
 *
 * ⚠️ 平面必须在 `pointerdown` 时**冻结**（`point` 取按下瞬间的 TCP，法线按模式固定）。
 * 若每帧按当前 TCP 重建平面，会形成自反馈环：
 *   目标点变 → 关节变 → TCP 变 → 平面又跟着变 → 手势"飘走"且永远收敛不到鼠标处。
 * 冻结后平面与鼠标之间是**静态映射**，手感才跟手。
 */
import type { Vec3 } from '../model/Pose';

/** 拖动平面模式 */
export type DragPlaneMode =
  /** 水平面（法线 = 世界 Z）：只改 X / Y，Z 锁死在锚点高度 */
  | 'xy'
  /** 朝向相机的平面（法线 = 视线反向）：X / Y / Z 全自由 */
  | 'camera'
  /** 矢状面（法线 = 世界 Y）：只改 X / Z，Y 锁死在锚点 */
  | 'xz';

export interface PlaneSpec {
  /** 平面上一点（拖动期间冻结，= pointerdown 瞬间的 TCP） */
  point: Vec3;
  /** 单位法线 */
  normal: Vec3;
}

/** 与平面近乎平行 / 交点数值不稳定时的判定阈值 */
const EPS_PARALLEL = 1e-9;

export const DRAG_PLANE_MODES: readonly DragPlaneMode[] = ['xy', 'camera', 'xz'];

export function dragPlaneLabel(mode: DragPlaneMode): string {
  switch (mode) {
    case 'xy':
      return '水平面 XY（锁 Z）';
    case 'camera':
      return '朝向相机（自由）';
    case 'xz':
      return '矢状面 XZ（锁 Y）';
  }
}

// ---------------------------------------------------------------------------
// 向量小工具（保持零依赖，故不引 three）
// ---------------------------------------------------------------------------

function sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function addScaled(a: Vec3, dir: Vec3, k: number): Vec3 {
  return [a[0] + dir[0] * k, a[1] + dir[1] * k, a[2] + dir[2] * k];
}

function dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function length(v: Vec3): number {
  return Math.hypot(v[0], v[1], v[2]);
}

/** 归一化；零向量返回 null（调用方据此回退），避免产生 NaN 法线 */
export function normalizeVec3(v: Vec3): Vec3 | null {
  const len = length(v);
  if (!(len > EPS_PARALLEL)) return null;
  return [v[0] / len, v[1] / len, v[2] / len];
}

// ---------------------------------------------------------------------------
// 平面
// ---------------------------------------------------------------------------

/**
 * 求某模式的平面法线。
 *
 * `cameraDir` 语义 = **相机视线方向**（即 `camera.getWorldDirection()`，由相机指向场景）。
 * 朝向相机的平面，其法线应为反方向（指向相机），故取负。
 * 相机方向退化（零向量）时回退到水平面法线，宁可锁 Z 也不产生 NaN。
 */
export function planeNormalFor(mode: DragPlaneMode, cameraDir: Vec3): Vec3 {
  switch (mode) {
    case 'xy':
      return [0, 0, 1];
    case 'xz':
      return [0, 1, 0];
    case 'camera':
      return normalizeVec3([-cameraDir[0], -cameraDir[1], -cameraDir[2]]) ?? [0, 0, 1];
  }
}

/** 构造冻结平面：过 `anchorTcp`，法线由模式决定 */
export function makePlane(
  mode: DragPlaneMode,
  anchorTcp: Vec3,
  cameraDir: Vec3,
): PlaneSpec {
  return {
    point: [anchorTcp[0], anchorTcp[1], anchorTcp[2]],
    normal: planeNormalFor(mode, cameraDir),
  };
}

/**
 * 把平面上的点吸附到该模式允许的自由度上。
 *
 * `xy` / `xz` 模式显式取回锚点对应分量，让「锁死」成为**精确相等**而非
 * 「理论上相等、浮点上差 1e-15」—— 测试可以直接断言 `=== anchor[2]`。
 */
export function snapToMode(mode: DragPlaneMode, point: Vec3, anchor: Vec3): Vec3 {
  switch (mode) {
    case 'xy':
      return [point[0], point[1], anchor[2]];
    case 'xz':
      return [point[0], anchor[1], point[2]];
    case 'camera':
      return [point[0], point[1], point[2]];
  }
}

// ---------------------------------------------------------------------------
// 射线 ∩ 平面
// ---------------------------------------------------------------------------

/**
 * 射线与平面求交。
 *
 * `dir` 无需归一化（内部按参数 t 求解，与长度无关）。
 * 返回 null 的两种情况：
 *   - **平行**：`|n·d| < 1e-9`，无交点或整条射线都在平面内；
 *   - **背向**：交点位于相机后方（`t < 0`）—— 鼠标"看向"平面之外，此时不该产生目标点。
 */
export function intersectRayPlane(
  origin: Vec3,
  dir: Vec3,
  plane: PlaneSpec,
): Vec3 | null {
  const denom = dot(plane.normal, dir);
  if (Math.abs(denom) < EPS_PARALLEL) return null;

  const t = dot(plane.normal, sub(plane.point, origin)) / denom;
  if (!(t >= 0)) return null;

  return addScaled(origin, dir, t);
}

/**
 * 拖动链的完整一步：鼠标射线 → 冻结平面 → 吸附自由度 → 目标点。
 * 任一步失败（平行 / 背向）返回 null，调用方保持上一目标不动。
 */
export function dragTarget(
  mode: DragPlaneMode,
  plane: PlaneSpec,
  rayOrigin: Vec3,
  rayDir: Vec3,
): Vec3 | null {
  const hit = intersectRayPlane(rayOrigin, rayDir, plane);
  if (!hit) return null;
  return snapToMode(mode, hit, plane.point);
}

/** 两点是否在允许误差内重合（用于判断"有没有真的动"） */
export function pointsCoincide(a: Vec3, b: Vec3, tolMm = 1e-9): boolean {
  return (
    Math.abs(a[0] - b[0]) <= tolMm &&
    Math.abs(a[1] - b[1]) <= tolMm &&
    Math.abs(a[2] - b[2]) <= tolMm
  );
}
