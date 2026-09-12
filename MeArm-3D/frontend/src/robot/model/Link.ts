/**
 * Link（连杆）——纯几何量。
 *
 * `length` 是**运动学量**：本连杆「近端关节坐标系 → 远端关节坐标系」的距离，
 * 方向为近端坐标系 +Z。它必须等于其子关节 `origin.position` 的模长（由模型校验强制），
 * 以保证「连杆长度」与「关节位置」不会出现两个互相矛盾的真值来源。
 *
 * `geometry` / `details` 只用于 3D 显示，**不参与任何运动学计算**。
 * 全部件由参数化基础体（plate / box / cylinder / sphere / servo）拼装，
 * 后续可替换为 GLB/GLTF/OBJ 而不影响运动学（spec §29 / §30）。
 */
import type { EulerDeg, Vec3 } from './Pose';

/**
 * 倒角薄板：低多边形工程外观的主力件（底盘 / 立柱侧板 / 臂板 / 爪片）。
 * 对应 three.js `RoundedBoxGeometry`。`size = [x, y, z]`（mm），位置为几何中心。
 */
export interface PlateGeometry {
  type: 'plate';
  size: Vec3;
  /** 倒角半径（mm）；缺省用 DEFAULT_PLATE_CORNER_RADIUS，渲染时按最小边自动钳位 */
  cornerRadius?: number;
  position?: Vec3;
  rotation?: EulerDeg;
  color?: string;
}

/**
 * 舵机壳体：壳体 + 两侧安装耳 + 金属输出轴 + 舵盘。
 * **本体局部坐标系约定：`+Z` 为输出轴方向，原点在壳体中心。**
 * 因此「轴朝上」不写 rotation，「轴朝 +Y」写 rotation: [-90, 0, 0]。
 */
export interface ServoGeometry {
  type: 'servo';
  /** 壳体尺寸 [x, y, z]（mm），缺省为常见微型舵机 DEFAULT_SERVO_SIZE */
  size?: Vec3;
  /** 输出轴伸出长度（mm），缺省 DEFAULT_SERVO_SHAFT_LENGTH */
  shaftLength?: number;
  /** 是否绘制两侧安装耳，缺省 true */
  ears?: boolean;
  position?: Vec3;
  rotation?: EulerDeg;
  /** 壳体颜色，缺省 DEFAULT_SERVO_COLOR */
  color?: string;
  /** 输出轴 / 舵盘颜色，缺省 DEFAULT_SERVO_SHAFT_COLOR */
  shaftColor?: string;
}

export type LinkGeometry =
  | { type: 'none'; color?: string }
  | { type: 'box'; size: Vec3; position?: Vec3; rotation?: EulerDeg; color?: string }
  | PlateGeometry
  | {
      type: 'cylinder';
      radius: number;
      height: number;
      radialSegments?: number;
      position?: Vec3;
      rotation?: EulerDeg;
      color?: string;
    }
  | { type: 'sphere'; radius: number; position?: Vec3; rotation?: EulerDeg; color?: string }
  | ServoGeometry;

export interface Link {
  id: string;
  name: string;
  /** 父连杆 id；根连杆为 null / undefined */
  parent?: string | null;
  /** 连杆长度（mm），沿近端关节坐标系 +Z */
  length: number;
  geometry: LinkGeometry;
  /** 附加显示件（舵机 / 螺栓 / 轴销等），挂在同一坐标系下，仅用于 3D 显示 */
  details?: LinkGeometry[];
}

// ---------------------------------------------------------------------------
// 默认值（全部为参数化默认，可在 robot.yaml 中逐项覆盖）
// ---------------------------------------------------------------------------

export const DEFAULT_LINK_COLOR = '#8b93a7';
/** 薄板默认倒角半径（mm）——「边缘适当倒角」的最小成本实现（spec §5） */
export const DEFAULT_PLATE_CORNER_RADIUS = 1.6;
/** 常见微型舵机壳体尺寸（SG90 / MG90S 量级），仅作默认值，非实测 */
export const DEFAULT_SERVO_SIZE: Vec3 = [22.8, 12.2, 22.5];
export const DEFAULT_SERVO_SHAFT_LENGTH = 5;
export const DEFAULT_SERVO_COLOR = '#1b1f28';
export const DEFAULT_SERVO_SHAFT_COLOR = '#c9ced8';
/** jaw_link.geometry 未给出尺寸时的爪片默认值 [宽, 厚, 长] */
export const DEFAULT_JAW_SIZE: Vec3 = [10, 5, 30];

export function isRootLink(link: Link): boolean {
  return link.parent === null || link.parent === undefined;
}

/** 附加显示件（缺省空数组，避免调用方到处写 `?? []`） */
export function linkDetails(link: Link): LinkGeometry[] {
  return link.details ?? [];
}

/**
 * 显示几何相对近端关节坐标系的平移偏移。
 * 省略 `position` 时按「连杆中段」放置（0, 0, length/2），这是最符合直觉的默认值。
 */
export function geometryPosition(geometry: LinkGeometry, linkLength = 0): Vec3 {
  if (geometry.type === 'none') return [0, 0, 0];
  if (geometry.position) return [...geometry.position];
  return [0, 0, linkLength / 2];
}

/** 显示几何相对近端关节坐标系的旋转（缺省零） */
export function geometryRotation(geometry: LinkGeometry): EulerDeg {
  if (geometry.type === 'none') return [0, 0, 0];
  return geometry.rotation ? [...geometry.rotation] : [0, 0, 0];
}

export function geometryColor(geometry: LinkGeometry): string {
  return geometry.color ?? DEFAULT_LINK_COLOR;
}

/** 薄板倒角半径：按最小边自动钳位，防止半径大于半边长导致几何自交 */
export function plateCornerRadius(geometry: PlateGeometry): number {
  const requested = geometry.cornerRadius ?? DEFAULT_PLATE_CORNER_RADIUS;
  const halfMin = Math.min(...geometry.size) / 2;
  if (!(halfMin > 0)) return 0;
  return Math.max(0, Math.min(requested, halfMin - 0.01));
}

/** 舵机渲染参数（补齐全部默认值，渲染层不再关心缺省逻辑） */
export interface ServoRenderSpec {
  size: Vec3;
  shaftLength: number;
  ears: boolean;
  bodyColor: string;
  shaftColor: string;
}

export function servoRenderSpec(geometry: ServoGeometry): ServoRenderSpec {
  return {
    size: geometry.size ? [...geometry.size] : [...DEFAULT_SERVO_SIZE],
    shaftLength: geometry.shaftLength ?? DEFAULT_SERVO_SHAFT_LENGTH,
    ears: geometry.ears ?? true,
    bodyColor: geometry.color ?? DEFAULT_SERVO_COLOR,
    shaftColor: geometry.shaftColor ?? DEFAULT_SERVO_SHAFT_COLOR,
  };
}

/** 爪片尺寸 [宽, 厚, 长]；由 jaw_link.geometry 派生，缺省用 DEFAULT_JAW_SIZE */
export function jawPlateSize(link: Link | undefined): Vec3 {
  const geometry = link?.geometry;
  if (geometry && (geometry.type === 'plate' || geometry.type === 'box')) {
    return [...geometry.size];
  }
  return [...DEFAULT_JAW_SIZE];
}
