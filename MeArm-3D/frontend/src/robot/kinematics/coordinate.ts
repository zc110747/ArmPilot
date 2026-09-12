/**
 * 坐标系与单位转换 —— **全项目唯一的转换层**（spec §九）。
 *
 * ```
 *   机器人世界系（FK / IK / RobotModel）   Z-up 右手系，mm，degree
 *              │
 *      robotPositionToScene / scenePositionToRobot
 *              ↓
 *   Three.js 场景系（渲染）                同样 Z-up，1 scene unit = 1 mm
 * ```
 *
 * **禁止**在业务代码里出现散落的 `-x` / `-y` / `90 - angle` / `180 - angle`。
 * 关节方向、舵机方向、镜像等一切“反着来”的调整，都必须落到：
 *   - `config/robot.yaml` 的 `joint.axis` / `actuator.reverse`（首选，纯配置）
 *   - 或本文件（真正跨坐标系时）
 */
import type { EulerDeg, JointState, Vec3 } from '../model/Pose';
import { degToRad, radToDeg } from '../model/Pose';

/** 机器人世界系定义：右手系，Z 上 / X 前 / Y 左 */
export const ROBOT_FRAME = {
  /** 上下轴 */
  up: [0, 0, 1] as Vec3,
  /** 前后轴（+X 为正前方） */
  forward: [1, 0, 0] as Vec3,
  /** 左右轴（+Y 为左） */
  left: [0, 1, 0] as Vec3,
} as const;

/**
 * Three.js 场景单位与 mm 的比例。
 * 取 1 是为了让「FK 计算出的 XYZ」与「Three.js 模型世界坐标」可以**直接逐值比较**，
 * 不存在任何隐藏缩放导致的验收误差。
 */
export const SCENE_UNITS_PER_MM = 1;

/** 位置：机器人系(场景单位) -> Three.js 世界坐标 */
export function robotPositionToScene(p: Vec3): Vec3 {
  return [p[0] * SCENE_UNITS_PER_MM, p[1] * SCENE_UNITS_PER_MM, p[2] * SCENE_UNITS_PER_MM];
}

/** 位置：Three.js 世界坐标 -> 机器人系(mm) */
export function scenePositionToRobot(p: Vec3): Vec3 {
  return [p[0] / SCENE_UNITS_PER_MM, p[1] / SCENE_UNITS_PER_MM, p[2] / SCENE_UNITS_PER_MM];
}

/** 脚本/位移 -> 场景单位 */
export function mmToScene(mm: number): number {
  return mm * SCENE_UNITS_PER_MM;
}

export function sceneToMm(units: number): number {
  return units / SCENE_UNITS_PER_MM;
}

export function eulerDegToRadVec(rotationDeg: EulerDeg): Vec3 {
  return [degToRad(rotationDeg[0]), degToRad(rotationDeg[1]), degToRad(rotationDeg[2])];
}

export function eulerRadToDegVec(rotationRad: Vec3): EulerDeg {
  return [radToDeg(rotationRad[0]), radToDeg(rotationRad[1]), radToDeg(rotationRad[2])];
}

export function degreesToRadians(deg: number): number {
  return degToRad(deg);
}

export function radiansToDegrees(rad: number): number {
  return radToDeg(rad);
}

/** 平面极坐标：目标点 (x, y) -> 底座偏转角（degree），θ=0 指向 +X */
export function planarAzimuthDeg(x: number, y: number): number {
  return radToDeg(Math.atan2(y, x));
}

/** 笛卡尔 (x, y) -> 水平半径 r（mm，恒 >= 0） */
export function planarRadius(x: number, y: number): number {
  return Math.hypot(x, y);
}

/** 只保留可动关节的键，去除未知关节，避免脏 key 污染 JointState */
export function sanitizeJointState(state: JointState, allowedJointIds: readonly string[]): JointState {
  const out: JointState = {};
  for (const id of allowedJointIds) {
    if (state[id] !== undefined) out[id] = state[id]!;
  }
  return out;
}
