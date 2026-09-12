/**
 * 基础几何 / 状态类型。
 *
 * 单位约定（详见 docs/coordinate-system.md）：
 *   - 位置：mm
 *   - 角度：degree（欧拉角为 intrinsic XYZ，与 Three.js Euler order 'XYZ' 完全一致）
 *   - 坐标系：右手系 Z-up（X 前 / Y 左 / Z 上）
 *
 * 本文件属于机器人模型层，**不依赖 Three.js**，可在 node 中独立测试。
 */

/** 三维向量（长度 mm） */
export type Vec3 = [number, number, number];

/** 欧拉角（intrinsic XYZ，单位 degree） */
export type EulerDeg = [number, number, number];

/**
 * 关节角集合（单位 **degree**），key = Joint.id。
 * 这是虚拟机械臂与真实机械臂之间唯一的“共同语言”。
 */
export type JointState = Record<string, number>;

/** 单个坐标系位姿（位置 mm / 朝向 degree） */
export interface Transform {
  position: Vec3;
  rotation: EulerDeg;
}

/** 末端位姿（与 Transform 同构，语义别名） */
export type Pose = Transform;

/** 控制源：谁在驱动机械臂 */
export type ControlSource = 'virtual' | 'real' | 'command';

export function degToRad(deg: number): number {
  return (deg * Math.PI) / 180;
}

export function radToDeg(rad: number): number {
  return (rad * 180) / Math.PI;
}

export function vec3(x: number, y: number, z: number): Vec3 {
  return [x, y, z];
}

export function cloneVec3(v: Vec3): Vec3 {
  return [v[0], v[1], v[2]];
}

export function cloneJointState(state: JointState): JointState {
  return { ...state };
}

/** 关节角限幅（不改入参） */
export function clamp(value: number, min: number, max: number): number {
  return value < min ? min : value > max ? max : value;
}
