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
 * 三数欧拉角三元组的**约定**。
 *
 * - `'xyz'`（**缺省**）：intrinsic XYZ，`R = Rx · Ry · Rz`。
 *   与 Three.js `Euler` 的 order `'XYZ'` 逐值一致，也是 MeArm-V1 一直在用的约定。
 * - `'rpy'`：fixed-axis（extrinsic）XYZ，`R = Rz(yaw) · Ry(pitch) · Rx(roll)`。
 *   这是 **URDF `<origin rpy="r p y">`** 的约定。
 *
 * 为什么需要它：官方 URDF 的 `rpy` 与 `'xyz'` **不是同一种参数化**
 * （extrinsic XYZ(x,y,z) ≡ intrinsic ZYX(z,y,x)，只在特定角组合下才巧合相等）。
 * 若不让配置自己声明约定，接入官方 URDF 时就必须把每个 `rpy` 换算成 intrinsic XYZ ——
 * yaml 里会出现一批**在官方文件里查不到的数**，之后无人能复核它有没有抄错。
 * 声明约定后，官方数值可**逐个原样**落进配置，可审计性最高。
 *
 * ⚠️ 缺省（`undefined`）语义 = `'xyz'`，保证引入本字段**不改变任何既有行为**。
 */
export type RotationConvention = 'xyz' | 'rpy';

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
