/**
 * Joint（关节）——运动学量。
 *
 * 语义（与 FK / Three.js Joint Tree 严格一致）：
 *   - 每个关节拥有一个**自己的坐标系**（joint frame）。该坐标系随关节角度旋转，
 *     所以「关节 = 一个 THREE.Group」。
 *   - `origin` 描述「父关节坐标系 → 本关节坐标系」的固定变换：
 *        T_joint = T_parent · Translate(origin.position) · R_eulerXYZ(origin.rotation) · R_axis(θ)
 *     即先把父系平移到本关节位置、应用固定旋转，再绕 `axis` 按右手定则转 θ。
 *   - θ = 0 时，子连杆沿本坐标系 +Z 方向伸展 `childLink.length`。
 *
 * 本文件属于机器人模型层，**不依赖 Three.js**。
 */
import type { EulerDeg, Vec3 } from './Pose';
import { clamp } from './Pose';

export type JointType = 'revolute' | 'fixed';

/**
 * 关节角色。用于 UI 命名（J1/J2/J3/Gripper）与业务逻辑识别，
 * 避免业务代码去硬编码关节 id 字符串。
 */
export type JointRole = 'base' | 'shoulder' | 'elbow' | 'tool' | 'gripper';

export interface JointOrigin {
  /** 父关节坐标系 -> 本关节坐标系 平移（mm） */
  position: Vec3;
  /** 父关节坐标系 -> 本关节坐标系 固定旋转（intrinsic XYZ, degree） */
  rotation: EulerDeg;
}

export interface JointLimits {
  /** 关节角下限（degree） */
  min: number;
  /** 关节角上限（degree） */
  max: number;
}

/**
 * 关节耦合 —— 平行四连杆机构（meArm 小臂）的建模手段。
 *
 * 实测事实（2026-09-12，白色分割背景 + 舵机逐度扫描 + 照片骨架拟合）：
 *   扫 S7 使**大臂/肩**转过 52.91° 时，**小臂**在画面里的绝对倾角只漂移 10.08°
 *   （而相对角变了 42.83°）⇒ 小臂不是串在大臂上的，而是由独立舵机经平行四连杆驱动，
 *     其**绝对倾角只由该舵机决定**，与大臂角无关。
 *
 * 串联网里表达「本关节存的是绝对角」的方式就是声明本耦合：
 *
 *     relative = value + gain × otherJointValue
 *
 * 平行四连杆的 gain 恒为 **−1**（relative = absolute − 父关节绝对角），
 * 于是「拖动大臂时小臂保持绝对角不动」——与真机一致。
 *
 * 注意：`value`（存进 JointState 的那个数）语义是**绝对角**，
 * 因此它的限位区间与父关节无关，标定仍是逐关节仿射映射（calibration 层不用改）。
 */
export interface JointCoupling {
  /** 与之耦合的关节 id（通常是本关节在链上的父关节） */
  jointId: string;
  /** 耦合增益；平行四连杆取 −1 */
  gain: number;
}

export interface Joint {
  id: string;
  name: string;
  role?: JointRole;
  parentLink: string;
  childLink: string;
  type: JointType;
  /** 本关节坐标系下的旋转轴（单位向量） */
  axis: Vec3;
  origin: JointOrigin;
  limits: JointLimits;
  /** 可选：本关节的角度是「绝对角」时，声明与父关节的耦合关系 */
  coupling?: JointCoupling;
}

/**
 * 是否为「可动关节」（有真实自由度的关节）。
 * 固定关节（如腕部 tool）在 `RobotModel.joints` 中存在以便完整描述链，
 * 但不进入 UI 滑杆、不进入 JointState、不参与 IK。
 */
export function isMovableJoint(joint: Joint): boolean {
  return joint.type === 'revolute' && joint.limits.max > joint.limits.min;
}

export function clampJointAngle(joint: Joint, angleDeg: number): number {
  return clamp(angleDeg, joint.limits.min, joint.limits.max);
}

export function jointMidAngle(joint: Joint): number {
  return (joint.limits.min + joint.limits.max) / 2;
}
