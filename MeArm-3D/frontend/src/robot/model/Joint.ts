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
import type { EulerDeg, RotationConvention, Vec3 } from './Pose';
import { clamp } from './Pose';

/**
 * 关节类型。
 *
 * - `revolute` 可动关节：有独立输入（舵机），值进 JointState / UI / JR 协议。
 * - `fixed`    固定关节：完全不转（`origin.rotation` 已表达其固定朝向），
 *              在 `joints` 里存在只为把链描述完整。
 * - `passive`  **被动关节**：会转，但**没有独立输入** —— 其角度完全由 `coupling`
 *              从别的关节派生。它**不进 JointState / UI 滑杆 / JR 协议**，
 *              也不需要执行器与 homePose；它的值恒取 `limits.min`
 *              （因此校验层强制要求被动关节 `min === max`，见 `RobotModel.validateRobotModel`）。
 *
 * 本机的被动关节是 `tool`（腕）：爪的绝对倾角被平行四连杆锁死，
 * 于是它在串联网里的局部旋转 = 90 + (−1) × elbow（实测依据见 config/robot.yaml）。
 */
export type JointType = 'revolute' | 'fixed' | 'passive';

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
  /**
   * `rotation` 的欧拉角约定。**缺省 = `'xyz'`（intrinsic XYZ，既有行为）**。
   *
   * 取 `'rpy'` 时 `rotation` 按 URDF `<origin rpy>` 的 fixed-axis XYZ 解释。
   * 引入它的理由见 `Pose.ts → RotationConvention`：让官方 URDF 的数值可以原样落配置。
   */
  rotationConvention?: RotationConvention;
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
 * 是否为「可动关节」（有真实**独立**自由度的关节 = 一个舵机一个关节）。
 *
 * 这条判据同时决定三件事，所以三处必须同源：
 *   ① 谁进 `JointState`（= 状态帧 / JR 协议 / UI 滑杆）；
 *   ② 谁参与 IK 的未知量；
 *   ③ 谁必须有执行器与 homePose。
 *
 * - 固定关节（`fixed`）：完全不转 ⇒ 否；
 * - 被动关节（`passive`，本机的腕）：会转但**没有输入** ⇒ 否
 *   （它的角度由 `coupling` 派生，混进状态帧会让 JR 从四元组变五元组）。
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
