/**
 * FK（正运动学）—— `JointState` → 全部关节坐标系位姿 + 末端位姿。
 *
 * ```
 *   J1 / J2 / J3 / Gripper
 *            ↓
 *   forwardKinematics()
 *            ↓
 *   { joints: Record<id, Transform>, endEffector: Pose }
 * ```
 *
 * 与 Three.js 的**严格等价关系**（Phase 3 验收条件，误差必须 < 0.1mm）：
 *
 *   本文件的 FK 链       T_J = T_parent · Tz(parentLink.length) · T(origin.position)
 *                              · R_euler(origin.rotation, origin.rotationConvention) · R_axis(θ)
 *   Three.js Joint Tree  group.position = origin.position
 *                        group.quaternion = qEuler(origin.rotation, convention) · qAxisAngle(axis, θ)
 *                        （父 group 已承担 Tz(parentLink.length)）
 *
 * 其中 `R_euler(·, convention)` 缺省为 intrinsic XYZ（既有行为），声明 `'rpy'` 时
 * 按 URDF `<origin rpy>` 的 fixed-axis XYZ 解释 —— 见 `transform.ts → mat4EulerByConvention`。
 *
 * 单位：输入关节角 degree，输出位置 mm / 欧拉角 degree。
 * 本文件**不依赖 Three.js**，可在 node 中直接做验收测试。
 */
import type { Joint } from '../model/Joint';
import type { JointState, Transform, Vec3 } from '../model/Pose';
import { degToRad } from '../model/Pose';
import type { RobotModel } from '../model/RobotModel';
import { movableJoints, requireJoint, rootLink } from '../model/RobotModel';
import type { Link } from '../model/Link';
import {
  type Mat4,
  mat4AxisAngle,
  mat4EulerByConvention,
  mat4GetEulerXYZ,
  mat4GetPosition,
  mat4Identity,
  mat4Multiply,
  mat4MultiplyAll,
  mat4ToTransform,
  mat4TransformPoint,
  mat4Translation,
} from './transform';

/** FK 输出：所有关节坐标系位姿 + 末端位姿 */
export interface RobotPose {
  /** key = Joint.id（含固定关节与叶关节，便于 3D 放置与调试） */
  joints: Record<string, Transform>;
  /** 末端（TCP）位姿 */
  endEffector: Transform;
}

/** 关节在给定状态下的角度（固定关节恒为 0；缺省按 limits.min 处理） */
export function jointAngleOf(state: JointState, joint: Joint): number {
  if (joint.type === 'fixed') return 0;
  return state[joint.id] ?? joint.limits.min;
}

/**
 * 关节的**实际旋转角** —— 在 `jointAngleOf` 基础上叠加耦合项。
 *
 *   实际旋转 = value + gain × otherJointValue
 *
 * 用于平行四连杆机构（meArm 小臂）：`value` 存的是**绝对倾角**，
 * 而串联网里该关节的局部旋转必须是「相对父关节」的角，因此减去父关节角
 * （gain = −1）。没有 `coupling` 的关节即等于 `jointAngleOf`。
 *
 * ⚠️ FK 与 Three.js 渲染层必须**同时**使用本函数，否则两者会不一致。
 */
export function effectiveJointAngle(
  model: RobotModel,
  state: JointState,
  joint: Joint,
): number {
  const value = jointAngleOf(state, joint);
  const coupling = joint.coupling;
  if (!coupling) return value;
  const other = model.joints.find((j) => j.id === coupling.jointId);
  if (!other) return value;
  return value + coupling.gain * jointAngleOf(state, other);
}

/** 把关节状态夹到模型限位内（非破坏性） */
export function clampJointState(model: RobotModel, state: JointState): JointState {
  const out: JointState = {};
  for (const joint of movableJoints(model)) {
    const value = state[joint.id] ?? joint.limits.min;
    out[joint.id] = value < joint.limits.min ? joint.limits.min : value > joint.limits.max ? joint.limits.max : value;
  }
  return out;
}

/** 关节状态是否全部落在限位内 */
export function isJointStateWithinLimits(
  model: RobotModel,
  state: JointState,
  epsilonDeg = 1e-9,
): boolean {
  return movableJoints(model).every((joint) => {
    const value = state[joint.id];
    if (value === undefined) return false;
    return value >= joint.limits.min - epsilonDeg && value <= joint.limits.max + epsilonDeg;
  });
}

/**
 * 逐个关节坐标系的世界变换矩阵（沿 RobotModel 的连杆图遍历，天然支持分支 / 叶关节）。
 * 顺序与 `kinematicChain()` 一致（root -> tip）。
 */
export function jointMatrices(model: RobotModel, state: JointState): Map<string, Mat4> {
  const out = new Map<string, Mat4>();
  const jointsByParent = new Map<string, Joint[]>();
  for (const joint of model.joints) {
    const list = jointsByParent.get(joint.parentLink) ?? [];
    list.push(joint);
    jointsByParent.set(joint.parentLink, list);
  }

  const walk = (link: Link, parentJointMatrix: Mat4): void => {
    // 连杆自身的伸展：父关节坐标系 -> 本连杆远端（= 子关节坐标系原点）
    const linkAdvance = mat4Translation([0, 0, link.length]);
    for (const joint of jointsByParent.get(link.id) ?? []) {
      const originTranslation = mat4Translation(joint.origin.position);
      // 按关节**自己声明的**欧拉角约定解释 origin.rotation（缺省 intrinsic XYZ，既有行为不变）。
      // 直接调 mat4EulerXYZ 会把约定硬编码，使任何 'rpy' 来源的配置被静默按错约定解读。
      const originRotation = mat4EulerByConvention(
        joint.origin.rotation,
        joint.origin.rotationConvention,
      );
      const jointRotation =
        joint.type === 'fixed'
          ? mat4Identity()
          : mat4AxisAngle(joint.axis, degToRad(effectiveJointAngle(model, state, joint)));

      const matrix = mat4MultiplyAll(
        parentJointMatrix,
        linkAdvance,
        originTranslation,
        originRotation,
        jointRotation,
      );
      out.set(joint.id, matrix);

      const child = model.links.find((l) => l.id === joint.childLink);
      if (child) walk(child, matrix);
    }
  };

  walk(rootLink(model), mat4Identity());
  return out;
}

/** 末端（TCP）世界变换矩阵 */
export function endEffectorMatrix(model: RobotModel, state: JointState): Mat4 {
  const matrices = jointMatrices(model, state);
  const reference = matrices.get(model.tcp.joint) ?? mat4Identity();
  return mat4Multiply(reference, mat4Translation(model.tcp.offset));
}

/** 正运动学：关节角 → 末端位姿 + 全部关节坐标系位姿 */
export function forwardKinematics(model: RobotModel, state: JointState): RobotPose {
  const matrices = jointMatrices(model, state);
  const joints: Record<string, Transform> = {};
  for (const [id, matrix] of matrices) joints[id] = mat4ToTransform(matrix);
  return { joints, endEffector: mat4ToTransform(endEffectorMatrix(model, state)) };
}

/** 只要末端位置（mm）的快捷方式 */
export function endEffectorPosition(model: RobotModel, state: JointState): Vec3 {
  return mat4GetPosition(endEffectorMatrix(model, state));
}

/** 只要末端位置与欧拉角 */
export function endEffectorPose(model: RobotModel, state: JointState): Transform {
  const matrix = endEffectorMatrix(model, state);
  return { position: mat4GetPosition(matrix), rotation: mat4GetEulerXYZ(matrix) };
}

/**
 * 某个关节坐标系下的点 → 世界坐标。
 * 用于「3D 场景里挂载的传感器 / 标注点」等需要跟随关节的显示对象。
 */
export function pointInJointFrameToWorld(
  model: RobotModel,
  state: JointState,
  jointId: string,
  localPoint: Vec3,
): Vec3 {
  requireJoint(model, jointId);
  const matrices = jointMatrices(model, state);
  const matrix = matrices.get(jointId);
  if (!matrix) throw new Error(`[fk] 关节 ${jointId} 不在连杆图上`);
  return mat4TransformPoint(matrix, localPoint);
}
