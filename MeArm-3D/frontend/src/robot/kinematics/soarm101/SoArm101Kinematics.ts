/**
 * `SoArm101Kinematics` —— `KinematicsEngine` 在 **SO-ARM101** 上的实现。
 *
 * ## 本文件最重要的一句话
 *
 * > **它只实现 FK。IK 明确地"没有实现"，而且这件事是可被机器检查的。**
 *
 * 这不是偷懒，是 spec §「不伪造」的正面执行。SO-101 的逆解**没有任何可靠性依据**：
 *
 * | 如果硬做一个 IK | 后果 |
 * |---|---|
 * | 抄 MeArm 的平面 2R 解析解 | SO-101 不是平面 2R 机构（腕部另有 2 个自由度、且各轴不是共面串联）⇒ 解出来的角**必然错**，而 `positionError` 还会显示成一个"很小的残差"（因为它用自己的 FK 自证） |
 * | 塞一个数值迭代 | 没有关节雅可比维数的验证、没有收敛判据的实测、没有工作空间边界 ⇒ 返回的"解"无法与"最优的错"区分 |
 * | 直接返回 `homePose` | `success: true` + 一个巨大的 `positionError` —— 上层若只看 `success` 就会把机械臂派到错误的位姿 |
 *
 * ⇒ 所以 `inverse()` 返回 `ikFailure('NOT_IMPLEMENTED', …)`：
 *   `success: false` / `joints: {}` / `positionError: null`。
 *   **失败是这里唯一诚实的结果形状**，而且它是"说得清楚为什么"的失败。
 *
 * ## 与 MeArm 实现的关系
 *
 * `forward` / `forwardAll` 同样是**纯委托**，转给既有的通用 `fk.ts` ——
 * 一行运动学算法都不复制：
 *
 * ```text
 *   SoArm101Kinematics.forward()    ──▶  fk.forwardKinematics().endEffector
 *   SoArm101Kinematics.forwardAll() ──▶  fk.forwardKinematics()
 *   SoArm101Kinematics.inverse()    ──▶  ikFailure('NOT_IMPLEMENTED')   ← 唯一的分歧点
 * ```
 *
 * 之所以能直接复用 `fk.ts`：`jointMatrices()` 是**沿连杆图遍历**的通用实现
 * （不是为 MeArm 写死的四关节直链），它天然支持 SO-101 的 6 铰链 + 1 固定帧结构。
 * 这不是巧合 —— 见 ADR：FK 的通用性正是本阶段抽象成立的前提之一。
 *
 * ## ⚠️ 与"Golden Baseline"的边界
 *
 * 本文件**不修改任何 MeArm-V1 的行为**：它只读取自己那份 `RobotDefinition`，
 * 而 `SoArm101Kinematics` 在引入之前对 MeArm 完全不存在。
 */
import { defineRobot, type RobotDefinition } from '../../definition/RobotDefinition';
import type { RobotModel } from '../../model/RobotModel';
import { loadRobotModel } from '../../model/loadRobotModel';
import { SO_ARM101_ROBOT_ID } from '../../model/robotIds';
import type { JointState, Pose, Vec3 } from '../../model/Pose';
import { forwardKinematics } from '../fk';
import { ikFailure, type IKResult } from '../IKResult';
import type {
  InverseOptions,
  KinematicsCapability,
  KinematicsEngine,
  RobotPoses,
} from '../KinematicsEngine';

/**
 * SO-ARM101 的能力声明。
 *
 * | 字段 | 取值 | 依据 |
 * |---|---|---|
 * | `positioningDof` | `5` | 机构事实：6 个铰链里 `shoulder_pan / shoulder_lift / elbow_flex / wrist_flex / wrist_roll` 这 5 个改变 TCP **位置**，`gripper` 只开合爪（不影响 TCP 点，TCP 帧 `gripperframe` 挂在 `gripper` body 上而非活动爪上） |
 * | `supportsOrientation` | `false` | **能力**事实：本工程**没有** SO-101 的姿态求解器，且 `inverse()` 接口只收位置目标（spec §12）。声明 `true` 等于承诺一个不存在的东西 |
 * | `solverKind` | `'none'` | 明确的"没有逆解器" —— 这也是 spec 用 `solverKind` 而非 `supportsIK` 布尔表达支持度的原因：未来加数值解时改这里是**一处** |
 *
 * ⚠️ `supportsOrientation: false` **不是**在说"SO-101 结构上不能定姿"。
 *    它说的是"**ArmPilot 此刻不提供**姿态目标接口"。机构本身有 2 个腕自由度，
 *    将来实现数值 IK 时，应同时把这里改成 `true` 并开始返回真实的
 *    `IKResult.orientationError` —— 两者必须**一起**改，不能只改一个。
 */
const SO_ARM101_CAPABILITY: KinematicsCapability = {
  positioningDof: 5,
  supportsOrientation: false,
  solverKind: 'none',
};

/** 只提示一次，避免拖动/循环调用时刷屏 */
let warnedNotImplemented = false;

/**
 * 一次性告警。
 *
 * 为什么要有：`ikFailure` 是**返回值**，调用方完全可能不检查 `success` 就继续用
 * （那时 `joints` 是空对象，行为会表现为"机械臂不动"）。控制台留一条明确的
 * 说明，能让"为什么不动"在 30 秒内被定位，而不是被当成"是不是链路断了"。
 */
function warnNotImplementedOnce(): void {
  if (warnedNotImplemented) return;
  warnedNotImplemented = true;
  console.warn(
    '[SoArm101Kinematics] 逆运动学未实现（solverKind="none"）。' +
      'inverse() 返回 ikFailure("NOT_IMPLEMENTED")，不会给出任何关节解。' +
      '这是刻意留白而非缺陷：SO-101 的 IK 目前没有可靠依据，' +
      '伪造一个"看起来能用"的求解器会静默产出错误的关节角。',
  );
}

/** 仅测试使用：重置一次性告警状态 */
export function resetSoArm101IkWarning(): void {
  warnedNotImplemented = false;
}

export class SoArm101Kinematics implements KinematicsEngine {
  readonly definition: RobotDefinition;
  readonly capability: KinematicsCapability = SO_ARM101_CAPABILITY;

  constructor(definition: RobotDefinition = defineRobot(loadRobotModel(SO_ARM101_ROBOT_ID))) {
    this.definition = definition;
  }

  /** 底层 `RobotModel`（= `definition.robotModel`，即通用 FK 直接消费的那一份） */
  private get model(): RobotModel {
    return this.definition.robotModel;
  }

  /**
   * 正运动学：关节角 → 末端位姿。
   *
   * 与 `MeArmKinematics.forward()` 同样返回 `forwardKinematics().endEffector`
   * 的**同一个对象引用**（不是拷贝）—— 于是"引擎层与直接调用逐位一致"是
   * 结构上成立的，而不是靠容差逼近。
   */
  forward(joints: JointState): Pose {
    return forwardKinematics(this.model, joints).endEffector;
  }

  /** 正运动学（完整）：全部关节坐标系 + 末端 */
  forwardAll(joints: JointState): RobotPoses {
    return forwardKinematics(this.model, joints);
  }

  /**
   * 逆运动学：**未实现**（见文件头）。
   *
   * 签名与 `KinematicsEngine` 完全一致（收位置目标、可选 options），
   * 但**忽略入参**并返回诚实的失败结果。
   *
   * `options` 被忽略是刻意的：解析它（比如校验 `prefer` 是否为已知支）
   * 会暗示"这些选项是有意义的"，而实际上没有任何求解路径会使用它们。
   * MeArm 那边对 `prefer` 做严格校验，是因为它**真的有**多支解可选。
   */
  inverse(_target: Vec3, _options?: InverseOptions): IKResult {
    warnNotImplementedOnce();
    return ikFailure(
      'NOT_IMPLEMENTED',
      'SO-ARM101 的逆运动学尚未实现（KinematicsCapability.solverKind = "none"）。' +
        '本工程拒绝为它生成不可靠的关节解：位置型解析解（MeArm 的平面 2R）不适用于本机构，' +
        '而数值解缺少收敛性与工作空间边界依据。正运动学（forward / forwardAll）可用。',
    );
  }
}

/**
 * 建立 SO-ARM101 的运动学引擎。
 *
 * 缺省用 `config/robots/so-arm101/robot.yaml`（`loadRobotModel` 的结果按 id 缓存，
 * 所以反复调用不会重复解析）。
 */
export function createSoArm101Kinematics(model?: RobotModel): SoArm101Kinematics {
  return new SoArm101Kinematics(defineRobot(model ?? loadRobotModel(SO_ARM101_ROBOT_ID)));
}
