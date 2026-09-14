/**
 * `MeArmKinematics` —— `KinematicsEngine` 在 **MeArm-V1** 上的实现。
 *
 * ## 本文件最重要的一句话
 *
 * > **它里面一行运动学算法都没有。**
 *
 * 每一个方法都是**纯委托**，转发给既有的、已经通过全部验收的
 * `fk.ts` / `ik.ts`：
 *
 * ```text
 *   MeArmKinematics.forward()   ──▶  fk.forwardKinematics().endEffector
 *   MeArmKinematics.forwardAll()──▶  fk.forwardKinematics()
 *   MeArmKinematics.inverse()   ──▶  ik.solveIk()  ──▶  fromMeArmIkResult()
 * ```
 *
 * 这是**刻意**的，也是本阶段抽象能成立的前提（spec §19 / §26）：
 *
 * | 如果这么做 | 后果 |
 * |---|---|
 * | 在引擎里"重新实现一版更干净的 FK" | 两套 FK 必然漂移；且 Three.js 共用的是 `fk.ts`，渲染与判据会分家 |
 * | 在引擎里"顺手把 residual 换成 positionError 的语义" | 「抽象前后逐位一致」的回归失去意义 —— 那正是本阶段唯一的验收原则 |
 * | 在引擎里缓存 `ikGeometry` | `ik.ts` 已经用 `WeakMap` 按模型缓存了；再缓存一层只会制造失效时机问题 |
 *
 * ## 唯一真正"新增"的东西
 *
 * 只有**形状转换**：`ik.solveIk()` 的 `residual / branch / reason`
 * 被归一化成 `IKResult` 的 `positionError / solutionType / error`
 * （见 `IKResult.fromMeArmIkResult`），以及 `prefer` 字符串的**显式校验**
 * （见下方 `toIkOptions`）。
 *
 * 后者值得一提：`InverseOptions.prefer` 是 `string`，而 `ik.ts` 只认
 * `'elbow-up' | 'elbow-down' | 'nearest'`。若直接 `as` 强转，
 * 传错一个字符串（比如 `'elbow_up'`）会被 `ik.ts` 当作"未知支"而
 * **静默退化为 `elbow-up`** —— 上层以为指定了支，实际没有。
 * 所以这里显式校验并抛错：**把静默失效变成明确报错**。
 */
import { defineRobot, type RobotDefinition } from '../../definition/RobotDefinition';
import type { RobotModel } from '../../model/RobotModel';
import { loadRobotModel } from '../../model/loadRobotModel';
import { MEARM_V1_ROBOT_ID } from '../../model/robotIds';
import type { JointState, Pose, Vec3 } from '../../model/Pose';
import { forwardKinematics } from '../fk';
import { solveIk, type IkPreference } from '../ik';
import { fromMeArmIkResult, type IKResult } from '../IKResult';
import type {
  InverseOptions,
  KinematicsCapability,
  KinematicsEngine,
  RobotPoses,
} from '../KinematicsEngine';

/** `ik.ts` 认得的支解偏好 —— 与 `IkPreference` 保持同源（改一处必须改两处，故在此显式列出） */
const MEARM_KINEMATICS_PREFERENCES: readonly IkPreference[] = [
  'elbow-up',
  'elbow-down',
  'nearest',
];

/**
 * `InverseOptions` → `ik.ts` 的 `IkOptions`。
 *
 * ⚠️ 唯一做校验的地方：`prefer` 不在已知集合里就**抛错**，不静默退化。
 */
function toIkOptions(options: InverseOptions | undefined) {
  if (!options) return undefined;

  const out: {
    prefer?: IkPreference;
    near?: JointState;
    seed?: JointState;
    tolerance?: number;
  } = {};

  if (options.prefer !== undefined) {
    if (!MEARM_KINEMATICS_PREFERENCES.includes(options.prefer as IkPreference)) {
      throw new Error(
        `[MeArmKinematics] 未知的解支偏好 "${options.prefer}"；` +
          `可选值：${MEARM_KINEMATICS_PREFERENCES.join(' / ')}。` +
          '（不静默退化 —— 传错支而悄悄用另一支，会让拖动时翻肘且无从排查）',
      );
    }
    out.prefer = options.prefer as IkPreference;
  }
  if (options.near !== undefined) out.near = options.near;
  if (options.seed !== undefined) out.seed = options.seed;
  if (options.toleranceMm !== undefined) out.tolerance = options.toleranceMm;
  return out;
}

/** MeArm-V1 的能力声明 */
const MEARM_V1_CAPABILITY: KinematicsCapability = {
  // 1 个绕 Z 的偏航关节 + 矢状面内一组平面 2R ⇒ 3 个定位自由度
  positioningDof: 3,
  // 位置型机构：没有姿态自由度。**必须**说 false —— 见 IKResult 的头部说明
  supportsOrientation: false,
  solverKind: 'analytic',
};

export class MeArmKinematics implements KinematicsEngine {
  readonly definition: RobotDefinition;
  readonly capability: KinematicsCapability = MEARM_V1_CAPABILITY;

  constructor(definition: RobotDefinition = defineRobot(loadRobotModel())) {
    this.definition = definition;
  }

  /** 底层 `RobotModel`（= `definition.robotModel`，即既有算法直接消费的那一份） */
  private get model(): RobotModel {
    return this.definition.robotModel;
  }

  /**
   * 正运动学：关节角 → 末端位姿。
   *
   * 返回的是 `forwardKinematics()` 结果里 `endEffector` 的**同一个对象引用**
   * （不是拷贝）—— 于是"抽象层与直接调用逐位一致"是**结构上成立**的，
   * 而不是靠一个容差去逼近。测试里用 `toBe` 断言这一点。
   */
  forward(joints: JointState): Pose {
    return forwardKinematics(this.model, joints).endEffector;
  }

  /** 正运动学（完整）：全部关节坐标系 + 末端 */
  forwardAll(joints: JointState): RobotPoses {
    return forwardKinematics(this.model, joints);
  }

  /**
   * 逆运动学：末端**位置**目标 → 关节解（统一 `IKResult` 形状）。
   *
   * `ik.ts` 的原始返回被保留在它自己的导出里（它仍导出 `candidates` 等诊断信息）；
   * 需要那些信息时直接调 `solveIk()` —— 本层刻意不把它们搬进通用形状。
   */
  inverse(target: Vec3, options?: InverseOptions): IKResult {
    return fromMeArmIkResult(solveIk(this.model, target, toIkOptions(options)));
  }
}

/**
 * 建立 MeArm-V1 的运动学引擎。
 *
 * 缺省用随包内置的 `config/robot.yaml`（`loadRobotModel` 的结果按 id 缓存，
 * 所以反复调用不会重复解析）。
 *
 * ⚠️ id 显式写死为 `'mearm-v1'` 而**不是**读选择器的 `default`：
 * 本类**就是** MeArm-V1 的实现，它的缺省值不该随"当前默认机器人"变化。
 * （改选择器的 default 后，`createMeArmKinematics()` 仍应造出 MeArm 引擎。）
 */
export function createMeArmKinematics(model?: RobotModel): MeArmKinematics {
  return new MeArmKinematics(defineRobot(model ?? loadRobotModel(MEARM_V1_ROBOT_ID)));
}
