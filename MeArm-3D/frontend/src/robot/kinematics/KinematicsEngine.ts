/**
 * `KinematicsEngine` —— 运动学算法的**统一调用面**（spec §10）。
 *
 * ## 核心原则：**接口统一，算法不统一**
 *
 * ```
 *   KinematicsEngine (接口)
 *        ▲
 *        │ implements
 *   MeArmKinematics          ← 当前唯一实现：纯委托给既有 fk.ts / ik.ts
 * ```
 *
 * spec §10 用了一句很重的话来描述反面：
 *
 * > 不要创建假的 `GenericIK`。
 *
 * 这一条被认真执行了 —— 本项目**没有**、也不会有 `GenericIK`：
 * 一台平面 2R 机构与一台 6 DOF 串联臂的求解方式毫无共同之处，
 * 强行合并只会得到一个"对谁都不合适"的中间层，而 MeArm 的解析解
 * 会因为被塞进这个壳子而失去它的**明确适用范围声明**
 * （见 `ik.ts` 的 `assertPlanar2R` / `assertPlanarWrist` ——
 * 那两条断言的价值恰恰在于"不满足就直接拒绝，而不是给个错的解"）。
 *
 * ⇒ 所以本阶段的抽象只做**一件事**：把「上层怎么调用运动学」这件事定下来。
 *   将来接 6 DOF 臂时，新增一个 `KinematicsEngine` 实现即可，
 *   **MeArm-V1 一行不改**（spec §26 的收尾承诺）。
 *
 * ## DOF 不进通用接口（spec §12）
 *
 * 明确**禁止**把接口写成：
 *
 * ```ts
 * // ✗ 错误：把 6 DOF 的能力写死进通用 API
 * function solveIK(x, y, z, roll, pitch, yaw): IKResult
 * ```
 *
 * 因为 4 / 5 / 6 DOF 的**任务空间能力不同**：对 MeArm-V1 而言
 * `roll/pitch/yaw` 是三个永远填了也没用的参数，而一个"看起来能接受姿态"的签名
 * 会让调用方以为它被支持了。
 *
 * 正解是：接口只收**位置目标**（`Vec3`），姿态能力由
 * `KinematicsEngine.capability` **显式声明**，上层据此判断，
 * 而不是靠"填了参数没报错"来猜。
 */
import type { RobotDefinition } from '../definition/RobotDefinition';
import type { JointState, Pose, Vec3 } from '../model/Pose';
import type { IKResult } from './IKResult';

/**
 * 引擎能力声明 —— 让上层能**判断**能力，而不是猜。
 *
 * 它的存在本身就是对 spec §12 的执行：一个位置型机构必须能说出
 * "我不支持姿态目标"，否则上层只能通过"填了姿态参数、没报错"来推断支持 ——
 * 而 MeArm 恰恰会**静默忽略**姿态参数（它压根不解析这些量）。
 */
export interface KinematicsCapability {
  /** 定位自由度数（本机 = 3：偏航 + 平面 2R）。**不是**关节总数 */
  positioningDof: number;
  /**
   * 是否支持姿态（朝向）目标。
   *
   * `false` ⇒ `IKResult.orientationError` 恒为 `null`，
   * 调用方**不得**下任何姿态断言（包括 `=== 0` 这种看起来无害的）。
   */
  supportsOrientation: boolean;
  /** 求解方式：`analytic`（解析解）/ `numeric`（数值迭代）/ `none` */
  solverKind: 'analytic' | 'numeric' | 'none';
}

/**
 * 逆解选项。
 *
 * 刻意只有三个字段 —— 它们都是"多解机构的通用需求"，
 * 而不是 MeArm 特有参数（MeArm 的实现把它们映射到自己的
 * `IkOptions.prefer / near / seed`，映射逻辑在 `MeArmKinematics` 一处）。
 */
export interface InverseOptions {
  /**
   * 多解偏好。语义由实现定义：
   * 本机取 `'elbow-up' | 'elbow-down' | 'nearest'`（缺省 `'nearest'`）。
   */
  prefer?: string;
  /** 当前关节状态 —— 用于就近选支，也作为未参与解算关节的取值来源 */
  near?: JointState;
  /** 未参与解算的关节（如夹爪）取值来源 */
  seed?: JointState;
  /** 残差容差（mm）；超出即视为实现缺陷而非目标不可达 */
  toleranceMm?: number;
}

/** 全链位姿：每个关节坐标系 + 末端。结构上与 `fk.ts` 的 `RobotPose` 同构 */
export interface RobotPoses {
  /** key = Joint.id（含固定关节与被动关节，便于 3D 放置与逐段诊断） */
  joints: Record<string, Pose>;
  endEffector: Pose;
}

/**
 * 运动学引擎。
 *
 * 实现者必须保证：`forward` 与 `inverse` 的结果**与其底层算法的直接调用逐位一致** ——
 * 抽象层不得引入任何"顺手修正"（见 `MeArmKinematics` 与对应回归测试）。
 */
export interface KinematicsEngine {
  /** 本引擎所属的机器人定义（与 `RobotDefinition.robotModel` 同一份数据） */
  readonly definition: RobotDefinition;

  /** 能力声明（上层据此判断能否下姿态断言 / 能否做取向约束） */
  readonly capability: KinematicsCapability;

  /**
   * 正运动学：关节角 → 末端位姿。
   *
   * 输入 `JointState`（degree，绝对角语义），输出 `Pose`（位置 mm + 欧拉角 degree）。
   */
  forward(joints: JointState): Pose;

  /**
   * 逆运动学：末端**位置**目标（mm）→ 关节解。
   *
   * ⚠️ 只收位置。姿态能力见 `capability.supportsOrientation`（spec §12）。
   */
  inverse(target: Vec3, options?: InverseOptions): IKResult;

  /**
   * 正运动学（完整）：关节角 → **全部关节坐标系** + 末端位姿。
   *
   * 需要它的场景：Three.js 对齐判据要把**每一个关节 Group** 的
   * `matrixWorld` 与 FK 结果逐元素比对 —— 只比末端会漏掉
   * "中间某一段转错了、但末端恰好抵消"这类错误。
   */
  forwardAll(joints: JointState): RobotPoses;
}
