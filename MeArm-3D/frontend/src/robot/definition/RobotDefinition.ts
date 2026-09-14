/**
 * `RobotDefinition` —— 机器人定义的**最小统一入口**（spec §9）。
 *
 * ## 它是什么（以及不是什么）
 *
 * 它**不是**一个新数据结构，而是给"已经存在的那一份 `RobotModel`"一个
 * **分节的、可被引用的名字**。所以：

 * ```text
 *   RobotDefinition.metadata   ← robot / id / name / model / units
 *   RobotDefinition.links      ← 同一个数组（非拷贝）
 *   RobotDefinition.joints     ← 同一个数组（非拷贝）
 *   RobotDefinition.actuators  ← 同一个数组（非拷贝）
 *   RobotDefinition.tcp        ← 同一个对象
 *   RobotDefinition.homePose   ← 同一个对象
 *   RobotDefinition.limits     ← 派生视图（joints 的投影，**不是新数据**）
 *   RobotDefinition.robotModel ← 底层模型，供既有算法直接消费
 * ```
 *
 * ## 为什么保留 `robotModel` 这条"后门"
 *
 * 因为本阶段的抽象目标是「**把现有 MeArm 实现放到清晰的接口后面**」（spec §8），
 * 而不是「重新实现一遍模型层」。既有算法（`fk.ts` / `ik.ts` / 渲染层
 * `buildRobotObject3D.ts`）已经**直接消费 `RobotModel` 且完全正确**。
 *
 * 若抽象层把 `links` / `joints` 复制成一套自己的结构，就必须写一份
 * "RobotModel → 新结构"的转换器 —— 而**任何转换都是漂移的入口**：
 * 少拷一个 `coupling` 字段，整个平行四连杆就在抽象层里静默失效，
 * 而且 FK 自查同样是错的，**不会自己暴露**。
 *
 * ⇒ 所以这里**零转换、零拷贝**：同一批对象引用，换一个分节的读法。
 *   `test_kinematicsEngine.test.ts` 里有一条断言专门盯这件事（引用相等）。
 *
 * ## 哪些"未来字段"刻意不设计
 *
 * spec §9 明确说：「不要一次设计大量未来字段。只抽取当前 MeArm 已经真实存在的信息。」
 * 因此这里**没有** `simulation` / `kinematics` 分节 ——
 * 仿真参数在 `config/physics.yaml`（那是另一份真值，且与运动学正交），
 * 运动学算法在 `KinematicsEngine`（那是**行为**，不是**定义**）。
 * 把它们塞进 RobotDefinition 只会制造两处真值。
 */
import type { Actuator } from '../model/Actuator';
import type { Joint, JointLimits } from '../model/Joint';
import { isMovableJoint } from '../model/Joint';
import type { Link } from '../model/Link';
import type { JointState } from '../model/Pose';
import type { RobotModel, TcpSpec } from '../model/RobotModel';

/** 机器人身份 —— 只读元数据，**不参与任何运动学计算** */
export interface RobotMetadata {
  /** 机器人 id（如 `mearm`） */
  id: string;
  /** 人类可读名（如 `mARM`） */
  name: string;
  /** 模型标识（如 `MeArm-V1`）。缺省表示"未标注版本" */
  model?: string;
  /** 模型语义版本（如 `1.0.0`） */
  modelVersion?: string;
  /** robot.yaml 的 schema 版本（顶层 `version`），**与 modelVersion 不是一回事** */
  schemaVersion: number;
  units: 'mm';
}

/** 单个可动关节的限位条目（**派生视图**，不新增真值） */
export interface JointLimitEntry {
  id: string;
  /** 关节角色（base / shoulder / elbow / gripper…） */
  role?: string;
  /** 该关节是否参与定位（由 IK 求解） */
  limits: JointLimits;
}

/**
 * 机器人定义 —— 冻结的、可被引用的 Robot Model。
 *
 * 生命周期：由 `defineRobot(model)` 建立一次，之后**只读**。
 * 有意要改模型时改 `config/robot.yaml` 并重新 `loadRobotModel()`，
 * **不要**在运行期改这里的字段。
 */
export interface RobotDefinition {
  readonly metadata: RobotMetadata;
  readonly links: readonly Link[];
  readonly joints: readonly Joint[];
  readonly actuators: readonly Actuator[];
  readonly tcp: TcpSpec;
  /** 开机 / HOME 关节位姿（degree） */
  readonly homePose: JointState;
  /** 可动关节的限位表（= `joints` 的投影，顺序与 `movableJoints()` 一致） */
  readonly limits: readonly JointLimitEntry[];

  /**
   * 底层 `RobotModel` —— **同一个对象**，不是拷贝。
   *
   * 既有算法层（`fk.ts` / `ik.ts` / `buildRobotObject3D.ts`）继续直接消费它，
   * 于是"抽象层"与"算法层"共享同一份数据，结构上不可能漂移。
   */
  readonly robotModel: RobotModel;
}

/**
 * 从 `RobotModel` 建立 `RobotDefinition`。
 *
 * ⚠️ 纯视图，**零转换零拷贝**：`links` / `joints` / `actuators` / `tcp` / `homePose`
 * 全部是原对象引用。这一点由测试断言（`toBe`），因为一旦变成拷贝，
 * 就会出现"抽象层看的是旧模型"这类只在热重载/换模型时才暴露的幽灵 bug。
 */
export function defineRobot(model: RobotModel): RobotDefinition {
  return {
    metadata: {
      id: model.id,
      name: model.name,
      ...(model.model !== undefined ? { model: model.model } : {}),
      ...(model.modelVersion !== undefined ? { modelVersion: model.modelVersion } : {}),
      schemaVersion: model.version,
      units: model.units,
    },
    links: model.links,
    joints: model.joints,
    actuators: model.actuators,
    tcp: model.tcp,
    homePose: model.homePose,
    limits: model.joints.filter(isMovableJoint).map((j) => ({
      id: j.id,
      ...(j.role !== undefined ? { role: j.role } : {}),
      limits: j.limits,
    })),
    robotModel: model,
  };
}

/**
 * 定义是否属于指定模型标识（如 `MeArm-V1`）。
 *
 * 用途：让「抽象前后的行为必须一致」这句验收话可以被**机器检查** ——
 * 黄金基线里记着自己属于哪个模型，回归测试核对当前加载的是不是同一个。
 * 未标注 `model` 时返回 `false`（**不猜测**）。
 */
export function isModel(def: RobotDefinition, modelId: string): boolean {
  return def.metadata.model === modelId;
}
