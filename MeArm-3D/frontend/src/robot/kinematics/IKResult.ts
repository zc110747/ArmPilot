/**
 * `IKResult` —— 逆运动学的**统一结果形状**（spec §11）。
 *
 * ## 它解决什么问题
 *
 * 上层代码（UI / 拖动 / 示教 / 未来的编排层）不应该依赖**某一台机器人专属**的
 * IK 返回格式。现在 MeArm 的 `solveIk()` 返回的是判别联合
 * `IkSuccess | IkFailure`（字段名是 `residual` / `branch` / `reason`），
 * 那是 MeArm 的实现细节；换个求解器，字段名必然不同，上层就得跟着改。
 *
 * `IKResult` 把这些**归一化**成一个与算法无关的形状。
 * 归一化是**单向适配**（`fromMeArmIkResult`），**不是**替换：
 * 现有 `solveIk` 与它的 2000 组闭环验收、e2e 探针**一个字都不动**。
 *
 * ## 两条诚实性约束（本项目最看重的东西）
 *
 * ### ① `orientationError` 对 MeArm-V1 恒为 `null`
 *
 * 本机是 **4 DOF 位置型**机构（1 偏航 + 1 组平面 2R + 1 被动腕），
 * **没有姿态自由度**。所以：
 *
 * - 允许 `orientationError = null`，**而不是**填 `0`
 * - 更**不允许**为了"看起来通用"而把接口写成 `solveIK(x, y, z, roll, pitch, yaw)`
 *   （spec §12 明令禁止）—— 那会凭空承诺一个本机做不到的能力
 *
 * 填 `0` 是最坏的选择：它同时骗过调用方**和**测试 —— 断言 `orientationError < ε`
 * 会永远为真，而那个 `ε` 从来没有被验证过。
 *
 * ### ② 失败时 `positionError = null`，不是 `0`
 *
 * 没有解就没有"误差"这个概念。填 `0` 会被读成"完美命中"，
 * 填 `Infinity` 会被读成"差无穷远" —— 两者都是编造。
 * 失败的信息在 `error`（机器可读码）+ `message`（人可读说明）里。
 */
import type { JointState } from '../model/Pose';

/**
 * 统一的逆解结果。
 *
 * 与 spec §11 的示例结构对照（差异都已在注释里说明理由）：
 *
 * | spec 示例 | 本实现 | 理由 |
 * |---|---|---|
 * | `joints: number[]` | `JointState`（`Record<jointId, deg>`） | 本项目全程用"关节 id → 角"的映射；裸数组需要额外的位次约定，而位次约定在本项目已经踩过坑（JR 四元组 vs qpos 五元组） |
 * | `positionError: number` | `number \| null` | 失败时无解 ⇒ 无误差；填 `0` 是编造 |
 * | `orientationError?: number` | `number \| null`（本机恒 `null`） | 显式表达"本机没有姿态自由度"，而不是留一个可能被误判为 0 的缺省 |
 * | `solutionType: string` | 同（本机取 `'elbow-up' \| 'elbow-down' \| 'none'`） | 保留原语义 |
 * | `error?: string` | 同（本机取 `'OUT_OF_WORKSPACE' \| 'JOINT_LIMIT' \| 'MODEL_ERROR'`） | 与 `docs/serial-v1.md` 的错误码同源 |
 */
export interface IKResult {
  /** 解算是否成功 */
  success: boolean;
  /**
   * 关节解（含未参与解算的关节，如夹爪）—— 可直接喂给 `KinematicsEngine.forward()`。
   *
   * **失败时为空对象 `{}`**：刻意不塞"最接近的那一支"，
   * 否则 `joints` 的语义变成"可能是解、可能是残次品"，调用方无法分辨。
   * 需要诊断信息请直接用 `solveIk()`（它仍导出 `candidates`）。
   */
  joints: JointState;
  /**
   * 末端**位置**误差（mm）。
   *
   * - 成功：`|FK(joints) − target|`
   * - 失败：`null`（没有解就没有误差可说）
   */
  positionError: number | null;
  /**
   * 末端**姿态**误差（radian 或 degree，由实现定义）。
   *
   * MeArm-V1 是位置型机构 ⇒ **恒为 `null`**。
   * 未来的 6 DOF 求解器在这里填真实值；上层若拿到 `null` 就必须
   * **明确不下姿态断言**，而不是把它当作 0。
   */
  orientationError: number | null;
  /** 解是否满足全部约束（限位 / 工作空间 / 自碰撞…） */
  constraintsSatisfied: boolean;
  /**
   * 解的类型 / 分支。本机取 `'elbow-up' | 'elbow-down'`；无解时 `'none'`。
   *
   * 用意与 `IkBranch` 一致：多解机构需要让上层知道"拿到的是哪一支"，
   * 否则拖动时翻支会表现为机械臂突然抽搐。
   */
  solutionType: string;
  /** 失败码（机器可读）。成功时不出现 */
  error?: string;
  /** 人类可读说明。成功时不出现 */
  message?: string;
}

/** 无解时的统一结果（`solutionType = 'none'`，两个误差均为 `null`） */
export function ikFailure(error: string, message?: string): IKResult {
  return {
    success: false,
    joints: {},
    positionError: null,
    orientationError: null,
    constraintsSatisfied: false,
    solutionType: 'none',
    error,
    ...(message !== undefined ? { message } : {}),
  };
}

/** 成功解的构造（供各求解器实现使用） */
export function ikSuccess(
  joints: JointState,
  positionError: number,
  solutionType: string,
  orientationError: number | null = null,
): IKResult {
  return {
    success: true,
    joints,
    positionError,
    orientationError,
    constraintsSatisfied: true,
    solutionType,
  };
}

/**
 * MeArm 的 `solveIk()` 结果 → 统一 `IKResult`。
 *
 * ★ **Phase 2 起已搬进包**：`robot-package/mearm-v1/kinematics/fromMeArmIkResult.ts`。
 *
 * 为什么搬：字段映射表（`residual` → `positionError` 等）是**这一台机器人**的实现细节，
 * 换个求解器字段名必然不同。留在 Core 会让"Core 里出现 MeArm 的字段名"这件事
 * 变成架构上的既成事实。
 *
 * 契约（Core 侧只有形状与构造子）：
 *   - `IKResult` / `ikFailure` / `ikSuccess` / `positionErrorOf` ← 留在这里
 *   - `fromMeArmIkResult` ← 在包内
 */

/** 供上层断言用的工具：位置误差（`null` ⇒ `Infinity`，便于直接比大小） */
export function positionErrorOf(result: IKResult): number {
  return result.positionError === null ? Number.POSITIVE_INFINITY : result.positionError;
}
