/**
 * `fromMeArmIkResult` —— **MeArm 原生 IK 结果 → 契约 `IKResult`** 的适配器。
 *
 * ## 为什么它在**包里**而不在 Core
 *
 * `IKResult`（形状）是 Core 的契约：上层 UI / 拖动 / 示教只依赖它。
 * 但"MeArm 的 `solveIk()` 返回了什么字段、怎么搬过去"是**这一台机器人**的实现细节 ——
 * 换个求解器（字段名必然不同），要改的是这里，而不是 Core。
 * spec §39 把这类"原生形状 → 契约"的适配器明确划给包。
 *
 * ## 两条不许越界的纪律
 *
 * ⚠️ 这是**纯字段搬运**，不含任何计算。任何"顺手修正一下"（比如把 `residual`
 * 抹到 0、把 `reason` 翻译成更友好的码）都会让「抽象前后逐位一致」的回归失去意义 ——
 * 那正是本阶段要守的判据。
 *
 * | MeArm `IkSuccess` | → | `IKResult` |
 * |---|---|---|
 * | `joints` | → | `joints` |
 * | `residual` | → | `positionError` |
 * | `branch` | → | `solutionType` |
 * | （无） | → | `orientationError = null`（MeArm 是位置型，**没有**姿态自由度） |
 *
 * | MeArm `IkFailure` | → | `IKResult` |
 * |---|---|---|
 * | `{}` | → | `joints: {}` |
 * | （无） | → | `positionError = null` |
 * | `reason` | → | `error` |
 * | `message` | → | `message` |
 */
import { ikFailure, ikSuccess, type IKResult } from '@robot/kinematics/IKResult';
import type { IkResult as MeArmIkResult } from './ik';

export function fromMeArmIkResult(result: MeArmIkResult): IKResult {
  if (result.success) {
    return ikSuccess(result.joints, result.residual, result.branch, null);
  }
  return ikFailure(result.reason, result.message);
}
