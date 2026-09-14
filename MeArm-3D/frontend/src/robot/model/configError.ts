/**
 * `RobotConfigError` —— **解析级**配置错误（字段类型 / 必填项 / 枚举取值不合法）。
 *
 * ## 为什么单独一个文件
 *
 * 它同时被两条链需要：
 *
 * ```text
 *   loadRobotModel.ts        ← 解析 robot.yaml（字段级）
 *   robotConfigRegistry.ts   ← 解析 robots.yaml（选择器级）
 * ```
 *
 * 若把类留在 `loadRobotModel.ts` 里，两者就形成**循环导入**
 * （loadRobotModel ← registry ← loadRobotModel）。ESM 能容忍它，
 * 但那是"靠使用时机的运气"，一旦将来有人在模块顶层用一次就会变成 TDZ 崩溃。
 * 抽成一个零依赖的叶子模块，循环从结构上消失。
 *
 * ## 与 `RobotModelError` 的分工（不要混用）
 *
 * | 错误类 | 触发时机 | 含义 |
 * |---|---|---|
 * | `RobotConfigError` | 解析字段时 | "这份配置**读不出来**"——写错了 |
 * | `RobotModelError` | 模型级校验后 | "配置读出来了，但**这台机器人自相矛盾**"——携带完整 issue 列表 |
 *
 * 两者都是 `Error` 子类，调用方按 `name` 或 `instanceof` 区分。
 */
export class RobotConfigError extends Error {
  /**
   * @param message 人类可读说明
   * @param source  出错的文件标签。缺省 `robot.yaml`（= 引入选择器之前的唯一来源，
   *                故既有调用点的报错文本**逐字不变**）。
   */
  constructor(message: string, source = 'robot.yaml') {
    super(`[${source}] ${message}`);
    this.name = 'RobotConfigError';
  }
}
