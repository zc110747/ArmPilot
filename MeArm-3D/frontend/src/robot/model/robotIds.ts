/**
 * 机器人 id 常量 —— **零依赖的叶子模块**。
 *
 * ## 为什么单独一个文件
 *
 * 这些 id 同时被三处需要：
 *
 * ```text
 *   kinematics/mearm/MeArmKinematics.ts      ← 引擎的缺省加载目标
 *   registry/RobotRegistry.ts                ← 工厂表的 key
 * ```
 *
 * 若把常量定义在 `RobotRegistry.ts` 里，实现文件就要反向 import registry，
 * 形成 `registry ↔ 实现` 循环；若定义在实现文件里，registry 要 import 实现的
 * **非引擎副作用**，语义上也很别扭。放在最底层的叶子模块里，两边都只向下依赖。
 *
 * ## 与 `config/robots.yaml` 的关系（重要）
 *
 * 这些常量是**代码侧**的类型化别名，`config/robots.yaml` 才是**配置侧**的
 * 权威声明。两者的取值必须一致 —— 由 `robotRegistry.test.ts` 断言
 * 「这两个常量都出现在选择器里」来钉住，避免"改了 yaml 忘了改常量"。
 *
 * ⚠️ 但**不要**反过来让 yaml 去读常量：选择器必须能在不碰代码的情况下增删机器人。
 */
export const MEARM_V1_ROBOT_ID = 'mearm-v1';
