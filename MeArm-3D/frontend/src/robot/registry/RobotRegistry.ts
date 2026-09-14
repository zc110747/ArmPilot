/**
 * `RobotRegistry` —— 「机器人 id → （定义 + 运动学引擎）」的**唯一分派表**。
 *
 * ## 它存在的理由（spec §「禁止 if robot == …」）
 *
 * 引入第二台机器人之后，上层代码会自然长出这种写法：
 *
 * ```ts
 * // ✗ 禁止：特判散落在业务代码里
 * const model = robot === 'so-arm101' ? loadRobotModel('so-arm101') : loadRobotModel('mearm-v1');
 * const ik = robot === 'so-arm101' ? null : solveMeArmIk(...);
 * ```
 *
 * 这类特判的问题是**它们会繁殖**：每加一台机器人、每加一处特性
 * （IK / 工作空间 / 示教 / 遥测），就多一圈 `if`，而漏掉的那一处表现为
 * "新机器人在某个功能上静默走了旧机器人的分支"。
 *
 * ⇒ 所以分派被**收敛到本文件的一张表**里。业务代码只问
 *   `loadRobot(id)`，拿到同一形状的 `definition` + `kinematics`。
 *   「谁有 IK、谁没有」这件事由 `kinematics.capability` **声明**（数据），
 *   而不是由调用方去猜（逻辑）。
 *
 * ## 缓存与单例
 *
 * 同一个 id 只构造一次（`RobotDefinition` 与 `KinematicsEngine` 都无状态，
 * 反复构造只有开销没有收益）。不同 id **各自独立** ⇒ 两台机器人可以同时在内存里，
 * 这是 Phase 8 切换压力回归能逐位比对的前提。
 *
 * ## 与选择器的关系
 *
 * 机器人**有哪些**由 `config/robots.yaml` 声明（配置）；
 * 机器人**怎么算**由本文件的工厂表声明（代码）。两者必须一一对应，
 * 由 `assertRegistryCoverage()` 把"配置里加了一台但没写引擎"变成明确的错误。
 */
import { defineRobot, type RobotDefinition } from '../definition/RobotDefinition';
import { MeArmKinematics } from '../kinematics/mearm/MeArmKinematics';
import type { KinematicsEngine } from '../kinematics/KinematicsEngine';
import { SoArm101Kinematics } from '../kinematics/soarm101/SoArm101Kinematics';
import { loadRobotModel } from '../model/loadRobotModel';
import { MEARM_V1_ROBOT_ID, SO_ARM101_ROBOT_ID } from '../model/robotIds';
import {
  defaultRobotId,
  listRobotIds,
  loadRobotSelector,
  resetRobotSelectorCache,
} from '../model/robotConfigRegistry';

/**
 * 运动学引擎工厂表 —— 本文件**唯一**允许出现"按机器人分派"的地方。
 *
 * 每个工厂只做一件事：把一个 `RobotDefinition` 包成一个引擎。
 * 刻意让工厂**强制接收 definition**（而不是让引擎自己在工厂里 load）：
 * 这样"引擎算的是哪一个定义"永远由调用方决定，不会出现
 * "registry 拿了 A 的定义、引擎内部又 load 了 B"这种静默错配。
 */
const KINEMATICS_FACTORIES: Readonly<Record<string, (definition: RobotDefinition) => KinematicsEngine>> =
  {
    [MEARM_V1_ROBOT_ID]: (definition) => new MeArmKinematics(definition),
    [SO_ARM101_ROBOT_ID]: (definition) => new SoArm101Kinematics(definition),
  };

/** 一条机器人注册记录 —— 上层只依赖这两个字段 */
export interface RobotRegistryEntry {
  /** 机器人 id（= `config/robots.yaml` 的 key） */
  id: string;
  /** 人类可读名（来自选择器，仅供 UI / 日志） */
  name: string;
  /** 冻结的机器人定义（`links` / `joints` / `tcp` / `homePose` …） */
  definition: RobotDefinition;
  /** 运动学引擎（能力声明见 `kinematics.capability`） */
  kinematics: KinematicsEngine;
}

const cache = new Map<string, RobotRegistryEntry>();

/**
 * 加载一台机器人（按 id 缓存）。
 *
 * @param robotId 机器人 id。**省略时读选择器的 `default`**。
 *
 * 未知 id 抛错（不回退）；id 合法但缺引擎实现也抛错（见 `assertRegistryCoverage`）。
 */
export function loadRobot(robotId?: string): RobotRegistryEntry {
  const id = robotId ?? defaultRobotId();
  const cached = cache.get(id);
  if (cached) return cached;

  const selectorEntry = loadRobotSelector().robots.find((robot) => robot.id === id);
  if (!selectorEntry) {
    throw new Error(
      `[RobotRegistry] 未知机器人 id "${id}"；` +
        `config/robots.yaml 声明了: ${listRobotIds().join(' / ')}`,
    );
  }

  const factory = KINEMATICS_FACTORIES[id];
  if (!factory) {
    // "配置里有、代码里没有"是必然会发生的中间状态（先加配置再加引擎）。
    // 把它变成一条自解释的错误，而不是 `undefined is not a function`。
    throw new Error(
      `[RobotRegistry] 机器人 "${id}" 在 config/robots.yaml 中存在，但 KINEMATICS_FACTORIES ` +
        `里没有对应的引擎实现。请在 frontend/src/robot/registry/RobotRegistry.ts 补一行工厂。`,
    );
  }

  const definition = defineRobot(loadRobotModel(id));
  const entry: RobotRegistryEntry = {
    id,
    name: selectorEntry.name,
    definition,
    kinematics: factory(definition),
  };
  cache.set(id, entry);
  return entry;
}

/**
 * 选择器声明的全部机器人（**不做**模型解析，故不会因某台机器人的 yaml 损坏而失败）。
 *
 * 供 UI 列出可选项用。
 */
export function listRobots(): readonly { id: string; name: string }[] {
  return loadRobotSelector().robots.map((robot) => ({ id: robot.id, name: robot.name }));
}

/**
 * 自检：选择器声明的每一台机器人都必须有引擎实现。
 *
 * **不抛错**，只返回缺口列表 —— 让调用方（测试 / 启动期诊断）自己决定
 * 是"失败"还是"报告"。既有测试 `robotRegistry.test.ts` 断言它为空。
 */
export function assertRegistryCoverage(): { ok: boolean; missing: readonly string[] } {
  const missing = listRobotIds().filter((id) => KINEMATICS_FACTORIES[id] === undefined);
  return { ok: missing.length === 0, missing };
}

/** 全部已注册的机器人 id（= 工厂表的 key，供反向诊断） */
export function listRegisteredRobotIds(): readonly string[] {
  return Object.keys(KINEMATICS_FACTORIES);
}

/** 清空缓存（仅测试 / 热重载使用）。选择器缓存一并清掉，避免两个缓存不一致 */
export function resetRobotRegistryCache(): void {
  cache.clear();
  resetRobotSelectorCache();
}
