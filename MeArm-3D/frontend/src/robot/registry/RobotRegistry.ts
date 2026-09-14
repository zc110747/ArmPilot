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
 * ## ★ Phase 2 起：这张表**不再是手写的**
 *
 * 原先 `KINEMATICS_FACTORIES` 是一张写死的 `{ 'mearm-v1': … }` 字面量。
 * 它的失效方式是本项目最贵的那一类：**加了包、忘了加表 ⇒ 新机器人在运行时
 * 表现为"没有引擎"**，而没有任何东西提示"你漏了一行"。
 *
 * 现在改为 `import.meta.glob` **自动发现**：
 *
 * ```text
 *   robot-package/<id>/kinematics/engine.ts      ← 包自带引擎（约定位置 + 统一出口名）
 *              ↓  import.meta.glob（构建期登记，新增包无需改本文件）
 *   KINEMATICS_FACTORIES  = { <id>: createKinematicsEngine }
 * ```
 *
 * 两条刻意保留的**响亮失败**（"自动发现"绝不能变成"发现不到就算了"）：
 *   1. 两个包命中同一个 id ⇒ 抛错（包定位二义）；
 *   2. 命中了文件但**没有导出** `createKinematicsEngine` ⇒ 抛错（并说明约定名）。
 *
 * 而"选择器里有、包里没有"仍是**可报告的缺口**（`assertRegistryCoverage`）——
 * 那是"先加配置再加实现"的正常中间态，不是错误。
 *
 * ## 缓存与单例
 *
 * 同一个 id 只构造一次（`RobotDefinition` 与 `KinematicsEngine` 都无状态，
 * 反复构造只有开销没有收益）。不同 id **各自独立** ⇒ 两台机器人可以同时在内存里，
 * 这是 Phase 8 切换压力回归能逐位比对的前提。
 */
import { defineRobot, type RobotDefinition } from '../definition/RobotDefinition';
import type { KinematicsEngine } from '../kinematics/KinematicsEngine';
import { loadRobotModel } from '../model/loadRobotModel';
import {
  defaultRobotId,
  listRobotIds,
  loadRobotSelector,
  resetRobotSelectorCache,
} from '../model/robotConfigRegistry';

/**
 * 包引擎模块必须导出的**约定名**。
 *
 * 每个 `robot-package/<id>/kinematics/engine.ts` 都要有同名导出，
 * 签名为 `(definition: RobotDefinition) => KinematicsEngine`。
 * 强制接收 definition（而不是让它自己去 load）是为了杜绝
 * "Registry 拿了 A 的定义、引擎内部又 load 了 B"这种静默错配。
 */
export const ENGINE_FACTORY_EXPORT = 'createKinematicsEngine';

interface EngineModule {
  [ENGINE_FACTORY_EXPORT]?: (definition: RobotDefinition) => KinematicsEngine;
}

/**
 * 包内引擎的**静态登记**（构建期完成，不打网络、不做 fs 探测）。
 *
 * ⚠️ glob 的键形如 `'../../../robot-package/mearm-v1/kinematics/engine.ts'`
 * —— 前缀是**相对本文件**的，属于实现细节（改本文件所在目录就会变）。
 * 所以取 id 时用**标记定位**（`MARKER`），而不是 `slice(固定长度)`：
 * 后者在目录层级一变就静默取到错误的子串（甚至取到 `../..` 里的点）。
 */
const MARKER = 'robot-package/';
const ENGINE_MODULES = import.meta.glob('../../../../robot-package/*/kinematics/engine.ts', {
  eager: true,
}) as Record<string, EngineModule>;

/** 从 glob 键里取出包 id（`.../robot-package/<id>/kinematics/engine.ts` → `<id>`）。 */
function packageIdOfGlobKey(key: string): string {
  const at = key.indexOf(MARKER);
  if (at < 0) {
    throw new Error(
      `[RobotRegistry] glob 键 ${key} 里找不到标记 ${MARKER}；` +
        '取包 id 的规则是"标记之后的第一段"，标记没了就必须显式失败，不能猜。',
    );
  }
  const id = key.slice(at + MARKER.length).split('/')[0];
  if (!id) {
    throw new Error(`[RobotRegistry] 无法从 glob 键 ${key} 取出包 id`);
  }
  return id;
}

/**
 * 从 glob 键里取出**仓库相对路径**（`../../../../robot-package/x/y.ts` → `robot-package/x/y.ts`）。
 *
 * 与 `packageIdOfGlobKey` 同源：都靠 `MARKER` 定位。存在的意义是让
 * **manifest 里声明的 `kinematics.engine.entry`** 能和"glob 真的发现了哪个文件"
 * 对起来 —— 否则两者各自自洽，文件一搬就分家且不报错（这类"路径二义"是本项目
 * 最贵的一类缺陷，见 `declared_path` 的注释）。
 */
function repoRelativeOfGlobKey(key: string): string {
  const at = key.indexOf(MARKER);
  if (at < 0) {
    throw new Error(`[RobotRegistry] glob 键 ${key} 里找不到标记 ${MARKER}`);
  }
  return key.slice(at);
}

/**
 * 运动学引擎工厂表 —— 从包内引擎文件**自动构建**。
 *
 * 构建在**模块加载期**完成：包写坏了要在 import 那一刻就炸，
 * 而不是等到某次 `loadRobot()` 才表现成"这台机器人不能用"。
 */
const KINEMATICS_FACTORIES: Readonly<
  Record<string, (definition: RobotDefinition) => KinematicsEngine>
> = (() => {
  const out: Record<string, (definition: RobotDefinition) => KinematicsEngine> = {};
  for (const [key, mod] of Object.entries(ENGINE_MODULES)) {
    const id = packageIdOfGlobKey(key);
    if (id in out) {
      throw new Error(
        `[RobotRegistry] 包 id ${id} 被两个引擎文件命中（${key}）⇒ 包定位二义。` +
          '一个包只能有一个 kinematics/engine.ts。',
      );
    }
    const factory = mod[ENGINE_FACTORY_EXPORT];
    if (typeof factory !== 'function') {
      throw new Error(
        `[RobotRegistry] ${key} 没有导出 \`${ENGINE_FACTORY_EXPORT}\`。` +
          '包的引擎模块必须导出这个**约定名**' +
          '（签名：`(definition: RobotDefinition) => KinematicsEngine`）' +
          '—— 否则"自动发现"会退化成"静默忽略这个包"。',
      );
    }
    out[id] = factory;
  }
  return out;
})();

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
      `[RobotRegistry] 机器人 "${id}" 在 config/robots.yaml 中存在，但 ` +
        `robot-package/${id}/kinematics/engine.ts 没有被发现。请在包里补上该文件并导出` +
        ` \`${ENGINE_FACTORY_EXPORT}\`（签名：(definition: RobotDefinition) => KinematicsEngine）。`,
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

/** 全部已注册的机器人 id（= 自动发现到的包，供反向诊断） */
export function listRegisteredRobotIds(): readonly string[] {
  return Object.keys(KINEMATICS_FACTORIES);
}

/**
 * 自动发现到的**引擎文件路径**（仓库相对，形如 `robot-package/<id>/kinematics/engine.ts`）。
 *
 * 供契约测试把 manifest 的 `kinematics.engine.entry` 与"glob 真的发现了哪个文件"
 * 对上（`robotManifest.test.ts` 的 `engine.entry` 那条断言）。**按从 glob 键里
 * 剥出真实路径实现**，不是拿 id 拼字符串 —— 拼出来的字符串永远"等于"声明值，
 * 那样这条断言就成了同义反复。
 */
export function listRegisteredEngineFiles(): readonly string[] {
  return Object.keys(ENGINE_MODULES).map(repoRelativeOfGlobKey).sort();
}

/** 清空缓存（仅测试 / 热重载使用）。选择器缓存一并清掉，避免两个缓存不一致 */
export function resetRobotRegistryCache(): void {
  cache.clear();
  resetRobotSelectorCache();
}
