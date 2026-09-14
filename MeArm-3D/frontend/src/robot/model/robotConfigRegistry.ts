/**
 * `robotConfigRegistry` —— 「机器人 id → robot.yaml 原文」的**构建期登记表**。
 *
 * ## 为什么需要它（而不是把选择器当明文解析就完事）
 *
 * Vite 的 `?raw` 导入**必须是静态的**：`import x from '...' + path + '...?raw'` 这种
 * 运行期拼路径的写法在打包器里根本不存在（不是"不推荐"，是"不可能"）。
 * 但"该加载哪台机器人"又必须是**运行期可配**的（否则加一台机器人就要改代码）。
 *
 * 两者的交点就是本文件：
 *
 * ```text
 *   import.meta.glob(...)   ← 构建期：把**所有**机器人的 robot.yaml 原文登记进来
 *   config/robots.yaml      ← 运行期：说"现在用哪一个"（见 parseSelector）
 * ```
 *
 * ⇒ 于是「新增一台机器人」= 丢一个 `config/robots/<id>/robot.yaml` + 在
 *   `config/robots.yaml` 加一行。**一行代码都不用改**。
 *   （这与 `textureRegistry` / `meshRegistry` 是同一个范式，刻意保持一致。）
 *
 * ## ⚠️ 与 MeArm 阶段的"唯一真值源"铁律的关系
 *
 * 铁律说的是「**同一台**机器人的参数只有一份」。本文件**不复制任何参数** ——
 * 它登记的只是"原文在哪"，值仍然只在各自的 `robot.yaml` 里。
 * 选择器（`config/robots.yaml`）里也**只允许**出现 id / 名字 / 路径三种字段。
 *
 * ## 缺省行为的兼容性
 *
 * `defaultRobotId()` 读的是选择器的 `default`（当前 = `mearm-v1`）。
 * 所以 `loadRobotModel()` 无参调用与改前**行为完全相同** ——
 * 既有 22 处调用点（已全部显式化）与全部黄金基线不受影响。
 */
import { parse as parseYaml } from 'yaml';
import selectorYamlText from '@config/robots.yaml?raw';
import { RobotConfigError } from './configError';

/** 选择器文件的错误标签（`[robots.yaml] …`），与 robot.yaml 的报错可区分 */
const SEL = 'robots.yaml';

/** 选择器错误（统一标签） */
function selectorError(message: string): RobotConfigError {
  return new RobotConfigError(message, SEL);
}

/**
 * 所有机器人的 `robot.yaml` 原文，key = **glob 给出的仓库相对路径**。
 *
 * 刻意只列两条明确的模式（而**不是** `config/**` 全收）——
 * 因为 `config/` 下还有 `physics.yaml` / `baseline-*.json` 这些**不是** robot.yaml
 * 的文件，用宽模式会让它们进入解析路径，把一个配置笔误变成启动期崩溃。
 */
const CONFIG_TEXTS: Record<string, string> = import.meta.glob(
  ['../../../../config/robot.yaml', '../../../../config/robots/*/robot.yaml'],
  { eager: true, query: '?raw', import: 'default' },
) as Record<string, string>;

/** glob key 里用于裁出仓库相对路径的标记 */
const MARKER = 'config/';

/** `../..//../config/robots/so-arm101/robot.yaml` → `config/robots/so-arm101/robot.yaml` */
function repoRelativePath(globKey: string): string {
  const index = globKey.lastIndexOf(MARKER);
  return index >= 0 ? globKey.slice(index) : globKey;
}

/** 构建期登记表：仓库相对路径 → 原文 */
const TEXT_BY_PATH: ReadonlyMap<string, string> = new Map(
  Object.entries(CONFIG_TEXTS).map(([globKey, text]) => [repoRelativePath(globKey), text]),
);

/** 选择器里的一条机器人记录 —— 只有 id / name / 指针（**禁止放数值**） */
export interface RobotSelectorEntry {
  /** 机器人 id（选择器的 key，也是 `loadRobotModel(id)` 收的那个值） */
  id: string;
  /** 人类可读名（仅供 UI / 日志，不参与任何计算） */
  name: string;
  /** 该机器人 `robot.yaml` 的仓库相对路径 */
  config: string;
  /** `physics.yaml` 的仓库相对路径（缺省按 `config/robots/<id>/physics.yaml` 约定） */
  physics: string;
  /** MJCF 的仓库相对路径；`null` = 由 gen_model.py 从 robot.yaml 生成（MeArm 路线） */
  mjcf: string | null;
  /** MJCF 里代表 TCP 的 site 名 */
  tcpSite: string;
}

export interface RobotSelector {
  /** 选择器 schema 版本（顶层 `version`） */
  version: number;
  /** 缺省机器人 id */
  default: string;
  /** 按声明顺序排列的机器人列表 */
  robots: readonly RobotSelectorEntry[];
}

function asDict(value: unknown, path: string): Record<string, unknown> {
  if (typeof value !== 'object' || value === null || Array.isArray(value)) {
    throw selectorError(`${path} 必须是对象`);
  }
  return value as Record<string, unknown>;
}

/** 取一个「必须是非空字符串」的字段；错误信息带上完整字段路径 */
function requireString(value: unknown, path: string): string {
  if (typeof value !== 'string' || value.trim() === '') {
    throw selectorError(`${path} 必须是非空字符串`);
  }
  return value;
}

/**
 * 解析选择器。
 *
 * 校验原则：**每一个字段都必须与登记表对得上**，宁可在这里炸也不要等到
 * 运行期拿到 `undefined` 原文再炸 —— 后者会表现为"加载出一台空机器人"。
 */
export function parseSelector(text: string): RobotSelector {
  let raw: unknown;
  try {
    raw = parseYaml(text);
  } catch (error) {
    throw selectorError(`YAML 语法错误: ${(error as Error).message}`);
  }

  const root = asDict(raw, 'root');
  const version = typeof root['version'] === 'number' ? root['version'] : 1;

  const robotsRaw = asDict(root['robots'], 'robots');
  const robots: RobotSelectorEntry[] = [];

  for (const [id, entryRaw] of Object.entries(robotsRaw)) {
    const entry = asDict(entryRaw, `robots.${id}`);
    const name = entry['name'];
    const config = entry['config'];

    if (typeof name !== 'string' || name.trim() === '') {
      throw selectorError(`robots.${id}.name 必须是非空字符串`);
    }
    if (typeof config !== 'string' || config.trim() === '') {
      throw selectorError(`robots.${id}.config 必须是非空字符串`);
    }
    if (!TEXT_BY_PATH.has(config)) {
      throw selectorError(
        `robots.${id}.config = "${config}" 未在构建期登记。` +
          `登记模式为 config/robot.yaml 与 config/robots/*/robot.yaml，` +
          `已登记: ${[...TEXT_BY_PATH.keys()].join(', ') || '（空）'}`,
      );
    }
    // ── 指针字段（同样"只放路径 / 名字，不放数值"） ─────────────────────────
    //
    // `physics` 缺省按**约定**推导（`config/robots/<id>/physics.yaml`），
    // 但**不校验文件是否存在** —— 前端不需要 physics 原文（那三类真值分别归
    // 各自 robot.yaml / physics.yaml / MJCF，前端只消费 robot.yaml），
    // 在这里做存在性检查会把"后端/Python 才需要的文件"变成前端启动期硬依赖。
    // 真正需要它的两端（Go / Python）各自有存在性检查，报错更贴近使用现场。
    const physicsRaw = entry['physics'];
    const physics =
      physicsRaw === undefined
        ? `config/robots/${id}/physics.yaml`
        : requireString(physicsRaw, `robots.${id}.physics`);

    const simRaw = entry['simulation'];
    let mjcf: string | null = null;
    let tcpSite = 'tcp';
    if (simRaw !== undefined) {
      const sim = asDict(simRaw, `robots.${id}.simulation`);
      const mjcfRaw = sim['mjcf'];
      if (mjcfRaw !== undefined && mjcfRaw !== null) {
        mjcf = requireString(mjcfRaw, `robots.${id}.simulation.mjcf`);
      }
      const siteRaw = sim['tcpSite'];
      if (siteRaw !== undefined) {
        tcpSite = requireString(siteRaw, `robots.${id}.simulation.tcpSite`);
      }
    }

    robots.push({ id, name, config, physics, mjcf, tcpSite });
  }

  if (robots.length === 0) throw selectorError('robots 不能为空');

  const defaultId = root['default'];
  if (typeof defaultId !== 'string' || defaultId.trim() === '') {
    throw selectorError('default 必须是非空字符串');
  }
  if (!robots.some((robot) => robot.id === defaultId)) {
    throw selectorError(
      `default = "${defaultId}" 不在 robots 中，可选: ${robots
        .map((robot) => robot.id)
        .join(' / ')}`,
    );
  }

  return { version, default: defaultId, robots };
}

let cachedSelector: RobotSelector | undefined;

/** 选择器单例（解析一次，之后只读） */
export function loadRobotSelector(): RobotSelector {
  if (!cachedSelector) cachedSelector = parseSelector(selectorYamlText);
  return cachedSelector;
}

/** 缺省机器人 id（= 选择器的 `default`） */
export function defaultRobotId(): string {
  return loadRobotSelector().default;
}

/** 全部机器人 id（按选择器声明顺序） */
export function listRobotIds(): readonly string[] {
  return loadRobotSelector().robots.map((robot) => robot.id);
}

/** 按 id 取选择器条目；未知 id 返回 `undefined`（**不猜测、不回退**） */
export function findRobotEntry(robotId: string): RobotSelectorEntry | undefined {
  return loadRobotSelector().robots.find((robot) => robot.id === robotId);
}

/**
 * 按 id 取 `robot.yaml` 原文。
 *
 * 未知 id **抛错**（而不是回退到缺省）—— 这是刻意的：
 * 回退会让"配置写错了一个字的 id"表现为"静默加载了另一台机器人"，
 * 而那种错误在本项目里正是最贵的一类（模型不对，所有 FK/限位判据全部失真却都能跑）。
 */
export function robotYamlTextById(robotId: string): string {
  const entry = findRobotEntry(robotId);
  if (!entry) {
    throw selectorError(`未知机器人 id "${robotId}"；可选: ${listRobotIds().join(' / ')}`);
  }
  const text = TEXT_BY_PATH.get(entry.config);
  if (text === undefined) {
    // parseSelector 已经查过一次，这里只是"理论上不可达"的兜底 ——
    // 但它把「登记表与选择器脱钩」这类构建期事故变成一条明确的错误。
    throw selectorError(`robots.${robotId}.config = "${entry.config}" 的原文未登记`);
  }
  return text;
}

/** 清空缓存（仅测试 / 热重载使用） */
export function resetRobotSelectorCache(): void {
  cachedSelector = undefined;
}
