/**
 * TS 侧的 **Robot Package 路径解析器**（与 Python 的 `robopkg.declared_path` 同一份契约）。
 *
 * ## 为什么需要它
 *
 * Phase 2 把用例数据从仓库根的 `tests/baseline/<id>/` 搬进了各包自己的
 * `robot-package/<id>/tests/cases/`。搬迁时最容易出错的地方不是"搬"，而是
 * **"散落在各处自己拼路径的读者"**：
 *
 * - 漏改的那一处**不一定报错** —— 它可能仍然解析到一个**存在的**文件
 *   （只是那已经属于另一台机器人，或是一份过期的副本）；
 * - 报错的那种还算好，至少它响了。
 *
 * ⇒ 所以路径**只声明在 manifest**，解析**只发生在这一个函数**里：
 *
 * ```text
 * robot-package/<id>/manifest.yaml
 *        └─ tests.cases / model.config / simulation.mjcf …   ← 唯一声明处
 *                  ↓  （本文件 + Python robopkg.declared_path）
 *            绝对路径                                          ← 唯一解析处
 * ```
 *
 * ## 为什么用 `import.meta.glob` 而不是 `fs.readdirSync`
 *
 * manifest 是**构建期**已知的静态文件集合。用 glob 让 Vite 在打包/测试时把它
 * 登记进来，于是"包里多了一个 manifest"这件事**不需要改任何代码**就会生效 ——
 * 这正是 spec §41 想要的形态（Agent 生成一个包 = 只丢一个目录）。
 *
 * ⚠️ 与 Python 侧的差别：Python 用 `os.listdir` 动态发现，TS 用 glob 静态登记。
 * 两者的**契约必须一致**（键集合 / 取值规则），由
 * `frontend/tests/unit/robotManifest.test.ts` 与 `core/tests/test_package_contract.py`
 * 两侧同时盯着。
 */
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { parse as parseYaml } from 'yaml';

const HERE = path.dirname(fileURLToPath(import.meta.url));
/** `frontend/tests/helpers` → 仓库根（ArmPilot/MeArm-3D） */
export const REPO_ROOT = path.resolve(HERE, '..', '..', '..');

interface ManifestShape {
  readonly id: string;
  readonly name: string;
  readonly model: { readonly config: string; readonly physics?: string };
  readonly simulation: { readonly mjcf?: string; readonly tcp_site: string };
  readonly kinematics: {
    readonly engine?: { readonly entry?: string };
    readonly ik?: { readonly type?: string; readonly entry?: string };
  };
  readonly tests: { readonly cases: string; readonly frozen?: string };
}

const MANIFEST_TEXTS = import.meta.glob('../../../robot-package/*/manifest.yaml', {
  eager: true,
  query: '?raw',
  import: 'default',
}) as Record<string, string>;

/** id → manifest（模块加载时一次性解析；坏 YAML 直接抛，不静默跳过） */
const MANIFESTS: Record<string, ManifestShape> = (() => {
  const out: Record<string, ManifestShape> = {};
  for (const [file, text] of Object.entries(MANIFEST_TEXTS)) {
    const m = parseYaml(text) as ManifestShape;
    if (m.id in out) {
      throw new Error(`manifest id 重复：${m.id}（${file}）⇒ 包定位二义`);
    }
    out[m.id] = m;
  }
  return out;
})();

/** manifest 里可被解析的路径键（与 Python `robopkg.declared_paths_keys()` 对齐） */
export type DeclaredPathKey =
  | 'model.config'
  | 'model.physics'
  | 'simulation.mjcf'
  | 'kinematics.engine.entry'
  | 'kinematics.ik.entry'
  | 'tests.cases'
  | 'tests.frozen';

export function packageIds(): string[] {
  return Object.keys(MANIFESTS).sort();
}

export function manifestOf(robotId: string): ManifestShape {
  const m = MANIFESTS[robotId];
  if (m === undefined) {
    throw new Error(
      `没有包 ${robotId}（robot-package/<id>/manifest.yaml 未登记）；已登记: ${packageIds().join(' / ') || '(空)'}`,
    );
  }
  return m;
}

/**
 * 取包在 manifest 里**声明**的路径的绝对路径。
 *
 * 未知键 / 未声明 / 值不是字符串 ⇒ **报错**（与两端"不许兜底"同一条纪律）。
 */
export function declaredPath(robotId: string, key: DeclaredPathKey): string {
  const m = manifestOf(robotId);
  let rel: string | undefined;
  switch (key) {
    case 'model.config':
      rel = m.model.config;
      break;
    case 'model.physics':
      rel = m.model.physics;
      break;
    case 'simulation.mjcf':
      rel = m.simulation.mjcf;
      break;
    case 'kinematics.engine.entry':
      rel = m.kinematics.engine?.entry;
      break;
    case 'kinematics.ik.entry':
      rel = m.kinematics.ik?.entry;
      break;
    case 'tests.cases':
      rel = m.tests.cases;
      break;
    case 'tests.frozen':
      rel = m.tests.frozen;
      break;
    default: {
      const never: never = key;
      throw new Error(`未登记的声明路径键 ${String(never)}`);
    }
  }
  if (typeof rel !== 'string' || rel.trim() === '') {
    throw new Error(`${robotId}.${key} 未在 manifest 里声明`);
  }
  return path.resolve(REPO_ROOT, rel);
}
