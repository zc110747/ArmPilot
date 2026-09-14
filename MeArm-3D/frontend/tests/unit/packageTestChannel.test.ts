/**
 * **前端包内测试通道的守卫**（Phase 2 步⑤；Python 侧对应
 * `core/tests/test_package_contract.py::test_pytest_testpaths_covers_package_dirs`）。
 *
 * ## 这条测试防的是"绿得没有意义"
 *
 * 「测试存在」和「测试被执行」是两件事。`vite.config.ts` 的 `test.include`
 * 漏掉 `robot-package` 的话，`vitest run` 会显示全绿 —— 而那些包内测试
 * **从未跑过**。这类"伪造通过"比缺测试更危险：它看起来是绿的，
 * 于是没人会去补。
 *
 * 同理，manifest 里的 `tests.local` 如果写了一个不存在的目录、
 * 或者目录里一个 `test_*.ts` 都没有，那这条声明就是一段没人看的注释。
 *
 * ⇒ 两道判据都在这里钉住：**通道开着** + **声明是真的**。
 */
import { existsSync, readFileSync, readdirSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import { parse as parseYaml } from 'yaml';
import { listRobots } from '@robot/index';

/** 仓库根：`frontend/tests/unit/` → 上三级 */
const REPO_ROOT = fileURLToPath(new URL('../../../', import.meta.url));

const VITE_CONFIG = import.meta.glob('../../vite.config.ts', {
  eager: true,
  query: '?raw',
  import: 'default',
}) as Record<string, string>;

interface ManifestLike {
  readonly tests: { readonly cases: string; readonly local?: string };
}

/** 包内测试目录里的 `*.test.ts`（只看一层：包内测试目录是扁平的） */
function tsTestsIn(dir: string): string[] {
  if (!existsSync(dir)) return [];
  return readdirSync(dir)
    .filter((name) => /\.test\.tsx?$/.test(name))
    .sort();
}

const PACKAGES = listRobots().map((robot) => {
  const manifestPath = path.join(REPO_ROOT, 'robot-package', robot.id, 'manifest.yaml');
  const manifest = parseYaml(readFileSync(manifestPath, 'utf-8')) as ManifestLike;
  const localRel = manifest.tests.local;
  const localDir = localRel ? path.join(REPO_ROOT, localRel) : null;
  return { id: robot.id, localRel, localDir, tsTests: localDir ? tsTestsIn(localDir) : [] };
});

describe('前端包内测试通道（robot-package/<id>/tests/）', () => {
  it('vitest 的 include 覆盖 robot-package（否则包内测试"存在但不执行"）', () => {
    const text = Object.values(VITE_CONFIG)[0];
    expect(text, 'vite.config.ts 没被 glob 到').toBeTypeOf('string');
    expect(
      text,
      'vite.config.ts 的 test.include 没有覆盖 robot-package ⇒ 包内测试**存在但不会被执行**，' +
        '`vitest run` 会显示全绿。请把 \'../robot-package/*/tests/**/*.test.ts\' 加进 include。',
    ).toMatch(/include:\s*\[[^\]]*robot-package[^\]]*\]/);
    expect(text).toContain("'tests/**/*.test.ts'");
  });

  it('manifest.tests.local 声明的目录必须存在（声明不许是空头支票）', () => {
    const declared = PACKAGES.filter((p) => p.localRel !== undefined);
    expect(declared.length, '至少要有一个包声明包内测试目录').toBeGreaterThan(0);
    for (const pkg of declared) {
      expect(existsSync(pkg.localDir!), `${pkg.id}.tests.local = ${pkg.localRel} 不存在`).toBe(
        true,
      );
      // 路径必须在**自己的包**内 —— "包内测试"随包走的前提
      expect(
        path.relative(path.join(REPO_ROOT, 'robot-package', pkg.id), pkg.localDir!),
        `${pkg.id}.tests.local 不在自己的包里`,
      ).not.toMatch(/^\.\./);
    }
  });

  it('至少有一个包内 TS 测试（否则上面两条断言是空转）', () => {
    const total = PACKAGES.reduce((n, p) => n + p.tsTests.length, 0);
    expect(
      total,
      '没有任何包内 TS 测试 ⇒ 这条通道还没被真正使用；' +
        '要么补一条（判据：这条断言换台机器人还成立吗），要么删掉通道别留空壳',
    ).toBeGreaterThan(0);
  });

  it('包内测试与 Core 测试**不重名**（重名会让收集器二选一，静默少跑一个）', () => {
    // 默认 import 模式下，同名测试文件会以同一个模块名登记，pytest 会直接
    // 报 "import file mismatch"，而 vitest 的表现是**其中一个被静默丢掉**。
    const coreTests = new Set(
      readdirSync(path.join(REPO_ROOT, 'frontend', 'tests'), { recursive: true })
        .map((e) => path.basename(String(e)))
        .filter((n) => /\.test\.tsx?$/.test(n)),
    );
    for (const pkg of PACKAGES) {
      for (const name of pkg.tsTests) {
        expect(coreTests.has(name), `${pkg.id} 的包内测试 ${name} 与 Core 里的同名`).toBe(false);
      }
    }
  });
});
