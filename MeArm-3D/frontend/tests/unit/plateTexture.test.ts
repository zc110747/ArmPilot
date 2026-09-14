/**
 * 照片纹理接入的守卫测试（外观层）。
 *
 * 为什么要专门守这一层：贴图失败**不会报错**。`buildRobotObject3D` 在
 * 「key 未登记」时静默回退纯色（这是刻意的：缺一块装饰不该白屏），
 * 于是「文件名打错一个字」「走 import.meta.glob 没扫到」这类失误
 * 会安静地表现为「板没变色」——**没有断言就只能靠肉眼发现**。
 *
 * 三道判据：
 *   ① yaml 里声明的 key 必须真的被登记（否则等于配了一张不存在的图）
 *   ② 纹理长宽比必须与该板**大面**的长宽比一致（抓"把 forearm 的图配到 upper_arm"）
 *   ③ flip 缺省必须是 false（避免"默认翻转"这种隐式行为）
 */
import { existsSync, readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import type { LinkGeometry, PlateGeometry } from '@robot/model/Link';
import { plateTexture } from '@robot/model/Link';
import { loadRobotModel } from '@robot/model/loadRobotModel';
import { listTextureKeys, resolveTextureUrl } from '@robot/model/textureRegistry';

const REPO_ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../..');

/** 读出 PNG 的 IHDR 宽高（无需依赖图像库） */
function pngSize(file: string): { w: number; h: number } {
  const buf = readFileSync(file);
  if (buf.readUInt32BE(0) !== 0x89504e47) throw new Error(`${file} 不是 PNG`);
  return { w: buf.readUInt32BE(16), h: buf.readUInt32BE(20) };
}

/** 收集模型里所有配了纹理的 plate（含 details 附加件） */
function texturedPlates(): Array<{ linkId: string; geometry: PlateGeometry }> {
  const model = loadRobotModel('mearm-v1');
  const out: Array<{ linkId: string; geometry: PlateGeometry }> = [];
  const consider = (linkId: string, geometry: LinkGeometry): void => {
    if (geometry.type === 'plate' && geometry.texture) out.push({ linkId, geometry });
  };
  for (const link of model.links) {
    consider(link.id, link.geometry);
    for (const detail of link.details ?? []) consider(link.id, detail);
  }
  return out;
}

describe('照片纹理注册表', () => {
  it('至少登记了 assets/textures 下的两张臂板纹理', () => {
    const keys = listTextureKeys();
    expect(keys).toContain('mearm/tiles/upper_arm_link.png');
    expect(keys).toContain('mearm/tiles/forearm_link.png');
  });

  it('每个 key 都能解析为可用 URL', () => {
    for (const key of listTextureKeys()) {
      expect(resolveTextureUrl(key), key).toBeTruthy();
    }
  });

  it('未登记的 key 返回 undefined（调用方据此回退纯色）', () => {
    expect(resolveTextureUrl('mearm/tiles/__not_exist__.png')).toBeUndefined();
  });
});

describe('robot.yaml 的 texture 声明', () => {
  it('模型里确实有两块板配了照片纹理', () => {
    expect(texturedPlates().length).toBeGreaterThanOrEqual(2);
  });

  it('★ 声明的每个 key 都必须已登记 —— 否则贴图会静默失效', () => {
    for (const { linkId, geometry } of texturedPlates()) {
      const key = geometry.texture!;
      expect(resolveTextureUrl(key), `${linkId}: ${key} 未被 textureRegistry 登记`).toBeTruthy();
    }
  });

  it('★ 纹理长宽比必须与该板大面一致 —— 抓「把图配错板」', () => {
    for (const { linkId, geometry } of texturedPlates()) {
      const key = geometry.texture!;
      const file = path.join(REPO_ROOT, 'assets/textures', key);
      expect(existsSync(file), `${file} 不存在`).toBe(true);

      // 大面 = size 里排除最小那一维后的两维，按 [短, 长] 取比值
      const sorted = [...geometry.size].sort((a, b) => a - b);
      const faceRatio = sorted[2]! / sorted[1]!;

      const { w, h } = pngSize(file);
      const tileRatio = Math.max(w, h) / Math.min(w, h);

      expect(
        Math.abs(tileRatio - faceRatio) / faceRatio,
        `${linkId}: 纹理长宽比 ${tileRatio.toFixed(3)} vs 板大面 ${faceRatio.toFixed(3)}`,
      ).toBeLessThan(0.05);
    }
  });

  it('flipU / flipV 缺省为 false（不隐式翻转）', () => {
    for (const { geometry } of texturedPlates()) {
      const spec = plateTexture(geometry)!;
      expect(spec.flipU).toBe(false);
      expect(spec.flipV).toBe(false);
    }
  });
});
