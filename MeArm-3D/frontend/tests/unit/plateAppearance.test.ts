/**
 * 外观（渲染）参数的守卫测试（ADR D64）。
 *
 * 这一层同样**不会报错**：`applyAppearance()` 是"建好对象树后统一刷卡"，
 * 刷错对象、刷漏对象、或者刷卡范围外溢，在画面上都只表现为"整体亮了/没变"，
 * 没有任何异常可抓。所以这里钉三条契约：
 *
 *   ① `robot.yaml` 的 appearance 能解析，且**缺省逐值等于引入本特性前的行为**
 *      （environmentIntensity=0、exposureEv=0）—— 保证"没配 = 不变"。
 *   ② `exposureEv` 越界必须抛错（它是一个填错就会毁画面的量，不能静默接受）。
 *   ③ ★ `applyAppearance()` **只动照片纹理材质**，其余材质的每个量都必须原封不动。
 *      这条是 D64 的核心：环境反射一旦外溢到整机就是不可接受的副作用
 *      （实测底座蓝板被点亮 ×3.75），而它在画面上看着只像"光照调了一下"。
 */
import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import {
  applyAppearance,
} from '../../src/components/RobotScene/buildRobotObject3D';
import { loadRobotModel, parseRobotModel } from '@robot/model/loadRobotModel';
import { DEFAULT_APPEARANCE, EXPOSURE_EV_RANGE } from '@robot/model/RobotModel';

/**
 * 照片纹理材质的 name。
 *
 * 这里刻意写**字面量**而不是从源码 import 常量：万一常量被改名，
 * 下面的断言会因为"没刷到"而失败（响亮地坏），而不是悄悄退化成空测试。
 */
const TEXTURED_MATERIAL_NAME = 'plateTexture';

/** 造一棵最小对象树：一个照片纹理材质 + 一个普通材质 + 一个 basic 材质（模拟轴/标记） */
function makeTree() {
  const textured = new THREE.MeshStandardMaterial();
  textured.name = TEXTURED_MATERIAL_NAME;
  const texturedBefore = {
    roughness: textured.roughness,
    metalness: textured.metalness,
    color: textured.color.getHex(),
    envMap: textured.envMap,
    envMapIntensity: textured.envMapIntensity,
  };

  const plain = new THREE.MeshStandardMaterial({ color: 0x4d8fe8, roughness: 0.62, metalness: 0.2 });
  const plainBefore = {
    roughness: plain.roughness,
    metalness: plain.metalness,
    color: plain.color.getHex(),
    envMap: plain.envMap,
    envMapIntensity: plain.envMapIntensity,
  };

  const basic = new THREE.MeshBasicMaterial({ color: 0xff7a45 });

  const root = new THREE.Group();
  root.add(new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), textured));
  root.add(new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), plain));
  root.add(new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), basic));

  return { root, textured, texturedBefore, plain, plainBefore };
}

/**
 * 最小但**合法**的模型（links a/b + 可动关节 j + homePose）。
 * 校验器会拦掉不完整模型（childLink 不存在 / homePose 缺关节），所以夹具必须补齐。
 */
function minimalModelYaml(appearance?: unknown): Record<string, unknown> {
  return {
    version: 1,
    robot: { id: 'x', units: 'mm', homePose: { j: 0 } },
    links: [
      { id: 'a', length: 1, geometry: { type: 'none' } },
      { id: 'b', parent: 'a', length: 1, geometry: { type: 'none' } },
    ],
    joints: [
      {
        id: 'j',
        parentLink: 'a',
        childLink: 'b',
        axis: [0, 0, 1],
        origin: { position: [0, 0, 0], rotation: [0, 0, 0] },
        limit: { min: -1, max: 1 },
      },
    ],
    actuators: [],
    ...(appearance === undefined ? {} : { appearance }),
  };
}

describe('robot.yaml 的 appearance 声明', () => {
  it('内置配置能解析出 appearance', () => {
    const model = loadRobotModel();
    expect(model.appearance).toBeDefined();
    expect(model.appearance.texturedPlate).toBeDefined();
  });

  it('★ 声明值必须落在合法区间 —— 抓「多打一个 0」这类误填', () => {
    const plate = loadRobotModel().appearance.texturedPlate;
    expect(plate.exposureEv).toBeGreaterThanOrEqual(EXPOSURE_EV_RANGE.min);
    expect(plate.exposureEv).toBeLessThanOrEqual(EXPOSURE_EV_RANGE.max);
    // 强度是乘在 IBL 上的系数，>1 会明显过曝；上限留 2 足够实验
    expect(plate.environmentIntensity).toBeGreaterThanOrEqual(0);
    expect(plate.environmentIntensity).toBeLessThanOrEqual(2);
    expect(plate.roughness).toBeGreaterThan(0);
    expect(plate.roughness).toBeLessThanOrEqual(1);
    expect(plate.metalness).toBe(0);
  });

  it('appearance 整段缺省 = 引入本特性前的行为', () => {
    const model = parseRobotModel(minimalModelYaml());
    expect(model.appearance.texturedPlate).toEqual(DEFAULT_APPEARANCE.texturedPlate);
    expect(model.appearance.texturedPlate.environmentIntensity).toBe(0);
    expect(model.appearance.texturedPlate.exposureEv).toBe(0);
  });

  it('exposureEv 越界必须抛错（不静默接受）', () => {
    const build = (exposureEv: number) => () =>
      parseRobotModel(minimalModelYaml({ texturedPlate: { exposureEv } }));
    expect(build(EXPOSURE_EV_RANGE.max + 1)).toThrow(/exposureEv/);
    expect(build(EXPOSURE_EV_RANGE.min - 1)).toThrow(/exposureEv/);
    expect(build(EXPOSURE_EV_RANGE.max)).not.toThrow();
  });
});

describe('applyAppearance 的作用域', () => {
  it('★ 只刷照片纹理材质：普通材质的每个量都必须原封不动', () => {
    const { root, plain, plainBefore } = makeTree();
    const envMap = new THREE.Texture();

    applyAppearance(root, { texturedPlate: { environmentIntensity: 0.25, exposureEv: 0, roughness: 0.6, metalness: 0 } }, envMap);

    // 这是 D64 的核心断言：环境反射不得外溢
    expect(plain.roughness).toBe(plainBefore.roughness);
    expect(plain.metalness).toBe(plainBefore.metalness);
    expect(plain.color.getHex()).toBe(plainBefore.color);
    expect(plain.envMap).toBe(plainBefore.envMap);
    expect(plain.envMap).toBeNull();
    expect(plain.envMapIntensity).toBe(plainBefore.envMapIntensity);
  });

  it('照片纹理材质拿到 roughness / metalness / envMap / envMapIntensity', () => {
    const { root, textured } = makeTree();
    const envMap = new THREE.Texture();

    applyAppearance(root, { texturedPlate: { environmentIntensity: 0.25, exposureEv: 0, roughness: 0.6, metalness: 0 } }, envMap);

    expect(textured.roughness).toBe(0.6);
    expect(textured.metalness).toBe(0);
    expect(textured.envMap).toBe(envMap);
    expect(textured.envMapIntensity).toBe(0.25);
  });

  it('exposureEv 实现为线性 color 增益 2^EV，且不 clamp', () => {
    for (const ev of [0, 2, 4]) {
      const { root, textured } = makeTree();
      applyAppearance(
        root,
        { texturedPlate: { environmentIntensity: 0, exposureEv: ev, roughness: 0.6, metalness: 0 } },
        null,
      );
      expect(textured.color.r).toBeCloseTo(Math.pow(2, ev), 6);
      expect(textured.color.g).toBeCloseTo(Math.pow(2, ev), 6);
      expect(textured.color.b).toBeCloseTo(Math.pow(2, ev), 6);
    }
  });

  it('envMap = null 时不挂环境反射（关闭状态不留残余）', () => {
    const { root, textured } = makeTree();
    applyAppearance(
      root,
      { texturedPlate: { environmentIntensity: 0, exposureEv: 0, roughness: 0.85, metalness: 0 } },
      null,
    );
    expect(textured.envMap).toBeNull();
    expect(textured.envMapIntensity).toBe(0);
  });

  it('树里混有 MeshBasicMaterial（关节轴/TCP 标记）不抛异常', () => {
    const { root } = makeTree();
    expect(() =>
      applyAppearance(
        root,
        { texturedPlate: { environmentIntensity: 0.25, exposureEv: 0, roughness: 0.6, metalness: 0 } },
        new THREE.Texture(),
      ),
    ).not.toThrow();
  });
});
