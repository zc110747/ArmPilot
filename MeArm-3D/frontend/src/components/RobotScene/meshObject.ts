/**
 * 外部网格件（STL）→ three.js 对象。
 *
 * ## 为什么单独一个文件而不是塞进 buildRobotObject3D
 *
 * 因为它是本项目**第一处异步几何**：其余件（plate / box / cylinder / servo / jaw）
 * 都是"参数 → 顶点"同步算出来的，而网格要**先取 URL、再下载、再解析**。
 * 这条异步路径附带三条必须成对出现的纪律（见下），单独成文件才好一眼看全。
 *
 * ## 三条纪律（缺一条就会以"静默"方式出错）
 *
 * 1. **无 DOM 不加载**。`STLLoader` 走 `FileLoader` → `fetch`/XHR，
 *    而单元与验收测试跑在 **node 环境**（`vite.config.ts` 的 `environment: 'node'`）。
 *    不守卫就会连带打挂几何类测试（与 `plateTexture` 遇到的是同一个坑）。
 * 2. **key 未登记 ⇒ 告警 + 回退"不画这个件"**，绝不抛异常让整棵树构建失败。
 *    贴图失败是静默的，网格失败同样静默 —— 只有告警 + 测试能兜住。
 * 3. **几何按 URL 缓存、材质逐实例**。几何可能上千个三角形、且被多个 link 引用
 *    （SO-ARM101 的 `sts3215_03a_v1.stl` 就出现在 5 个 link 上），每次重下重解析
 *    纯属浪费；而材质必须逐实例 —— 幽灵臂（半透明）会**原地改材质**，
 *    共享材质会让主臂跟着变半透明。
 */

import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import type { MeshGeometry } from '@robot/model/Link';
import { geometryColor } from '@robot/model/Link';
import { resolveMeshUrl } from '@robot/model/meshRegistry';

type Disposables = Array<{ dispose(): void }>;

/** 本环境能否发起网格请求（早于 `plateTexture` 的同一判据） */
function canLoadMesh(): boolean {
  return typeof document !== 'undefined';
}

/** URL → 已解析几何（进行中 / 已完成）。几何**跨实例共享**，因此不交给调用方 dispose */
const GEOMETRY_CACHE = new Map<string, Promise<THREE.BufferGeometry>>();

/** 已告警过的 key —— 同一个缺件每帧刷屏会把真正的日志冲掉 */
const WARNED = new Set<string>();
function warnOnce(key: string, message: string): void {
  if (WARNED.has(key)) return;
  WARNED.add(key);
  console.warn(`[mesh] ${message}`);
}

/** 外部网格的缺省材质参数：官方件多为**打印件 / 舵机壳**，非金属且偏哑光 */
export const DEFAULT_MESH_METALNESS = 0.0;
export const DEFAULT_MESH_ROUGHNESS = 0.62;

function loadGeometry(url: string): Promise<THREE.BufferGeometry> {
  const cached = GEOMETRY_CACHE.get(url);
  if (cached) return cached;
  const task = new STLLoader().loadAsync(url).catch((error: unknown) => {
    // 失败不进缓存，留出重试机会（网络抖动不该变成"这个件永久消失"）
    GEOMETRY_CACHE.delete(url);
    throw error;
  });
  GEOMETRY_CACHE.set(url, task);
  return task;
}

/**
 * 由 `MeshGeometry` 产出对象。
 *
 * 返回的 Group **立即**可用（可能暂时是空的），网格解析完成后自动挂上去；
 * `onReady` 在挂上后调用一次，供场景在"按需渲染"模式下请求重绘。
 * 位置 / 姿态由调用方 `applyPlacement()` 负责，与本函数无关。
 */
export function createMeshObject(
  geometry: MeshGeometry,
  disposables: Disposables,
  onReady?: () => void,
): THREE.Object3D {
  const group = new THREE.Group();
  group.name = `mesh:${geometry.file}`;

  if (!canLoadMesh()) {
    warnOnce(`nodom:${geometry.file}`, `当前环境无 DOM，跳过网格加载：${geometry.file}`);
    return group;
  }

  const url = resolveMeshUrl(geometry.file);
  if (!url) {
    warnOnce(
      `missing:${geometry.file}`,
      `网格未登记（应为 assets/models/ 下的相对路径）：${geometry.file} —— 本件将不显示`,
    );
    return group;
  }

  // 材质逐实例：幽灵臂会原地把它改成半透明，共享材质会连主臂一起改掉。
  const material = new THREE.MeshStandardMaterial({
    color: new THREE.Color(geometryColor(geometry)),
    metalness: geometry.metalness ?? DEFAULT_MESH_METALNESS,
    roughness: geometry.roughness ?? DEFAULT_MESH_ROUGHNESS,
  });
  disposables.push(material);

  loadGeometry(url)
    .then((source) => {
      const mesh = new THREE.Mesh(source, material);
      mesh.name = 'mesh';
      if (geometry.scale) mesh.scale.set(geometry.scale[0], geometry.scale[1], geometry.scale[2]);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      group.add(mesh);
      onReady?.();
    })
    .catch((error: unknown) => {
      warnOnce(`fail:${url}`, `加载网格失败：${geometry.file} —— ${String(error)}`);
    });

  return group;
}
