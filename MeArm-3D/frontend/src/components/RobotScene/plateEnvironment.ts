/**
 * 照片纹理件的**环境反射纹理**（ADR D64）。
 *
 * 为什么需要环境反射：黑亚克力（板面在照片里落在 8bit 的 1~3 码值）的漫反射项
 * 几乎恒等于 0，加多少平行光都是等比例放大，救不回来；但**镜面项不乘这个暗反照率**
 * （非金属 F0 ≈ 0.04），所以一块黑板能不能被"看见形状"，取决于它反射了什么。
 * 实测：无环境反射时板面渲染输出 L≈0.08（max 2.4，纯黑剪影）；
 * 挂上环境反射后 L≈16.6，纹理与螺栓都出来了。
 *
 * 为什么用 `material.envMap` 而**不是** `scene.environment`：
 * `scene.environment` 是全局的，而且 three 在「材质没有自己的 envMap」时会把
 * `material.envMapIntensity` **覆盖**成 `scene.environmentIntensity`
 * （见 `WebGLRenderer.js`：`m_uniforms.envMapIntensity.value = scene.environmentIntensity`），
 * 所以逐材质降级根本做不到 —— 实测整机非贴图件会被一并点亮（底座蓝板 ×3.75）。
 * 各材质自带 envMap 之后，环境反射**只作用于照片纹理件**，其余材质逐值不变。
 *
 * 用 `RoomEnvironment` + `PMREMGenerator` **程序化**生成，不引入任何外部 HDR 资产：
 * 离线可复现、不给产物加体积、也不受网络影响。
 */
import * as THREE from 'three';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

/**
 * 按 (renderer, intensity) 缓存的共享环境纹理。
 *
 * 同一 Canvas 下 `gl` 是稳定的，所以正常只会生成一张；改强度时才重建并释放旧的。
 * 做成模块级缓存而不是每个组件各持一份：这是**共享 GPU 资源**，
 * 主臂与幽灵臂各建一份既浪费显存，又会因为两张纹理不同而在画面上留下细微分界。
 */
let cached: { gl: THREE.WebGLRenderer; intensity: number; texture: THREE.Texture } | null = null;

/** 取（必要时创建）照片纹理件的环境反射纹理；`intensity <= 0` 时返回 null（= 完全关闭） */
export function plateEnvMap(gl: THREE.WebGLRenderer, intensity: number): THREE.Texture | null {
  if (intensity <= 0) return null;
  if (cached && cached.gl === gl && cached.intensity === intensity) return cached.texture;

  cached?.texture.dispose();
  const generator = new THREE.PMREMGenerator(gl);
  const target = generator.fromScene(new RoomEnvironment(), 0.04);
  // 生成器可立刻释放；已生成的 target.texture 仍然有效
  generator.dispose();

  cached = { gl, intensity, texture: target.texture };
  return cached.texture;
}
