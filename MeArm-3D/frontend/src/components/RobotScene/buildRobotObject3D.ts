/**
 * RobotModel → Three.js 对象树（**纯 three.js，无 React**）。
 *
 * 为什么单独抽出来：这样「3D 场景里实际存在的 Joint Tree」与「node 验收测试里构建的对象树」
 * 是**同一份代码**产出的同一棵图，Phase 3 的「FK 计算结果 == Three.js 实际模型位置」才是
 * 真正可验证的，而不是两套各自实现的东西互相印证。
 *
 * 结构严格遵循：
 * ```
 *   RobotGroup
 *    └── JointGroup(base)          ← 一个关节 = 一个 THREE.Group
 *         ├── LinkObject(column)   ← 沿 +Z 从 0 伸展 length，全部几何由 robot.yaml 参数化
 *         └── JointGroup(shoulder)
 *              ├── LinkObject(upper_arm)
 *              └── ...
 * ```
 * **禁止**通过改 Mesh 的 position 来模拟关节运动：关节运动一律体现为 JointGroup 的旋转。
 *
 * 显示几何全部是 `robot.yaml` 里 links[].geometry / links[].details 的参数化拼装
 * （plate 倒角薄板 / box / cylinder / sphere / servo 舵机），
 * 渲染层只负责「参数 → 网格」，不含任何机构尺寸常量（spec §9 / §30）。
 */
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import type { Link, LinkGeometry, PlateGeometry, PlateTextureSpec, ServoGeometry } from '@robot/model/Link';
import {
  geometryColor,
  geometryPosition,
  geometryRotation,
  jawPlateSize,
  linkDetails,
  plateCornerRadius,
  plateTexture,
  servoRenderSpec,
} from '@robot/model/Link';
import { resolveTextureUrl } from '@robot/model/textureRegistry';
import type { Joint } from '@robot/model/Joint';
import type { JointState, Vec3 } from '@robot/model/Pose';
import { degToRad } from '@robot/model/Pose';
import type { Appearance, RobotModel } from '@robot/model/RobotModel';
import { jointById, jointByRole, linkById, rootLink } from '@robot/model/RobotModel';
import { effectiveJointAngle } from '@robot/kinematics/fk';

/** 机器人在场景中的单位换算唯一来源是 kinematics/coordinate.ts（1 场景单位 = 1 mm） */

/** 关节原点小球颜色（调试元素，spec §20） */
const JOINT_ORIGIN_COLOR = '#ffd166';

export interface RobotObjects {
  /** 机器人根 Group（其坐标系 = 根连杆的近端坐标系） */
  root: THREE.Group;
  /** jointId -> 关节坐标系 Group */
  jointGroups: Map<string, THREE.Group>;
  /** jointId -> 关节固定旋转（origin.rotation），applyJointState 需要它做合成 */
  jointOriginQuaternions: Map<string, THREE.Quaternion>;
  /** linkId -> 该连杆的可见对象（geometry.type === 'none' 且无 details 的连杆不产生对象） */
  linkObjects: Map<string, THREE.Object3D>;
  /** 末端 TCP 标记 */
  tcpMarker: THREE.Object3D;
  /** 关节轴向可视化集合（可整体开关） */
  jointAxisHelpers: THREE.Object3D[];
  /** 关节原点小球集合（可整体开关） */
  jointOriginHelpers: THREE.Object3D[];
  /** 夹爪两片爪（开合显示用） */
  gripperJaws: { left: THREE.Object3D; right: THREE.Object3D; axis: THREE.Vector3 } | null;
  /** 需要 dispose 的资源 */
  disposables: Array<{ dispose(): void }>;
}

type Disposables = RobotObjects['disposables'];

function toVector3(v: readonly number[]): THREE.Vector3 {
  return new THREE.Vector3(v[0] ?? 0, v[1] ?? 0, v[2] ?? 0);
}

/** 统一材质：简单 PBR，不用贴图（spec §24） */
function createMaterial(color: string, metalness = 0.35, roughness = 0.55): THREE.MeshStandardMaterial {
  return new THREE.MeshStandardMaterial({
    color: new THREE.Color(color),
    metalness,
    roughness,
  });
}

/** "实际臂"（ghost）半透明度：够淡以不挡主臂，够实以看清它落在哪 */
const GHOST_OPACITY = 0.28;

/**
 * 把整棵对象树改成"幽灵"外观：半透明 + 不写深度。
 *
 * 为什么用**透明度**而不是换个醒目颜色来区分「实际臂」：
 * 本项目的 UI 约定是"无装饰色"（见 `styles.css` 头部），而且颜色不属于数据。
 * 这里的信号是**位置分离**本身 —— 主臂在命令位、幽灵在实际位，
 * 两者分开多少就是滞后多少；收敛时两者重合（此时几乎看不见幽灵，正是想要的结论）。
 *
 * `depthWrite = false` 是必需的：否则半透明面之间会互相遮挡出"脏面"，
 * 从某些角度看整条臂会变成硬边色块。
 */
function makeGhost(root: THREE.Object3D): void {
  const patched = new Set<THREE.Material>();
  root.traverse((object) => {
    const mesh = object as THREE.Mesh;
    if (!mesh.isMesh) return;
    const materials = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
    for (const material of materials) {
      if (patched.has(material)) continue;
      patched.add(material);
      material.transparent = true;
      material.opacity = GHOST_OPACITY;
      material.depthWrite = false;
    }
  });
  root.renderOrder = 1;
}

/** 把几何体摆到它在「近端关节坐标系」中的位置 / 姿态 */
function applyPlacement(
  object: THREE.Object3D,
  geometry: LinkGeometry,
  linkLength: number,
): void {
  object.position.copy(toVector3(geometryPosition(geometry, linkLength)));
  const rotation = geometryRotation(geometry);
  object.rotation.set(
    degToRad(rotation[0]),
    degToRad(rotation[1]),
    degToRad(rotation[2]),
    'XYZ',
  );
}

/** 倒角盒：半径按最小边钳位，过小则退化为直角盒（避免退化几何） */
function createRoundedBox(size: Vec3, radius: number): THREE.BufferGeometry {
  const [sx, sy, sz] = size;
  const clamped = Math.min(radius, Math.min(sx, sy, sz) / 2 - 0.01);
  if (clamped < 0.05) return new THREE.BoxGeometry(sx, sy, sz);
  return new RoundedBoxGeometry(sx, sy, sz, 2, clamped);
}

/**
 * `BoxGeometry` / `RoundedBoxGeometry` 的 materialIndex → 面。
 * **实测得出**（`.workbuddy/captures/uv_probe.mjs`，见 ADR D63），非按记忆推定。
 */
const FACE_GROUPS = [
  { normal: '+X', du: '-Z', dv: '+Y' },
  { normal: '-X', du: '+Z', dv: '+Y' },
  { normal: '+Y', du: '+X', dv: '-Z' },
  { normal: '-Y', du: '+X', dv: '+Z' },
  { normal: '+Z', du: '+X', dv: '+Y' },
  { normal: '-Z', du: '-X', dv: '+Y' },
] as const;

/**
 * 「大面」（贴照片的那两个面）在材质数组里的下标。
 *
 * 大面 = `size` 里**最小那一维**所在的轴（薄板的两个最大平面就垂直于它）。
 * 例：大臂板 `[22, 5, 74]` 最小维是 Y ⇒ 大面 = group 2 / 3。
 */
function bigFaceIndices(size: Vec3): [number, number] {
  let axis = 0;
  for (let i = 1; i < 3; i++) if (size[i]! < size[axis]!) axis = i;
  return [axis * 2, axis * 2 + 1];
}

/**
 * 同一块板「两个大面里哪一面需要镜像、镜像在哪个 uv 轴」。
 *
 * **两条依据，缺一不可：**
 *
 * 1）几何必然（上面实测表）：两个大面的 Δu/Δv 必有一个相反 ⇒ 同一张照片
 *    只能在一面上原样显示，另一面必须镜像，否则从两侧看会有一次左右/上下翻转。
 *
 * 2）★ **`TextureLoader` 默认 `flipY = true`**：上传时图像被垂直翻转，
 *    因此 **图像顶行 ⇔ `uv_v = 1`**（而不是 `v = 0`）。这一条极易漏，
 *    漏掉会让所有 v 方向的推理**整体反号** —— 本轮就是这么错了一次：
 *    受控实验（四象限探针纹理，`.workbuddy/captures/probe_axes.py`）拍到
 *    屏幕上 tile 的 `v>0.5` 出现在**上方**，才定位到多翻了一次。
 *
 * 于是「基准面」定为 **+ 轴侧那个 group**（`thinAxis * 2`）：照片的"上"是物理世界的"上"，
 * 大面里 Δv 与「局部 + 轴」同向的那一个直接可用，另一个才镜像。
 * 例（`±Y` 大面，本轮实测）：面 2 (+Y) Δv = `-Z` ⇒ 需镜像；面 3 (−Y) Δv = `+Z` ⇒ 原样。
 *
 * 照片那一侧（哪端朝上、是否整体翻转）属于外观事实，交给 `robot.yaml` 的 `textureFlipU/V`。
 */
function mirrorAxisForThinAxis(thinAxis: number): 'u' | 'v' {
  const front = FACE_GROUPS[thinAxis * 2]!;
  const back = FACE_GROUPS[thinAxis * 2 + 1]!;
  return front.du !== back.du ? 'u' : 'v';
}

/**
 * 本环境能否加载图片纹理。
 *
 * `THREE.TextureLoader` 内部会 `document.createElementNS('img')`，
 * 而本项目的单元/验收测试跑在 **node 环境**（`vite.config.ts` 的 `environment: 'node'`）——
 * 那里没有 DOM，硬加载会直接抛 `ReferenceError: document is not defined`，
 * 把整棵对象树建不出来。纹理只影响外观、不参与任何几何或 FK 判定，
 * 所以在无 DOM 的环境按「没配纹理」处理是正确取舍，不是降级 hack。
 */
const CAN_LOAD_TEXTURE = typeof document !== 'undefined';

/**
 * 照片纹理材质：一张图 → `MeshStandardMaterial`。
 *
 * 三个必须显式设置的量：
 * - `colorSpace = SRGBColorSpace`：纹理是 sRGB 图，不声明会让整块板偏暗（three 默认按线性解释）。
 * - `ClampToEdgeWrapping`：翻转靠 `repeat` 取负值实现，`Repeat` 会让边缘采到对侧像素。
 * - `anisotropy`：板面在场景里常以大角度出现，各向异性过滤能救回边缘清晰度。
 *
 * 翻转用 `repeat/offset` 而不是 `texture.flipY`：`flipY` 只在**上传时**生效
 * （对 ImageBitmap 等路径无效），`repeat` 是在着色器里做的，一律可靠。
 */
function createTextureMaterial(
  spec: PlateTextureSpec,
  url: string,
  mirrorAxis: 'u' | 'v' | null,
  disposables: Disposables,
): THREE.MeshStandardMaterial {
  const material = new THREE.MeshStandardMaterial({
    // 照片本身已含颜色；材质基色保持白，避免二次染色
    color: 0xffffff,
  });
  // ⚠️ roughness / metalness / 曝光补偿**不在这里定死**：它们来自 robot.yaml 的
  //    appearance 段，由 applyPlateAppearance() 在对象树建好后统一应用。
  //    name 是这个后处理的**唯一标识**（不靠 `map != null` 猜，那样会把将来的
  //    其它贴图材质一并卷进来）。
  material.name = TEXTURED_PLATE_MATERIAL;

  const texture = new THREE.TextureLoader().load(url);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.wrapS = THREE.ClampToEdgeWrapping;
  texture.wrapT = THREE.ClampToEdgeWrapping;
  texture.anisotropy = 4;

  const repeatU = (spec.flipU ? -1 : 1) * (mirrorAxis === 'u' ? -1 : 1);
  const repeatV = (spec.flipV ? -1 : 1) * (mirrorAxis === 'v' ? -1 : 1);
  texture.repeat.set(repeatU, repeatV);
  texture.offset.set(repeatU < 0 ? 1 : 0, repeatV < 0 ? 1 : 0);

  material.map = texture;
  disposables.push(texture, material);
  // 贴了照片的板就不再是单色板：金属度压低、粗糙度抬高，让照片自己说话
  return material;
}

/**
 * 薄板材质：无纹理时返回**单个材质**，有纹理时返回**6 元材质数组**
 * （大面贴照片，其余四面仍是板色）。
 *
 * 为什么不给整块板一个材质：`RoundedBoxGeometry` 的 UV 是「每个面各自铺满 [0,1]」，
 * 一个材质会让 5mm 窄边也被整张照片铺满 —— 侧面出现被拉扁的螺栓，比不贴更假。
 */
function createPlateMaterial(
  geometry: PlateGeometry,
  disposables: Disposables,
): THREE.Material | THREE.Material[] {
  const base = createMaterial(geometryColor(geometry), 0.2, 0.62);
  disposables.push(base);

  const spec = plateTexture(geometry);
  if (!spec || !CAN_LOAD_TEXTURE) return base;

  const url = resolveTextureUrl(spec.key);
  if (!url) {
    // 纹理缺失不该让场景崩掉：留下配色板，并把情况说出来
    console.warn(`[robot] 纹理未登记，回退纯色: ${spec.key}（可用的 key 见 listTextureKeys()）`);
    return base;
  }

  const [frontIndex, backIndex] = bigFaceIndices(geometry.size);
  const thinAxis = Math.round(frontIndex / 2);
  const mirrorAxis = mirrorAxisForThinAxis(thinAxis);

  // 基准面 = + 轴侧（thinAxis*2）需要镜像，− 轴侧原样 —— 理由见 mirrorAxisForThinAxis 注释
  const front = createTextureMaterial(spec, url, mirrorAxis, disposables);
  const back = createTextureMaterial(spec, url, null, disposables);

  const materials: THREE.Material[] = [base, base, base, base, base, base];
  materials[frontIndex] = front;
  materials[backIndex] = back;
  return materials;
}

/** 倒角薄板（低多边形工程外观主力件）：边缘适当倒角，spec §5 */
function createPlateMesh(geometry: PlateGeometry, disposables: Disposables): THREE.Mesh {
  const objectGeometry = createRoundedBox(geometry.size, plateCornerRadius(geometry));
  const material = createPlateMaterial(geometry, disposables);
  const mesh = new THREE.Mesh(objectGeometry, material);
  mesh.name = 'plate';
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  disposables.push(objectGeometry);
  return mesh;
}

/**
 * 舵机：壳体 + 两侧安装耳 + 金属输出轴 + 圆舵盘。
 * 本体局部约定 **+Z = 输出轴方向、原点 = 壳体中心**（见 model/Link.ts 的 ServoGeometry 注释）。
 * 舵盘刻意做成**圆盘**而非带臂舵盘：带臂件在关节旋转时会产生方向错觉（§35 结构正确性优先）。
 */
function createServoObject(geometry: ServoGeometry, disposables: Disposables): THREE.Object3D {
  const spec = servoRenderSpec(geometry);
  const [sx, sy, sz] = spec.size;

  const group = new THREE.Group();
  group.name = 'servo';

  const bodyMaterial = createMaterial(spec.bodyColor, 0.22, 0.65);
  const bodyGeometry = createRoundedBox(spec.size, Math.min(1.6, Math.min(sx, sy, sz) / 2 - 0.01));
  const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
  body.castShadow = true;
  body.receiveShadow = true;
  group.add(body);
  disposables.push(bodyGeometry, bodyMaterial);

  if (spec.ears) {
    const earLength = 10.6;
    const earThickness = 2.8;
    const earGeometry = createRoundedBox([earLength, sy, earThickness], 0.8);
    for (const sign of [-1, 1]) {
      const ear = new THREE.Mesh(earGeometry, bodyMaterial);
      ear.position.set(sign * (sx / 2 + earLength / 2), 0, sz / 2 - earThickness / 2 - 0.3);
      ear.castShadow = true;
      group.add(ear);
    }
    disposables.push(earGeometry);
  }

  const shaftMaterial = createMaterial(spec.shaftColor, 0.85, 0.28);
  const shaftGeometry = new THREE.CylinderGeometry(3, 3, spec.shaftLength, 16);
  const shaft = new THREE.Mesh(shaftGeometry, shaftMaterial);
  shaft.position.set(0, 0, sz / 2 + spec.shaftLength / 2);
  shaft.rotation.x = Math.PI / 2; // 圆柱默认轴为 +Y，转到 +Z
  group.add(shaft);

  const hubGeometry = new THREE.CylinderGeometry(5.2, 5.2, 2.6, 20);
  const hub = new THREE.Mesh(hubGeometry, shaftMaterial);
  hub.position.set(0, 0, sz / 2 + spec.shaftLength + 1.3);
  hub.rotation.x = Math.PI / 2;
  group.add(hub);

  disposables.push(shaftGeometry, hubGeometry, shaftMaterial);
  return group;
}

/** 由 LinkGeometry 产出一个（或一组）可见对象；'none' 返回 null */
function createGeometryObject(
  geometry: LinkGeometry,
  linkLength: number,
  disposables: Disposables,
): THREE.Object3D | null {
  switch (geometry.type) {
    case 'none':
      return null;

    case 'plate': {
      const mesh = createPlateMesh(geometry, disposables);
      applyPlacement(mesh, geometry, linkLength);
      return mesh;
    }

    case 'servo': {
      const group = createServoObject(geometry, disposables);
      applyPlacement(group, geometry, linkLength);
      return group;
    }

    case 'box':
    case 'cylinder':
    case 'sphere': {
      let objectGeometry: THREE.BufferGeometry;
      if (geometry.type === 'box') {
        objectGeometry = new THREE.BoxGeometry(geometry.size[0], geometry.size[1], geometry.size[2]);
      } else if (geometry.type === 'cylinder') {
        objectGeometry = new THREE.CylinderGeometry(
          geometry.radius,
          geometry.radius,
          geometry.height,
          geometry.radialSegments ?? 24,
        );
      } else {
        objectGeometry = new THREE.SphereGeometry(geometry.radius, 24, 16);
      }
      const material = createMaterial(geometryColor(geometry));
      const mesh = new THREE.Mesh(objectGeometry, material);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      applyPlacement(mesh, geometry, linkLength);
      disposables.push(objectGeometry, material);
      return mesh;
    }
  }
}

/** 一根连杆 → 一个 Group（主体几何 + details 附加件，全部参数化） */
function createLinkObject(link: Link, disposables: Disposables): THREE.Object3D | null {
  const parts: THREE.Object3D[] = [];

  const main = createGeometryObject(link.geometry, link.length, disposables);
  if (main) parts.push(main);

  for (const detail of linkDetails(link)) {
    // 附加件省略 position 时落在本坐标系原点（而非连杆中段），这是附加件最合理的缺省
    const object = createGeometryObject(detail, 0, disposables);
    if (object) parts.push(object);
  }

  if (parts.length === 0) return null;

  const group = new THREE.Group();
  group.name = `link:${link.id}`;
  for (const part of parts) group.add(part);
  return group;
}

/** 关节轴向可视化（实体细杆 + 锥头）。轴向在本关节坐标系下定义，因此挂在关节 Group 上。
 *  用圆柱而不是 THREE.Line：WebGL 下 LineBasicMaterial 的 linewidth 恒为 1px，几乎看不见。 */
function createJointAxisObject(joint: Joint, disposables: Disposables): THREE.Object3D {
  const axis = toVector3(joint.axis);
  if (axis.lengthSq() === 0) axis.set(0, 0, 1);
  axis.normalize();

  const length = 34;
  const shaftRadius = 0.8;
  const color = new THREE.Color('#ff7a45');
  const headLength = 9;
  const headWidth = 4.5;

  const group = new THREE.Group();
  group.name = `jointAxisGroup:${joint.id}`;

  const shaftGeometry = new THREE.CylinderGeometry(shaftRadius, shaftRadius, length, 8);
  const material = new THREE.MeshBasicMaterial({ color });
  const shaft = new THREE.Mesh(shaftGeometry, material);
  shaft.position.copy(axis.clone().multiplyScalar(length / 2));
  shaft.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis);

  const headGeometry = new THREE.ConeGeometry(headWidth, headLength, 12);
  const head = new THREE.Mesh(headGeometry, material);
  head.position.copy(axis.clone().multiplyScalar(length + headLength / 2));
  head.quaternion.copy(shaft.quaternion);

  group.add(shaft, head);
  disposables.push(shaftGeometry, headGeometry, material);
  return group;
}

/** 关节原点小球（调试元素，spec §20） */
function createJointOriginObject(disposables: Disposables): THREE.Object3D {
  const geometry = new THREE.SphereGeometry(2.6, 12, 10);
  const material = new THREE.MeshBasicMaterial({ color: JOINT_ORIGIN_COLOR });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.name = 'jointOrigin';
  disposables.push(geometry, material);
  return mesh;
}

/** TCP 标记：小球 + 三轴短十字，让「末端」在场景里可见 */
function createTcpMarker(disposables: Disposables): THREE.Object3D {
  const marker = new THREE.Group();
  marker.name = 'tcp';

  const dotGeometry = new THREE.SphereGeometry(4.6, 16, 12);
  const dotMaterial = new THREE.MeshBasicMaterial({ color: '#3fb950' });
  marker.add(new THREE.Mesh(dotGeometry, dotMaterial));
  disposables.push(dotGeometry, dotMaterial);

  const armLength = 26;
  const axes: Array<{ dir: THREE.Vector3; color: string }> = [
    { dir: new THREE.Vector3(1, 0, 0), color: '#ff5555' },
    { dir: new THREE.Vector3(0, 1, 0), color: '#55ff77' },
    { dir: new THREE.Vector3(0, 0, 1), color: '#6699ff' },
  ];
  for (const { dir, color: axisColor } of axes) {
    const geometry = new THREE.CylinderGeometry(0.8, 0.8, armLength, 6);
    const material = new THREE.MeshBasicMaterial({ color: axisColor });
    const bar = new THREE.Mesh(geometry, material);
    bar.position.copy(dir.clone().multiplyScalar(armLength / 2));
    bar.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    marker.add(bar);
    disposables.push(geometry, material);
  }

  return marker;
}

/**
 * 夹爪两片爪的显示体（开合角由 gripper 关节角驱动）。
 *
 * 爪片尺寸来自 `jaw_link.geometry`（plate/box 的 size = [宽, 厚, 长]），因此改 yaml 即改爪型。
 * 绕 X 铰轴对称开合：左爪（y<0）+θ/2、右爪（y>0）−θ/2 —— 符号使两爪**向外张开**；
 * 反过来写会让两爪互穿（见 applyJointState 注释）。
 */
function createGripperJaws(
  size: Vec3,
  color: string,
  disposables: Disposables,
): { left: THREE.Object3D; right: THREE.Object3D } {
  // size = [宽, 厚, 长]；宽由 createRoundedBox(size) 直接消费，此处只需厚与长
  const [, thickness, length] = size;
  const geometry = createRoundedBox(size, 1.1);
  const material = createMaterial(color, 0.35, 0.5);
  disposables.push(geometry, material);

  const make = (sign: 1 | -1) => {
    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = true;
    // 爪片沿 +Z 伸出、沿 ±Y 向两侧让开，铰点位于连杆中线上
    mesh.position.set(0, sign * (thickness / 2), length / 2);
    const pivot = new THREE.Group();
    pivot.name = sign > 0 ? 'jaw:right' : 'jaw:left';
    pivot.add(mesh);
    return pivot;
  };
  return { left: make(-1), right: make(1) };
}

/** 照片纹理材质的 name —— `applyPlateAppearance()` 靠它识别要调整哪一类材质 */
const TEXTURED_PLATE_MATERIAL = 'plateTexture';

/**
 * 把 `robot.yaml` 的 appearance 段应用到对象树，**并且把环境反射限定在照片纹理件上**。
 *
 * 为什么做成「建好树后统一应用」而不是把参数一路透传进
 * `createLinkObject → createGeometryObject → createPlateMesh → createPlateMaterial`：
 * 外观参数是整机级的，透传要改 5 个纯几何职责的函数签名；后处理只有一个入口，
 * 将来新增贴图件也自动被覆盖。识别身份用 `material.name`，不用 `map != null`
 * 这类启发式 —— 后者会把别的贴图材质悄悄卷进来。
 *
 * 四个量（实测依据见 ADR D64）：
 * - `roughness` / `metalness`：亚克力是**光泽非金属**。黑件的形状可读性来自镜面反射，
 *   做成纯漫反射只会得到一块黑（实测板面输出 L≈0.08、max=2.4）。
 * - `exposureEv`：线性曝光补偿，实现为 `material.color = 2^EV`。three 的
 *   `diffuse = color ⊗ map`，且 `Color` 分量**不 clamp**，所以这就是一次精确的线性提亮。
 *   ⚠️ 它同时抬高 F0（`mix(0.04, diffuse, metalness)`）：只要 `metalness ≠ 0`，
 *   高光就会被一起放大成镜面，所以 yaml 里把 metalness 钉在 0。
 * - `environmentIntensity` + `envMap`：环境反射**逐材质挂 `material.envMap`**，
 *   而不是设 `scene.environment` —— 后者是全局的，且 three 会对「没有自带 envMap」
 *   的材质用 `scene.environmentIntensity` **覆盖**其 `envMapIntensity`
 *   （`WebGLRenderer.js`），逐材质降级根本做不到；实测整机非贴图件会被一并点亮
 *   （底座蓝板 ×3.75）。自带 envMap 后只有照片纹理件收到环境反射。
 */
export function applyAppearance(
  root: THREE.Object3D,
  appearance: Appearance,
  envMap: THREE.Texture | null = null,
): void {
  const { environmentIntensity, exposureEv, roughness, metalness } = appearance.texturedPlate;
  const gain = Math.pow(2, exposureEv);

  root.traverse((object) => {
    const mesh = object as THREE.Mesh;
    if (!mesh.isMesh) return;
    const materials = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
    for (const material of materials) {
      if (material.name !== TEXTURED_PLATE_MATERIAL) continue;
      const standard = material as THREE.MeshStandardMaterial;
      standard.roughness = roughness;
      standard.metalness = metalness;
      standard.color.setScalar(gain);
      standard.envMap = envMap;
      standard.envMapIntensity = environmentIntensity;
      // 换 envMap 会切换 shader 的 ENVMAP 宏分支，必须重编译
      standard.needsUpdate = true;
    }
  });
}

/**
 * 由 RobotModel 构建完整 Three.js 对象树。
 * 只构建一次；之后每次关节变化调用 `applyJointState()`。
 */
export interface BuildRobotObject3DOptions {
  /**
   * 构建「实际臂幽灵」而非主臂：整体半透明、不写深度，且**不带调试元素**
   * （关节轴 / 关节原点小球）—— 幽灵只负责表达位姿，画上调试球只会让人看花眼。
   */
  ghost?: boolean;
  /**
   * 照片纹理件的环境反射纹理（见 `plateEnvironment.ts`）。
   *
   * 由调用方（React 侧，持有 renderer）创建后传入 —— 本模块必须保持
   * **不依赖 DOM / WebGL**，否则 node 环境下的验收测试建不出对象树。
   * 缺省 `null` = 不给贴图件加环境反射（行为与引入本特性前一致）。
   */
  envMap?: THREE.Texture | null;
}

export function buildRobotObject3D(
  model: RobotModel,
  options: BuildRobotObject3DOptions = {},
): RobotObjects {
  const disposables: Disposables = [];
  const jointGroups = new Map<string, THREE.Group>();
  const jointOriginQuaternions = new Map<string, THREE.Quaternion>();
  const linkObjects = new Map<string, THREE.Object3D>();
  const jointAxisHelpers: THREE.Object3D[] = [];
  const jointOriginHelpers: THREE.Object3D[] = [];
  let gripperJaws: RobotObjects['gripperJaws'] = null;

  const jointsByParentLink = new Map<string, Joint[]>();
  for (const joint of model.joints) {
    const list = jointsByParentLink.get(joint.parentLink) ?? [];
    list.push(joint);
    jointsByParentLink.set(joint.parentLink, list);
  }

  const root = new THREE.Group();
  root.name = `robot:${model.id}`;

  // 按 role 定位夹爪关节，避免在渲染代码里硬编码关节 id
  const gripperJoint = jointByRole(model, 'gripper') ?? null;
  // 夹爪连杆由「两片爪绕铰轴对称开合」特例渲染，其自身 geometry 仅用于提供爪片尺寸
  const jawLinkId = gripperJoint?.childLink ?? null;

  /**
   * 把一根连杆挂到 parentObject 上，并递归其子关节。
   * parentObject 的坐标系 = 该连杆的近端（父关节）坐标系。
   */
  const attachLink = (link: Link, parentObject: THREE.Object3D): void => {
    if (link.id !== jawLinkId) {
      const object = createLinkObject(link, disposables);
      if (object) {
        parentObject.add(object);
        linkObjects.set(link.id, object);
      }
    }

    const childJoints = jointsByParentLink.get(link.id) ?? [];

    for (const joint of childJoints) {
      const jointGroup = new THREE.Group();
      jointGroup.name = `joint:${joint.id}`;

      // 父连杆沿 +Z 伸展 length -> 本关节坐标系原点，再叠加 origin.position 附加偏移
      jointGroup.position.set(0, 0, link.length);
      jointGroup.position.add(toVector3(joint.origin.position));

      const originQuaternion = new THREE.Quaternion().setFromEuler(
        new THREE.Euler(
          degToRad(joint.origin.rotation[0]),
          degToRad(joint.origin.rotation[1]),
          degToRad(joint.origin.rotation[2]),
          'XYZ',
        ),
      );
      jointGroup.quaternion.copy(originQuaternion);

      jointOriginQuaternions.set(joint.id, originQuaternion);
      jointGroups.set(joint.id, jointGroup);
      parentObject.add(jointGroup);

      const axisObject = createJointAxisObject(joint, disposables);
      axisObject.visible = false;
      jointGroup.add(axisObject);
      jointAxisHelpers.push(axisObject);

      const originObject = createJointOriginObject(disposables);
      originObject.visible = false;
      jointGroup.add(originObject);
      jointOriginHelpers.push(originObject);

      const childLink = linkById(model, joint.childLink);
      if (childLink) attachLink(childLink, jointGroup);
    }

    // 夹爪特例：gripper 关节的 childLink(jaw_link) 无实体几何，改由「两片爪绕关节轴 ±θ/2
    // 对称开合」表达。爪片挂在父坐标系（工具系）下，使开合绕掌心中线对称；
    // 而 gripper 关节 Group 自身的旋转仍严格等于 FK 结果。
    if (gripperJoint && gripperJoint.parentLink === link.id) {
      const jawLink = linkById(model, gripperJoint.childLink);
      const jawColor = jawLink ? geometryColor(jawLink.geometry) : '#d29922';
      const jaws = createGripperJaws(jawPlateSize(jawLink), jawColor, disposables);
      const pivot = new THREE.Group();
      pivot.name = 'gripperPalm';
      pivot.position.set(0, 0, link.length);
      pivot.add(jaws.left, jaws.right);
      parentObject.add(pivot);
      gripperJaws = {
        left: jaws.left,
        right: jaws.right,
        axis: toVector3(gripperJoint.axis).normalize(),
      };
    }
  };

  attachLink(rootLink(model), root);

  // 外观（渲染）参数统一应用：见 applyAppearance 注释。
  // 放在 attachLink 之后、makeGhost 之前 —— 与半透明化是两个正交的关注点。
  applyAppearance(root, model.appearance, options.envMap ?? null);

  // TCP 标记：挂在 tcp.joint 对应的关节 Group 下，偏移 tcp.offset
  const tcpParent = jointGroups.get(model.tcp.joint) ?? root;
  const tcpMarker = createTcpMarker(disposables);
  tcpMarker.position.copy(toVector3(model.tcp.offset));
  tcpParent.add(tcpMarker);

  if (options.ghost) {
    makeGhost(root);
    for (const helper of jointAxisHelpers) helper.visible = false;
    for (const helper of jointOriginHelpers) helper.visible = false;
  }

  return {
    root,
    jointGroups,
    jointOriginQuaternions,
    linkObjects,
    tcpMarker,
    jointAxisHelpers,
    jointOriginHelpers,
    gripperJaws,
    disposables,
  };
}

/**
 * 应用关节状态：**只改 JointGroup 的旋转**，从不改 Mesh 的 position。
 * 合成顺序 `qOrigin · qAxis(θ)` 与 FK 的 `R_euler · R_axis(θ)` 完全一致。
 */
export function applyJointState(
  objects: RobotObjects,
  model: RobotModel,
  state: JointState,
): void {
  for (const [jointId, group] of objects.jointGroups) {
    const joint = jointById(model, jointId);
    if (!joint) continue;

    const originQuaternion = objects.jointOriginQuaternions.get(jointId);
    if (originQuaternion) group.quaternion.copy(originQuaternion);

    if (joint.type !== 'revolute') continue;

    // ⚠️ 必须用 effectiveJointAngle（含 coupling），与 fk.ts 保持逐值一致；
    //    直接用 state[jointId] 会在平行四连杆关节上让 3D 与 FK 分家。
    const angleDeg = effectiveJointAngle(model, state, joint);
    const axis = toVector3(joint.axis);
    if (axis.lengthSq() === 0) axis.set(0, 0, 1);
    axis.normalize();

    group.quaternion.multiply(
      new THREE.Quaternion().setFromAxisAngle(axis, degToRad(angleDeg)),
    );
  }

  // 夹爪开合显示：两片爪绕关节轴对称 ±θ/2（相对掌心中线）。
  // 符号必须让两爪向外张开：左爪(y<0) 取 +θ/2、右爪(y>0) 取 −θ/2；写反会导致两爪互穿。
  if (objects.gripperJaws) {
    const gripperJoint = jointByRole(model, 'gripper');
    const { left, right, axis } = objects.gripperJaws;
    if (gripperJoint) {
      const angleDeg = state[gripperJoint.id] ?? gripperJoint.limits.min;
      const half = degToRad(angleDeg / 2);
      left.quaternion.setFromAxisAngle(axis, half);
      right.quaternion.setFromAxisAngle(axis, -half);
    }
  }
}

/** 释放 GPU 资源（模型热切换时使用） */
export function disposeRobotObject3D(objects: RobotObjects): void {
  for (const item of objects.disposables) item.dispose();
}
