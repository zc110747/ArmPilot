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
import type { Link, LinkGeometry, PlateGeometry, ServoGeometry } from '@robot/model/Link';
import {
  geometryColor,
  geometryPosition,
  geometryRotation,
  jawPlateSize,
  linkDetails,
  plateCornerRadius,
  servoRenderSpec,
} from '@robot/model/Link';
import type { Joint } from '@robot/model/Joint';
import type { JointState, Vec3 } from '@robot/model/Pose';
import { degToRad } from '@robot/model/Pose';
import type { RobotModel } from '@robot/model/RobotModel';
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

/** 倒角薄板（低多边形工程外观主力件）：边缘适当倒角，spec §5 */
function createPlateMesh(geometry: PlateGeometry, disposables: Disposables): THREE.Mesh {
  const objectGeometry = createRoundedBox(geometry.size, plateCornerRadius(geometry));
  const material = createMaterial(geometryColor(geometry), 0.2, 0.62);
  const mesh = new THREE.Mesh(objectGeometry, material);
  mesh.name = 'plate';
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  disposables.push(objectGeometry, material);
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
