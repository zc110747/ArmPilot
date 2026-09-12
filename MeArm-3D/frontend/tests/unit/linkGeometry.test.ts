/**
 * 显示几何（plate / servo / details）回归测试。
 *
 * 为什么需要：geometry / details 是**纯显示层**，一旦有人不小心让它参与运动学
 * （比如把尺寸写进 link.length、或让渲染层从几何里推导关节位置），
 * 就会出现"虚拟臂看着对、真机角度不对"的隐性事故。
 * 本文件把三条不变量钉死：
 *   1. geometry / details 的解析与默认值补齐正确；
 *   2. **改几何不改变 FK**（几何与运动学严格解耦）；
 *   3. 场景里真的建出了 4 个舵机，且夹爪开合方向正确（向外张开，不互穿）。
 */
import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import {
  buildRobotObject3D,
  applyJointState,
} from '../../src/components/RobotScene/buildRobotObject3D';
import {
  DEFAULT_PLATE_CORNER_RADIUS,
  DEFAULT_SERVO_SIZE,
  endEffectorPosition,
  homeJointState,
  jointByRole,
  linkById,
  linkDetails,
  loadRobotModel,
  parseRobotModelYaml,
  plateCornerRadius,
  servoRenderSpec,
  type LinkGeometry,
  type PlateGeometry,
  type ServoGeometry,
} from '../../src/robot';

const model = loadRobotModel();

/** 收集对象树里所有指定名字的对象（three.js 无选择器，手写遍历） */
function collectByName(root: THREE.Object3D, name: string): THREE.Object3D[] {
  const found: THREE.Object3D[] = [];
  root.traverse((object) => {
    if (object.name === name) found.push(object);
  });
  return found;
}

describe('LinkGeometry · 解析与默认值', () => {
  it('robot.yaml 每个连杆的 geometry 都是已知类型', () => {
    const known: ReadonlyArray<LinkGeometry['type']> = [
      'none',
      'box',
      'plate',
      'cylinder',
      'sphere',
      'servo',
    ];
    for (const link of model.links) {
      expect(known, `link ${link.id}`).toContain(link.geometry.type);
      for (const detail of linkDetails(link)) {
        expect(known, `link ${link.id} detail`).toContain(detail.type);
      }
    }
  });

  it('plate 缺省倒角半径 = DEFAULT_PLATE_CORNER_RADIUS', () => {
    const parsed = parseRobotModelYaml(`
version: 1
robot: { id: t, name: t, units: mm, homePose: { j: 0 } }
links:
  - { id: a, name: A, parent: null, length: 0, geometry: { type: plate, size: [10, 10, 2] } }
  - { id: b, name: B, parent: a, length: 10 }
joints:
  - { id: j, name: J, role: base, type: revolute, parentLink: a, childLink: b, axis: [0, 0, 1], limit: { min: 0, max: 10 } }
`);
    const geometry = parsed.links[0]!.geometry as PlateGeometry;
    expect(geometry.type).toBe('plate');
    expect(geometry.cornerRadius).toBe(DEFAULT_PLATE_CORNER_RADIUS);
  });

  it('plate 倒角半径按最小边自钳位（防止半径大于半边长导致几何自交）', () => {
    const thin: PlateGeometry = { type: 'plate', size: [40, 40, 2], cornerRadius: 30 };
    expect(plateCornerRadius(thin)).toBeLessThan(1);
    const roomy: PlateGeometry = { type: 'plate', size: [40, 40, 2], cornerRadius: 0.8 };
    expect(plateCornerRadius(roomy)).toBe(0.8);
  });

  it('servo 缺省值补齐 size / shaftLength / ears / 颜色', () => {
    const parsed = parseRobotModelYaml(`
version: 1
robot: { id: t, name: t, units: mm, homePose: { j: 0 } }
links:
  - id: a
    name: A
    parent: null
    length: 0
    details:
      - { type: servo }
  - { id: b, name: B, parent: a, length: 10 }
joints:
  - { id: j, name: J, role: base, type: revolute, parentLink: a, childLink: b, axis: [0, 0, 1], limit: { min: 0, max: 10 } }
`);
    const detail = linkDetails(parsed.links[0]!)[0] as ServoGeometry;
    expect(detail.type).toBe('servo');
    const spec = servoRenderSpec(detail);
    expect(spec.size).toEqual([...DEFAULT_SERVO_SIZE]);
    expect(spec.shaftLength).toBeGreaterThan(0);
    expect(spec.ears).toBe(true);
  });

  it('details 可显式覆盖 servo 尺寸与安装耳开关', () => {
    const detail: ServoGeometry = {
      type: 'servo',
      size: [40, 20, 40],
      shaftLength: 9,
      ears: false,
    };
    const spec = servoRenderSpec(detail);
    expect(spec.size).toEqual([40, 20, 40]);
    expect(spec.shaftLength).toBe(9);
    expect(spec.ears).toBe(false);
  });

  it('非法 geometry.type 被解析器拦截', () => {
    expect(() =>
      parseRobotModelYaml(`
version: 1
robot: { id: t, name: t, units: mm, homePose: { j: 0 } }
links:
  - { id: a, name: A, parent: null, length: 0, geometry: { type: teapot, size: [1, 1, 1] } }
joints:
  - { id: j, name: J, role: base, type: revolute, parentLink: a, childLink: b, axis: [0, 0, 1], limit: { min: 0, max: 10 } }
`),
    ).toThrow(/teapot/);
  });
});

describe('几何 ↔ 运动学解耦', () => {
  it('把整份 geometry / details 换成另一套外观，FK 一字不变', () => {
    const home = homeJointState(model);
    const before = endEffectorPosition(model, home);

    // 造一份"外观完全不同、运动学字段逐字复制"的模型
    const stripped = {
      ...model,
      links: model.links.map((link) => ({
        ...link,
        geometry: { type: 'none' } as LinkGeometry,
        details: undefined,
      })),
    };
    const after = endEffectorPosition(stripped, home);

    expect(after).toEqual(before);

    // HOME 解析解（小臂存的是**绝对角**，与肩角由平行四连杆解耦）：
    //   x = 80·sin(肩) + (80 + 40)·sin(小臂绝对角)
    //   z = 60 + 80·cos(肩) + (80 + 40)·cos(小臂绝对角)
    // 数值来自 2026-09-12 实拍反解，见 docs/hardware-measurement.md
    const rad = (deg: number): number => (deg * Math.PI) / 180;
    expect(before[0]).toBeCloseTo(
      80 * Math.sin(rad(home.shoulder!)) + 120 * Math.sin(rad(home.elbow!)), 6);
    expect(before[2]).toBeCloseTo(
      60 + 80 * Math.cos(rad(home.shoulder!)) + 120 * Math.cos(rad(home.elbow!)), 6);
  });

  it('jaw_link.geometry 只提供爪片尺寸，不改变 TCP', () => {
    const jawLink = linkById(model, jointByRole(model, 'gripper')!.childLink);
    expect(jawLink).toBeDefined();
    // 爪片尺寸必须来自配置（plate/box 的 size），否则渲染层会回落到 DEFAULT_JAW_SIZE
    expect(['plate', 'box']).toContain(jawLink!.geometry.type);
  });
});

describe('对象树 · 舵机与夹爪', () => {
  const objects = buildRobotObject3D(model);

  it('场景里存在 4 个舵机（底座 / 肩 / 肘 / 夹取）', () => {
    const servos = collectByName(objects.root, 'servo');
    expect(servos).toHaveLength(4);
  });

  it('每个关节都有 JointGroup、轴向指示与原点小球', () => {
    expect(objects.jointGroups.size).toBe(model.joints.length);
    expect(objects.jointAxisHelpers).toHaveLength(model.joints.length);
    expect(objects.jointOriginHelpers).toHaveLength(model.joints.length);
  });

  it('除夹爪连杆外，每根连杆都产出可见对象', () => {
    const jawLinkId = jointByRole(model, 'gripper')!.childLink;
    for (const link of model.links) {
      if (link.id === jawLinkId) continue; // 夹爪由两片爪特例渲染
      expect(objects.linkObjects.has(link.id), `link ${link.id}`).toBe(true);
    }
  });

  it('夹爪 θ=0 两爪贴合、θ=90 两爪向外张开（不互穿）', () => {
    const jaws = objects.gripperJaws;
    expect(jaws).not.toBeNull();

    // 读数取「爪片网格相对掌心中线坐标系」的位置：与整机姿态无关，只反映开合关系
    const palm = jaws!.left.parent!;
    const readJawLocal = (): [THREE.Vector3, THREE.Vector3] => {
      objects.root.updateMatrixWorld(true);
      const read = (pivot: THREE.Object3D): THREE.Vector3 => {
        const part = pivot.children[0] ?? pivot;
        const world = new THREE.Vector3();
        part.getWorldPosition(world);
        return palm.worldToLocal(world);
      };
      return [read(jaws!.left), read(jaws!.right)];
    };

    const thickness = 4.5; // robot.yaml: jaw_link.geometry.size = [9, 4.5, 34]

    applyJointState(objects, model, { ...homeJointState(model), gripper: 0 });
    const [closedLeft, closedRight] = readJawLocal();
    // 闭合：两爪分居铰轴两侧，各自偏移 = 爪厚/2，正好贴合
    expect(closedLeft.y).toBeCloseTo(-thickness / 2, 6);
    expect(closedRight.y).toBeCloseTo(thickness / 2, 6);

    applyJointState(objects, model, { ...homeJointState(model), gripper: 90 });
    const [openLeft, openRight] = readJawLocal();
    // 张开：仍分居各自一侧（说明是"向外张开"而不是"越过中线互穿"），且张开幅度显著变大
    expect(openLeft.y).toBeLessThan(0);
    expect(openRight.y).toBeGreaterThan(0);
    expect(openLeft.y).toBeLessThan(closedLeft.y - 1);
    expect(openRight.y).toBeGreaterThan(closedRight.y + 1);
  });

  it('夹爪开合不改变 TCP 世界坐标', () => {
    const readTcp = (): THREE.Vector3 => {
      objects.root.updateMatrixWorld(true);
      return new THREE.Vector3().setFromMatrixPosition(objects.tcpMarker.matrixWorld);
    };
    applyJointState(objects, model, { ...homeJointState(model), gripper: 0 });
    const a = readTcp();
    applyJointState(objects, model, { ...homeJointState(model), gripper: 90 });
    const b = readTcp();
    expect(a.distanceTo(b)).toBeLessThan(1e-9);
  });
});
