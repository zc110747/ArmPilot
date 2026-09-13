/**
 * 实际臂幽灵回归（Phase 12 · 虚拟/真实同步）。
 *
 * 本组件的风险是"看起来加了一条臂，实际它跟错了源"——主臂跟 command、幽灵跟 actual，
 * 若两者被接反，画面上一切正常（两条臂都在动），只有把鼠标拖到真机上才会发现。
 * 所以这里把三条不变量钉死：
 *   1. 幽灵**必须**半透明且不写深度（否则会出现脏面，且会遮住主臂）；
 *   2. 幽灵**不得**带调试元素（关节轴 / 关节原点小球）；
 *   3. 幽灵渲染出的 TCP **逐值等于 FK(actualJoints)**，且与主臂（FK(commandJoints)）不同。
 * 第 3 条是接反正/接错源唯一能被单测抓到的地方。
 */
import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import {
  applyJointState,
  buildRobotObject3D,
} from '../../src/components/RobotScene/buildRobotObject3D';
import { endEffectorPosition, loadRobotModel, type JointState } from '../../src/robot';

const model = loadRobotModel();

const COMMAND: JointState = { base: -18, shoulder: 8, elbow: 118, gripper: 38 };
const ACTUAL: JointState = { base: 26, shoulder: 34, elbow: 132, gripper: 58 };

function meshesOf(root: THREE.Object3D): THREE.Mesh[] {
  const found: THREE.Mesh[] = [];
  root.traverse((object) => {
    if ((object as THREE.Mesh).isMesh) found.push(object as THREE.Mesh);
  });
  return found;
}

/** 渲染后的 TCP 世界坐标（必须 updateMatrixWorld，否则读到的还是建树时的零位） */
function tcpWorld(objects: ReturnType<typeof buildRobotObject3D>): THREE.Vector3 {
  objects.root.updateMatrixWorld(true);
  return new THREE.Vector3().setFromMatrixPosition(objects.tcpMarker.matrixWorld);
}

const distance = (v: THREE.Vector3, xyz: readonly number[]): number =>
  Math.hypot(v.x - xyz[0], v.y - xyz[1], v.z - xyz[2]);

describe('ActualGhostArm · 对象树', () => {
  it('主臂材质保持不透明（幽灵不应污染主臂）', () => {
    const main = buildRobotObject3D(model);
    const meshes = meshesOf(main.root);
    expect(meshes.length).toBeGreaterThan(4);
    for (const mesh of meshes) {
      const materials = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
      for (const material of materials) {
        expect(material.transparent, mesh.name).toBe(false);
        expect(material.depthWrite, mesh.name).toBe(true);
      }
    }
  });

  it('幽灵所有材质半透明且不写深度（否则半透明面互相遮挡出脏面）', () => {
    const ghost = buildRobotObject3D(model, { ghost: true });
    const meshes = meshesOf(ghost.root);
    expect(meshes.length).toBeGreaterThan(4);
    for (const mesh of meshes) {
      const materials = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
      for (const material of materials) {
        expect(material.transparent, mesh.name).toBe(true);
        expect(material.depthWrite, mesh.name).toBe(false);
        expect(material.opacity).toBeLessThan(0.5);
        expect(material.opacity).toBeGreaterThan(0.05);
      }
    }
  });

  it('幽灵不带调试元素（关节轴 / 关节原点小球一律隐藏）', () => {
    const ghost = buildRobotObject3D(model, { ghost: true });
    expect(ghost.jointAxisHelpers.length).toBeGreaterThan(0);
    expect(ghost.jointOriginHelpers.length).toBeGreaterThan(0);
    for (const helper of ghost.jointAxisHelpers) expect(helper.visible).toBe(false);
    for (const helper of ghost.jointOriginHelpers) expect(helper.visible).toBe(false);
  });
});

describe('ActualGhostArm · 跟随源', () => {
  it('幽灵渲染的 TCP 逐值等于 FK(actualJoints)，且与主臂的 FK(commandJoints) 不同', () => {
    const main = buildRobotObject3D(model);
    const ghost = buildRobotObject3D(model, { ghost: true });
    applyJointState(main, model, COMMAND);
    applyJointState(ghost, model, ACTUAL);

    const mainTcp = tcpWorld(main);
    const ghostTcp = tcpWorld(ghost);
    const expectedCommand = endEffectorPosition(model, COMMAND);
    const expectedActual = endEffectorPosition(model, ACTUAL);

    // 主臂 = 意图，幽灵 = 现状；两者各自与自己的 FK 逐值一致
    expect(distance(mainTcp, expectedCommand)).toBeLessThan(1e-9);
    expect(distance(ghostTcp, expectedActual)).toBeLessThan(1e-9);

    // 若把两条臂的源接反，下面这条会失败 —— 这是本组唯一能抓到"接反"的断言
    expect(distance(mainTcp, expectedActual)).toBeGreaterThan(1);
    expect(distance(ghostTcp, expectedCommand)).toBeGreaterThan(1);

    // 滞后量应当与关节差同量级：至少有 10mm，否则这组用例证明不了什么
    expect(mainTcp.distanceTo(ghostTcp)).toBeGreaterThan(10);
  });
});
