/**
 * Phase 3 验收（关键卡点）：**FK 计算结果 == Three.js 实际模型位置**，误差必须 < 0.1 mm。
 *
 * 这里不是"两套实现互相印证"，而是：
 *   - FK            ← 纯数学实现（src/robot/kinematics/fk.ts）
 *   - Three.js 对象树 ← 与实际 3D 场景**同一份构建代码**（components/RobotScene/buildRobotObject3D.ts）
 * 因此本测试通过 = 屏幕上的机械臂就是 FK 算出来的那一台。
 */
import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import { buildRobotObject3D, applyJointState } from '../../src/components/RobotScene/buildRobotObject3D';
import {
  endEffectorPosition,
  forwardKinematics,
  homeJointState,
  jointMatrices,
  loadRobotModel,
  movableJoints,
  type JointState,
  type RobotModel,
} from '../../src/robot';

const TOLERANCE_MM = 0.1;

/** 可复现的伪随机数（LCG），保证验收结果可复盘 */
function makeRandom(seed: number): () => number {
  let state = seed >>> 0;
  return () => {
    state = (state * 1664525 + 1013904223) >>> 0;
    return state / 0x100000000;
  };
}

function randomJointState(model: RobotModel, random: () => number): JointState {
  const state: JointState = {};
  for (const joint of movableJoints(model)) {
    const { min, max } = joint.limits;
    state[joint.id] = min + (max - min) * random();
  }
  return state;
}

/** 把 Three.js 世界矩阵转成 4x4 数组，便于与 FK 矩阵逐元素比较 */
function matrixToArray(object: THREE.Object3D): number[] {
  object.updateMatrixWorld(true);
  return object.matrixWorld.toArray();
}

function maxMatrixDelta(a: readonly number[], b: readonly number[]): number {
  let max = 0;
  for (let i = 0; i < 16; i += 1) {
    max = Math.max(max, Math.abs((a[i] ?? 0) - (b[i] ?? 0)));
  }
  return max;
}

describe('Phase 3 · FK 与 Three.js 位姿一致性（< 0.1 mm）', () => {
  const model = loadRobotModel();

  it('零位：末端应在正上方 60+80+80+40 = 260 mm 处', () => {
    const zero: JointState = { base: 0, shoulder: 0, elbow: 0, gripper: 0 };
    const position = endEffectorPosition(model, zero);
    expect(position[0]).toBeCloseTo(0, 9);
    expect(position[1]).toBeCloseTo(0, 9);
    expect(position[2]).toBeCloseTo(260, 9);
  });

  it('HOME 位末端位置与解析解一致', () => {
    // HOME 由 2026-09-12 实拍反解（见 docs/hardware-measurement.md）。
    // 注意小臂存的是**绝对角**（平行四连杆解耦），所以解析式里直接用它，不再叠肩角。
    const home = homeJointState(model);
    const [x, y, z] = endEffectorPosition(model, home);
    // 平面 2R：r = 80·sin(肩) + (80+40)·sin(小臂绝对角)
    //          z = 60 + 80·cos(肩) + (80+40)·cos(小臂绝对角)
    const rad = (deg: number): number => (deg * Math.PI) / 180;
    const r = 80 * Math.sin(rad(home.shoulder!)) + 120 * Math.sin(rad(home.elbow!));
    const zz = 60 + 80 * Math.cos(rad(home.shoulder!)) + 120 * Math.cos(rad(home.elbow!));
    expect(x).toBeCloseTo(r, 6);
    expect(y).toBeCloseTo(0, 9);
    expect(z).toBeCloseTo(zz, 6);
  });

  it('随机关节状态下，每个关节坐标系与 Three.js 的位姿误差 < 0.1（含旋转矩阵元素）', () => {
    const objects = buildRobotObject3D(model);
    const random = makeRandom(20260912);

    let maxPositionError = 0;
    let maxRotationError = 0;
    const samples = 200;

    for (let i = 0; i < samples; i += 1) {
      const state = randomJointState(model, random);
      applyJointState(objects, model, state);
      objects.root.updateMatrixWorld(true);

      const fkMatrices = jointMatrices(model, state);
      for (const [jointId, fkMatrix] of fkMatrices) {
        const group = objects.jointGroups.get(jointId);
        expect(group, `关节 ${jointId} 缺少 THREE.Group`).toBeDefined();
        const threeMatrix = matrixToArray(group!);
        maxRotationError = Math.max(maxRotationError, maxMatrixDelta(fkMatrix, threeMatrix));
      }

      // 末端 TCP
      const threeTcp = new THREE.Vector3().setFromMatrixPosition(objects.tcpMarker.matrixWorld);
      const fkTcp = endEffectorPosition(model, state);
      const positionError = Math.hypot(
        threeTcp.x - fkTcp[0],
        threeTcp.y - fkTcp[1],
        threeTcp.z - fkTcp[2],
      );
      maxPositionError = Math.max(maxPositionError, positionError);
    }

    console.log(
      `[Phase3] ${samples} 组随机关节状态：末端位置最大误差 = ${maxPositionError.toExponential(3)} mm，` +
        `关节坐标系矩阵最大元素误差 = ${maxRotationError.toExponential(3)}`,
    );

    expect(maxPositionError).toBeLessThan(TOLERANCE_MM);
    expect(maxRotationError).toBeLessThan(1e-6);
  });

  it('forwardKinematics 返回的 joints 位姿与直接矩阵读取一致', () => {
    const state: JointState = { base: -30, shoulder: 40, elbow: 55, gripper: 20 };
    const pose = forwardKinematics(model, state);
    for (const [jointId, matrix] of jointMatrices(model, state)) {
      const transform = pose.joints[jointId]!;
      const group = new THREE.Matrix4().fromArray(matrix);
      const position = new THREE.Vector3().setFromMatrixPosition(group);
      expect(transform.position[0]).toBeCloseTo(position.x, 9);
      expect(transform.position[1]).toBeCloseTo(position.y, 9);
      expect(transform.position[2]).toBeCloseTo(position.z, 9);
    }
    expect(pose.endEffector.position).toEqual(endEffectorPosition(model, state));
  });

  it('夹爪开合不改变末端定位（IK 只解 J1/J2/J3 的前提）', () => {
    const base: JointState = { base: 15, shoulder: 30, elbow: 40, gripper: 0 };
    for (const gripper of [0, 30, 60, 90]) {
      const position = endEffectorPosition(model, { ...base, gripper });
      expect(position[0]).toBeCloseTo(endEffectorPosition(model, base)[0], 9);
      expect(position[1]).toBeCloseTo(endEffectorPosition(model, base)[1], 9);
      expect(position[2]).toBeCloseTo(endEffectorPosition(model, base)[2], 9);
    }
  });

  it('末端高度随肩肘增大而降低（方向语义正确：+θ 向前倾倒）', () => {
    const upright = endEffectorPosition(model, { base: 0, shoulder: 0, elbow: 0, gripper: 0 });
    const lean = endEffectorPosition(model, { base: 0, shoulder: 30, elbow: 0, gripper: 0 });
    expect(lean[2]).toBeLessThan(upright[2]);
    expect(lean[0]).toBeGreaterThan(upright[0]);

    // 底座旋转：需先让机械臂偏离 Z 轴（末端不在 Z 轴上时绕 Z 旋转才会产生 Y 位移）
    const bent: JointState = { base: 0, shoulder: 30, elbow: 30, gripper: 0 };
    const before = endEffectorPosition(model, bent);
    expect(before[1]).toBeCloseTo(0, 9);
    const left = endEffectorPosition(model, { ...bent, base: 30 });
    expect(left[1]).toBeGreaterThan(0); // +θ 绕 +Z，X 轴转向 +Y（左侧）
    // 旋转不改变高度与水平半径
    expect(left[2]).toBeCloseTo(before[2], 9);
    expect(Math.hypot(left[0], left[1])).toBeCloseTo(Math.hypot(before[0], before[1]), 9);
  });
});
