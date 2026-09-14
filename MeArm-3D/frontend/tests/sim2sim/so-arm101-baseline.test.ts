/**
 * Sim2Sim · **SO-ARM101** 基线回归（`tests/baseline/so-arm101/sim2sim.json`）。
 *
 * ## 与 MeArm 那份的分工
 *
 * |                  | `mearm-v1-baseline.test.ts` | 本文件 |
 * |---|---|---|
 * | 基线文件          | `tests/baseline/mearm-v1/*.json`（4 份） | `tests/baseline/so-arm101/sim2sim.json`（1 份） |
 * | 判据侧            | FK / Three.js / IK / 工作空间 / 抽象层等价 | FK / Three.js / **能力诚实性** |
 * | 为什么只有 FK     | —— | SO-101 **没有逆解器**，也没有工作空间数据。没有的东西不做基线、不写判据 |
 *
 * 两份基线都由**同一套**判据生成（`simulation/mujoco/sim2sim.py:runSim2Sim(robot)`），
 * 这正是"统一框架"能不能成立的关键：如果 SO-101 另写一套验收，那"两台机器人
 * 用同一套判据"就只是说法，不是事实。
 *
 * ## 本文件在验什么（三条互相独立的判据）
 *
 * | # | 判据 | 为什么它不是自证 |
 * |---|---|---|
 * | ① | `forwardKinematics(model, q).endEffector` == 基线的 `tcpFrontend` | 基线由**另一个进程**（Python + MuJoCo）采集；这里只是复现 |
 * | ② | Three.js `tcpMarker.matrixWorld` == 基线 `tcpFrontend` | 读的是**真实渲染对象的矩阵**，不是把数学再算一遍 —— 挂错树 / 可见性误关 / 变换漏乘都会被抓到 |
 * | ③ | `kinematics.inverse()` 一律 `NOT_IMPLEMENTED` 且**不返回关节角** | 断言"没有的东西确实没有"，防止将来有人塞一个"看起来能用"的解 |
 *
 * ## 容差
 *
 * - 对基线：`1e-9 mm`（冻结与回归之间只应差浮点噪声）
 * - Three.js ↔ FK：`1e-6 mm`（与 MeArm 那份同值；项目既有契约是 0.1 mm，**只收紧不放松**）
 *
 * ⚠️ 这里**刻意不设**"参考 ↔ MuJoCo"判据：那是 µm 级差异，由官方两份文件
 * 自身的精度差导致（见 `simulation/mujoco/sim2sim.py` 的 `FK_TOL_MM`），
 * 属于 Python 侧（pytest）的判据。前端能独立验证的是"渲染 == FK == 录制值"。
 */
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import {
  applyJointState,
  buildRobotObject3D,
  disposeRobotObject3D,
} from '../../src/components/RobotScene/buildRobotObject3D';
import { SO_ARM101_ROBOT_ID, forwardKinematics, loadRobot } from '../../src/robot';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(HERE, '..', '..', '..');
const SNAPSHOT = path.join(REPO_ROOT, 'tests', 'baseline', 'so-arm101', 'sim2sim.json');

/** 对基线的位置容差（mm） */
const TOL_VS_BASELINE_MM = 1e-9;
/** Three.js 世界矩阵 vs FK 的容差（mm） */
const TOL_THREE_MM = 1e-6;

interface SnapCase {
  id: string;
  joints: Record<string, number>;
  tcpFrontend: number[];
  tcpRef: number[];
  tcpMujoco: number[];
}

interface SnapIkCase {
  id: string;
  target: number[];
  outcome: string;
  reason: string | null;
  joints: Record<string, number> | null;
  closedLoopMm: number | null;
}

interface Snapshot {
  robotId: string;
  model: string;
  modelVersion: string;
  jointOrder: string[];
  capability: { solverKind: string; positioningDof: number; supportsOrientation: boolean };
  toleranceMm: number;
  ikSupported: boolean;
  ikNote: string;
  timestepS: number;
  maxVelocityDegS: number;
  fkCases: SnapCase[];
  ikCases: SnapIkCase[];
}

function loadSnapshot(): Snapshot {
  return JSON.parse(readFileSync(SNAPSHOT, 'utf8')) as Snapshot;
}

const snapshot = loadSnapshot();
const entry = loadRobot(SO_ARM101_ROBOT_ID);
const model = entry.definition.robotModel;

function dist(a: readonly number[], b: readonly number[]): number {
  return Math.hypot(a[0]! - b[0]!, a[1]! - b[1]!, a[2]! - b[2]!);
}

// ---------------------------------------------------------------------------
// 0. 基线与当前模型同一
// ---------------------------------------------------------------------------

describe('Sim2Sim(SO-ARM101) · 基线与当前模型同一', () => {
  it('基线声明的 id / 模型 / 关节顺序与现在加载到的一致', () => {
    expect(snapshot.robotId).toBe(SO_ARM101_ROBOT_ID);
    expect(snapshot.model).toBe('SO-ARM101');
    expect(entry.id).toBe(snapshot.robotId);
    expect(entry.name).toBe(snapshot.model);
    expect(model.id).toBe('so-arm101');
    // 基线的 `jointOrder` = **可动关节**顺序（`RobotCfg.joint_order()` 的语义）。
    // 模型里还带着固定帧（`gripper_frame`，TCP 的挂载点），它不是可动关节，
    // 所以这里按 `type === 'revolute'` 过滤 —— 用整表比会多出那一个。
    expect(model.joints.filter((j) => j.type === 'revolute').map((j) => j.id)).toEqual(
      snapshot.jointOrder,
    );
  });

  it('基线声明的能力与引擎的 capability 一致（声明即契约）', () => {
    expect(entry.kinematics.capability.solverKind).toBe(snapshot.capability.solverKind);
    expect(entry.kinematics.capability.positioningDof).toBe(
      snapshot.capability.positioningDof,
    );
    expect(entry.kinematics.capability.supportsOrientation).toBe(
      snapshot.capability.supportsOrientation,
    );
    expect(snapshot.capability.solverKind).toBe('none');
  });

  it('每一条用例的关节集合都恰好是这台机器人的可动关节', () => {
    const want = model.joints.filter((j) => j.type === 'revolute').map((j) => j.id);
    expect(snapshot.fkCases.length).toBeGreaterThan(10);
    for (const c of snapshot.fkCases) {
      expect(Object.keys(c.joints).sort()).toEqual([...want].sort());
    }
  });
});

// ---------------------------------------------------------------------------
// 1. Joint → FK
// ---------------------------------------------------------------------------

describe('Sim2Sim(SO-ARM101) · Joint → FK', () => {
  it(`${snapshot.fkCases.length} 例全部与基线 tcpFrontend 一致`, () => {
    let maxErr = 0;
    let worst = '';
    for (const c of snapshot.fkCases) {
      const got = forwardKinematics(model, c.joints).endEffector.position;
      const err = dist(got, c.tcpFrontend);
      if (err > maxErr) {
        maxErr = err;
        worst = c.id;
      }
    }
    console.log(
      `[sim2sim·so-arm101·FK] N=${snapshot.fkCases.length} · max|Δ| = ` +
        `${maxErr.toExponential(3)} mm（最差 ${worst}；容差 ${TOL_VS_BASELINE_MM}）`,
    );
    expect(maxErr).toBeLessThan(TOL_VS_BASELINE_MM);
  });

  it('抽象层 forward() 与直接 FK 在全部用例上逐位一致', () => {
    let maxErr = 0;
    for (const c of snapshot.fkCases) {
      const viaEngine = entry.kinematics.forward(c.joints).position;
      const direct = forwardKinematics(model, c.joints).endEffector.position;
      maxErr = Math.max(maxErr, dist(viaEngine, direct));
    }
    expect(maxErr).toBe(0);
  });

  it('基线的 tcpRef 与 tcpFrontend 在同一处（两份独立实现互证）', () => {
    // 前端(fk.ts) 与 Python(fkref.py)是两个独立实现，都消费同一份 robot.yaml
    // ⇒ 它们之间只应有浮点噪声。这条**必须**严于 TOL_VS_BASELINE 之外的那条
    // 「参考↔MuJoCo」判据（后者含官方文件精度差，量级 3e-3 mm）。
    let maxErr = 0;
    for (const c of snapshot.fkCases) {
      maxErr = Math.max(maxErr, dist(c.tcpFrontend, c.tcpRef));
    }
    console.log(`[sim2sim·so-arm101·双实现] max|前端 − 参考| = ${maxErr.toExponential(3)} mm`);
    expect(maxErr).toBeLessThan(TOL_VS_BASELINE_MM);
  });
});

// ---------------------------------------------------------------------------
// 2. Joint → Three.js（真实渲染对象）
// ---------------------------------------------------------------------------

describe('Sim2Sim(SO-ARM101) · Joint → Three.js', () => {
  it(`${snapshot.fkCases.length} 例：渲染树的 TCP 矩阵与 FK 基线一致`, () => {
    const objects = buildRobotObject3D(model);
    let maxErr = 0;
    let worst = '';

    try {
      for (const c of snapshot.fkCases) {
        applyJointState(objects, model, c.joints);
        objects.root.updateMatrixWorld(true);

        // ★ 读的是**渲染真实用的矩阵**，不是把 FK 再算一遍。
        //   挂错父节点 / 变换漏乘 / Group 被跳过，都会让这条断言失败。
        const tcp = new THREE.Vector3().setFromMatrixPosition(objects.tcpMarker.matrixWorld);
        const err = Math.hypot(
          tcp.x - c.tcpFrontend[0]!,
          tcp.y - c.tcpFrontend[1]!,
          tcp.z - c.tcpFrontend[2]!,
        );
        if (err > maxErr) {
          maxErr = err;
          worst = c.id;
        }
      }

      console.log(
        `[sim2sim·so-arm101·Three.js] N=${snapshot.fkCases.length} · 末端 max|Δ| = ` +
          `${maxErr.toExponential(3)} mm（最差 ${worst}；容差 ${TOL_THREE_MM}）`,
      );
      expect(maxErr).toBeLessThan(TOL_THREE_MM);
    } finally {
      disposeRobotObject3D(objects);
    }
  });

  it('渲染树为这台机器人建出了与关节数相匹配的 Group（结构不变量）', () => {
    const objects = buildRobotObject3D(model);
    try {
      for (const jid of snapshot.jointOrder) {
        expect(objects.jointGroups.get(jid), `缺少关节 Group: ${jid}`).toBeDefined();
      }
      // MeArm 的关节名不该出现在 SO-101 的树里（切换不彻底的最直接症状）
      for (const mearmOnly of ['base', 'shoulder', 'elbow']) {
        expect(objects.jointGroups.get(mearmOnly)).toBeUndefined();
      }
    } finally {
      disposeRobotObject3D(objects);
    }
  });
});

// ---------------------------------------------------------------------------
// 3. 能力诚实性（**禁止伪造 IK**）
// ---------------------------------------------------------------------------

describe('Sim2Sim(SO-ARM101) · 能力诚实性', () => {
  it('inverse() 对基线的每个探测目标都返回 NOT_IMPLEMENTED 且不给出任何关节角', () => {
    expect(snapshot.ikSupported).toBe(false);
    expect(snapshot.ikCases.length).toBeGreaterThan(0);

    for (const c of snapshot.ikCases) {
      const got = entry.kinematics.inverse(c.target as [number, number, number]);
      expect(got.success, `${c.id} 竟然解出来了 —— 这就是伪造 IK`).toBe(false);
      // 统一 `IKResult` 里错误码字段叫 `error`（不是 MeArm 原生结果里的 `reason`）
      expect(got.error).toBe('NOT_IMPLEMENTED');
      expect(got.joints).toEqual({});
      expect(got.positionError).toBeNull();
      expect(got.constraintsSatisfied).toBe(false);
      expect(got.solutionType).toBe('none');
      // 基线的记录也必须同样是"没算"
      expect(c.outcome).toBe('unavailable');
      expect(c.reason).toBe('NOT_IMPLEMENTED');
      expect(c.joints).toBeNull();
      expect(c.closedLoopMm).toBeNull();
    }
  });

  it('基线里没有工作空间 / 2R 几何数据（没测过的东西不写）', () => {
    const raw = loadSnapshot() as unknown as Record<string, unknown>;
    expect(raw['workspace']).toBeUndefined();
    expect(raw['geometry']).toBeUndefined();
    // 能力声明里的 supportsOrientation 必须是 false：接口目前只收位置目标，
    // 声明 true 等于承诺一个不存在的姿态求解器。
    expect(snapshot.capability.supportsOrientation).toBe(false);
  });

  it('ikNote 明确写出"暂不提供"与原因（UI / 报告要靠它解释"为什么不动"）', () => {
    expect(snapshot.ikNote).toContain('NOT_IMPLEMENTED');
    expect(snapshot.ikNote).toContain('不含任何伪造');
  });

  it('两台机器人的 capability 判据是数据而不是 id 特判', () => {
    // 同一个 `loadRobot` 入口，两台机器人给出不同的 solverKind —— 这就是
    // 「分派收敛到一张表」的可观测证据。
    expect(entry.kinematics.capability.solverKind).not.toBe('analytic');
    expect(snapshot.capability.solverKind).toBe(entry.kinematics.capability.solverKind);
  });
});
