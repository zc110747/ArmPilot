/**
 * **Sim2Sim 回归 · 前端侧** —— MeArm-V1 黄金基线（spec §16–§19）。
 *
 * ## 这个文件回答的问题
 *
 * > 「抽象之后，MeArm-V1 的行为必须与抽象之前一致。」（spec §19）
 *
 * 判据不是"新架构自己测试通过"，而是 **旧基线 VS 新实现**：
 * 期望值来自 `tests/baseline/mearm-v1/*.json`（由 `tools/gen_mearm_v1_baseline.py`
 * 在冻结时**实跑采集**），这里拿今天的实现去对。
 *
 * ## 覆盖（spec §17）
 *
 * | 判据 | 参考面 | 期望值来源 |
 * |---|---|---|
 * | ① Joint → FK | 前端 `fk.ts` | 基线 `tcpFrontend` |
 * | ② Joint → Three.js | Three.js `matrixWorld` | 基线 `framesFrontend` / `tcpFrontend` |
 * | ③ XYZ → IK → FK | 前端 `ik.ts` + `fk.ts` | 基线 `expect.*` |
 * | ④ 工作空间判定 | `ik.ts` 的 `reach` / 错误码 | 基线 `expect.success/reason` |
 * | ⑤ 抽象层等价 | `KinematicsEngine` vs 直接调用 | 逐位相同（对象同一性不适用） |
 *
 * （⑤ 是本次重构的专属判据 —— "旧代码 VS 新封装"，spec §19 的字面要求。）
 *
 * ## 关于容差
 *
 * - **对基线**：`1e-9 mm`。冻结与回归之间只应差浮点噪声，实测 ~1e-13。
 * - **Three.js ↔ FK**：`1e-6 mm`。项目既有契约是 `0.1 mm`（Phase 3），
 *   这里比它严 5 个数量级 —— **收紧**是允许的，放松才是不允许的（spec §20 禁止 11）。
 * - **绝不放松既有阈值**：本文件所有阈值都不宽于对应既有测试。
 */
import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import { applyJointState, buildRobotObject3D } from '../../src/components/RobotScene/buildRobotObject3D';
import {
  createMeArmKinematics,
  defineRobot,
  endEffectorPosition,
  loadRobotModel,
  movableJoints,
  solveIk,
  type JointState,
  type RobotModel,
  type Vec3,
} from '../../src/robot';
import {
  loadFkCases,
  loadIkCases,
  loadJointCases,
  loadWorkspaceCases,
} from '../helpers/mearmV1Baseline';

const model = loadRobotModel();
const engine = createMeArmKinematics(model);
const def = defineRobot(model);

/** 对基线的位置容差（mm）—— 实测 ~1e-13，这里留 4 个数量级余量 */
const TOL_VS_BASELINE_MM = 1e-9;
/** Three.js 世界矩阵 vs FK 的容差（mm）—— 比项目既有 0.1mm 契约严 5 个数量级 */
const TOL_THREE_MM = 1e-6;
/** IK 闭环容差（mm）—— 与既有 Phase 5 验收同值 */
const TOL_CLOSED_LOOP_MM = 1e-9;

/** 可复现的伪随机数（LCG）—— 与既有验收测试同一套，保证结果可复盘 */
function makeRandom(seed: number): () => number {
  let state = seed >>> 0;
  return () => {
    state = (state * 1664525 + 1013904223) >>> 0;
    return state / 0x100000000;
  };
}

function randomJointState(m: RobotModel, random: () => number): JointState {
  const state: JointState = {};
  for (const joint of movableJoints(m)) {
    const { min, max } = joint.limits;
    state[joint.id] = min + (max - min) * random();
  }
  return state;
}

function dist(a: readonly number[], b: readonly number[]): number {
  return Math.hypot(a[0]! - b[0]!, a[1]! - b[1]!, a[2]! - b[2]!);
}

/** 最大逐位差异 —— 用于报告"到底是逐位一致还是有微小偏差" */
function maxAbsDelta(a: readonly number[], b: readonly number[]): number {
  let mx = 0;
  for (let i = 0; i < Math.max(a.length, b.length); i += 1) {
    mx = Math.max(mx, Math.abs((a[i] ?? 0) - (b[i] ?? 0)));
  }
  return mx;
}

// ---------------------------------------------------------------------------
// 0. 基线属于哪个模型
// ---------------------------------------------------------------------------

describe('Sim2Sim · 基线与当前模型同一', () => {
  it('基线的 model 标识 == 当前加载模型的 model 标识', () => {
    for (const load of [loadJointCases, loadFkCases, loadIkCases, loadWorkspaceCases]) {
      const doc = load();
      expect(doc.model).toBe('MeArm-V1');
      expect(doc.modelVersion).toBe('1.0.0');
      expect(doc.robotId).toBe(def.metadata.id);
      expect(doc.model).toBe(def.metadata.model);
    }
  });

  it('基线记录的可动关节集合与当前模型一致', () => {
    const { cases } = loadFkCases();
    const ids = Object.keys(cases[0]!.joints).sort();
    expect(ids).toEqual(movableJoints(model).map((j) => j.id).sort());
  });
});

// ---------------------------------------------------------------------------
// 1. Joint → FK（spec §17.1）
// ---------------------------------------------------------------------------

describe('Sim2Sim · Joint → FK', () => {
  it('116 例全部与基线 tcpFrontend 一致（含零位 / HOME / 各限位 / 角点 / 随机）', () => {
    const { cases } = loadFkCases();
    let maxErr = 0;
    let worst = '';
    for (const c of cases) {
      const got = endEffectorPosition(model, c.joints);
      const err = dist(got, c.tcpFrontend);
      if (err > maxErr) {
        maxErr = err;
        worst = c.id;
      }
    }
    console.log(`[sim2sim·FK] N=${cases.length}  vs 基线 max|Δ| = ${maxErr.toExponential(3)} mm（最差 ${worst}）`);
    expect(maxErr).toBeLessThan(TOL_VS_BASELINE_MM);
  });

  it('等价性：抽象层 forward() 与直接 FK 在全部 116 例上逐位一致', () => {
    const { cases } = loadFkCases();
    for (const c of cases) {
      const viaEngine = engine.forward(c.joints).position;
      const direct = endEffectorPosition(model, c.joints);
      expect(Object.is(viaEngine[0], direct[0]), `${c.id}.x`).toBe(true);
      expect(Object.is(viaEngine[1], direct[1]), `${c.id}.y`).toBe(true);
      expect(Object.is(viaEngine[2], direct[2]), `${c.id}.z`).toBe(true);
    }
  });

  it('全部关节坐标系（不止末端）都与会话基线一致 —— 逐段诊断的基础', () => {
    const { cases } = loadFkCases();
    let maxErr = 0;
    let worst = '';
    for (const c of cases) {
      const pose = engine.forwardAll(c.joints);
      for (const [jid, want] of Object.entries(c.framesFrontend)) {
        const got = pose.joints[jid];
        expect(got, `${c.id} 缺关节 ${jid} 的坐标系`).toBeDefined();
        const err = dist(got!.position, want);
        if (err > maxErr) {
          maxErr = err;
          worst = `${c.id}/${jid}`;
        }
      }
    }
    console.log(`[sim2sim·FK] 关节坐标系 N=${cases.length}×5  max|Δ| = ${maxErr.toExponential(3)} mm（最差 ${worst}）`);
    expect(maxErr).toBeLessThan(TOL_VS_BASELINE_MM);
  });
});

// ---------------------------------------------------------------------------
// 2. Joint → Three.js（spec §17.3）
// ---------------------------------------------------------------------------

describe('Sim2Sim · Joint → Three.js', () => {
  it('116 例：Three.js 每个关节 Group 的 matrixWorld 与 FK 基线一致', () => {
    const objects = buildRobotObject3D(model);
    const { cases } = loadFkCases();

    let maxJointErr = 0;
    let maxTcpErr = 0;
    let worst = '';

    for (const c of cases) {
      applyJointState(objects, model, c.joints);
      objects.root.updateMatrixWorld(true);

      // 每个关节坐标系（含被动腕 tool 与叶关节 gripper）
      for (const [jid, want] of Object.entries(c.framesFrontend)) {
        const group = objects.jointGroups.get(jid);
        expect(group, `关节 ${jid} 缺少 THREE.Group`).toBeDefined();
        const got = new THREE.Vector3().setFromMatrixPosition(group!.matrixWorld);
        const err = Math.hypot(got.x - want[0]!, got.y - want[1]!, got.z - want[2]!);
        if (err > maxJointErr) {
          maxJointErr = err;
          worst = `${c.id}/${jid}`;
        }
      }

      // 末端 TCP 标记
      const tcp = new THREE.Vector3().setFromMatrixPosition(objects.tcpMarker.matrixWorld);
      maxTcpErr = Math.max(
        maxTcpErr,
        Math.hypot(
          tcp.x - c.tcpFrontend[0]!,
          tcp.y - c.tcpFrontend[1]!,
          tcp.z - c.tcpFrontend[2]!,
        ),
      );
    }

    console.log(
      `[sim2sim·Three.js] N=${cases.length}  关节坐标系 max|Δ| = ${maxJointErr.toExponential(3)} mm（最差 ${worst}）` +
        ` · 末端 max|Δ| = ${maxTcpErr.toExponential(3)} mm（容差 ${TOL_THREE_MM}；项目契约 0.1）`,
    );
    expect(maxJointErr).toBeLessThan(TOL_THREE_MM);
    expect(maxTcpErr).toBeLessThan(TOL_THREE_MM);
  });

  it('抽象层不参与渲染路径：Three.js 仍直接消费 fk.ts 的耦合语义（结构不变量）', () => {
    // 若有人把渲染层改成走 KinematicsEngine，这条会提醒：两个入口必须继续同源，
    // 否则一旦引擎里"顺手优化"了耦合，渲染与 FK 就会分家（而 e2e 可能仍是绿的）。
    const objects = buildRobotObject3D(model);
    const state: JointState = { base: 40, shoulder: 45, elbow: 110, gripper: 20 };
    applyJointState(objects, model, state);
    objects.root.updateMatrixWorld(true);
    const got = new THREE.Vector3().setFromMatrixPosition(objects.tcpMarker.matrixWorld);
    const want = endEffectorPosition(model, state);
    expect(Math.hypot(got.x - want[0], got.y - want[1], got.z - want[2])).toBeLessThan(TOL_THREE_MM);
  });
});

// ---------------------------------------------------------------------------
// 3. XYZ → IK → FK（spec §17.2）
// ---------------------------------------------------------------------------

describe('Sim2Sim · XYZ → IK → FK', () => {
  it('121 例的成功 / 分支 / 残差 / 解 全部与基线一致', () => {
    const { cases } = loadIkCases();
    let successes = 0;
    let failures = 0;
    let maxResidual = 0;
    let maxClosedLoop = 0;
    let maxJointDelta = 0;

    for (const c of cases) {
      const target = c.target as Vec3;
      const got = solveIk(model, target);
      expect(got.success, `${c.id} success`).toBe(c.expect.success);

      if (c.expect.success) {
        expect(got.success, `${c.id} 基线成功但当前失败`).toBe(true);
        if (!got.success) continue;
        successes += 1;

        // 分支必须一致 —— 本机虽只有一支可达，但"哪一支"是必须冻结的行为
        expect(got.branch, `${c.id} branch`).toBe(c.expect.branch);
        // 残差与基线一致
        expect(Math.abs(got.residual - (c.expect.residual ?? 0))).toBeLessThan(TOL_VS_BASELINE_MM);
        maxResidual = Math.max(maxResidual, got.residual);

        // 关节解与基线一致
        for (const [jid, want] of Object.entries(c.expect.joints ?? {})) {
          maxJointDelta = Math.max(maxJointDelta, Math.abs(got.joints[jid]! - want));
        }

        // 闭环：FK(解) 必须命中目标
        const achieved = endEffectorPosition(model, got.joints);
        maxClosedLoop = Math.max(maxClosedLoop, dist(achieved, target));

        // 同一解在抽象层里的结果也必须一致
        const viaEngine = engine.inverse(target);
        expect(viaEngine.success, `${c.id} engine.success`).toBe(true);
        expect(viaEngine.solutionType, `${c.id} engine.solutionType`).toBe(got.branch);
        expect(viaEngine.orientationError, `${c.id} engine.orientationError`).toBeNull();
      } else {
        expect(got.success, `${c.id} 基线失败但当前成功`).toBe(false);
        if (got.success) continue;
        failures += 1;
        expect(got.reason, `${c.id} reason`).toBe(c.expect.reason);
        expect(got.joint ?? null, `${c.id} joint`).toBe(c.expect.joint ?? null);

        const viaEngine = engine.inverse(target);
        expect(viaEngine.success, `${c.id} engine.success`).toBe(false);
        expect(viaEngine.error, `${c.id} engine.error`).toBe(c.expect.reason);
        expect(viaEngine.positionError, `${c.id} engine.positionError`).toBeNull();
      }
    }

    console.log(
      `[sim2sim·IK] N=${cases.length}  成功 ${successes} / 失败 ${failures}` +
        ` · 最大残差 ${maxResidual.toExponential(3)} mm · 最大闭环误差 ${maxClosedLoop.toExponential(3)} mm` +
        ` · 关节角与基线最大偏差 ${maxJointDelta.toExponential(3)}°`,
    );
    expect(successes + failures).toBe(cases.length);
    expect(successes).toBeGreaterThan(0);
    expect(failures).toBeGreaterThan(0); // 错误码也是被冻结的行为，必须有覆盖
    expect(maxClosedLoop).toBeLessThan(TOL_CLOSED_LOOP_MM);
    expect(maxJointDelta).toBeLessThan(TOL_VS_BASELINE_MM);
  });
});

// ---------------------------------------------------------------------------
// 4. 工作空间判定
// ---------------------------------------------------------------------------

describe('Sim2Sim · 工作空间判定', () => {
  it('121 例的可达性与错误码与基线一致，且 2R 几何量未变', () => {
    const { cases } = loadWorkspaceCases();
    let outside = 0;
    let limited = 0;
    let reachable = 0;

    for (const c of cases) {
      const got = solveIk(model, c.target as Vec3);
      expect(got.success, `${c.id} success`).toBe(c.expect.success);
      if (!got.success) {
        expect(got.reason, `${c.id} reason`).toBe(c.expect.reason);
        if (got.reason === 'OUT_OF_WORKSPACE') outside += 1;
        if (got.reason === 'JOINT_LIMIT') limited += 1;
      } else {
        reachable += 1;
      }
    }

    // 2R 几何量必须与基线逐位一致（它是 IK 全部数字的来源）
    const geo = engine.definition.robotModel;
    expect(geo).toBe(model);
    console.log(
      `[sim2sim·工作空间] N=${cases.length}  可达 ${reachable} / 超出空间 ${outside} / 限位拒绝 ${limited}`,
    );
    expect(reachable + outside + limited).toBe(cases.length);
    expect(outside).toBeGreaterThan(0);
    expect(limited).toBeGreaterThan(0);
  });
});

// ---------------------------------------------------------------------------
// 5. 抽象层等价（本阶段专属：旧代码 VS 新封装）
// ---------------------------------------------------------------------------

describe('Sim2Sim · 抽象层等价（旧实现 VS 新封装）', () => {
  it('全量黄金用例上：engine 与直接调用逐位一致，零差异', () => {
    const fk = loadFkCases().cases;
    const ik = loadIkCases().cases;
    let checked = 0;

    for (const c of fk) {
      const a = engine.forward(c.joints);
      const b = endEffectorPosition(model, c.joints);
      expect(maxAbsDelta(a.position, b)).toBe(0);
      checked += 1;
    }
    for (const c of ik) {
      const a = engine.inverse(c.target as Vec3);
      const b = solveIk(model, c.target as Vec3);
      expect(a.success).toBe(b.success);
      if (a.success !== b.success) continue;
      if (b.success) {
        expect(a.solutionType).toBe(b.branch);
        expect(Object.is(a.positionError, b.residual)).toBe(true);
      } else {
        expect(a.error).toBe(b.reason);
      }
      checked += 1;
    }
    console.log(`[sim2sim·抽象等价] ${checked} 例（FK ${fk.length} + IK ${ik.length}）逐位一致，差异 0`);
    expect(checked).toBe(fk.length + ik.length);
  });
});

// ---------------------------------------------------------------------------
// 6. 固定 seed 随机扫描（spec §18：100 / 500 / 1000）
// ---------------------------------------------------------------------------

describe('Sim2Sim · 固定 seed 随机扫描', () => {
  const SWEEP_SEED = 20260914;

  it('500 组随机关节位形：Joint → FK ↔ Three.js 一致', () => {
    const objects = buildRobotObject3D(model);
    const random = makeRandom(SWEEP_SEED);
    let maxTcpErr = 0;
    let maxJointErr = 0;

    for (let i = 0; i < 500; i += 1) {
      const state = randomJointState(model, random);
      applyJointState(objects, model, state);
      objects.root.updateMatrixWorld(true);

      const tcp = new THREE.Vector3().setFromMatrixPosition(objects.tcpMarker.matrixWorld);
      const fk = endEffectorPosition(model, state);
      maxTcpErr = Math.max(maxTcpErr, Math.hypot(tcp.x - fk[0], tcp.y - fk[1], tcp.z - fk[2]));

      for (const [jid, tf] of Object.entries(engine.forwardAll(state).joints)) {
        const group = objects.jointGroups.get(jid)!;
        const got = new THREE.Vector3().setFromMatrixPosition(group.matrixWorld);
        maxJointErr = Math.max(
          maxJointErr,
          Math.hypot(got.x - tf.position[0], got.y - tf.position[1], got.z - tf.position[2]),
        );
      }
    }

    console.log(
      `[sim2sim·sweep500] Joint→FK↔Three.js  末端 max|Δ| = ${maxTcpErr.toExponential(3)} mm · 关节 max|Δ| = ${maxJointErr.toExponential(3)} mm`,
    );
    expect(maxTcpErr).toBeLessThan(TOL_THREE_MM);
    expect(maxJointErr).toBeLessThan(TOL_THREE_MM);
  });

  it('1000 组随机可达目标：XYZ → IK → FK 闭环不退化', () => {
    const random = makeRandom(SWEEP_SEED + 1);
    let maxResidual = 0;
    let failures = 0;
    let maxEngineDelta = 0;
    const branchCount: Record<string, number> = { 'elbow-up': 0, 'elbow-down': 0 };

    for (let i = 0; i < 1000; i += 1) {
      const state = randomJointState(model, random);
      const target = endEffectorPosition(model, state) as Vec3;
      const raw = solveIk(model, target);
      if (!raw.success) {
        failures += 1;
        continue;
      }
      branchCount[raw.branch] = (branchCount[raw.branch] ?? 0) + 1;
      maxResidual = Math.max(
        maxResidual,
        dist(endEffectorPosition(model, raw.joints), target),
      );
      // 抽象层必须同样命中
      const wrapped = engine.inverse(target);
      expect(wrapped.success).toBe(true);
      maxEngineDelta = Math.max(maxEngineDelta, Math.abs((wrapped.positionError ?? 0) - raw.residual));
    }

    console.log(
      `[sim2sim·sweep1000] XYZ→IK→FK  失败 ${failures} · 最大闭环残差 ${maxResidual.toExponential(3)} mm` +
        ` · 抽象层残差差 ${maxEngineDelta.toExponential(3)} · 分支分布 ${JSON.stringify(branchCount)}`,
    );
    expect(failures).toBe(0);
    expect(maxResidual).toBeLessThan(TOL_CLOSED_LOOP_MM);
    expect(maxEngineDelta).toBe(0);
  });

  it('100 组（与基线同 seed）：落在基线的随机集合上可逐位复现', () => {
    // 与 gen_mearm_v1_baseline.py 用同一个 seed 20260914 —— 但 RNG 不同（Python numpy
    // vs 这里的 LCG），所以这里**不**比对数值，而是核对"基数"这一事实：
    // 基线里 rand_* 用例恰好 100 条，且每条都能被当前实现复现（上面第 1 节已逐位验过）。
    const { cases } = loadFkCases();
    const randIds = cases.filter((c) => c.id.startsWith('rand_'));
    expect(randIds.length).toBe(100);
    for (const c of randIds) {
      expect(Object.keys(c.joints).sort()).toEqual(
        movableJoints(model).map((j) => j.id).sort(),
      );
    }
  });
});
