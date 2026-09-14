/**
 * `RobotDefinition` / `KinematicsEngine` / `IKResult` 的单元验收。
 *
 * ## 本文件唯一的验收原则
 *
 * > **抽象层必须与其底层算法逐位一致。**
 *
 * 因此这里的判据是**逐位数值相等**（`Object.is`）与 RobotDefinition 的
 * **引用同一性**（`toBe`，那条 `defineRobot` 确实返回原对象引用，可观测），
 * 而**不是**"误差小于某个容差"。用容差去验一件声称"完全相同"的事，
 * 恰好会把要找的那类缺陷放过去。
 *
 * 一个具体例子：若抽象层把 `joints[].coupling` 在拷贝时漏掉，
 * 平行四连杆失效，末端会偏出**几十毫米** —— 容差判据会抓到它；
 * 但若抽象层把 `gain: -1` 写成 `-0.9999999`，末端只偏 1e-5 mm，
 * 容差判据（1e-3）会**放过**它，而逐位判据不会。
 *
 * ⚠️ 注意区分两种"一致"：
 *   · `defineRobot()` 是**视图**，返回原对象引用 ⇒ 用 `toBe` 断言
 *   · `forward()` / `forwardAll()` 是**每次调用新构造**的结果对象
 *     ⇒ 只能断言**内容逐位相同**，主张"同一对象"是错的
 */
import { describe, expect, it } from 'vitest';
import {
  defineRobot,
  endEffectorPosition,
  forwardKinematics,
  isModel,
  loadRobotModel,
  movableJoints,
  type JointState,
  type RobotModel,
} from '../../src/robot';
// MeArm 的引擎与解析解住在**包内**（Phase 2 步④）
import { createMeArmKinematics } from '../../../robot-package/mearm-v1/kinematics/engine';
import { solveIk } from '../../../robot-package/mearm-v1/kinematics/ik';
import { loadFkCases, loadIkCases } from '../helpers/mearmV1Baseline';

const model = loadRobotModel('mearm-v1');
const def = defineRobot(model);
const engine = createMeArmKinematics(model);

// ---------------------------------------------------------------------------
// 1. RobotDefinition：是"命名视图"，不是"新数据结构"
// ---------------------------------------------------------------------------

describe('RobotDefinition · 零转换零拷贝', () => {
  it('metadata 携带 MeArm-V1 标识（spec §3 / 验收 A）', () => {
    expect(def.metadata.id).toBe('mearm');
    expect(def.metadata.name).toBe('mARM');
    expect(def.metadata.model).toBe('MeArm-V1');
    expect(def.metadata.modelVersion).toBe('1.0.0');
    // schema 版本（yaml 顶层 version）与模型语义版本是两回事
    expect(def.metadata.schemaVersion).toBe(1);
    expect(def.metadata.units).toBe('mm');
    expect(isModel(def, 'MeArm-V1')).toBe(true);
    expect(isModel(def, 'MeArm-V2')).toBe(false);
  });

  it('links / joints / actuators / tcp / homePose 与 RobotModel 是**同一批对象**（不是拷贝）', () => {
    // 这条断言的价值：一旦变成拷贝，就会出现"抽象层看的是旧模型"这类
    // 只在热重载 / 换模型时才暴露的幽灵 bug。
    expect(def.links).toBe(model.links);
    expect(def.joints).toBe(model.joints);
    expect(def.actuators).toBe(model.actuators);
    expect(def.tcp).toBe(model.tcp);
    expect(def.homePose).toBe(model.homePose);
    expect(def.robotModel).toBe(model);
  });

  it('limits 是 joints 的**投影**（可动关节子集，限位对象仍是同一引用）', () => {
    expect(def.limits.map((l) => l.id)).toEqual(movableJoints(model).map((j) => j.id));
    for (const entry of def.limits) {
      const joint = model.joints.find((j) => j.id === entry.id)!;
      expect(entry.limits).toBe(joint.limits);
      expect(entry.role).toBe(joint.role);
    }
    // 本机：4 个可动关节（被动腕 tool 不在其中）
    expect(def.limits.map((l) => l.id)).toEqual(['base', 'shoulder', 'elbow', 'gripper']);
  });

  it('defineRobot 不修改输入模型（纯函数）', () => {
    const before = JSON.stringify(model);
    defineRobot(model);
    expect(JSON.stringify(model)).toBe(before);
  });
});

// ---------------------------------------------------------------------------
// 2. KinematicsEngine：纯委托 —— 逐位一致 + 对象同一性
// ---------------------------------------------------------------------------

/**
 * 递归逐位比对（`Object.is`，即"完全相同"而非"接近"）。
 *
 * 为什么不用 `toBe`：`forwardKinematics()` 每次调用都构造**新对象**，
 * 所以"对象同一性"根本不是这里该主张的性质 —— 那是把"没有拷贝"和
 * "内容相同"两件事混为一谈。真正要守的是**内容逐位相同**。
 */
function expectSameNumbers(a: unknown, b: unknown, path = '$'): void {
  if (typeof a === 'number' && typeof b === 'number') {
    expect(Object.is(a, b), `${path}: ${a} vs ${b}`).toBe(true);
    return;
  }
  if (Array.isArray(a) && Array.isArray(b)) {
    expect(a.length, `${path}.length`).toBe(b.length);
    a.forEach((v, i) => expectSameNumbers(v, b[i], `${path}[${i}]`));
    return;
  }
  if (a !== null && b !== null && typeof a === 'object' && typeof b === 'object') {
    const ka = Object.keys(a as object).sort();
    const kb = Object.keys(b as object).sort();
    expect(ka, `${path} keys`).toEqual(kb);
    for (const k of ka) {
      expectSameNumbers((a as Record<string, unknown>)[k], (b as Record<string, unknown>)[k], `${path}.${k}`);
    }
    return;
  }
  expect(a, path).toEqual(b);
}

describe('KinematicsEngine · 与底层算法逐位一致', () => {
  it('capability 如实声明"位置型机构"（spec §12：不把 DOF 写死进接口）', () => {
    expect(engine.capability.positioningDof).toBe(3);
    expect(engine.capability.supportsOrientation).toBe(false);
    expect(engine.capability.solverKind).toBe('analytic');
  });

  it('forward() 与 forwardKinematics().endEffector 逐位一致', () => {
    const state: JointState = { base: 12.5, shoulder: 30, elbow: 120, gripper: 44 };
    expectSameNumbers(engine.forward(state), forwardKinematics(model, state).endEffector);
  });

  it('forwardAll() 与 forwardKinematics() 逐位一致（含全部关节坐标系）', () => {
    const state: JointState = { base: -20, shoulder: 5, elbow: 138, gripper: 0 };
    expectSameNumbers(engine.forwardAll(state), forwardKinematics(model, state));
  });

  it('全部黄金关节用例：forward() 与 endEffectorPosition() 逐位一致', () => {
    const { cases } = loadFkCases();
    expect(cases.length).toBeGreaterThan(0);
    for (const c of cases) {
      const got = engine.forward(c.joints).position;
      const want = endEffectorPosition(model, c.joints);
      expect(Object.is(got[0], want[0]), `${c.id} x`).toBe(true);
      expect(Object.is(got[1], want[1]), `${c.id} y`).toBe(true);
      expect(Object.is(got[2], want[2]), `${c.id} z`).toBe(true);
    }
  });

  it('全部黄金 IK 用例：inverse() 与 solveIk() 逐字段一致', () => {
    const { cases } = loadIkCases();
    expect(cases.length).toBeGreaterThan(0);
    let compared = 0;

    for (const c of cases) {
      const raw = solveIk(model, c.target as [number, number, number]);
      const wrapped = engine.inverse(c.target as [number, number, number]);

      expect(wrapped.success, `${c.id} success`).toBe(raw.success);
      expect(wrapped.constraintsSatisfied, `${c.id} constraintsSatisfied`).toBe(raw.success);
      // 本机没有姿态自由度 ⇒ 永远 null（**不是 0**）
      expect(wrapped.orientationError, `${c.id} orientationError`).toBeNull();

      if (raw.success && wrapped.success) {
        expect(wrapped.solutionType, `${c.id} solutionType`).toBe(raw.branch);
        expect(wrapped.error, `${c.id} error`).toBeUndefined();
        // residual → positionError：逐位一致（不是"接近"）
        expect(Object.is(wrapped.positionError, raw.residual), `${c.id} positionError`).toBe(true);
        expect(Object.keys(wrapped.joints).sort()).toEqual(Object.keys(raw.joints).sort());
        for (const [jid, v] of Object.entries(raw.joints)) {
          expect(Object.is(wrapped.joints[jid], v), `${c.id} joints.${jid}`).toBe(true);
        }
        compared += 1;
      } else if (!raw.success && !wrapped.success) {
        expect(wrapped.solutionType, `${c.id} solutionType`).toBe('none');
        expect(wrapped.error, `${c.id} error`).toBe(raw.reason);
        expect(wrapped.message, `${c.id} message`).toBe(raw.message);
        // 失败时 positionError 必须是 null —— 填 0 会被读成"完美命中"，那是编造
        expect(wrapped.positionError, `${c.id} positionError`).toBeNull();
        expect(wrapped.joints, `${c.id} joints`).toEqual({});
        compared += 1;
      }
    }
    expect(compared).toBe(cases.length);
  });

  it('inverse() 的 near / seed / prefer 透传后仍与 solveIk() 逐位一致', () => {
    const near: JointState = { base: 5, shoulder: 25, elbow: 125, gripper: 50 };
    const seed: JointState = { base: 5, shoulder: 25, elbow: 125, gripper: 77 };
    const target = endEffectorPosition(model, near) as [number, number, number];

    for (const prefer of ['nearest', 'elbow-up', 'elbow-down'] as const) {
      const raw = solveIk(model, target, { prefer, near, seed });
      const wrapped = engine.inverse(target, { prefer, near, seed });
      expect(wrapped.success).toBe(raw.success);
      if (raw.success && wrapped.success) {
        expect(wrapped.solutionType).toBe(raw.branch);
        expect(Object.is(wrapped.positionError, raw.residual)).toBe(true);
        for (const [jid, v] of Object.entries(raw.joints)) {
          expect(Object.is(wrapped.joints[jid], v), `${prefer}.${jid}`).toBe(true);
        }
      } else if (!raw.success) {
        expect(wrapped.error).toBe(raw.reason);
      }
    }
  });

  it('未知的 prefer 必须**显式抛错**，不静默退化', () => {
    // 若强转不校验，ik.ts 会把未知支当作 `elbow-up` —— 上层以为指定了支，实际没有
    expect(() => engine.inverse([120, 0, 90], { prefer: 'elbow_up' })).toThrow(
      /未知的解支偏好/,
    );
  });
});

// ---------------------------------------------------------------------------
// 3. 抽象层不改变模型：往返一致性
// ---------------------------------------------------------------------------

describe('KinematicsEngine · 抽象前后模型不变', () => {
  it('复用同一模型建立多个引擎，definition.robotModel 恒为同一对象', () => {
    const a = createMeArmKinematics();
    const b = createMeArmKinematics();
    expect(a.definition.robotModel).toBe(model);
    expect(b.definition.robotModel).toBe(model);
    expect(a.forward).toBeTypeOf('function');
  });

  it('外部传入另一份 RobotModel 时，引擎消费的是那一份（不是隐藏的全局单例）', () => {
    const other: RobotModel = { ...model };
    const engine2 = createMeArmKinematics(other);
    expect(engine2.definition.robotModel).toBe(other);
    expect(engine2.definition.robotModel).not.toBe(model);
  });

  it('engine.inverse() 的解可直接喂回 engine.forward()（闭环自洽）', () => {
    const target = endEffectorPosition(model, {
      base: 8, shoulder: 22, elbow: 122, gripper: 50,
    }) as [number, number, number];
    const res = engine.inverse(target);
    expect(res.success).toBe(true);
    const back = engine.forward(res.joints).position;
    const err = Math.hypot(back[0] - target[0], back[1] - target[1], back[2] - target[2]);
    expect(err).toBeLessThan(1e-9);
    // 而且这个误差就是 IKResult 自己上报的 positionError
    expect(res.positionError).not.toBeNull();
    expect(Math.abs((res.positionError ?? 0) - err)).toBeLessThan(1e-12);
  });
});
