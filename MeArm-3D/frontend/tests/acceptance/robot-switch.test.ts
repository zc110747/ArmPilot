/**
 * 模型切换验收（Phase 4 · 三维模型切换 / Phase 8 · 切换压力回归）。
 *
 * 单机器人（mearm-v1）下，本文件验证：
 *  - 缺省加载的就是选择器 default；
 *  - 切换守卫（已接传输时拒绝、未知 id 拒绝且不回退）仍然成立；
 *  - 切到同一台是幂等的（不复位姿态）；
 *  - 重复 setRobot 不漂移（Phase 8 压力回归的退化形态）。
 *
 * 注：多机器人"几何完全不同"的切换对照，依赖第二台机器人存在；
 * 当前仓库只有 mearm-v1，故这里只守"单机器人下的切换机制不退化"。
 * 一旦新增第二台机器人，本文件应补回"切到另一台 ⇒ 关节集合整体替换"的用例。
 */
import { beforeEach, describe, expect, it } from 'vitest';
import {
  MEARM_V1_ROBOT_ID,
  assertRegistryCoverage,
  endEffectorPose,
  homeJointState,
  jointIds,
  loadRobot,
  loadRobotModel,
} from '@robot/index';
import { useRobotStore } from '@/store/robotStore';

const MEARM = loadRobotModel(MEARM_V1_ROBOT_ID);

/** 与"当前活动模型"绑定的状态字段（`log` / 相机 / 开关等与模型无关，不参与比对） */
const MODEL_BOUND_KEYS = [
  'robotId',
  'commandJoints',
  'actualJoints',
  'endEffector',
  'actualEndEffector',
  'target',
  'ikStatus',
  'alignmentErrorMm',
] as const;

function snapshot() {
  const s = useRobotStore.getState();
  return JSON.parse(JSON.stringify(Object.fromEntries(MODEL_BOUND_KEYS.map((k) => [k, s[k]]))));
}

/** 把 store 复位到「MeArm + HOME + 未接传输」——切换的守卫依赖 `transportDriven` */
function resetStore(): void {
  const home = homeJointState(MEARM);
  const pose = endEffectorPose(MEARM, home);
  useRobotStore.setState({
    robotId: MEARM_V1_ROBOT_ID,
    model: MEARM,
    commandJoints: { ...home },
    actualJoints: { ...home },
    endEffector: pose,
    actualEndEffector: pose,
    target: [pose.position[0], pose.position[1], pose.position[2]],
    transportKind: null,
    transportDriven: false,
    connection: 'disconnected',
    mode: 'simulation',
    alignmentErrorMm: null,
    ikStatus: null,
    log: [],
  });
}

/** 每个关节的值是否都落在**该模型自己的**限位内 —— 独立于 store 的判据 */
function outOfLimitJoints(
  model: ReturnType<typeof loadRobotModel>,
  joints: Record<string, number>,
): string[] {
  const bad: string[] = [];
  for (const id of jointIds(model)) {
    const joint = model.joints.find((j) => j.id === id);
    if (!joint) continue;
    const v = joints[id];
    if (v === undefined) {
      bad.push(`${id}:缺失`);
      continue;
    }
    const eps = 1e-6 * Math.max(1, Math.abs(joint.limits.min), Math.abs(joint.limits.max));
    if (v < joint.limits.min - eps || v > joint.limits.max + eps) {
      bad.push(`${id}:${v} ∉ [${joint.limits.min}, ${joint.limits.max}]`);
    }
  }
  return bad;
}

beforeEach(() => {
  resetStore();
});

describe('Phase 4 · 活动机器人', () => {
  it('初值是选择器的 default，且与「缺省加载」是同一个对象', () => {
    const s = useRobotStore.getState();
    expect(s.robotId).toBe(MEARM_V1_ROBOT_ID);
    // `loadRobotModel` 按 id 缓存 ⇒ 引用相等。用 `toBe` 而不是 `toEqual`：
    // 若哪天变成"每次 new 一个"，切换压力回归会因对象身份漂移而失去意义。
    expect(s.model).toBe(MEARM);
  });

  it('选择器声明的每台机器人都有引擎实现（配置 ↔ 代码一一对应）', () => {
    expect(assertRegistryCoverage()).toEqual({ ok: true, missing: [] });
  });

  it('切到同一台是幂等的 —— 不会把姿态复位掉', () => {
    const base = MEARM.joints.find((j) => j.role === 'base')!;
    const moved = base.limits.min + 5;
    useRobotStore.getState().setJoint(base.id, moved);

    const result = useRobotStore.getState().setRobot(MEARM_V1_ROBOT_ID);
    expect(result.ok).toBe(true);
    expect(useRobotStore.getState().commandJoints[base.id]).toBeCloseTo(moved, 9);
  });

  it('切换后 goHome / setJoint 走本模型的 homePose 与限位', () => {
    useRobotStore.getState().setRobot(MEARM_V1_ROBOT_ID);

    useRobotStore.getState().goHome();
    expect(outOfLimitJoints(MEARM, useRobotStore.getState().commandJoints)).toEqual([]);

    const base = MEARM.joints.find((j) => j.role === 'base')!;
    useRobotStore.getState().setJoint(base.id, 1e6);
    expect(useRobotStore.getState().commandJoints[base.id]).toBeCloseTo(base.limits.max, 6);
    useRobotStore.getState().setJoint(base.id, -1e6);
    expect(useRobotStore.getState().commandJoints[base.id]).toBeCloseTo(base.limits.min, 6);
  });

  it('MeArm 的 moveTo 走解析解（能力门没有误伤 Golden Baseline）', () => {
    const mearm = loadRobot(MEARM_V1_ROBOT_ID);
    expect(mearm.kinematics.capability.solverKind).toBe('analytic');

    useRobotStore.getState().goHome();
    const home = useRobotStore.getState().endEffector.position;
    const result = useRobotStore.getState().moveTo([home[0], home[1], home[2]]);
    expect(result.success).toBe(true);
  });
});

describe('Phase 4 · 切换的守卫（拒绝而不是静默）', () => {
  // ⚠️ 「已接入传输时拒绝切换」守卫在**单机器人模式下无法被触发**：
  //   守卫针对的是「在驱动链路上把当前模型换成**另一台**机器人」（换模型 = 换限位/标定，
  //   会悄悄改变能发给真机的指令）。单机器人模式下能调用的只有 setRobot(mearm-v1)，
  //   而它就是当前活动模型 ⇒ 命中 setRobot 的**幂等分支**（ok:true，不动任何状态），
  //   根本走不到 transportDriven 守卫。该守卫的回归留给「存在第二台机器人」的环境
  //   （切 A→B 且已连接时应被拒）。见 frontend/src/store/robotStore.ts 的
  //   setRobot：幂等分支在前，transportDriven 守卫在后。

  it('未知 id 拒绝且**不回退**到缺省模型', () => {
    const before = snapshot();
    const result = useRobotStore.getState().setRobot('does-not-exist');

    expect(result.ok).toBe(false);
    expect(useRobotStore.getState().robotId).toBe(MEARM_V1_ROBOT_ID);
    expect(useRobotStore.getState().model).toBe(MEARM);
    expect(snapshot()).toEqual(before);
  });
});

describe('Phase 8 · 切换压力回归', () => {
  it('重复 200 轮 setRobot(mearm-v1) 后状态与初始**逐位相同**（无漂移）', () => {
    const initial = snapshot();

    for (let i = 0; i < 200; i += 1) {
      const a = useRobotStore.getState().setRobot(MEARM_V1_ROBOT_ID);
      expect(a.ok).toBe(true);
      const s1 = useRobotStore.getState();
      expect(s1.model).toBe(MEARM);
      expect(Object.keys(s1.commandJoints).sort()).toEqual(jointIds(MEARM).slice().sort());
      expect(outOfLimitJoints(MEARM, s1.commandJoints)).toEqual([]);
    }

    // 换回去必须与最开始**完全相同** —— 这条能抓住"某轮之后残留了另一台的状态"
    expect(snapshot()).toEqual(initial);
  });

  it('压力回归期间日志不会无限增长（pushLog 有上限）', () => {
    for (let i = 0; i < 60; i += 1) {
      useRobotStore.getState().setRobot(MEARM_V1_ROBOT_ID);
    }
    expect(useRobotStore.getState().log.length).toBeLessThanOrEqual(200);
  });
});
