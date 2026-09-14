/**
 * 模型切换验收（Phase 4 · 三维模型切换 / Phase 8 · 切换压力回归）。
 *
 * ## 本文件要回答的问题
 *
 * 「在**完全不修改 MeArm-V1 核心模型**的前提下，能否把活动机器人换成另一台
 *  **几何/关节数/关节名/限位全都不同**的机器人，并让上层状态（关节、末端、目标、
 *  IK 状态）随之整体切换、且不残留旧模型的任何东西？」
 *
 * ## 判据为什么不是"再算一遍"
 *
 * 本文件刻意**不去**重新推导 FK / 限位（那是自证：把 store 用的同一个函数再跑一次，
 * 无论 store 用的是对模型还是错模型都会相等）。用的是三类**外部可对照**的判据：
 *
 * | 判据 | 为什么它独立 |
 * |---|---|
 * | `Object.keys(commandJoints)` === `jointIds(新模型)` | 关节名集合由**各自 yaml 独立加载**得来，不经过 store |
 * | 每个关节值 ∈ 新模型自己的 `limits` | 限位是**独立加载**的常量；旧模型的限位不同 ⇒ 用错模型必被发现 |
 * | 切换 A→B→A 后与初始快照**逐位相同** | 只依赖"两次相同输入应产出相同结果"，与算法无关 |
 *
 * 第 2 条是核心：`mearm-v1` 的肩限位是 0..180 一类的**舵机行程**，
 * 而 `so-arm101` 的是 **−100..100 的关节角**（`unit: joint`）。
 * 两者区间不重叠 ⇒ "用旧模型的限位去裁新模型的关节"不可能蒙混过关。
 */
import { beforeEach, describe, expect, it } from 'vitest';
import {
  MEARM_V1_ROBOT_ID,
  SO_ARM101_ROBOT_ID,
  assertRegistryCoverage,
  endEffectorPose,
  homeJointState,
  jointIds,
  loadRobot,
  loadRobotModel,
} from '@robot/index';
import { useRobotStore } from '@/store/robotStore';

const MEARM = loadRobotModel(MEARM_V1_ROBOT_ID);
const SO101 = loadRobotModel(SO_ARM101_ROBOT_ID);

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
    // 浮点容差：限位本身是 6 位有效数字截断的产物（见 SOURCE.md §4.2）
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

describe('Phase 4 · 活动机器人的切换', () => {
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

  it('切到 so-arm101：关节集合换成新模型的，MeArm 独有的键**整个消失**', () => {
    const before = useRobotStore.getState();
    expect(Object.keys(before.commandJoints).sort()).toEqual(jointIds(MEARM).slice().sort());

    const result = useRobotStore.getState().setRobot(SO_ARM101_ROBOT_ID);
    expect(result).toMatchObject({ ok: true, robotId: SO_ARM101_ROBOT_ID });

    const after = useRobotStore.getState();
    expect(after.model).toBe(SO101);
    // ★ 关节名集合必须**恰好**是新模型的
    expect(Object.keys(after.commandJoints).sort()).toEqual(jointIds(SO101).slice().sort());
    expect(Object.keys(after.actualJoints).sort()).toEqual(jointIds(SO101).slice().sort());

    // ★ 直接证据：MeArm 独有的关节在切换后**键不存在**（不是"值被改了"，是整个键消失）
    for (const id of jointIds(MEARM)) {
      if (jointIds(SO101).includes(id)) continue;
      expect(after.commandJoints).not.toHaveProperty(id);
      expect(after.actualJoints).not.toHaveProperty(id);
    }
    // 两台模型的关节集合本身必须不同，否则上面那条循环是空转
    expect(jointIds(MEARM)).not.toEqual(jointIds(SO101));
  });

  it('★ `gripper` 是两台**同名但不同义**的关节 —— 键集合相等对它无效，只有限位能兜住', () => {
    const shared = jointIds(MEARM).filter((id) => jointIds(SO101).includes(id));
    // 这是事实、也是隐患：唯一同名的那一个关节，其语义与限位都不同。
    // 断言下来是为了拦住"以后有人把 jointIds 相等当成模型一致的判据"。
    expect(shared).toEqual(['gripper']);

    const a = MEARM.joints.find((j) => j.id === 'gripper')!;
    const b = SO101.joints.find((j) => j.id === 'gripper')!;
    expect([a.limits.min, a.limits.max]).not.toEqual([b.limits.min, b.limits.max]);
    // 两台模型各自都有一个"限位最宽的定位关节"（MeArm 的小臂绝对角 vs SO-101 的腕自转），
    // 它们的区间**不重叠** —— 这正是"用错模型的限位去裁关节"必被抓到的原因。
    const mearmElbow = MEARM.joints.find((j) => j.role === 'elbow')!;
    const soWristRoll = SO101.joints.find((j) => j.id === 'wrist_roll')!;
    expect(soWristRoll.limits.min).toBeLessThan(soWristRoll.limits.max);
    expect(mearmElbow.limits.min).toBeGreaterThan(SO101.joints.find((j) => j.id === 'elbow_flex')!.limits.max);
  });

  it('切换后的关节值都在**新模型**的限位内', () => {
    // 先把 MeArm 摆到一个远离 HOME 的姿态，再切 —— 若切换没有整体复位，
    // 这些旧模型关节空间里的值会被带过去，下面立刻报出来。
    const mearmBase = MEARM.joints.find((j) => j.role === 'base')!;
    useRobotStore.getState().setJoint(mearmBase.id, mearmBase.limits.max);
    const carried = useRobotStore.getState().commandJoints[mearmBase.id];

    useRobotStore.getState().setRobot(SO_ARM101_ROBOT_ID);
    const s = useRobotStore.getState();
    expect(outOfLimitJoints(SO101, s.commandJoints)).toEqual([]);
    expect(outOfLimitJoints(SO101, s.actualJoints)).toEqual([]);
    // MeArm 的 base 在这个姿态下是**满量程**的 60°，而 SO-101 的 pan 区间虽更宽（±110），
    // 语义完全不同 ⇒ 判据不是"数值越界"，而是"这个键压根不该存在"。
    expect(s.commandJoints).not.toHaveProperty(mearmBase.id);
    expect(typeof carried).toBe('number');
  });

  it('切换后 goHome / setJoint 走的是新模型的 homePose 与限位', () => {
    useRobotStore.getState().setRobot(SO_ARM101_ROBOT_ID);

    useRobotStore.getState().goHome();
    expect(outOfLimitJoints(SO101, useRobotStore.getState().commandJoints)).toEqual([]);

    const pan = SO101.joints.find((j) => j.id === 'shoulder_pan')!;
    useRobotStore.getState().setJoint('shoulder_pan', 1e6);
    expect(useRobotStore.getState().commandJoints['shoulder_pan']).toBeCloseTo(pan.limits.max, 6);
    useRobotStore.getState().setJoint('shoulder_pan', -1e6);
    expect(useRobotStore.getState().commandJoints['shoulder_pan']).toBeCloseTo(pan.limits.min, 6);
  });

  it('★ 不伪造 IK：SO-101 的 moveTo 返回 NO_SOLVER，且**关节一位都不动**', () => {
    useRobotStore.getState().setRobot(SO_ARM101_ROBOT_ID);
    const before = useRobotStore.getState().commandJoints;

    // 随手取一个"看起来合理"的目标点。MeArm 的解析解会**成功**返回一组角
    // （把 2R 几何套在 6 铰链机上），所以这条断言真正测的是"能力门有没有生效"。
    const result = useRobotStore.getState().moveTo([200, 0, 150]);

    expect(result.success).toBe(false);
    expect(result.success === false && result.reason).toBe('NO_SOLVER');
    expect(useRobotStore.getState().commandJoints).toEqual(before);
    expect(useRobotStore.getState().ikStatus).toMatchObject({ ok: false, reason: 'NO_SOLVER' });
    // 目标**保留**下来（与"越界也保留 target"同一取向：让用户看到我想去哪）
    expect(useRobotStore.getState().target).toEqual([200, 0, 150]);
  });

  it('MeArm 自身的 moveTo 仍然走解析解（能力门没有误伤 Golden Baseline）', () => {
    const mearm = loadRobot(MEARM_V1_ROBOT_ID);
    expect(mearm.kinematics.capability.solverKind).toBe('analytic');

    useRobotStore.getState().goHome();
    const home = useRobotStore.getState().endEffector.position;
    const result = useRobotStore.getState().moveTo([home[0], home[1], home[2]]);
    expect(result.success).toBe(true);
  });
});

describe('Phase 4 · 切换的守卫（拒绝而不是静默）', () => {
  it('已接入传输时拒绝切换，且状态一位不变', () => {
    useRobotStore.setState({ transportKind: 'websocket', transportDriven: true });
    const before = snapshot();

    const result = useRobotStore.getState().setRobot(SO_ARM101_ROBOT_ID);

    expect(result.ok).toBe(false);
    expect(result.robotId).toBe(MEARM_V1_ROBOT_ID);
    expect(result.reason).toContain('已接入传输');
    expect(snapshot()).toEqual(before);
    // 拒绝必须留下一条说明"为什么 / 怎么修"的日志（与 setMode 同一纪律）
    const log = useRobotStore.getState().log;
    expect(log[log.length - 1]?.kind).toBe('err');
    expect(log[log.length - 1]?.text).toContain('断开');
  });

  it('未知 id 拒绝且**不回退**到缺省模型', () => {
    const before = snapshot();
    const result = useRobotStore.getState().setRobot('does-not-exist');

    expect(result.ok).toBe(false);
    expect(useRobotStore.getState().robotId).toBe(MEARM_V1_ROBOT_ID);
    expect(useRobotStore.getState().model).toBe(MEARM);
    expect(snapshot()).toEqual(before);
  });

  it('切到同一台是幂等的 —— 不会把姿态复位掉', () => {
    const base = MEARM.joints.find((j) => j.role === 'base')!;
    const moved = base.limits.min + 5;
    useRobotStore.getState().setJoint(base.id, moved);

    const result = useRobotStore.getState().setRobot(MEARM_V1_ROBOT_ID);
    expect(result.ok).toBe(true);
    expect(useRobotStore.getState().commandJoints[base.id]).toBeCloseTo(moved, 9);
  });
});

describe('Phase 8 · 切换压力回归', () => {
  it('往返 200 轮后回到 MeArm 的状态与初始**逐位相同**（无漂移）', () => {
    const initial = snapshot();

    for (let i = 0; i < 200; i += 1) {
      const a = useRobotStore.getState().setRobot(SO_ARM101_ROBOT_ID);
      expect(a.ok).toBe(true);
      const s1 = useRobotStore.getState();
      expect(s1.model).toBe(SO101);
      expect(Object.keys(s1.commandJoints).sort()).toEqual(jointIds(SO101).slice().sort());
      expect(outOfLimitJoints(SO101, s1.commandJoints)).toEqual([]);

      const b = useRobotStore.getState().setRobot(MEARM_V1_ROBOT_ID);
      expect(b.ok).toBe(true);
      const s2 = useRobotStore.getState();
      expect(s2.model).toBe(MEARM);
      expect(Object.keys(s2.commandJoints).sort()).toEqual(jointIds(MEARM).slice().sort());
      expect(outOfLimitJoints(MEARM, s2.commandJoints)).toEqual([]);
    }

    // 换回去必须与最开始**完全相同** —— 这条能抓住"某轮之后残留了另一台的状态"
    // 这类只在长循环里才暴露的漂移。
    expect(snapshot()).toEqual(initial);
  });

  it('压力回归期间日志不会无限增长（pushLog 有上限）', () => {
    for (let i = 0; i < 60; i += 1) {
      useRobotStore.getState().setRobot(SO_ARM101_ROBOT_ID);
      useRobotStore.getState().setRobot(MEARM_V1_ROBOT_ID);
    }
    expect(useRobotStore.getState().log.length).toBeLessThanOrEqual(200);
  });
});
