/**
 * RobotModel —— 整个 ArmPilot 的**唯一数据源**。
 *
 * ```
 *                RobotModel
 *                     │
 *          ┌──────────┴──────────┐
 *          ↓                     ↓
 *     Virtual Robot          Real Robot
 *     （Three.js）           （mARM / AVR）
 * ```
 *
 * 禁止在前端 / 后端 / 固件侧分别维护两套 Link / Joint / Angle / Limit / Coordinate：
 * 一切几何量、限位、标定、home 位姿都必须从本结构派生。
 *
 * 本文件属于机器人模型层，**不依赖 Three.js**，可在 node 中独立测试。
 */
import type { Actuator } from './Actuator';
import { actuatorJointToServo, actuatorServoRangeForLimits } from './Actuator';
import type { Joint, JointRole } from './Joint';
import { isMovableJoint } from './Joint';
import type { Link } from './Link';
import type { JointState, Vec3 } from './Pose';

/** TCP（工具中心点 / End Effector）定义 */
export interface TcpSpec {
  /** TCP 参考的关节 id（TCP = 该关节坐标系下的固定偏移点） */
  joint: string;
  /** 相对该关节坐标系的固定偏移（mm） */
  offset: Vec3;
}

export interface RobotModel {
  version: number;
  id: string;
  name: string;
  units: 'mm';
  links: Link[];
  joints: Joint[];
  actuators: Actuator[];
  /** 开机 / HOME 关节位姿（degree） */
  homePose: JointState;
  /** TCP 定义 */
  tcp: TcpSpec;
}

// ---------------------------------------------------------------------------
// 索引 / 遍历
// ---------------------------------------------------------------------------

export function linkById(model: RobotModel, id: string): Link | undefined {
  return model.links.find((l) => l.id === id);
}

export function requireLink(model: RobotModel, id: string): Link {
  const link = linkById(model, id);
  if (!link) throw new Error(`[RobotModel] 找不到连杆: ${id}`);
  return link;
}

export function jointById(model: RobotModel, id: string): Joint | undefined {
  return model.joints.find((j) => j.id === id);
}

export function requireJoint(model: RobotModel, id: string): Joint {
  const joint = jointById(model, id);
  if (!joint) throw new Error(`[RobotModel] 找不到关节: ${id}`);
  return joint;
}

export function jointByRole(model: RobotModel, role: JointRole): Joint | undefined {
  return model.joints.find((j) => j.role === role);
}

/** 关节对应的执行器（双舵机关节会返回多个） */
export function actuatorsForJoint(model: RobotModel, jointId: string): Actuator[] {
  return model.actuators.filter((a) => a.jointId === jointId);
}

export function actuatorById(model: RobotModel, id: string): Actuator | undefined {
  return model.actuators.find((a) => a.id === id);
}

export function actuatorByChannel(model: RobotModel, channel: number): Actuator | undefined {
  return model.actuators.find((a) => a.channel === channel);
}

export function rootLink(model: RobotModel): Link {
  const roots = model.links.filter((l) => l.parent === null || l.parent === undefined);
  if (roots.length !== 1) {
    throw new Error(`[RobotModel] 期望恰好 1 个根连杆，实际 ${roots.length} 个`);
  }
  return roots[0]!;
}

/** 有真实自由度的关节（revolute 且限位跨度 > 0），即 UI 中的 J1/J2/J3/Gripper */
export function movableJoints(model: RobotModel): Joint[] {
  return model.joints.filter(isMovableJoint);
}

/** 所有可动关节的 id（= JointState 的 key 集合） */
export function jointIds(model: RobotModel): string[] {
  return movableJoints(model).map((j) => j.id);
}

function childJointMap(model: RobotModel): Map<string, Joint> {
  const map = new Map<string, Joint>();
  for (const joint of model.joints) {
    // parentLink 唯一：一个连杆只能有一个近端关节
    if (!map.has(joint.parentLink)) map.set(joint.parentLink, joint);
  }
  return map;
}

/**
 * 从根连杆沿 childLink 走到末端的完整关节链（含叶关节，如 gripper）。
 * 顺序保证 root -> tip，与 FK 累乘顺序一致。
 */
export function kinematicChain(model: RobotModel): Joint[] {
  const byParent = childJointMap(model);
  const chain: Joint[] = [];
  const visited = new Set<string>();
  let link: Link | undefined = rootLink(model);

  while (link) {
    if (visited.has(link.id)) break;
    visited.add(link.id);
    const joint: Joint | undefined = byParent.get(link.id);
    if (!joint) break;
    chain.push(joint);
    link = linkById(model, joint.childLink);
  }
  return chain;
}

/** 从根到指定关节的路径（含该关节） */
export function chainToJoint(model: RobotModel, jointId: string): Joint[] {
  const full = kinematicChain(model);
  const index = full.findIndex((j) => j.id === jointId);
  return index < 0 ? [] : full.slice(0, index + 1);
}

/** 关节在链中的深度（根关节 = 0，叶关节最大）；不在链上返回 -1 */
export function jointDepth(model: RobotModel, jointId: string): number {
  return kinematicChain(model).findIndex((j) => j.id === jointId);
}

// ---------------------------------------------------------------------------
// JointState <-> 舵机角
// ---------------------------------------------------------------------------

/**
 * 关节空间**原点**位姿：各可动关节取 0°。
 *
 * ⚠️ 关节限位不一定把 0° 包含在内。本机就是一个真实例子：`elbow` 存的是
 * **绝对倾角**（0° = 小臂指向天顶），而真机经平行四连杆后可达区间只有
 * 108.44°..141.86°，所以 0° 物理上不可达。上层（store / UI）取用本函数后
 * 必须再经限位钳位，得到的是「最接近原点且可达」的位姿 —— 这是真机的
 * 真实约束，不是实现缺陷。
 */
export function zeroJointState(model: RobotModel): JointState {
  const state: JointState = {};
  for (const joint of movableJoints(model)) state[joint.id] = 0;
  return state;
}

/** 拷贝 home 位姿 */
export function homeJointState(model: RobotModel): JointState {
  const state: JointState = {};
  for (const joint of movableJoints(model)) {
    state[joint.id] = model.homePose[joint.id] ?? 0;
  }
  return state;
}

/**
 * 关节角 → 舵机角通道映射（`Joint Angle → Calibration → Servo Angle`）。
 * 输出 key = 舵机 channel（6/7/8/9），value = 舵机角度（degree）。
 */
export function jointStateToServoAngles(
  model: RobotModel,
  state: JointState,
): Record<number, number> {
  const out: Record<number, number> = {};
  for (const actuator of model.actuators) {
    const jointAngle = state[actuator.jointId] ?? 0;
    out[actuator.channel] = actuatorJointToServo(actuator, jointAngle);
  }
  return out;
}

// ---------------------------------------------------------------------------
// 校验
// ---------------------------------------------------------------------------

export type ModelIssueLevel = 'error' | 'warning';

export interface ModelIssue {
  level: ModelIssueLevel;
  code: string;
  message: string;
}

export class RobotModelError extends Error {
  readonly issues: ModelIssue[];
  constructor(issues: ModelIssue[]) {
    const errors = issues.filter((i) => i.level === 'error');
    super(
      `[RobotModel] 模型校验失败（${errors.length} 项错误）:\n` +
        errors.map((e) => `  - [${e.code}] ${e.message}`).join('\n'),
    );
    this.name = 'RobotModelError';
    this.issues = issues;
  }
}

const EPS = 1e-6;

function length3(v: Vec3): number {
  return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/** 校验模型自洽性；返回全部 issue（error + warning） */
export function validateRobotModel(model: RobotModel): ModelIssue[] {
  const issues: ModelIssue[] = [];
  const err = (code: string, message: string) => issues.push({ level: 'error', code, message });
  const warn = (code: string, message: string) => issues.push({ level: 'warning', code, message });

  // 1. 基本字段
  if (!model.id) err('ROBOT_ID', 'robot.id 不能为空');
  if (model.units !== 'mm') err('UNITS', `units 必须为 "mm"，实际 "${String(model.units)}"`);
  if (!Array.isArray(model.links) || model.links.length === 0) err('LINKS_EMPTY', 'links 不能为空');
  if (!Array.isArray(model.joints) || model.joints.length === 0) err('JOINTS_EMPTY', 'joints 不能为空');

  // 2. id 唯一性
  const dupCheck = (items: { id: string }[], label: string) => {
    const seen = new Set<string>();
    for (const item of items) {
      if (!item.id) {
        err(`${label}_ID_EMPTY`, `${label} 存在空 id`);
        continue;
      }
      if (seen.has(item.id)) err(`${label}_ID_DUP`, `${label} id 重复: ${item.id}`);
      seen.add(item.id);
    }
  };
  dupCheck(model.links, 'LINK');
  dupCheck(model.joints, 'JOINT');
  dupCheck(model.actuators, 'ACTUATOR');

  // 3. 引用完整性
  for (const link of model.links) {
    if (link.parent !== null && link.parent !== undefined && !linkById(model, link.parent)) {
      err('LINK_PARENT_MISSING', `连杆 ${link.id} 的 parent="${link.parent}" 不存在`);
    }
    if (link.length < 0) err('LINK_LENGTH_NEG', `连杆 ${link.id} 长度 ${link.length} < 0`);
  }
  for (const joint of model.joints) {
    if (!linkById(model, joint.parentLink)) {
      err('JOINT_PARENT_LINK', `关节 ${joint.id} 的 parentLink="${joint.parentLink}" 不存在`);
    }
    if (!linkById(model, joint.childLink)) {
      err('JOINT_CHILD_LINK', `关节 ${joint.id} 的 childLink="${joint.childLink}" 不存在`);
    }
    if (joint.type !== 'revolute' && joint.type !== 'fixed') {
      err('JOINT_TYPE', `关节 ${joint.id} 类型非法: ${String(joint.type)}`);
    }
    if (joint.type === 'revolute') {
      if (length3(joint.axis) < EPS) err('JOINT_AXIS_ZERO', `关节 ${joint.id} 的 axis 为零向量`);
      if (!(joint.limits.max > joint.limits.min)) {
        err('JOINT_LIMIT_RANGE', `关节 ${joint.id} 限位非法: [${joint.limits.min}, ${joint.limits.max}]`);
      }
    }
    // origin.position 只是「可选附加偏移」；父连杆 length 已把坐标系推到本关节处。
    // 常见笔误：把连杆长度写进 origin.position 而 link.length 留 0 —— 这里给一个告警兜底。
    const parentLink = linkById(model, joint.parentLink);
    if (parentLink) {
      const d = length3(joint.origin.position);
      if (d > EPS && parentLink.length === 0) {
        warn(
          'JOINT_OFFSET_SUSPICIOUS',
          `关节 ${joint.id} 在父连杆 ${parentLink.id}(length=0) 上使用了 origin.position=` +
            `[${joint.origin.position.join(', ')}]（模长 ${d.toFixed(2)}）。` +
            `若意图是「连杆长度」，应写进 links[].length，否则机构尺寸不会生效。`,
        );
      }
    }
  }

  // 4. 树结构：唯一根 + 全连通 + 无环
  const roots = model.links.filter((l) => l.parent === null || l.parent === undefined);
  if (roots.length !== 1) {
    err('ROOT_COUNT', `期望恰好 1 个根连杆（parent 为空），实际 ${roots.length} 个`);
  }
  const childJointCount = new Map<string, number>();
  for (const joint of model.joints) {
    childJointCount.set(joint.childLink, (childJointCount.get(joint.childLink) ?? 0) + 1);
  }
  for (const link of model.links) {
    const n = childJointCount.get(link.id) ?? 0;
    if (link.parent === null || link.parent === undefined) {
      if (n > 1) err('ROOT_FANIN', `根连杆 ${link.id} 被 ${n} 个关节作为 childLink 引用`);
    } else if (n !== 1) {
      err('LINK_FANIN', `连杆 ${link.id} 被 ${n} 个关节作为 childLink 引用（应为 1）`);
    }
  }
  if (roots.length === 1) {
    const reached = new Set<string>();
    const walk = (link: Link, stack: Set<string>) => {
      if (reached.has(link.id)) return;
      if (stack.has(link.id)) {
        err('CYCLE', `连杆图存在环，涉及 ${link.id}`);
        return;
      }
      reached.add(link.id);
      const next = new Set(stack).add(link.id);
      for (const joint of model.joints.filter((j) => j.parentLink === link.id)) {
        const child = linkById(model, joint.childLink);
        if (child) walk(child, next);
      }
    };
    walk(roots[0]!, new Set());
    for (const link of model.links) {
      if (!reached.has(link.id)) err('LINK_DISCONNECTED', `连杆 ${link.id} 无法从根到达`);
    }
  }

  // 5. 链上关节的角色唯一
  const roles = new Map<string, string[]>();
  for (const joint of model.joints) {
    if (!joint.role) continue;
    roles.set(joint.role, [...(roles.get(joint.role) ?? []), joint.id]);
  }
  for (const [role, ids] of roles) {
    if (ids.length > 1) err('ROLE_DUP', `role="${role}" 被多个关节使用: ${ids.join(', ')}`);
  }

  // 6. 执行器
  for (const actuator of model.actuators) {
    const joint = jointById(model, actuator.jointId);
    if (!joint) {
      err('ACTUATOR_JOINT', `执行器 ${actuator.id} 的 jointId="${actuator.jointId}" 不存在`);
      continue;
    }
    if (!isMovableJoint(joint)) {
      err('ACTUATOR_FIXED_JOINT', `执行器 ${actuator.id} 指向非可动关节 ${joint.id}`);
    }
    if (actuator.scale === 0) err('ACTUATOR_SCALE_ZERO', `执行器 ${actuator.id} scale 为 0`);
    if (!(actuator.limits.max > actuator.limits.min)) {
      err('ACTUATOR_LIMIT_RANGE', `执行器 ${actuator.id} 舵机限位非法`);
    }
    if (actuator.limits.min < 0 || actuator.limits.max > 180) {
      err('ACTUATOR_LIMIT_180', `执行器 ${actuator.id} 舵机限位超出 0..180`);
    }
    if (!Number.isInteger(actuator.channel) || actuator.channel <= 0) {
      err('ACTUATOR_CHANNEL', `执行器 ${actuator.id} channel 非法: ${String(actuator.channel)}`);
    }
    // 关节限位映射到舵机空间后必须落在舵机硬限位内
    const range = actuatorServoRangeForLimits(actuator, joint.limits);
    const slack = 1e-3;
    if (range.min < actuator.limits.min - slack || range.max > actuator.limits.max + slack) {
      err(
        'ACTUATOR_REACH',
        `执行器 ${actuator.id}: 关节限位 [${joint.limits.min}, ${joint.limits.max}] 映射到舵机 ` +
          `[${range.min.toFixed(2)}, ${range.max.toFixed(2)}]，超出舵机硬限位 ` +
          `[${actuator.limits.min}, ${actuator.limits.max}]`,
      );
    }
  }
  const channelSeen = new Set<number>();
  for (const actuator of model.actuators) {
    if (channelSeen.has(actuator.channel)) {
      err('ACTUATOR_CHANNEL_DUP', `舵机通道 ${actuator.channel} 被多个执行器占用`);
    }
    channelSeen.add(actuator.channel);
  }
  for (const joint of movableJoints(model)) {
    if (actuatorsForJoint(model, joint.id).length === 0) {
      warn('JOINT_NO_ACTUATOR', `可动关节 ${joint.id} 没有执行器（仅虚拟仿真可用）`);
    }
  }

  // 6b. 关节耦合（平行四连杆机构）
  for (const joint of model.joints) {
    const coupling = joint.coupling;
    if (!coupling) continue;
    if (coupling.jointId === joint.id) {
      err('JOINT_COUPLING_SELF', `关节 ${joint.id} 的 coupling 指向自身`);
      continue;
    }
    const other = jointById(model, coupling.jointId);
    if (!other) {
      err(
        'JOINT_COUPLING_TARGET',
        `关节 ${joint.id} 的 coupling.jointId="${coupling.jointId}" 不存在`,
      );
      continue;
    }
    if (isMovableJoint(joint) && !isMovableJoint(other)) {
      err('JOINT_COUPLING_FIXED', `关节 ${joint.id} 耦合到非可动关节 ${other.id}`);
    }
    if (coupling.gain === 0) {
      warn('JOINT_COUPLING_ZERO', `关节 ${joint.id} 的 coupling.gain 为 0（等于没有耦合）`);
    }
  }

  // 7. homePose
  for (const joint of movableJoints(model)) {
    const value = model.homePose[joint.id];
    if (value === undefined) {
      err('HOME_MISSING', `homePose 缺少关节 ${joint.id}`);
      continue;
    }
    if (value < joint.limits.min - EPS || value > joint.limits.max + EPS) {
      err(
        'HOME_OUT_OF_LIMIT',
        `homePose.${joint.id}=${value} 超出关节限位 [${joint.limits.min}, ${joint.limits.max}]`,
      );
    }
    for (const actuator of actuatorsForJoint(model, joint.id)) {
      const servo = actuatorJointToServo(actuator, value);
      if (servo < actuator.limits.min - 1e-3 || servo > actuator.limits.max + 1e-3) {
        err(
          'HOME_SERVO_OUT_OF_LIMIT',
          `homePose.${joint.id}=${value} 标定后舵机 ${actuator.id}=${servo.toFixed(2)}°，` +
            `超出舵机限位 [${actuator.limits.min}, ${actuator.limits.max}]`,
        );
      }
    }
  }
  for (const key of Object.keys(model.homePose)) {
    if (!jointById(model, key)) {
      warn('HOME_UNKNOWN_JOINT', `homePose 含未知关节 "${key}"`);
    }
  }

  // 8. TCP
  if (!jointById(model, model.tcp.joint)) {
    err('TCP_JOINT', `tcp.joint="${model.tcp.joint}" 不存在`);
  }

  return issues;
}

/** 校验失败（存在 error 级 issue）时抛出，并携带完整 issue 列表 */
export function assertValidRobotModel(model: RobotModel): RobotModel {
  const issues = validateRobotModel(model);
  if (issues.some((i) => i.level === 'error')) throw new RobotModelError(issues);
  return model;
}
