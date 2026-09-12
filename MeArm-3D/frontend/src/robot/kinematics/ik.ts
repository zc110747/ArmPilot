/**
 * IK（逆运动学）—— 末端 **XYZ 位置** → `base / shoulder / elbow` 三个定位关节角。
 *
 * ## 为什么可以解析求解
 *
 * 本机是「1 个绕 Z 的偏航关节 + 1 组在竖直平面内摆动的 2R 连杆」，
 * 因此可以精确解耦成两步（几何关系见 `docs/coordinate-system.md` §3.1）：
 *
 * ```
 *   J1 = atan2(y, x)          ← 偏航，与高度无关，把三维问题降成矢状面内的二维
 *   r  = hypot(x, y)          ← 矢状面内的水平半径
 *   ┌─ 矢状面内 2R ─────────────────────────────────────────────┐
 *   │  L1 = 肩枢轴 → 肘枢轴         （大臂）                      │
 *   │  L2 = 肘枢轴 → TCP            （小臂 + 手腕 + 夹爪）        │
 *   │  dr = r − 枢轴水平半径,  dz = z − 枢轴高度                  │
 *   │  D  = hypot(dr, dz)                                        │
 *   │  α  = ±acos((D² − L1² − L2²) / (2·L1·L2))   ← 相对肘角      │
 *   │  φ  = atan2(dr, dz)                          ← 0 = 天顶     │
 *   │  θs = φ − atan2(L2·sinα, L1 + L2·cosα)                     │
 *   │  θe = θs + α                                               │
 *   └────────────────────────────────────────────────────────────┘
 * ```
 *
 * **所有几何量（L1 / L2 / 枢轴）都在运行时从 RobotModel 求导，不写死常数** ——
 * 改 `config/robot.yaml` 的 `length` 即改机构，本文件无需改动。
 *
 * ## ⚠️ 本机最特殊的一点：`elbow` 是**绝对角**
 *
 * 真机小臂由独立舵机经**平行四连杆**驱动，其绝对倾角与肩角解耦
 * （实测：肩转 52.91° 时小臂绝对角只漂 10.08°，见 `docs/hardware-measurement.md`）。
 * 因此 `JointState.elbow` 存的是**离开天顶的绝对倾角**，而不是相对大臂的夹角。
 *
 * 这带来一个反直觉的结论：**IK 解出的 `θe` 就是最终要写进 JointState 的值，不需要再叠加肩角**。
 * 串联网里的局部旋转由 FK 的 `effectiveJointAngle()` 负责（`relative = θe + (−1)·θs`）。
 * 若按常见 meArm 写法"再叠加一次肩角"，机构会立刻错位 —— 见 `docs/decisions.md` D18。
 *
 * ## 约束
 *
 * - IK **只解定位三关节**，绝不碰夹爪，更不知道舵机 / PWM / 标定
 *   （`coordinate-system.md` §4 明文规定：标定绝不进入 IK）。
 * - 夹爪角若传入 `seed` 则原样透传，否则取 `homePose`，保证返回值可直接喂给 FK 闭环验收。
 */
import type { Joint } from '../model/Joint';
import type { JointState, Vec3 } from '../model/Pose';
import { degToRad, radToDeg } from '../model/Pose';
import type { RobotModel } from '../model/RobotModel';
import { homeJointState, jointByRole, movableJoints } from '../model/RobotModel';
import { endEffectorPosition, jointMatrices } from './fk';
import { mat4GetPosition } from './transform';

/** 求解失败的原因 */
export type IkReason =
  /** 目标点超出连杆总长能触及的球壳范围（几何不可达） */
  | 'OUT_OF_WORKSPACE'
  /** 几何可达，但所需关节角超出该关节限位（真机做不出来） */
  | 'JOINT_LIMIT';

/**
 * 解的分支。
 *
 * 平面 2R 有两组解，由相对肘角 `α = θe − θs` 的符号区分：
 * - `elbow-up`：`α > 0`，肘部高于「肩→目标」连线 —— **真机 HOME 位所在支**
 * - `elbow-down`：`α < 0`，肘部低于该连线（小臂向回折）
 */
export type IkBranch = 'elbow-up' | 'elbow-down';

/** 多解选取策略 */
export type IkPreference = IkBranch | 'nearest';

/** 单个候选解（含被限位否决的那些，便于调试与可视化） */
export interface IkCandidate {
  branch: IkBranch;
  /** `α = θe − θs`（degree） */
  relativeAngle: number;
  /** 相对肘角绝对值（degree） */
  alphaAbsolute: number;
  /** 该支解（base/shoulder/elbow 三元组，degree） */
  joints: JointState;
  /** 是否全部落在限位内 */
  feasible: boolean;
  /** 不可行时，越界最严重的关节 id */
  violatedJoint?: string;
  /** 越界量（degree，0 表示刚好在界上） */
  violation: number;
}

export interface IkOptions {
  /**
   * 多解选取策略，默认 `'nearest'`（需要 `near`；缺失时自动退化为 `'elbow-up'`）。
   *
   * `nearest` 是为「鼠标拖动末端」准备的：末端在工作空间内边界附近移动时，
   * 两支解会互相翻越，固定选支会让机械臂突然翻肘；按「离当前位姿最近」选支可消除跳变。
   */
  prefer?: IkPreference;
  /** 当前关节状态；`prefer: 'nearest'` 用它做判据，也用作未解关节的取值来源 */
  near?: JointState;
  /** 未参与解算的关节（夹爪）取值来源；缺省取 `homePose` */
  seed?: JointState;
  /** 残差阈值（mm）；超出即视为实现缺陷而非目标不可达，默认 1e-9 */
  tolerance?: number;
}

export interface IkSuccess {
  success: true;
  /** 完整 JointState（含夹爪），可直接喂给 `forwardKinematics` */
  joints: JointState;
  branch: IkBranch;
  /** 解算后用 FK 自查的末端位置残差（mm） */
  residual: number;
  /** 实际采用的底座方位角（degree）；`r ≈ 0` 时该值由 `near` / 限位决定 */
  azimuth: number;
  /** `α = θe − θs`（degree） */
  relativeAngle: number;
}

export interface IkFailure {
  success: false;
  reason: IkReason;
  /** `JOINT_LIMIT` 时指出越界的关节 id */
  joint?: string;
  /** 人类可读说明，格式对齐 `protocol/serial-v1.md` 的 `ERR JOINT ...` */
  message: string;
  /** 已算出的候选（`OUT_OF_WORKSPACE` 时为空数组） */
  candidates: IkCandidate[];
}

export type IkResult = IkSuccess | IkFailure;

/**
 * 矢状面 2R 的几何量 —— 全部由模型求导。
 *
 * 求导方式：把全部关节置 0 跑一次 FK。此时机构整体沿 +Z 竖直伸展，
 * 于是「相邻枢轴的间距」直接就是连杆在矢状面内的等效长度，
 * 天然把 `origin.position` 附加偏移与 `tcp.offset` 一并算进去。
 */
export interface IkGeometry {
  baseId: string;
  shoulderId: string;
  elbowId: string;
  /** 肩枢轴高度（mm，底座平面之上） */
  pivotZ: number;
  /** 肩枢轴水平半径（mm，位于偏航轴上，正常为 0） */
  pivotR: number;
  /** 大臂等效长（肩枢轴 → 肘枢轴，mm） */
  l1: number;
  /** 小臂等效长（肘枢轴 → TCP，mm） */
  l2: number;
  /** 可达球壳：[min, max] 为枢轴到目标的最小 / 最大距离（mm） */
  reach: readonly [number, number];
}

/** 模型形状与平面 2R 假设不符时抛出（属配置错误，不是目标不可达） */
export class IkModelError extends Error {
  constructor(message: string) {
    super(`[ik] ${message}`);
    this.name = 'IkModelError';
  }
}

const EPS_DEG = 1e-9;
const EPS_MM = 1e-9;

function axisCloseTo(axis: Vec3, expected: readonly [number, number, number], tol = 1e-6): boolean {
  return (
    Math.abs(axis[0] - expected[0]) < tol &&
    Math.abs(axis[1] - expected[1]) < tol &&
    Math.abs(axis[2] - expected[2]) < tol
  );
}

function requireRoleJoint(model: RobotModel, role: 'base' | 'shoulder' | 'elbow'): Joint {
  const joint = jointByRole(model, role);
  if (!joint) throw new IkModelError(`模型缺少 role=${role} 的关节，无法做位置 IK`);
  return joint;
}

/**
 * 校验「解析式 2R」的前提在模型里成立。
 *
 * 解析解假定：底座绕 Z 偏航、肩与肘共用一根平行于 Y 的俯仰轴、且两关节无固定朝向偏转。
 * 一旦有人改了 `joints[].axis` / `origin.rotation`，解析式会**静默失效**（不报错但解错），
 * 所以这里主动拦下来，把"改配置后的隐形错误"变成一条明确报错。
 */
function assertPlanar2R(base: Joint, shoulder: Joint, elbow: Joint): void {
  if (base.type !== 'revolute' || shoulder.type !== 'revolute' || elbow.type !== 'revolute') {
    throw new IkModelError('base / shoulder / elbow 必须都是 revolute 关节');
  }
  if (!axisCloseTo(base.axis, [0, 0, 1])) {
    throw new IkModelError(
      `base.axis = [${base.axis.join(', ')}]，解析式 IK 要求 [0, 0, 1]（绕 Z 偏航）。` +
        '若确需改轴，请同步改写 ik.ts 的解算方式。',
    );
  }
  if (!axisCloseTo(shoulder.axis, [0, 1, 0]) || !axisCloseTo(elbow.axis, [0, 1, 0])) {
    throw new IkModelError(
      `shoulder.axis / elbow.axis 必须同为 [0, 1, 0]（共面俯仰轴），` +
        `当前为 [${shoulder.axis.join(', ')}] / [${elbow.axis.join(', ')}]`,
    );
  }
  for (const joint of [base, shoulder, elbow]) {
    const [rx, ry, rz] = joint.origin.rotation;
    if (Math.abs(rx) > 1e-6 || Math.abs(ry) > 1e-6 || Math.abs(rz) > 1e-6) {
      throw new IkModelError(
        `${joint.id}.origin.rotation = [${rx}, ${ry}, ${rz}]，解析式 IK 要求无固定朝向偏转（[0, 0, 0]）`,
      );
    }
  }
}

/** 从模型求导矢状面 2R 的几何量（带缓存） */
const geometryCache = new WeakMap<RobotModel, IkGeometry>();

export function ikGeometry(model: RobotModel): IkGeometry {
  const cached = geometryCache.get(model);
  if (cached) return cached;

  const base = requireRoleJoint(model, 'base');
  const shoulder = requireRoleJoint(model, 'shoulder');
  const elbow = requireRoleJoint(model, 'elbow');
  assertPlanar2R(base, shoulder, elbow);

  // 全部关节置 0 → 机构整体沿 +Z 竖直伸展，"枢轴间距"即矢状面内的等效连杆长度
  const upright: JointState = {};
  for (const joint of movableJoints(model)) upright[joint.id] = 0;
  // 偏航归零，使肩枢轴落在 XZ 平面内（枢轴位置与 shoulder/elbow 角无关）
  upright[base.id] = 0;

  const matrices = jointMatrices(model, upright);
  const shoulderMatrix = matrices.get(shoulder.id);
  const elbowMatrix = matrices.get(elbow.id);
  if (!shoulderMatrix || !elbowMatrix) {
    throw new IkModelError('FK 未产出 shoulder / elbow 坐标系，无法求导几何量');
  }

  const pShoulder = mat4GetPosition(shoulderMatrix);
  const pElbow = mat4GetPosition(elbowMatrix);
  const pTcp = endEffectorPosition(model, upright);

  const pivotZ = pShoulder[2];
  const pivotR = Math.hypot(pShoulder[0], pShoulder[1]);
  const l1 = dist(pShoulder, pElbow);
  const l2 = dist(pElbow, pTcp);

  if (!(l1 > EPS_MM) || !(l2 > EPS_MM)) {
    throw new IkModelError(
      `求导出的连杆长度非法（l1=${l1}, l2=${l2}）；请检查 links[].length 与 tcp.offset`,
    );
  }

  const geometry: IkGeometry = {
    baseId: base.id,
    shoulderId: shoulder.id,
    elbowId: elbow.id,
    pivotZ,
    pivotR,
    l1,
    l2,
    reach: [Math.abs(l1 - l2), l1 + l2],
  };
  geometryCache.set(model, geometry);
  return geometry;
}

function dist(a: Vec3, b: Vec3): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function limitViolation(joint: Joint, value: number): number {
  return Math.max(0, joint.limits.min - value, value - joint.limits.max);
}

/**
 * 求解全部候选解（不做多解选取）。
 *
 * 返回的候选**包含被限位否决的支**，便于 UI/调试展示「另一支差多少度」；
 * 调用方若只想拿可行解，过滤 `feasible === true` 即可。
 */
export function solveIkCandidates(model: RobotModel, target: Vec3): {
  reason?: IkReason;
  candidates: IkCandidate[];
  azimuth: number;
  azimuthIndeterminate: boolean;
} {
  const geometry = ikGeometry(model);
  const base = requireRoleJoint(model, 'base');
  const shoulder = requireRoleJoint(model, 'shoulder');
  const elbow = requireRoleJoint(model, 'elbow');
  const { l1, l2, pivotZ, pivotR } = geometry;

  const [x, y, z] = target;
  const r = Math.hypot(x, y);

  // ---- ① 偏航 -------------------------------------------------------------
  // r ≈ 0 时目标落在偏航轴上，方位角数学上不定（无唯一解），交由上层按 nearest / 限位决定。
  const azimuthIndeterminate = r < 1e-9;
  const azimuthRaw = azimuthIndeterminate ? 0 : radToDeg(Math.atan2(y, x));

  // ---- ② 矢状面内 2R -------------------------------------------------------
  const dr = r - pivotR;
  const dz = z - pivotZ;
  const d = Math.hypot(dr, dz);

  const [reachMin, reachMax] = geometry.reach;
  if (d > reachMax + 1e-9 || d < reachMin - 1e-9) {
    return {
      reason: 'OUT_OF_WORKSPACE',
      candidates: [],
      azimuth: azimuthRaw,
      azimuthIndeterminate,
    };
  }

  const clampedD = Math.min(Math.max(d, reachMin), reachMax);
  const cosAlpha = clamp1((clampedD * clampedD - l1 * l1 - l2 * l2) / (2 * l1 * l2));
  const alphaDeg = radToDeg(Math.acos(cosAlpha));
  const phiDeg = radToDeg(Math.atan2(dr, dz));

  const candidates: IkCandidate[] = [];
  for (const sign of [1, -1] as const) {
    const relativeAngle = sign * alphaDeg;
    const ths = phiDeg - radToDeg(Math.atan2(l2 * Math.sin(degToRad(relativeAngle)), l1 + l2 * Math.cos(degToRad(relativeAngle))));
    // ⚠️ elbow 存绝对角 ⇒ 直接 = ths + 相对角，不要再叠加肩角（见文件头说明）
    const the = ths + relativeAngle;

    const joints: JointState = { [base.id]: azimuthRaw, [shoulder.id]: ths, [elbow.id]: the };
    const vS = limitViolation(shoulder, ths);
    const vE = limitViolation(elbow, the);
    const violation = Math.max(vS, vE);
    const violatedJoint = violation <= EPS_DEG ? undefined : vE >= vS ? elbow.id : shoulder.id;

    candidates.push({
      branch: sign > 0 ? 'elbow-up' : 'elbow-down',
      relativeAngle,
      alphaAbsolute: alphaDeg,
      joints,
      feasible: violation <= EPS_DEG,
      ...(violatedJoint ? { violatedJoint } : {}),
      violation,
    });
  }
  return { candidates, azimuth: azimuthRaw, azimuthIndeterminate };
}

function clamp1(v: number): number {
  return v < -1 ? -1 : v > 1 ? 1 : v;
}

/** 把候选补成完整 JointState（夹爪等未解关节来自 seed / homePose） */
function completeState(model: RobotModel, partial: JointState, opts: IkOptions): JointState {
  const source = opts.seed ?? opts.near;
  const home = homeJointState(model);
  const out: JointState = {};
  for (const joint of movableJoints(model)) {
    const value = partial[joint.id] ?? source?.[joint.id] ?? home[joint.id] ?? joint.limits.min;
    out[joint.id] = clampToLimits(joint, value);
  }
  return out;
}

function clampToLimits(joint: Joint, value: number): number {
  return value < joint.limits.min ? joint.limits.min : value > joint.limits.max ? joint.limits.max : value;
}

/** 位置目标，供日志 / 错误信息使用 */
function fmt(p: Vec3): string {
  return `(${p[0].toFixed(3)}, ${p[1].toFixed(3)}, ${p[2].toFixed(3)})`;
}

/**
 * 逆运动学：末端位置 → 定位三关节角（`base / shoulder / elbow`）。
 *
 * ```
 *   const r = solveIk(model, [120, 0, 90]);
 *   if (r.success) apply(r.joints);      // r.joints 含夹爪，可直接喂 FK
 *   else console.warn(r.reason, r.message);
 * ```
 */
export function solveIk(model: RobotModel, target: Vec3, opts: IkOptions = {}): IkResult {
  const geometry = ikGeometry(model);
  const base = requireRoleJoint(model, 'base');
  const shoulder = requireRoleJoint(model, 'shoulder');
  const elbow = requireRoleJoint(model, 'elbow');

  const { candidates, reason, azimuth, azimuthIndeterminate } = solveIkCandidates(model, target);

  if (reason === 'OUT_OF_WORKSPACE') {
    const [reachMin, reachMax] = geometry.reach;
    const d = Math.hypot(Math.hypot(target[0], target[1]) - geometry.pivotR, target[2] - geometry.pivotZ);
    return {
      success: false,
      reason: 'OUT_OF_WORKSPACE',
      message:
        `目标 ${fmt(target)} 超出工作空间：枢轴到目标距离 ${d.toFixed(3)}mm，` +
        `可达范围 ${reachMin.toFixed(3)}..${reachMax.toFixed(3)}mm ` +
        `（L1=${geometry.l1.toFixed(3)} + L2=${geometry.l2.toFixed(3)}）`,
      candidates: [],
    };
  }

  // ---- ③ 偏航限位（与支解无关，先判）-----------------------------------------
  let azimuthUsed = azimuth;
  if (azimuthIndeterminate) {
    // 目标在偏航轴上：方位角不定，取当前值（就近）或 home，再钳位
    const fallback = opts.near?.[base.id] ?? model.homePose[base.id] ?? 0;
    azimuthUsed = clampToLimits(base, fallback);
  }
  const baseViolation = azimuthIndeterminate ? 0 : limitViolation(base, azimuthUsed);

  // ---- ④ 选支 ---------------------------------------------------------------
  const preference: IkPreference = opts.prefer ?? 'nearest';
  const effectivePreference: IkPreference = preference === 'nearest' && !opts.near ? 'elbow-up' : preference;

  const feasible = candidates.filter((c) => c.feasible && baseViolation <= EPS_DEG);
  if (feasible.length === 0) {
    if (baseViolation > EPS_DEG) {
      return {
        success: false,
        reason: 'JOINT_LIMIT',
        joint: base.id,
        message:
          `JOINT ${base.id} ${azimuthUsed.toFixed(3)} ` +
          `(limit ${base.limits.min}..${base.limits.max})：目标方位角超出底座可达范围`,
        candidates,
      };
    }
    // 几何可达但两支解都超限 → 报越界最轻的那支
    const best = candidates.reduce((a, b) => (a.violation <= b.violation ? a : b));
    const joint = best.violatedJoint === elbow.id ? elbow : shoulder;
    const value = best.violatedJoint === elbow.id ? best.joints[elbow.id]! : best.joints[shoulder.id]!;
    return {
      success: false,
      reason: 'JOINT_LIMIT',
      joint: joint.id,
      message:
        `JOINT ${joint.id} ${value.toFixed(3)} ` +
        `(limit ${joint.limits.min}..${joint.limits.max})：` +
        `几何可达但两支解都越界，最接近的一支（${best.branch}）仍差 ${best.violation.toFixed(3)}°`,
      candidates,
    };
  }

  const chosen = selectCandidate(model, feasible, effectivePreference, opts);

  // ---- ⑤ 收敛：合成完整状态并用 FK 自查残差 -----------------------------------
  const joints = completeState(model, chosen.joints, opts);
  joints[base.id] = azimuthUsed;

  const achieved = endEffectorPosition(model, joints);
  const residual = dist(achieved, target);
  const tolerance = opts.tolerance ?? 1e-9;
  if (!(residual <= Math.max(tolerance, 1e-6))) {
    // 走到这里说明解算或模型有问题（例如连杆共线退化），必须显式暴露而不是静默返回
    throw new IkModelError(
      `解算自检失败：目标 ${fmt(target)}，FK 实得 ${fmt(achieved)}，残差 ${residual}mm ` +
        `（容差 ${tolerance}mm）—— 这是实现缺陷，请检查模型与 ik.ts 的一致性`,
    );
  }

  return {
    success: true,
    joints,
    branch: chosen.branch,
    residual,
    azimuth: azimuthUsed,
    relativeAngle: chosen.relativeAngle,
  };
}

function selectCandidate(
  model: RobotModel,
  feasible: IkCandidate[],
  preference: IkPreference,
  opts: IkOptions,
): IkCandidate {
  if (preference === 'nearest') {
    const near = opts.near!;
    return feasible.reduce((a, b) =>
      squaredDistanceTo(model, b.joints, near) < squaredDistanceTo(model, a.joints, near) ? b : a,
    );
  }
  const wanted = feasible.find((c) => c.branch === preference);
  if (wanted) return wanted;
  // 指定支不可行 → 退到可行解里的 elbow-up（真机 HOME 所在支），绝不静默返回不可行解
  return feasible.find((c) => c.branch === 'elbow-up') ?? feasible[0]!;
}

function squaredDistanceTo(model: RobotModel, a: JointState, b: JointState): number {
  let sum = 0;
  for (const joint of movableJoints(model)) {
    const av = a[joint.id];
    const bv = b[joint.id];
    if (av === undefined || bv === undefined) continue;
    const d = av - bv;
    sum += d * d;
  }
  return sum;
}

/** 全部候选解 + 选取结果，供调试面板 / 可视化使用 */
export function solveIkAll(
  model: RobotModel,
  target: Vec3,
  opts: IkOptions = {},
): { result: IkResult; candidates: IkCandidate[]; azimuthIndeterminate: boolean } {
  const probe = solveIkCandidates(model, target);
  const result = solveIk(model, target, opts);
  return {
    result,
    candidates: probe.candidates,
    azimuthIndeterminate: probe.azimuthIndeterminate,
  };
}
