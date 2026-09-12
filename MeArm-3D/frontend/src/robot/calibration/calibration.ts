/**
 * Calibration —— 关节角 ↔ 舵机角 标定层。
 *
 * ```
 *   Joint Angle           ← IK / UI / 轨迹规划只到这一层，永远不越界
 *        ↓
 *   Calibration           ← offset / scale / reverse / min / max
 *        ↓
 *   Servo Angle           ← Transport / 串口协议
 * ```
 *
 * 铁律：
 *   1. **标定绝不能进入 IK**。IK 只解关节角，标定是「关节空间 → 舵机空间」的下游映射。
 *   2. 真实机构装反 / 零点不同，一律改 `config/robot.yaml` 的 actuator 参数，不改代码。
 *   3. 一个机械关节由两个舵机共同驱动时（spec §二十五），在该关节下写两条 actuator 记录即可，
 *      上层（IK / UI / RobotState）完全无感。
 */
import type { Actuator } from '../model/Actuator';
import {
  actuatorJointToServo,
  actuatorServoRangeForLimits,
  actuatorServoToJoint,
  clampServoAngle,
} from '../model/Actuator';
import type { Joint } from '../model/Joint';
import type { RobotModel } from '../model/RobotModel';
import { actuatorsForJoint, jointById, movableJoints, requireJoint } from '../model/RobotModel';
import type { JointState } from '../model/Pose';

/** 单个关节的标定条目（一个关节可能有多条，对应多舵机） */
export interface CalibrationEntry {
  jointId: string;
  actuator: Actuator;
  /** 关节限位映射后的舵机范围（升序） */
  servoRange: { min: number; max: number };
  /** 是否多舵机共同驱动 */
  coupled: boolean;
}

export interface CalibrationTable {
  entries: CalibrationEntry[];
  /** jointId -> entries */
  byJoint: Map<string, CalibrationEntry[]>;
}

/** 从模型派生标定表（模型是唯一数据源，此处不存任何数值） */
export function buildCalibrationTable(model: RobotModel): CalibrationTable {
  const byJoint = new Map<string, CalibrationEntry[]>();
  const entries: CalibrationEntry[] = [];

  for (const joint of movableJoints(model)) {
    const actuators = actuatorsForJoint(model, joint.id);
    const list: CalibrationEntry[] = [];
    for (const actuator of actuators) {
      const entry: CalibrationEntry = {
        jointId: joint.id,
        actuator,
        servoRange: actuatorServoRangeForLimits(actuator, joint.limits),
        coupled: actuators.length > 1,
      };
      entries.push(entry);
      list.push(entry);
    }
    byJoint.set(joint.id, list);
  }

  return { entries, byJoint };
}

/** 关节角 → 舵机角（单执行器） */
export function jointToServo(actuator: Actuator, jointAngleDeg: number): number {
  return actuatorJointToServo(actuator, jointAngleDeg);
}

/** 舵机角 → 关节角（单执行器） */
export function servoToJoint(actuator: Actuator, servoAngleDeg: number): number {
  return actuatorServoToJoint(actuator, servoAngleDeg);
}

/**
 * 关节角 → 全部相关舵机角（多舵机关节会返回多条）。
 * 结果 key = 舵机 channel。
 */
export function jointToServoChannels(
  model: RobotModel,
  jointId: string,
  jointAngleDeg: number,
): Record<number, number> {
  const out: Record<number, number> = {};
  for (const actuator of actuatorsForJoint(model, jointId)) {
    out[actuator.channel] = actuatorJointToServo(actuator, jointAngleDeg);
  }
  return out;
}

/** 关节状态 → 全部舵机角 */
export function jointStateToServoChannels(
  model: RobotModel,
  state: JointState,
): Record<number, number> {
  const out: Record<number, number> = {};
  for (const actuator of model.actuators) {
    out[actuator.channel] = actuatorJointToServo(actuator, state[actuator.jointId] ?? 0);
  }
  return out;
}

/** 舵机状态 → 关节状态（用于「真实机械臂反馈 → 虚拟机械臂」同步） */
export function servoChannelsToJointState(
  model: RobotModel,
  servoAngles: Record<number, number>,
): JointState {
  const out: JointState = {};
  for (const joint of movableJoints(model)) {
    const entries = actuatorsForJoint(model, joint.id);
    const first = entries[0];
    if (!first) {
      out[joint.id] = 0;
      continue;
    }
    const servo = servoAngles[first.channel];
    out[joint.id] = servo === undefined ? 0 : actuatorServoToJoint(first, servo);
  }
  return out;
}

export interface CalibrationCheck {
  jointId: string;
  actuatorId: string;
  channel: number;
  servoRange: { min: number; max: number };
  hardwareLimits: { min: number; max: number };
  ok: boolean;
}

/** 逐执行器检查「关节限位 → 舵机角」是否落在舵机硬限位内 */
export function checkCalibration(model: RobotModel, epsilonDeg = 1e-3): CalibrationCheck[] {
  return buildCalibrationTable(model).entries.map((entry) => {
    const { servoRange, actuator } = entry;
    const ok =
      servoRange.min >= actuator.limits.min - epsilonDeg &&
      servoRange.max <= actuator.limits.max + epsilonDeg;
    return {
      jointId: entry.jointId,
      actuatorId: actuator.id,
      channel: actuator.channel,
      servoRange,
      hardwareLimits: actuator.limits,
      ok,
    };
  });
}

/** 单句人类可读的标定描述，用于 StatusPanel / 日志 */
export function describeCalibration(model: RobotModel): string[] {
  const lines: string[] = [];
  for (const entry of buildCalibrationTable(model).entries) {
    const { actuator, servoRange } = entry;
    const sign = actuator.reverse ? '-' : '+';
    lines.push(
      `S${actuator.channel} (${entry.jointId}): servo = ${sign}θ × ${actuator.scale} ` +
        `+ ${actuator.offset}  →  关节[${servoRange.min.toFixed(1)}, ${servoRange.max.toFixed(1)}]° ` +
        `∈ 舵机[${actuator.limits.min}, ${actuator.limits.max}]°` +
        (entry.coupled ? '  [多舵机关节]' : ''),
    );
  }
  return lines;
}

/** 安全发送前的兜底：把（关节角→舵机角）夹到舵机硬限位，并返回是否发生钳位 */
export function calibrateClamped(
  model: RobotModel,
  jointId: string,
  jointAngleDeg: number,
): { channels: Record<number, number>; clamped: boolean } {
  const joint: Joint = requireJoint(model, jointId);
  const inJointLimit =
    jointAngleDeg >= joint.limits.min - 1e-9 && jointAngleDeg <= joint.limits.max + 1e-9;

  const channels: Record<number, number> = {};
  let clamped = !inJointLimit;
  for (const actuator of actuatorsForJoint(model, jointId)) {
    const raw = actuatorJointToServo(actuator, jointAngleDeg);
    const limited = clampServoAngle(actuator, raw);
    if (Math.abs(limited - raw) > 1e-9) clamped = true;
    channels[actuator.channel] = limited;
  }
  return { channels, clamped };
}

/** 判断关节 id 是否在模型中存在且可动 */
export function isMovableJointId(model: RobotModel, jointId: string): boolean {
  const joint = jointById(model, jointId);
  return !!joint && joint.type === 'revolute' && joint.limits.max > joint.limits.min;
}
