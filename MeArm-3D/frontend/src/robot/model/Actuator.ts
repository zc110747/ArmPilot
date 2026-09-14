/**
 * Actuator（执行器）——关节空间 → 舵机空间的映射与舵机硬件限位。
 *
 * ⚠️ 关键设计约束（spec §二十四 / §二十五）：
 *   - 标定**只**存在于「Joint Angle → Servo Angle」这一层；
 *   - IK **永远只输出 Joint State**，绝不输出 Servo 角度 / PWM；
 *   - 若某个机械关节由两个舵机共同驱动，则一个 Joint 对应多个 Actuator，
 *     对 IK 与上层完全透明（本类型天然支持：`jointId` 可重复）。
 *
 * 映射公式：
 *   servo = reverse ? (-θ * scale + offset) : (θ * scale + offset)
 */
import type { JointLimits } from './Joint';

/**
 * 执行器角度的**语义空间**。
 *
 * - `'deg'`（**缺省**）：**舵机空间**。`limits` 是舵机硬限位（0..180°），
 *   `offset` / `scale` / `reverse` 是一段真实的仿射标定
 *   （"舵机装反了 / 装了舵盘后零点偏了"这类物理事实）。
 *   MeArm-V1 即此形态。
 * - `'joint'`：**关节空间**。该通道没有独立标定，`limits` 直接就是关节角的可达区间（degree）。
 *   SO-ARM101 即此形态：官方 MJCF 的执行器是 `<position>`，其 `ctrlrange` **等于**关节范围、
 *   `gear=1`，属**关节空间位置伺服** —— 官方模型里根本不存在 offset/scale/reverse。
 *
 * 为什么不能"用 scale 把 SO-101 压进 0..180"：那会凭空发明一段并不存在的标定
 * （官方文件里没有这些数），使"标定表"从**事实**退化成**为了让校验通过而凑的参数**。
 * 声明空间语义则如实反映"这条通道本来就没有标定"。
 *
 * ⚠️ 缺省（`undefined`）语义 = `'deg'`，保证引入本字段**不改变任何既有行为**。
 */
export type ActuatorUnit = 'deg' | 'joint';

export interface ActuatorLimits {
  /** 舵机强制下限（0..180 degree） */
  min: number;
  /** 舵机强制上限（0..180 degree） */
  max: number;
}

export interface Actuator {
  id: string;
  name?: string;
  /** 被驱动的关节 id */
  jointId: string;
  /** 串口协议中的舵机 ID（arm-device: 6 / 7 / 8 / 9） */
  channel: number;
  /** 舵机零点偏移（degree） */
  offset: number;
  /** 关节角 → 舵机角 比例 */
  scale: number;
  /** 是否反向 */
  reverse: boolean;
  /** **舵机**硬件强制范围（非关节范围） */
  limits: ActuatorLimits;
  /**
   * `limits` 所在的语义空间；**缺省 = `'deg'`（舵机空间 0..180°，既有行为）**。
   * 取 `'joint'` 表示本通道是关节空间位置伺服、`limits` 即关节可达区间。
   */
  unit?: ActuatorUnit;
}

/** 执行器的角度空间（补齐缺省，避免调用方到处写 `?? 'deg'`） */
export function actuatorUnit(actuator: Actuator): ActuatorUnit {
  return actuator.unit ?? 'deg';
}

/** 关节角（degree）→ 舵机角（degree） */
export function actuatorJointToServo(actuator: Actuator, jointAngleDeg: number): number {
  const scaled = jointAngleDeg * actuator.scale;
  return actuator.reverse ? -scaled + actuator.offset : scaled + actuator.offset;
}

/** 舵机角（degree）→ 关节角（degree），标定可逆（scale !== 0）时成立 */
export function actuatorServoToJoint(actuator: Actuator, servoAngleDeg: number): number {
  if (actuator.scale === 0) {
    throw new Error(`[Actuator] ${actuator.id}: scale 为 0，标定不可逆`);
  }
  const shifted = actuator.reverse ? actuator.offset - servoAngleDeg : servoAngleDeg - actuator.offset;
  return shifted / actuator.scale;
}

/** 关节限位映射到舵机空间，返回 [min, max]（已归一化为升序） */
export function actuatorServoRangeForLimits(
  actuator: Actuator,
  limits: JointLimits,
): { min: number; max: number } {
  const a = actuatorJointToServo(actuator, limits.min);
  const b = actuatorJointToServo(actuator, limits.max);
  return { min: Math.min(a, b), max: Math.max(a, b) };
}

/** 舵机角限幅到硬件强制范围 */
export function clampServoAngle(actuator: Actuator, servoAngleDeg: number): number {
  const { min, max } = actuator.limits;
  return servoAngleDeg < min ? min : servoAngleDeg > max ? max : servoAngleDeg;
}
