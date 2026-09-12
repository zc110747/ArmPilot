/**
 * RobotState —— 机器人唯一状态载体（spec §十七）。
 *
 * 任何机器人状态变化（虚拟操作 / 真实反馈 / 命令回显）都必须经过本结构，
 * 它是虚拟机械臂与真实机械臂之间同步的**唯一交换格式**。
 *
 * ```
 *   Virtual Robot  ←→  RobotState  ←→  Real Robot
 * ```
 *
 * `source` 用于打破「虚拟 → 真实 → 虚拟」的无限回环（spec §十九）：
 *   - `virtual`：用户在 3D 场景 / 滑杆上操作产生的状态
 *   - `command`：命令下发链路产生的状态（已发往真实机械臂，等待回执）
 *   - `real`   ：真实机械臂回传的状态（权威值，用于反向同步虚拟机械臂）
 */
import type { ControlSource, JointState, Pose } from './Pose';

export interface RobotState {
  joints: JointState;
  endEffector: Pose;
  timestamp: number;
  source: ControlSource;
}

export function createRobotState(
  joints: JointState,
  endEffector: Pose,
  source: ControlSource,
  timestamp: number = Date.now(),
): RobotState {
  return { joints: { ...joints }, endEffector, timestamp, source };
}

export function cloneRobotState(state: RobotState): RobotState {
  return {
    joints: { ...state.joints },
    endEffector: {
      position: [...state.endEffector.position],
      rotation: [...state.endEffector.rotation],
    },
    timestamp: state.timestamp,
    source: state.source,
  };
}

/** 两组关节角是否一致（每轴容差 eps 度） */
export function jointStatesEqual(a: JointState, b: JointState, eps = 1e-6): boolean {
  const keys = new Set([...Object.keys(a), ...Object.keys(b)]);
  for (const key of keys) {
    if (Math.abs((a[key] ?? 0) - (b[key] ?? 0)) > eps) return false;
  }
  return true;
}

/**
 * 逐关节误差（spec §三十四）：`Error = Actual − Command`。
 * 无位置反馈的舵机（MG90S 无回读）下 actual ≡ command，误差恒为 0；
 * 一旦固件具备回读能力，本函数无需改动即可显示真实误差。
 */
export function jointStateError(command: JointState, actual: JointState): JointState {
  const out: JointState = {};
  for (const key of Object.keys(command)) {
    out[key] = (actual[key] ?? command[key] ?? 0) - (command[key] ?? 0);
  }
  return out;
}

/** 末端位置误差（mm） */
export function positionError(a: Pose, b: Pose): number {
  return Math.hypot(
    a.position[0] - b.position[0],
    a.position[1] - b.position[1],
    a.position[2] - b.position[2],
  );
}
