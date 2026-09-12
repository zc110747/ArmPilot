/**
 * RobotCommand —— 一切控制意图的统一出口（spec §三十八）。
 *
 * 当前只有「关节角命令」一种；未来接入 AI / 视觉 / 语音 / MuJoCo 时，
 * 它们都必须先归一到 `RobotCommand` 再进入安全链路，而不是各自直连 Transport：
 *
 * ```
 *   Vision ─→ Target Pose ─→ IK ─┐
 *   AI ──────────────────────────┼─→ RobotCommand → Safety → Transport → Real Robot
 *   UI / 滑杆 / 轨迹 ────────────┘
 * ```
 *
 * 本阶段**不实现**任何 AI 能力，只保证该边界存在且稳定。
 */
import type { ControlSource, JointState } from './Pose';

export type RobotCommandType =
  /** 关节角命令（唯一进入真实机械臂的定位命令） */
  | 'joint_command'
  /** 回 HOME 位（必须经安全检查后才允许下发） */
  | 'home'
  /** 停止当前运动（轨迹/序列） */
  | 'stop'
  /** 急停：清空待发队列，真实机械臂进入安全状态 */
  | 'emergency_stop'
  /** 夹爪单独开合（不影响定位链路） */
  | 'gripper';

export interface RobotCommand {
  /** 协议版本，便于后续升级 */
  version: number;
  id: string;
  type: RobotCommandType;
  timestamp: number;
  /** 命令来源；用于打破虚拟/真实回环 */
  source: ControlSource;
  /** joint_command / home 时的目标关节角（degree） */
  joints?: JointState;
  /** 人类可读的备注，便于 Command Log 展示 */
  note?: string;
}

let commandSeq = 0;

export function nextCommandId(prefix = 'cmd'): string {
  commandSeq += 1;
  return `${prefix}-${Date.now().toString(36)}-${commandSeq.toString(36)}`;
}

export function createJointCommand(
  joints: JointState,
  source: ControlSource = 'virtual',
  note?: string,
): RobotCommand {
  return {
    version: 1,
    id: nextCommandId('joint'),
    type: 'joint_command',
    timestamp: Date.now(),
    source,
    joints: { ...joints },
    ...(note ? { note } : {}),
  };
}

export function createSimpleCommand(
  type: Exclude<RobotCommandType, 'joint_command'>,
  source: ControlSource = 'virtual',
  note?: string,
): RobotCommand {
  return {
    version: 1,
    id: nextCommandId(type),
    type,
    timestamp: Date.now(),
    source,
    ...(note ? { note } : {}),
  };
}
