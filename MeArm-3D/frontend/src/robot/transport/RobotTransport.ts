/**
 * RobotTransport —— 虚拟机械臂与真实机械臂之间的传输抽象（spec §二十 / §三十八）。
 *
 * 上层（Store / UI / 轨迹播放）只认识本接口，不认识 WebSocket / 串口 / 协议文本：
 *
 * ```
 *   RobotState ─→ RobotTransport ─→ MockTransport        （Phase 7：纯虚拟闭环）
 *                              └─→ WebSocketTransport    （Phase 8：浏览器 → Go → 串口 → AVR）
 * ```
 *
 * 消息概念三分（spec §十九），务必区分，否则会形成无限回环：
 *   - `command`：上层意图（我们要求机械臂做什么）
 *   - `state`  ：权威状态（机械臂现在处在哪）
 *   - `feedback`：真实机械臂对某条 command 的回执 / 实际值
 */
import type { JointState } from '../model/Pose';
import type { RobotCommand } from '../model/RobotCommand';
import type { RobotState } from '../model/RobotState';

export type TransportStatus = 'disconnected' | 'connecting' | 'connected' | 'error';

export interface TransportStatusDetail {
  status: TransportStatus;
  /** 断开/错误原因，用于状态面板 */
  reason?: string;
  timestamp: number;
}

export type Unsubscribe = () => void;

/**
 * 传输统计（spec §二十七 状态显示）。
 *
 * 只声明**所有传输都能提供**的字段；传输专有的调参（舵机角速度、重连次数…）
 * 一律可选 —— 这样状态面板只依赖本接口，换上 WebSocket / 串口后不用改。
 */
export interface TransportStats {
  /** 传输类型标识（mock / websocket / serial） */
  kind: string;
  /** 已受理的命令数（含被拒绝的） */
  sent: number;
  /** 已回推的状态帧数 */
  received: number;
  /** 因丢帧而未回推的帧数 */
  dropped: number;
  /** 被限位拒绝的命令数 */
  rejected: number;
  /** 当前是否正在逼近目标 */
  moving: boolean;
  /** 当前 |actual − command| 的最大关节差（deg） */
  lagDeg: number;
  /** Mock 专有：舵机最大角速度（deg/s） */
  maxSpeedDegPerSec?: number;
  /** Mock 专有：单程传输延迟（ms） */
  latencyMs?: number;
  /** Mock 专有：回推丢帧概率 */
  dropRate?: number;
  /** Mock 专有：是否按关节限位校验命令 */
  enforceLimits?: boolean;
  /** WebSocket 专有：重连次数 */
  reconnects?: number;
  /** WebSocket 专有：最近一次心跳往返时延（ms） */
  rttMs?: number | null;
}

export interface RobotTransport {
  /** 传输类型标识，用于日志与状态展示（mock / websocket） */
  readonly kind: string;

  connect(): Promise<void>;

  disconnect(): Promise<void>;

  /** 下发关节角命令（上层已完成安全检查） */
  sendJointState(state: JointState): Promise<void>;

  /** 下发结构化命令（可选能力：Mock 直接本地应用，WebSocket 走协议文本） */
  sendCommand?(command: RobotCommand): Promise<void>;

  /** 订阅机械臂状态（真实反馈 / Mock 回显） */
  onState(callback: (state: RobotState) => void): Unsubscribe;

  /** 订阅连接状态 */
  onStatus(callback: (detail: TransportStatusDetail) => void): Unsubscribe;

  /** 当前连接状态 */
  status(): TransportStatus;

  /** 传输统计（**可选能力**）；未实现时状态面板只显示连接信息 */
  stats?(): TransportStats;
}
