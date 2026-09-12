/**
 * MockTransport —— 纯虚拟闭环（spec §二十 / §三十八，Phase 7）。
 *
 * 它把自己伪装成"一台真的机械臂"：
 *
 *   上层 sendJointState(命令)  ──(传输延迟)──▶  固件受理
 *                                              │
 *                            有限角速度逼近（舵机不可能瞬间到位）
 *                                              │
 *   上层 onState(实际)        ◀──(状态回传)────  实际关节角
 *
 * **为什么不做成"零延迟等值回显"**：那样 `actual ≡ command`、误差恒为 0，
 * 链路虽然通了，但误差显示、收敛逻辑、丢帧容错**一条都没被验证**，
 * 全部推迟到 Phase 11 真机时第一次暴露。所以这里如实建模三件事：
 *
 *   1. **舵机有限角速度**（`maxSpeedDegPerSec`）—— 拖动中 Actual 真实滞后，松手后收敛
 *   2. **传输延迟**（`latencyMs`）—— 命令不是立刻生效
 *   3. **回推丢帧**（`dropRate`）—— 上位机可能收不到状态帧，但机械臂照常运动
 *
 * 另外可开启 `enforceLimits`：收到越界命令时回 `ERR JOINT ...` 并**保持不动**，
 * 复刻固件的行为，为 Phase 9/10 的错误路径铺路。
 *
 * ⚠️ 本类绝不回写 `commandJoints`：回推的状态只描述"机械臂现在在哪"，
 * 若它反过来改命令，就形成 `命令→状态→命令` 的无限回环（spec §十九）。
 */
import { endEffectorPose } from '../kinematics/fk';
import type { JointState } from '../model/Pose';
import type { RobotCommand } from '../model/RobotCommand';
import { homeJointState, movableJoints } from '../model/RobotModel';
import type { RobotModel } from '../model/RobotModel';
import { createRobotState, type RobotState } from '../model/RobotState';
import type {
  RobotTransport,
  TransportStatus,
  TransportStatusDetail,
  Unsubscribe,
} from './RobotTransport';
import { realTimer, type TimerLike } from './timer';

/** 关节角收敛阈值（degree）：小于它就算"到位"，停掉 tick 不再空转 */
const ARRIVED_EPS_DEG = 1e-6;

/** 可调运行参数（`ConnectionControl` 面板直接绑这几个） */
export interface MockTransportTuning {
  /** 舵机最大角速度（deg/s）；`0` 或非有限值 = 瞬时到位 */
  maxSpeedDegPerSec: number;
  /** 单程传输延迟（ms）：命令送达与状态回传各施加一次 */
  latencyMs: number;
  /** 回推丢帧概率 [0, 1) */
  dropRate: number;
  /** 是否按关节限位校验命令（模拟固件回 `ERR JOINT`） */
  enforceLimits: boolean;
}

export interface MockTransportOptions extends Partial<MockTransportTuning> {
  model: RobotModel;
  /** 可注入时钟（测试用虚拟时钟） */
  timer?: TimerLike;
  /** 仿真 tick 周期（ms） */
  tickMs?: number;
  /** 初始实际关节角（默认 HOME） */
  initialJoints?: JointState;
  /** 随机源；注入后丢帧可复现 */
  random?: () => number;
}

/** 传输统计（面板显示 + 测试断言） */
export interface MockTransportStats extends MockTransportTuning {
  /** 传输类型标识（供通用状态面板使用） */
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
  /** 当前 |actual − target| 的最大关节差（deg） */
  lagDeg: number;
}

/** 默认调参：MG90S 带载约 240°/s；15ms 约等于串口一次往返 */
export const DEFAULT_MOCK_TUNING: MockTransportTuning = {
  maxSpeedDegPerSec: 240,
  latencyMs: 15,
  dropRate: 0,
  enforceLimits: true,
};

/** 固件式错误文案：`ERR JOINT elbow 95 (limit 108.44..141.86)` */
function formatJointLimitError(jointId: string, value: number, min: number, max: number): string {
  return `ERR JOINT ${jointId} ${value.toFixed(2)} (limit ${min.toFixed(2)}..${max.toFixed(2)})`;
}

function maxAbsJointDiff(a: JointState, b: JointState, ids: string[]): number {
  let worst = 0;
  for (const id of ids) {
    const d = Math.abs((a[id] ?? 0) - (b[id] ?? 0));
    if (d > worst) worst = d;
  }
  return worst;
}

export class MockTransport implements RobotTransport {
  readonly kind = 'mock';

  private readonly model: RobotModel;
  private readonly timer: TimerLike;
  private readonly tickMs: number;
  private readonly random: () => number;
  private readonly ids: string[];
  private readonly tuning: MockTransportTuning;

  /** 机械臂"现在在哪"（模拟值） */
  private actual: JointState;
  /** 机械臂"要去哪"（已送达的最新命令） */
  private target: JointState;

  private connection: TransportStatus = 'disconnected';
  private tickHandle: number | null = null;
  /** 延迟队列里的待送达命令（`emergency_stop` 要能一键清空） */
  private inFlight: number[] = [];

  private stateListeners = new Set<(state: RobotState) => void>();
  private statusListeners = new Set<(detail: TransportStatusDetail) => void>();

  private counters = { sent: 0, received: 0, dropped: 0, rejected: 0 };

  constructor(options: MockTransportOptions) {
    this.model = options.model;
    this.timer = options.timer ?? realTimer;
    this.tickMs = options.tickMs ?? 20;
    this.random = options.random ?? Math.random;
    this.ids = movableJoints(this.model).map((joint) => joint.id);

    this.tuning = {
      maxSpeedDegPerSec: options.maxSpeedDegPerSec ?? DEFAULT_MOCK_TUNING.maxSpeedDegPerSec,
      latencyMs: options.latencyMs ?? DEFAULT_MOCK_TUNING.latencyMs,
      dropRate: options.dropRate ?? DEFAULT_MOCK_TUNING.dropRate,
      enforceLimits: options.enforceLimits ?? DEFAULT_MOCK_TUNING.enforceLimits,
    };

    const initial = options.initialJoints ?? homeJointState(this.model);
    this.actual = { ...initial };
    this.target = { ...initial };
  }

  // -------------------------------------------------------------------------
  // RobotTransport
  // -------------------------------------------------------------------------

  async connect(): Promise<void> {
    if (this.connection === 'connected') return;
    this.emitStatus('connecting');
    this.connection = 'connected';
    this.emitStatus('connected');
  }

  async disconnect(): Promise<void> {
    if (this.connection === 'disconnected') return;
    this.cancelInFlight();
    this.stopTicking();
    this.connection = 'disconnected';
    this.emitStatus('disconnected');
  }

  async sendJointState(state: JointState): Promise<void> {
    if (this.connection !== 'connected') {
      this.emitStatus('error', '未连接：命令被丢弃');
      return;
    }
    this.counters.sent += 1;

    const rejection = this.tuning.enforceLimits ? this.validate(state) : null;
    if (rejection) {
      this.counters.rejected += 1;
      this.emitStatus('error', rejection);
      // ⚠️ 被拒绝时**位置保持不动**：真实的固件也不会执行非法指令
      return;
    }

    // 传输延迟：命令不是立刻生效
    if (this.tuning.latencyMs <= 0) {
      this.applyTarget(state);
      return;
    }
    const handle = this.timer.setTimeout(() => {
      this.inFlight = this.inFlight.filter((h) => h !== handle);
      this.applyTarget(state);
    }, this.tuning.latencyMs);
    this.inFlight.push(handle);
  }

  async sendCommand(command: RobotCommand): Promise<void> {
    switch (command.type) {
      case 'joint_command':
      case 'gripper': {
        if (!command.joints) return;
        // `gripper` 只改夹爪，其余关节沿用当前目标 —— 满足"夹爪不影响定位链路"
        const merged: JointState =
          command.type === 'gripper' ? { ...this.target, ...command.joints } : command.joints;
        await this.sendJointState(merged);
        return;
      }
      case 'home': {
        await this.sendJointState(homeJointState(this.model));
        return;
      }
      case 'stop': {
        // 停在当下：目标立刻改为当前位置
        this.cancelInFlight();
        this.target = { ...this.actual };
        this.stopTicking();
        this.emitStatus('connected', 'STOP：冻结在当前姿态');
        return;
      }
      case 'emergency_stop': {
        this.cancelInFlight();
        this.target = { ...this.actual };
        this.stopTicking();
        this.emitStatus('error', 'EMERGENCY STOP：已清空待发队列并冻结');
        return;
      }
      default:
        return;
    }
  }

  onState(callback: (state: RobotState) => void): Unsubscribe {
    this.stateListeners.add(callback);
    return () => this.stateListeners.delete(callback);
  }

  onStatus(callback: (detail: TransportStatusDetail) => void): Unsubscribe {
    this.statusListeners.add(callback);
    return () => this.statusListeners.delete(callback);
  }

  status(): TransportStatus {
    return this.connection;
  }

  // -------------------------------------------------------------------------
  // Mock 专有：调参 / 统计
  // -------------------------------------------------------------------------

  /** 运行期调参（面板滑杆用）。改动立即影响后续 tick。 */
  tune(patch: Partial<MockTransportTuning>): void {
    Object.assign(this.tuning, patch);
    if (this.isMoving()) this.ensureTicking();
  }

  stats(): MockTransportStats {
    return {
      kind: this.kind,
      ...this.tuning,
      ...this.counters,
      moving: this.isMoving(),
      lagDeg: maxAbsJointDiff(this.actual, this.target, this.ids),
    };
  }

  /** 当前模拟的实际关节角（只读快照，测试用） */
  actualJoints(): JointState {
    return { ...this.actual };
  }

  // -------------------------------------------------------------------------
  // 内部
  // -------------------------------------------------------------------------

  private validate(state: JointState): string | null {
    for (const joint of movableJoints(this.model)) {
      const value = state[joint.id];
      if (value === undefined) continue;
      if (value < joint.limits.min - 1e-9 || value > joint.limits.max + 1e-9) {
        return formatJointLimitError(joint.id, value, joint.limits.min, joint.limits.max);
      }
    }
    return null;
  }

  private applyTarget(state: JointState): void {
    if (this.connection !== 'connected') return;
    this.target = { ...this.target, ...state };
    if (!this.isMoving()) return;
    // 瞬时到位模式下立刻收敛；有限速度模式下先走一步，再交给 tick 继续
    this.tick();
    if (this.isMoving()) this.ensureTicking();
  }

  private isMoving(): boolean {
    return maxAbsJointDiff(this.actual, this.target, this.ids) > ARRIVED_EPS_DEG;
  }

  private ensureTicking(): void {
    if (this.tickHandle !== null) return;
    this.tickHandle = this.timer.setInterval(() => this.tick(), this.tickMs);
  }

  private stopTicking(): void {
    if (this.tickHandle === null) return;
    this.timer.clearInterval(this.tickHandle);
    this.tickHandle = null;
  }

  private cancelInFlight(): void {
    for (const handle of this.inFlight) this.timer.clearTimeout(handle);
    this.inFlight = [];
  }

  /** 一个仿真步：每个关节最多前进 `maxSpeed × dt`，不越过目标 */
  private tick(): void {
    if (this.connection !== 'connected') return;

    const maxStep = this.tuning.maxSpeedDegPerSec * (this.tickMs / 1000);
    const instant = !Number.isFinite(maxStep) || maxStep <= 0;

    let changed = false;
    for (const id of this.ids) {
      const current = this.actual[id] ?? 0;
      const goal = this.target[id] ?? 0;
      const delta = goal - current;
      if (delta === 0) continue;
      if (instant || Math.abs(delta) <= maxStep) {
        this.actual[id] = goal;
      } else {
        this.actual[id] = current + Math.sign(delta) * maxStep;
      }
      changed = true;
    }

    if (changed) this.emitState();
    if (!this.isMoving()) this.stopTicking();
  }

  private emitState(): void {
    // 丢帧只影响"上报"，不影响机械臂的实际位置 —— 真实链路就是这样
    if (this.tuning.dropRate > 0 && this.random() < this.tuning.dropRate) {
      this.counters.dropped += 1;
      return;
    }
    this.counters.received += 1;
    const joints = { ...this.actual };
    const state = createRobotState(
      joints,
      endEffectorPose(this.model, joints),
      'real',
      this.timer.now(),
    );
    for (const listener of this.stateListeners) listener(state);
  }

  private emitStatus(status: TransportStatus, reason?: string): void {
    const detail: TransportStatusDetail = {
      status,
      timestamp: this.timer.now(),
      ...(reason ? { reason } : {}),
    };
    for (const listener of this.statusListeners) listener(detail);
  }
}
