/**
 * 全局唯一状态仓库（Zustand）。
 *
 * 设计约束（spec §十七 / §十九）：
 *   - 虚拟机械臂与真实机械臂共享**同一个 RobotState**；
 *   - 所有状态变化必须经 `RobotState`，不允许组件各自持有局部关节角副本；
 *   - `mode`（simulation / real）与 `controlSource` 分离：前者决定"要不要发给真实机械臂"，
 *     后者决定"当前姿态是谁驱动的"，用于打破虚拟↔真实回环；
 *   - `commandJoints` 与 `actualJoints` 严格区分（spec §二十六 / §三十四）：
 *       command = 我们要求机械臂去的角度
 *       actual  = 机械臂实际处在的角度（Phase 11 起由真实反馈驱动；仿真下等于 command）
 *
 * Phase 6 新增「末端目标」概念，三者的关系：
 *   - `target`        = 我想去哪（XYZ 输入或鼠标拖动指定；**越界也照常保留**）
 *   - `commandJoints` = 实际去哪（恒在限位内；目标越界时**逐位不变**）
 *   - `endEffector`   = commandJoints 的 FK 结果，它与 `target` 的差即"还差多远"
 *
 *   `target` 只在 `moveTo()` 时由用户显式指定。滑杆 / HOME / ZERO 属于**非目标驱动**，
 *   它们会让 `target` 自动跟随新 TCP —— 否则幽灵标记会停在旧位置，误报"未到位"。
 *
 * Phase 7 新增「传输驱动」概念，`actualJoints` 的写入者因此分两种：
 *   - `transportDriven === false`（未连接）：`actual ≡ command`，仿真下立即跟随（Phase 1–6 行为）
 *   - `transportDriven === true` （已连接）：`actualJoints` **只由 Transport 回推写入**，
 *     命令路径不再碰它 —— 否则 Mock 的滞后回推会被下一次命令立刻覆盖，误差永远显示 0
 *
 *   注意 3D 场景渲染的一直是 `commandJoints`（虚拟臂 = 命令的即时预测），
 *   `actualJoints` 只进状态面板，两者分离才能看出"真机跟上了没有"。
 */
import { create } from 'zustand';
import {
  endEffectorPose,
  homeJointState,
  jointByRole,
  jointIds,
  loadRobotModel,
  movableJoints,
  solveIk,
  zeroJointState,
  type DragPlaneMode,
  type IkBranch,
  type IkPreference,
  type IkReason,
  type IkResult,
  type JointState,
  type RobotModel,
  type Transform,
  type TransportStats,
  type Vec3,
} from '@robot/index';

export type RobotMode = 'simulation' | 'real';
export type RobotControlSource = 'virtual' | 'real' | 'command';

export interface LogEntry {
  id: number;
  time: string;
  // 'err' 是本地告警（例如 mode=real 准入校验失败、安全门拦截命令）——
  // 它不来自链路，所以不能复用 'in'/'out'；用独立 kind 才能在日志里一眼分辨。
  kind: 'in' | 'out' | 'sys' | 'err';
  text: string;
}

/** 最近一次末端目标求解的结果 */
export interface TargetStatus {
  ok: boolean;
  /** 失败原因（`ok === true` 时无） */
  reason?: IkReason;
  /** `JOINT_LIMIT` 时越界的关节 id */
  joint?: string;
  /** 人类可读说明，格式对齐 `protocol/serial-v1.md` 的 `ERR JOINT ...` */
  message?: string;
  /** 成功时的解支 */
  branch?: IkBranch;
  /** 成功时的 FK 自查残差（mm） */
  residual?: number;
  /** 成功时的底座方位角（degree） */
  azimuth?: number;
}

export type ToggleKey =
  | 'showJointAxes'
  | 'showJointOrigins'
  | 'showWorldAxes'
  | 'showRobotAxes'
  | 'showTcp';

interface RobotStore {
  model: RobotModel;
  /** 命令关节角（我们要求去的角度） */
  commandJoints: JointState;
  /** 实际关节角（机械臂真实所处角度） */
  actualJoints: JointState;
  /** 命令末端位姿（FK of commandJoints） */
  endEffector: Transform;
  /** 实际末端位姿（FK of actualJoints） */
  actualEndEffector: Transform;

  /** 末端目标点（"我想去哪"，mm）—— 越界时依然保留，便于用户看到差多远 */
  target: Vec3;
  /** 最近一次目标求解结果；滑杆等非目标驱动会清空 */
  ikStatus: TargetStatus | null;
  /** 拖动平面模式（拖动开始时被冻结进 DragHandle 的局部状态） */
  dragPlane: DragPlaneMode;
  /** 是否正在拖动末端（拖动期间需禁用轨道旋转，否则会边转相机边拖） */
  dragging: boolean;

  mode: RobotMode;
  controlSource: RobotControlSource;
  connection: 'connected' | 'disconnected';
  connectionLabel: string;
  /** 已连接的传输类型（`mock` / `websocket`）；null = 未接入 */
  transportKind: string | null;
  /**
   * `actualJoints` 是否由 Transport 回推驱动。
   * 为 true 时命令路径**不再**写 `actualJoints`（见文件头 Phase 7 说明）。
   */
  transportDriven: boolean;
  /** 传输统计（延迟 / 丢帧 / 累计帧数），由 `transportBridge` 定时回写 */
  transportStats: TransportStats | null;

  showJointAxes: boolean;
  showJointOrigins: boolean;
  showWorldAxes: boolean;
  showRobotAxes: boolean;
  showTcp: boolean;

  cameraResetToken: number;
  /** 运行期 FK ↔ Three.js 一致性误差（mm），由场景实时回写（Phase 3 验收的运行态证据） */
  alignmentErrorMm: number | null;
  log: LogEntry[];

  setJoint(jointId: string, angleDeg: number): void;
  setCommandJoints(next: JointState): void;
  setActualJoints(next: JointState): void;
  goHome(): void;
  goZero(): void;
  /** 求解末端目标并驱动关节；返回 IK 原始结果供调用方分支 */
  moveTo(xyz: Vec3, opts?: { prefer?: IkPreference }): IkResult;
  /** 把目标重置为当前 TCP（即"取消目标"） */
  resetTarget(): void;
  setDragPlane(mode: DragPlaneMode): void;
  setDragging(value: boolean): void;
  /**
   * 由 `transportBridge` 调用：登记 / 注销传输连接。
   * `kind === null` 表示断开，`transportDriven` 随之复位。
   */
  setConnection(
    kind: string | null,
    status: 'connected' | 'disconnected',
    label: string,
  ): void;
  setTransportStats(stats: TransportStats | null): void;
  setMode(mode: RobotMode): void;
  setControlSource(source: RobotControlSource): void;
  setToggle(key: ToggleKey, value: boolean): void;
  resetCamera(): void;
  setAlignmentError(value: number): void;
  pushLog(kind: LogEntry['kind'], text: string): void;
  clearLog(): void;
}

const model = loadRobotModel();

let logSeq = 0;
function makeLogEntry(kind: LogEntry['kind'], text: string): LogEntry {
  logSeq += 1;
  const now = new Date();
  const time = `${now.toTimeString().slice(0, 8)}.${String(now.getMilliseconds()).padStart(3, '0')}`;
  return { id: logSeq, time, kind, text };
}

function clipJointState(partial: Partial<JointState>): JointState {
  const out: JointState = {};
  for (const id of jointIds(model)) {
    const joint = model.joints.find((j) => j.id === id);
    const fallback = joint ? joint.limits.min : 0;
    const raw = partial[id] ?? fallback;
    out[id] = joint ? Math.min(joint.limits.max, Math.max(joint.limits.min, raw)) : raw;
  }
  return out;
}

/**
 * 非目标驱动的关节变化（滑杆 / HOME / ZERO）统一出口。
 * 与 `moveTo` 的区别只有两点：清空 `ikStatus`、让 `target` 跟随新 TCP。
 *
 * `transportDriven` 为 true 时**不写** `actualJoints` —— 那时它是 Transport 回推的领地，
 * 命令路径若也去写，就会把回推的滞后值立刻覆盖掉，误差显示永远为 0。
 */
function deriveVirtual(
  joints: JointState,
  syncTarget: boolean,
  transportDriven: boolean,
): Partial<RobotStore> {
  const pose = endEffectorPose(model, joints);
  const patch: Partial<RobotStore> = {
    commandJoints: joints,
    endEffector: pose,
    controlSource: 'virtual',
  };
  if (!transportDriven) {
    // 未接入传输：仿真下实际值立即跟随命令（Phase 1–6 行为）
    patch.actualJoints = joints;
    patch.actualEndEffector = pose;
  }
  if (syncTarget) {
    patch.target = [pose.position[0], pose.position[1], pose.position[2]];
    patch.ikStatus = null;
  }
  return patch;
}

export const useRobotStore = create<RobotStore>((set, get) => {
  const initial = homeJointState(model);
  const initialPose = endEffectorPose(model, initial);
  return {
    model,
    commandJoints: initial,
    actualJoints: initial,
    endEffector: initialPose,
    actualEndEffector: initialPose,

    // 初始把目标放在 HOME 的 TCP 上：幽灵标记与 TCP 重合，场景里不显眼
    target: [initialPose.position[0], initialPose.position[1], initialPose.position[2]],
    ikStatus: null,
    dragPlane: 'xy',
    dragging: false,

    // 默认 Simulation：防止网页一打开就直接控制真实机械臂（spec §三十一）
    mode: 'simulation',
    controlSource: 'virtual',
    connection: 'disconnected',
    connectionLabel: '未接入（可在 Connection 面板连接 MockTransport）',
    transportKind: null,
    transportDriven: false,
    transportStats: null,

    showJointAxes: true,
    showJointOrigins: false,
    showWorldAxes: true,
    showRobotAxes: false,
    showTcp: true,
    cameraResetToken: 0,
    alignmentErrorMm: null,
    log: [
      makeLogEntry(
        'sys',
        `RobotModel 载入：${model.name}（${model.links.length} 连杆 / ${model.joints.length} 关节 / ${model.actuators.length} 舵机）`,
      ),
    ],

    setJoint(jointId, angleDeg) {
      const next = clipJointState({ ...get().commandJoints, [jointId]: angleDeg });
      set(deriveVirtual(next, true, get().transportDriven));
    },

    setCommandJoints(next) {
      set(deriveVirtual(clipJointState(next), true, get().transportDriven));
    },

    setActualJoints(next) {
      const clipped = clipJointState(next);
      set({
        actualJoints: clipped,
        actualEndEffector: endEffectorPose(model, clipped),
        controlSource: 'real',
      });
    },

    goHome() {
      const home = clipJointState(homeJointState(model));
      set(deriveVirtual(home, true, get().transportDriven));
      get().pushLog('sys', `HOME 位姿 ${JSON.stringify(home)}`);
    },

    goZero() {
      // 关节空间原点：各关节 0°。小臂（绝对角）的真机可达区间是 108.44..141.86°，
      // 0° 不可达，故 clipJointState 会把它钳到最竖直的可达角 —— 结果是真机约束，不是 bug。
      const zero = clipJointState(zeroJointState(model));
      set(deriveVirtual(zero, true, get().transportDriven));
      get().pushLog('sys', `零位：各关节 0°（限位钳位后 ${JSON.stringify(zero)}）`);
    },

    moveTo(xyz, opts = {}) {
      const current = get();
      // `prefer: 'nearest'` + `near: 当前命令角` —— 拖动经过工作空间内边界时不翻支；
      // `seed: 当前命令角` 让夹爪等未参与解算的关节保持原值，返回值可直接喂 FK 闭环。
      const result = solveIk(model, xyz, {
        prefer: opts.prefer ?? 'nearest',
        near: current.commandJoints,
        seed: current.commandJoints,
      });

      if (result.success) {
        const joints = clipJointState(result.joints);
        set({
          ...deriveVirtual(joints, false, current.transportDriven),
          target: [xyz[0], xyz[1], xyz[2]],
          ikStatus: {
            ok: true,
            branch: result.branch,
            residual: result.residual,
            azimuth: result.azimuth,
          },
        });
      } else {
        // ⚠️ 目标越界时关节**逐位不变**（Phase 6 已拍板）：保留 target 让用户看到"差多远"，
        //    但绝不静默钳位 —— 钳位会让工作空间边界从界面上消失，也无法保证钳位点满足关节限位。
        set({
          target: [xyz[0], xyz[1], xyz[2]],
          ikStatus: {
            ok: false,
            reason: result.reason,
            joint: result.joint,
            message: result.message,
          },
        });
      }
      return result;
    },

    resetTarget() {
      const pose = get().endEffector.position;
      set({
        target: [pose[0], pose[1], pose[2]],
        ikStatus: null,
      });
    },

    setDragPlane(mode) {
      set({ dragPlane: mode });
    },

    setDragging(value) {
      set({ dragging: value });
    },

    setConnection(kind, status, label) {
      const driven = kind !== null && status === 'connected';
      const patch: Partial<RobotStore> = {
        transportKind: kind,
        transportDriven: driven,
        connection: status,
        connectionLabel: label,
      };
      if (!driven) {
        // 断开后回到「仿真立即跟随」：把 actual 拉回 command 并清掉统计。
        // 不这么做的话，最后那次回推的滞后值会永久挂在状态面板上显示一个假误差
        // —— 因为已经没有任何 transport 再去驱动它收敛了。
        const joints = get().commandJoints;
        patch.transportStats = null;
        patch.actualJoints = joints;
        patch.actualEndEffector = endEffectorPose(model, joints);
        patch.controlSource = 'virtual';
      }
      set(patch);
    },

    setTransportStats(stats) {
      set({ transportStats: stats });
    },

    setMode(mode) {
      // ⚠️ 这不是一个"纯 UI 开关"。
      //
      // 规范（本文件头部 §十七）要求 `mode` **决定"要不要发给真实机械臂"**，
      // 但早期实现只写了 `set({ mode })` + 日志 —— 于是点下「Real Robot」后
      // 命令照样走当时连着的 transport（多半是 Mock），**真机纹丝不动**，
      // 而 UI 却显示「Real（真实机械臂）」。这是一个"看起来成功了"的静默失败。
      //
      // 因此这里把 mode 变成**带校验的意图**：切到 real 时立刻核对链路是否具备
      // 驱动真机的能力，不具备就**明确告警**（而不是让用户以为已切过去）。
      set({ mode });

      if (mode !== 'real') {
        get().pushLog('sys', '切换到 Simulation（命令不再下发给真实机械臂）');
        return;
      }

      // ---- 以下为 mode === 'real' 的准入校验 ----
      const st = get();
      const stats = st.transportStats;
      const device = stats && 'device' in stats ? (stats.device as string | null) : null;

      if (st.transportKind === null || !st.transportDriven) {
        // 没有连接：命令无处可去。这是"点了 Real Robot 但真机不动"的第一大原因。
        get().pushLog(
          'err',
          'Real Robot 未生效：当前**未连接**任何传输。请先在 Connection 面板连接后端' +
            '（真机需用 config.serial.yaml 启动 armpilot-backend）',
        );
        return;
      }

      if (st.transportKind === 'mock') {
        get().pushLog(
          'err',
          'Real Robot 未生效：当前连接的是 **MockTransport**（浏览器内仿真，不碰硬件）。' +
            '请在 Connection 面板切到 WebSocket 并连接真机后端',
        );
        return;
      }

      if (device !== null && device !== 'serial') {
        // 后端连上了，但它自己也没接真机（device=sim 表示用的是内置假固件）。
        get().pushLog(
          'err',
          `Real Robot 未生效：后端链路末端是「${device}」而非 serial。` +
            '请用 config.serial.yaml 启动后端（并把机械臂接到配置的串口）',
        );
        return;
      }

      // 末端是 serial（或尚未收到 hello，暂按乐观放行并提示等待）
      get().pushLog(
        'sys',
        device === 'serial'
          ? 'Real Robot 已启用：命令将下发给真实机械臂（链路末端 serial）'
          : 'Real Robot 已启用：等待后端 hello 确认链路末端…',
      );
    },

    setControlSource(source) {
      set({ controlSource: source });
    },

    setToggle(key, value) {
      set({ [key]: value } as unknown as Partial<RobotStore>);
    },

    resetCamera() {
      set((state) => ({ cameraResetToken: state.cameraResetToken + 1 }));
    },

    setAlignmentError(value) {
      set({ alignmentErrorMm: value });
    },

    pushLog(kind, text) {
      set((state) => ({ log: [...state.log.slice(-199), makeLogEntry(kind, text)] }));
    },

    clearLog() {
      set({ log: [] });
    },
  };
});

// ---------------------------------------------------------------------------
// 派生选择器（供组件使用，避免在组件里重复写业务规则）
// ---------------------------------------------------------------------------

/** 关节显示名：J1/J2/J3 + Gripper */
export function jointLabel(jointId: string): string {
  const joint = model.joints.find((j) => j.id === jointId);
  if (!joint) return jointId;
  switch (joint.role) {
    case 'base':
      return 'J1 Base';
    case 'shoulder':
      return 'J2 Shoulder';
    case 'elbow':
      return 'J3 Elbow';
    case 'gripper':
      return 'Gripper';
    default:
      return joint.name;
  }
}

/** 定位关节（不含夹爪）—— IK 只解这三个（spec §十三） */
export function positioningJointIds(): string[] {
  return movableJoints(model)
    .filter((joint) => joint.role !== 'gripper')
    .map((joint) => joint.id);
}

export function gripperJointId(): string | null {
  return jointByRole(model, 'gripper')?.id ?? null;
}

/** 目标点与当前 TCP 的距离（mm）—— 0 表示已到位 */
export function targetGapMm(target: Vec3, joints: JointState): number {
  const tcp = endEffectorPose(model, joints).position;
  return Math.hypot(target[0] - tcp[0], target[1] - tcp[1], target[2] - tcp[2]);
}

export { model as robotModel };
