/**
 * 关节控制（Phase 4）：J1/J2/J3 + Gripper 滑杆，直接驱动 JointState。
 *
 * 「改变 Joint State → 立即更新 3D Model」由 store 统一驱动：
 * 组件只改命令关节角，实际 3D 姿态与末端 XYZ 全部由 RobotState / FK 派生，
 * 组件内不存任何角度副本。
 *
 * Real / Simulation（Phase 9 修正）：这两个按钮**不再是纯样式开关**。
 * 早期实现里 `setMode` 只改 UI，命令照样走当前 transport，于是"点了 Real Robot
 * 但真机不动"（或更糟：以为在仿真、其实在驱动真机）。
 * 现在：
 *   - `setMode` 会**校验链路**并在不具备时告警（见 robotStore.setMode）
 *   - `transportBridge.flush` 有**安全门**：simulation 模式下拒绝下发给真机链路
 *   - 本组件把"当前模式实际是否在下发"**显式显示出来**，不留静默状态
 */
import { movableJoints } from '@robot/index';
import { jointLabel, useRobotStore } from '@/store/robotStore';

export function JointControl() {
  const model = useRobotStore((s) => s.model);
  const commandJoints = useRobotStore((s) => s.commandJoints);
  const setJoint = useRobotStore((s) => s.setJoint);
  const goHome = useRobotStore((s) => s.goHome);
  const goZero = useRobotStore((s) => s.goZero);
  const mode = useRobotStore((s) => s.mode);
  const setMode = useRobotStore((s) => s.setMode);
  const transportKind = useRobotStore((s) => s.transportKind);
  const transportDriven = useRobotStore((s) => s.transportDriven);
  const transportStats = useRobotStore((s) => s.transportStats);

  // 链路末端（sim / serial）—— 决定 Real Robot 是否真的能驱动硬件。
  const device =
    transportStats !== null && 'device' in transportStats
      ? (transportStats.device as string | null)
      : null;
  const isRealLink = transportKind === 'websocket' && device === 'serial';

  // 当前模式下命令的实际去向 —— 三态，必须说清楚
  const routing = !transportDriven
    ? { text: '纯仿真（未连接，命令只改虚拟臂）', tone: 'dim' as const }
    : mode === 'simulation' && isRealLink
      ? { text: '已拦截：Simulation 模式下不下发给真机', tone: 'warn' as const }
      : mode === 'real' && isRealLink
        ? { text: '★ 正在驱动真实机械臂（链路末端 serial）', tone: 'live' as const }
        : mode === 'real' && transportKind === 'websocket'
          ? { text: `Real 模式下后端末端是「${device ?? '未知'}」，非真机`, tone: 'warn' as const }
          : mode === 'real'
            ? { text: 'Real 模式下连接的是 Mock，未驱动硬件', tone: 'warn' as const }
            : { text: `仿真（连接 ${transportKind ?? '—'}，命令照常下发到仿真链路）`, tone: 'dim' as const };

  return (
    <div className="card">
      <h2>关节控制 · Joint Control</h2>

      {movableJoints(model).map((joint) => {
        const value = commandJoints[joint.id] ?? joint.limits.min;
        return (
          <div className="slider-row" key={joint.id}>
            <div className="label">
              <span className="name">{jointLabel(joint.id)}</span>
              <span className="val">
                {value.toFixed(1)}
                <span className="dim">°</span>
              </span>
            </div>
            <input
              type="range"
              min={joint.limits.min}
              max={joint.limits.max}
              step={0.5}
              value={value}
              onChange={(event) => setJoint(joint.id, Number(event.target.value))}
            />
            <div className="limits">
              限位 {joint.limits.min}° … {joint.limits.max}°　轴 [{joint.axis.join(', ')}]
            </div>
          </div>
        );
      })}

      {/* 命令去向的显式提示：把"到底在驱动谁"摆在按钮旁边，不留静默状态 */}
      <div className={`mode-hint ${routing.tone}`} data-testid="mode-routing">
        {routing.text}
      </div>

      <div className="btn-row" style={{ marginTop: 10 }}>
        <button type="button" onClick={goHome}>
          HOME
        </button>
        <button type="button" onClick={goZero}>
          ZERO
        </button>
        <button
          type="button"
          className={mode === 'simulation' ? 'active' : ''}
          onClick={() => setMode('simulation')}
        >
          Simulation
        </button>
        <button
          type="button"
          className={mode === 'real' ? 'active' : ''}
          onClick={() => setMode('real')}
        >
          Real Robot
        </button>
      </div>
    </div>
  );
}
