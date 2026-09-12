/**
 * 状态面板（spec §二十七 / §三十四）。
 *
 * 必须显示：Connection / Robot / Control、每个关节的 Command 与 Actual、
 * 以及末端 XYZ。Command 与 Actual 严格区分，误差 = Actual − Command。
 */
import { jointStateError, movableJoints, positionError } from '@robot/index';
import { jointLabel, useRobotStore } from '@/store/robotStore';

export function StatusPanel() {
  const model = useRobotStore((s) => s.model);
  const commandJoints = useRobotStore((s) => s.commandJoints);
  const actualJoints = useRobotStore((s) => s.actualJoints);
  const endEffector = useRobotStore((s) => s.endEffector);
  const actualEndEffector = useRobotStore((s) => s.actualEndEffector);
  const mode = useRobotStore((s) => s.mode);
  const controlSource = useRobotStore((s) => s.controlSource);
  const connection = useRobotStore((s) => s.connection);
  const connectionLabel = useRobotStore((s) => s.connectionLabel);
  const alignmentErrorMm = useRobotStore((s) => s.alignmentErrorMm);
  const transportStats = useRobotStore((s) => s.transportStats);
  const transportDriven = useRobotStore((s) => s.transportDriven);

  const errors = jointStateError(commandJoints, actualJoints);
  const tcpError = positionError(endEffector, actualEndEffector);
  const aligned = alignmentErrorMm !== null && alignmentErrorMm < 0.1;

  return (
    <div className="card">
      <h2>状态 · Status</h2>

      <div className="kv">
        <span>Connection</span>
        <span>
          <span className={`badge ${connection === 'connected' ? 'ok' : 'bad'}`}>
            <i className="dot" />
            {connection === 'connected' ? 'Connected' : 'Disconnected'}
          </span>
        </span>
      </div>
      <div className="kv">
        <span />
        <span className="dim" style={{ fontSize: 11 }}>
          {connectionLabel}
        </span>
      </div>
      <div className="kv">
        <span>Robot</span>
        <span>{mode === 'real' ? 'Real（真实机械臂）' : 'Virtual（仿真）'}</span>
      </div>
      <div className="kv">
        <span>Control</span>
        <span>{controlSource}</span>
      </div>
      <div className="kv">
        <span>Transport</span>
        <span>
          {transportStats === null
            ? '—（未接入）'
            : `${transportStats.sent} 发 / ${transportStats.received} 收 · 丢 ${transportStats.dropped} · 滞后 ${transportStats.lagDeg.toFixed(2)}°`}
        </span>
      </div>

      <table className="grid" style={{ marginTop: 8 }}>
        <thead>
          <tr>
            <th>关节</th>
            <th>Command</th>
            <th>Actual</th>
            <th>Error</th>
          </tr>
        </thead>
        <tbody>
          {movableJoints(model).map((joint) => (
            <tr key={joint.id}>
              <td>{jointLabel(joint.id)}</td>
              <td>{(commandJoints[joint.id] ?? 0).toFixed(1)}°</td>
              <td>{(actualJoints[joint.id] ?? 0).toFixed(1)}°</td>
              <td>{(errors[joint.id] ?? 0).toFixed(2)}°</td>
            </tr>
          ))}
        </tbody>
      </table>

      <div className="kv" style={{ marginTop: 8 }}>
        <span>End Effector</span>
        <span>
          X {endEffector.position[0].toFixed(1)} · Y {endEffector.position[1].toFixed(1)} · Z{' '}
          {endEffector.position[2].toFixed(1)} mm
        </span>
      </div>
      <div className="kv">
        <span>Position Error</span>
        <span>{tcpError.toFixed(3)} mm</span>
      </div>

      <div className="kv">
        <span>FK ↔ Three.js</span>
        <span>
          <span className={`badge ${aligned ? 'ok' : 'bad'}`}>
            <i className="dot" />
            {alignmentErrorMm === null ? '检测中…' : `${alignmentErrorMm.toExponential(2)} mm`}
          </span>
        </span>
      </div>

      <div className="notes" style={{ marginTop: 6 }}>
        {transportDriven
          ? 'Actual 由传输回推驱动：拖动时滞后、松手后收敛（Mock 如实模拟了舵机的有限角速度）。'
          : '未接入传输：Actual 恒等于 Command（仿真立即跟随）。接入 MockTransport 后可看到真实的滞后与误差。'}
        <br />
        舵机（MG90S）无位置回读，Phase 11 接入真实反馈后本表无需改动即可显示真机误差。
      </div>
    </div>
  );
}
