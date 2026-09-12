/**
 * 模型 / 标定面板：把 `config/robot.yaml` 的实际生效值摊开给人看。
 * 用于验证「RobotModel 是唯一数据源」——这里显示的每个数字都直接从模型读，没有任何硬编码。
 */
import { buildCalibrationTable, describeCalibration } from '@robot/index';
import { useRobotStore } from '@/store/robotStore';

export function RobotModelPanel() {
  const model = useRobotStore((s) => s.model);
  const lines = describeCalibration(model);
  const table = buildCalibrationTable(model);

  return (
    <div className="card">
      <h2>模型 · robot.yaml</h2>

      <div className="kv">
        <span>id / name</span>
        <span>
          {model.id} / {model.name}
        </span>
      </div>
      <div className="kv">
        <span>units</span>
        <span>{model.units}</span>
      </div>
      <div className="kv">
        <span>links / joints / actuators</span>
        <span>
          {model.links.length} / {model.joints.length} / {model.actuators.length}
        </span>
      </div>
      <div className="kv">
        <span>TCP</span>
        <span>
          {model.tcp.joint} + [{model.tcp.offset.join(', ')}]
        </span>
      </div>

      <table className="grid" style={{ marginTop: 8 }}>
        <thead>
          <tr>
            <th>连杆</th>
            <th>length</th>
            <th>geometry</th>
          </tr>
        </thead>
        <tbody>
          {model.links.map((link) => (
            <tr key={link.id}>
              <td>{link.id}</td>
              <td>{link.length}</td>
              <td className="dim">{link.geometry.type}</td>
            </tr>
          ))}
        </tbody>
      </table>

      <h2 style={{ marginTop: 12 }}>标定 · Joint → Servo</h2>
      <table className="grid">
        <thead>
          <tr>
            <th>舵机</th>
            <th>关节</th>
            <th>映射后</th>
            <th>硬限位</th>
          </tr>
        </thead>
        <tbody>
          {table.entries.map((entry) => (
            <tr key={entry.actuator.id}>
              <td>S{entry.actuator.channel}</td>
              <td>{entry.jointId}</td>
              <td>
                {entry.servoRange.min.toFixed(0)}–{entry.servoRange.max.toFixed(0)}°
              </td>
              <td className="dim">
                {entry.actuator.limits.min}–{entry.actuator.limits.max}°
              </td>
            </tr>
          ))}
        </tbody>
      </table>

      <div className="notes mono" style={{ marginTop: 8, fontSize: 10.5 }}>
        {lines.map((line) => (
          <div key={line}>{line}</div>
        ))}
      </div>
    </div>
  );
}
