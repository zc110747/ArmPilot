/**
 * 视口叠加层：实时读数 + 显示开关 + 相机复位。
 * 这些读数的唯一来源是 RobotState（store），不重复计算。
 */
import { useRobotStore } from '@/store/robotStore';

export function ViewportOverlay() {
  const endEffector = useRobotStore((s) => s.endEffector);
  const mode = useRobotStore((s) => s.mode);
  const alignmentErrorMm = useRobotStore((s) => s.alignmentErrorMm);
  const showJointAxes = useRobotStore((s) => s.showJointAxes);
  const showJointOrigins = useRobotStore((s) => s.showJointOrigins);
  const showWorldAxes = useRobotStore((s) => s.showWorldAxes);
  const showRobotAxes = useRobotStore((s) => s.showRobotAxes);
  const showActualGhost = useRobotStore((s) => s.showActualGhost);
  const showTcp = useRobotStore((s) => s.showTcp);
  const setToggle = useRobotStore((s) => s.setToggle);
  const resetCamera = useRobotStore((s) => s.resetCamera);

  return (
    <div className="overlay">
      <div className="chip">
        TCP <b>X {endEffector.position[0].toFixed(1)}</b> <b>Y{' '}
        {endEffector.position[1].toFixed(1)}</b> <b>Z {endEffector.position[2].toFixed(1)}</b> mm
      </div>
      <div className="chip">
        Mode <b>{mode}</b>
      </div>
      <div className="chip">
        FK↔3D{' '}
        <b>
          {alignmentErrorMm === null ? '…' : `${alignmentErrorMm.toExponential(2)} mm`}
        </b>
      </div>
      <div className="chip axis-row">
        <label className="axis">
          <input
            type="checkbox"
            checked={showJointAxes}
            onChange={(e) => setToggle('showJointAxes', e.target.checked)}
          />
          Joint Axis
        </label>
        <label className="axis">
          <input
            type="checkbox"
            checked={showJointOrigins}
            onChange={(e) => setToggle('showJointOrigins', e.target.checked)}
          />
          Joint Origin
        </label>
        <label className="axis">
          <input
            type="checkbox"
            checked={showWorldAxes}
            onChange={(e) => setToggle('showWorldAxes', e.target.checked)}
          />
          World Axis
        </label>
        <label className="axis">
          <input
            type="checkbox"
            checked={showRobotAxes}
            onChange={(e) => setToggle('showRobotAxes', e.target.checked)}
          />
          Robot Axis
        </label>
        <label className="axis">
          <input
            type="checkbox"
            checked={showActualGhost}
            onChange={(e) => setToggle('showActualGhost', e.target.checked)}
          />
          Actual Arm
        </label>
        <label className="axis">
          <input
            type="checkbox"
            checked={showTcp}
            onChange={(e) => setToggle('showTcp', e.target.checked)}
          />
          TCP
        </label>
        <button type="button" onClick={resetCamera}>
          Reset Camera
        </button>
      </div>
    </div>
  );
}
