/**
 * 关节控制（Phase 4）：J1/J2/J3 + Gripper 滑杆，直接驱动 JointState。
 *
 * 「改变 Joint State → 立即更新 3D Model」由 store 统一驱动：
 * 组件只改命令关节角，实际 3D 姿态与末端 XYZ 全部由 RobotState / FK 派生，
 * 组件内不存任何角度副本。
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
