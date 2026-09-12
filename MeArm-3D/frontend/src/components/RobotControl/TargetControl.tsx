/**
 * 末端目标控制（Phase 6）：XYZ 直输 + 六向 Jog + 拖动平面选择 + IK 状态。
 *
 * 组件内**不存任何关节副本**；一切经 `store.moveTo()` 落到 RobotState。
 * XYZ 输入框是个"草稿"：正在编辑时不被拖动结果覆盖，避免手被打断。
 */
import { useEffect, useRef, useState } from 'react';
import { DRAG_PLANE_MODES, dragPlaneLabel, type DragPlaneMode } from '@robot/index';
import { useRobotStore } from '@/store/robotStore';

const JOG_STEP_MM = 10;

type Draft = [string, string, string];

function toDraft(v: [number, number, number]): Draft {
  return [v[0].toFixed(1), v[1].toFixed(1), v[2].toFixed(1)];
}

export function TargetControl() {
  const target = useRobotStore((s) => s.target);
  const moveTo = useRobotStore((s) => s.moveTo);
  const resetTarget = useRobotStore((s) => s.resetTarget);
  const ikStatus = useRobotStore((s) => s.ikStatus);
  const dragPlane = useRobotStore((s) => s.dragPlane);
  const setDragPlane = useRobotStore((s) => s.setDragPlane);
  const dragging = useRobotStore((s) => s.dragging);

  const [draft, setDraft] = useState<Draft>(() => toDraft(target));
  const editingRef = useRef(false);

  // 拖动 / Jog / HOME 改变了 target 时刷新输入框；但用户正在输入时不打断
  useEffect(() => {
    if (editingRef.current) return;
    setDraft(toDraft(target));
  }, [target]);

  const moveToXyz = (xyz: [number, number, number]) => {
    const result = moveTo(xyz);
    if (!result.success) {
      console.info(`[target] ${result.reason}${result.joint ? ` @ ${result.joint}` : ''}: ${result.message}`);
    }
  };

  const commit = () => {
    const parsed = draft.map((s) => Number(s)) as [number, number, number];
    if (parsed.some((v) => !Number.isFinite(v))) return;
    editingRef.current = false;
    moveToXyz(parsed);
  };

  const jog = (axis: 0 | 1 | 2, delta: number) => {
    const next: [number, number, number] = [target[0], target[1], target[2]];
    next[axis] = Math.round((next[axis] + delta) * 10) / 10;
    moveToXyz(next);
  };

  return (
    <div className="card">
      <h2>末端目标 · Target XYZ</h2>

      <div className="target-row">
        {(['X', 'Y', 'Z'] as const).map((axis, index) => (
          <label className="target-field" key={axis}>
            <span className="name">{axis}</span>
            <input
              type="number"
              step={1}
              value={draft[index]}
              onFocus={() => {
                editingRef.current = true;
              }}
              onBlur={() => {
                editingRef.current = false;
              }}
              onChange={(event) => {
                const next = [...draft] as Draft;
                next[index] = event.target.value;
                setDraft(next);
              }}
              onKeyDown={(event) => {
                if (event.key === 'Enter') commit();
              }}
            />
          </label>
        ))}
      </div>

      <div className="btn-row">
        <button type="button" onClick={commit}>
          Move
        </button>
        <button type="button" onClick={resetTarget}>
          Use TCP
        </button>
      </div>

      <div className="jog-grid">
        <button type="button" disabled={dragging} onClick={() => jog(0, -JOG_STEP_MM)}>
          X −
        </button>
        <button type="button" disabled={dragging} onClick={() => jog(0, JOG_STEP_MM)}>
          X +
        </button>
        <button type="button" disabled={dragging} onClick={() => jog(1, -JOG_STEP_MM)}>
          Y −
        </button>
        <button type="button" disabled={dragging} onClick={() => jog(1, JOG_STEP_MM)}>
          Y +
        </button>
        <button type="button" disabled={dragging} onClick={() => jog(2, -JOG_STEP_MM)}>
          Z −
        </button>
        <button type="button" disabled={dragging} onClick={() => jog(2, JOG_STEP_MM)}>
          Z +
        </button>
      </div>

      <div className="kv">
        <span>拖动平面</span>
        <select
          value={dragPlane}
          onChange={(event) => setDragPlane(event.target.value as DragPlaneMode)}
        >
          {DRAG_PLANE_MODES.map((mode) => (
            <option value={mode} key={mode}>
              {dragPlaneLabel(mode)}
            </option>
          ))}
        </select>
      </div>

      {ikStatus === null ? (
        <div className="ik-status dim">未指定目标（滑杆 / HOME / ZERO 驱动）</div>
      ) : ikStatus.ok ? (
        <div className="ik-status ok">
          OK · {ikStatus.branch} · 残差 {ikStatus.residual?.toExponential(1)} mm · 方位{' '}
          {ikStatus.azimuth?.toFixed(1)}°
        </div>
      ) : (
        <>
          <div className="ik-status bad">
            {ikStatus.reason}
            {ikStatus.joint ? ` @ ${ikStatus.joint}` : ''}
          </div>
          <div className="ik-status dim">{ikStatus.message}</div>
        </>
      )}

      <div className="dim" style={{ marginTop: 6 }}>
        {dragging
          ? `拖动中…平面已冻结为「${dragPlaneLabel(dragPlane)}」`
          : '在场景中拖动蓝色半透明球即可移动末端'}
      </div>
    </div>
  );
}
