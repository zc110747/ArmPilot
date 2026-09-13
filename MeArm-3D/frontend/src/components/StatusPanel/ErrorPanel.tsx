/**
 * 误差反馈面板（Phase 11）。
 *
 * 回答一个问题：**Actual 到底追上 Command 没有。**
 *
 * 为什么不是"再加一张表"
 * ---------------------
 * `StatusPanel` 已经把 Command / Actual / Error 三列数值列全了。但**光有数值看不出
 * 时序**：真机调试时最常问的是"这是还在追，还是卡住了？" —— 这需要趋势与时间，
 * 不是某一帧的数字。所以本面板给的是：
 *   - 逐关节**带符号的偏差条**（一眼看出哪个轴落下最多）
 *   - 误差**趋势 sparkline**（收敛 / 持平 / 发散）
 *   - 一句**健康结论**（已到位 / 跟踪中 / 异常），把"滞后"与"卡死"分开
 *
 * 判定逻辑全部在 `@robot` 的 `linkFeedback` 里（纯函数，可单测），
 * 本组件只负责采样与渲染。
 *
 * ⚠️ 能力边界（诚实声明）
 * ----------------------
 * MG90S 无位置回读，当前 `Actual` 是**由目标值反算**的（见 StatusPanel 注释与
 * ADR D34）。因此在 sim / 现有固件下，本面板能看到的是**链路时延与限位截断**，
 * 而不是"物理臂真的到了没有"—— 后者只能靠相机反解（Phase 4.5 工具链）。
 * 固件一旦具备回读，本面板无需改动即可显示真实误差。
 */
import { useEffect, useRef, useState } from 'react';
import {
  ERROR_TOL_DEG,
  classifyLinkHealth,
  classifyTrend,
  describeLinkHealth,
  describeTrend,
  jointStateError,
  jointStatesEqual,
  movableJoints,
  nextHistory,
  positionError,
} from '@robot/index';
import { jointLabel, useRobotStore } from '@/store/robotStore';

/** 偏差条满量程（deg）：|误差| ≥ 该值即画满。取 5° —— 真机实测标定偏置量级就在 3~5° */
const BAR_FULL_SCALE_DEG = 5;
/** sparkline 纵轴自适应下限（deg）：避免误差很小时把噪声放大成"剧烈波动" */
const SPARK_MIN_SCALE_DEG = 1.25;
const SPARK_W = 100;
const SPARK_H = 26;

export function ErrorPanel() {
  const model = useRobotStore((s) => s.model);
  const commandJoints = useRobotStore((s) => s.commandJoints);
  const actualJoints = useRobotStore((s) => s.actualJoints);
  const endEffector = useRobotStore((s) => s.endEffector);
  const actualEndEffector = useRobotStore((s) => s.actualEndEffector);
  const transportDriven = useRobotStore((s) => s.transportDriven);

  const joints = movableJoints(model);
  const errors = jointStateError(commandJoints, actualJoints);
  const lagDeg = joints.reduce((m, j) => Math.max(m, Math.abs(errors[j.id] ?? 0)), 0);
  const tcpErrorMm = positionError(endEffector, actualEndEffector);

  // 采样：`actualJoints` 每次回推都会换引用 ⇒ 天然以回推频率（约 200ms）采一格。
  // 命令一变就清空历史 —— 跨命令的趋势没有意义（详见 linkFeedback.nextHistory）。
  const [history, setHistory] = useState<number[]>([]);
  const [commandChangedAt, setCommandChangedAt] = useState(() => Date.now());
  const prevCommand = useRef(commandJoints);

  useEffect(() => {
    const changed = !jointStatesEqual(prevCommand.current, commandJoints);
    if (changed) {
      prevCommand.current = commandJoints;
      setCommandChangedAt(Date.now());
    }
    setHistory((h) => nextHistory(h, lagDeg, changed));
  }, [commandJoints, actualJoints, lagDeg]);

  const trend = classifyTrend(history);
  const health = classifyLinkHealth(history, {
    transportDriven,
    lagDeg,
    msSinceCommandChange: Date.now() - commandChangedAt,
  });
  const info = describeLinkHealth(health, lagDeg);

  const scale = Math.max(
    SPARK_MIN_SCALE_DEG,
    history.reduce((m, v) => Math.max(m, v), 0),
  );
  const sparkPoints = history
    .map((v, i) => {
      const x = history.length < 2 ? 0 : (i / (history.length - 1)) * SPARK_W;
      const y = SPARK_H - 1 - Math.min(1, v / scale) * (SPARK_H - 2);
      return `${x.toFixed(2)},${y.toFixed(2)}`;
    })
    .join(' ');

  return (
    <div className="card error-panel">
      <h2>链路误差 · Link Error</h2>

      <div className={`mode-hint ${info.tone}`} data-testid="link-health">
        {info.label}
      </div>

      <div className="err-rows" data-testid="err-rows">
        {joints.map((joint) => {
          const err = errors[joint.id] ?? 0;
          const over = Math.abs(err) > ERROR_TOL_DEG;
          const frac = Math.min(1, Math.abs(err) / BAR_FULL_SCALE_DEG);
          return (
            <div className="err-row" key={joint.id}>
              <span className="name">{jointLabel(joint.id)}</span>
              <span className="vals">
                {(commandJoints[joint.id] ?? 0).toFixed(1)}
                <span className="dim"> → </span>
                {(actualJoints[joint.id] ?? 0).toFixed(1)}
              </span>
              <span className="errbar" aria-hidden="true">
                <i className={over ? 'fill over' : 'fill'} style={{ width: `${frac * 100}%` }} />
              </span>
              <span className={over ? 'err-val over' : 'err-val'}>
                {err >= 0 ? '+' : ''}
                {err.toFixed(2)}°
              </span>
            </div>
          );
        })}
      </div>

      <div className="kv" style={{ marginTop: 8 }}>
        <span>TCP 位置误差</span>
        <span data-testid="tcp-error">{tcpErrorMm.toFixed(3)} mm</span>
      </div>
      <div className="kv">
        <span>趋势 · {describeTrend(trend)}</span>
        <span>{history.length} 采样</span>
      </div>

      <svg
        className="spark"
        viewBox={`0 0 ${SPARK_W} ${SPARK_H}`}
        preserveAspectRatio="none"
        role="img"
        aria-label="最大关节误差随时间的变化"
      >
        <polyline points={sparkPoints} />
      </svg>

      <div className="notes" style={{ marginTop: 6 }}>
        纵轴满量程 {BAR_FULL_SCALE_DEG}°（横轴自适应）。拖动时误差非零属正常 ——
        命令先到、物理后到；**命令停下后仍不收敛**才判异常。
        <br />
        当前固件无位置回读，Actual 由目标值反算，故本面板反映的是**链路时延与限位截断**。
      </div>
    </div>
  );
}
