/**
 * Command Log（spec §三十六）。
 * 网页侧只记录「离散动作」与「通讯事件」；连续滑杆拖动不逐帧记录，避免刷屏。
 */
import { useRobotStore } from '@/store/robotStore';

export function LogPanel() {
  const log = useRobotStore((s) => s.log);
  const clearLog = useRobotStore((s) => s.clearLog);

  return (
    <div className="card">
      <h2>
        日志 · Command Log
        <button
          type="button"
          style={{ float: 'right', padding: '1px 6px', fontSize: 11, marginTop: -3 }}
          onClick={clearLog}
        >
          清空
        </button>
      </h2>
      <div className="log">
        {log.length === 0 ? (
          <span>（空）</span>
        ) : (
          log.map((entry) => (
            <div key={entry.id}>
              <span className="dim">{entry.time}</span>{' '}
              <span className="dim">
                {entry.kind === 'out' ? 'SEND' : entry.kind === 'in' ? 'RECV' : 'SYS '}
              </span>{' '}
              {entry.text}
            </div>
          ))
        )}
      </div>
    </div>
  );
}
