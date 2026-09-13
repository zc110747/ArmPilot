/**
 * 示教录制 / 回放面板（Phase 13）。
 *
 * 解决的事：本项目此前只能**逐次**操作（拖滑杆 / 拖末端 / HOME），
 * 没有"把一串动作录下来再整体重放"的能力 —— 而这恰恰是臂类装置最常用的功能：
 * 手工摆出一个动作 → 录下来 → 回放给真机看一遍。
 *
 * 三个关键设计
 * ------------
 * 1. **录 command，不录 actual**。`actual` 是回推的滞后值（MG90S 无回读，
 *    甚至由目标值反算，见 ADR D34），录进去等于把链路时延焊进轨迹。
 * 2. **回放走既有命令路径**（`store.setCommandJoints`）。因此尾沿节流（30Hz）
 *    与安全门（Simulation 模式不驱动真机）全部自动生效 ——
 *    若为回放另开一条直发通道，等于绕过所有安全约束。
 * 3. **停止录制时强制补一帧**（`force`）。否则末帧可能停在被去抖吃掉的那个值上，
 *    于是轨迹终点 ≠ 臂当前所在位置，回放结束时会莫名回退一点。
 *
 * ⚠️ 能力边界
 * -----------
 * 回放是**开环**的：本面板只负责按时间轴把命令投出去，不校验真机是否跟上
 * （那件事由 Phase 11 误差面板负责）。固件若在某帧被限位截断，回放不会停下来 ——
 * 判据在 ErrorPanel 的"异常"结论里。
 */
import { useEffect, useMemo, useRef, useState } from 'react';
import {
  TeachPlayer,
  TEACH_MAX_FRAMES,
  parseTrack,
  sampleAt,
  serializeTrack,
  trackDurationMs,
  trackStats,
  type TeachPlaybackState,
} from '@robot/index';
import { jointLabel, useRobotStore } from '@/store/robotStore';

/** 列表最多渲染多少行（轨迹可达 2000 帧，全渲染既卡又没人看） */
const LIST_MAX_ROWS = 24;

const STATUS_TEXT: Record<TeachPlaybackState['status'], string> = {
  idle: '已就绪',
  playing: '回放中',
  paused: '已暂停',
  ended: '回放完成',
};

/** 进度条铺满量程用不到 —— 仅用于"是否回放中"的视觉提示，故无额外常量 */

export function TeachPanel() {
  const teachTrack = useRobotStore((s) => s.teachTrack);
  const teachRecording = useRobotStore((s) => s.teachRecording);
  const commandJoints = useRobotStore((s) => s.commandJoints);
  const setTeachRecording = useRobotStore((s) => s.setTeachRecording);
  const setTeachTrack = useRobotStore((s) => s.setTeachTrack);
  const appendTeachFrame = useRobotStore((s) => s.appendTeachFrame);
  const clearTeachTrack = useRobotStore((s) => s.clearTeachTrack);

  const [playback, setPlayback] = useState<TeachPlaybackState>({
    status: 'idle',
    tMs: 0,
    durationMs: 0,
    progress: 0,
  });
  const [notice, setNotice] = useState<string | null>(null);
  const fileRef = useRef<HTMLInputElement | null>(null);

  const player = useMemo(
    () =>
      new TeachPlayer({
        // 回放命令与滑杆走同一条路：节流 + 安全门自动生效
        onSample: (joints) => useRobotStore.getState().setCommandJoints(joints),
        onState: (state) => setPlayback(state),
      }),
    [],
  );

  // 轨迹变了就同步给播放器（load 会回到起点；录制期间新的帧不该续播）
  useEffect(() => {
    player.load(teachTrack);
  }, [player, teachTrack]);

  useEffect(() => () => player.dispose(), [player]);

  // 录制：命令一变就采一帧（间隔 / 静止由 appendFrame 内部判定）
  useEffect(() => {
    if (!teachRecording) return;
    // 纵深防御：正常路径下 play() 已经先关掉录制，这里再挡一次以防自录自放
    if (player.isPlaying()) return;
    appendTeachFrame(commandJoints);
  }, [commandJoints, teachRecording, appendTeachFrame, player]);

  const stats = trackStats(teachTrack);
  const durationMs = trackDurationMs(teachTrack);

  const rows = useMemo(() => {
    const frames = teachTrack.frames;
    const n = frames.length;
    if (n === 0) return [];
    if (n <= LIST_MAX_ROWS) return frames.map((f, i) => ({ i, frame: f }));
    const out: { i: number; frame: (typeof frames)[number] }[] = [];
    for (let k = 0; k < LIST_MAX_ROWS; k += 1) {
      const i = Math.round((k * (n - 1)) / (LIST_MAX_ROWS - 1));
      out.push({ i, frame: frames[i] as (typeof frames)[number] });
    }
    return out;
  }, [teachTrack]);

  const jointKeys = stats.joints;

  /** 停止录制（并把当前姿态强制补进轨迹，保证终点 = 臂当前所在位置） */
  const stopRecording = () => {
    setTeachRecording(false);
    appendTeachFrame(useRobotStore.getState().commandJoints, Date.now(), { force: true });
  };

  const handleRecord = () => {
    if (teachRecording) {
      stopRecording();
      return;
    }
    player.stop();
    setNotice(null);
    setTeachRecording(true);
  };

  const handlePlay = () => {
    if (teachRecording) stopRecording();
    setNotice(null);
    player.play();
  };

  const handleClear = () => {
    player.stop();
    clearTeachTrack();
    setNotice(null);
  };

  const handleExport = () => {
    const json = serializeTrack(teachTrack);
    const blob = new Blob([json], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `${teachTrack.name}.json`;
    a.click();
    URL.revokeObjectURL(url);
    setNotice(`已导出 ${stats.frames} 帧 / ${json.length} 字节`);
  };

  const handleImportFile = (file: File) => {
    const reader = new FileReader();
    reader.onload = () => {
      const parsed = parseTrack(String(reader.result ?? ''));
      if (parsed === null) {
        // 不静默失败：说清"为什么没载入"，否则用户只会看到"点了没反应"
        setNotice('导入失败：不是本工具导出的轨迹文件（format 标记不符或帧数据非法）');
        return;
      }
      player.stop();
      setTeachRecording(false);
      setTeachTrack(parsed);
      setNotice(`已导入 ${parsed.frames.length} 帧（名称 ${parsed.name}）`);
    };
    reader.onerror = () => setNotice('导入失败：文件读取错误');
    reader.readAsText(file);
  };

  const endJoints = sampleAt(teachTrack, durationMs);
  const recordFull = teachTrack.frames.length >= TEACH_MAX_FRAMES;

  return (
    <div className="card teach-panel">
      <h2>示教 · Teach</h2>

      <div className="kv">
        <span>轨迹名</span>
        <span>
          <input
            type="text"
            data-testid="teach-name"
            value={teachTrack.name}
            onChange={(e) => setTeachTrack({ ...teachTrack, name: e.target.value })}
          />
        </span>
      </div>

      <div className="btn-row">
        <button
          type="button"
          data-testid="teach-record"
          className={teachRecording ? 'active' : undefined}
          onClick={handleRecord}
          disabled={recordFull && !teachRecording}
        >
          {teachRecording ? 'Stop Rec' : 'Record'}
        </button>
        <button
          type="button"
          data-testid="teach-play"
          onClick={handlePlay}
          disabled={stats.frames === 0 || playback.status === 'playing'}
        >
          Play
        </button>
        <button
          type="button"
          data-testid="teach-pause"
          onClick={() => player.pause()}
          disabled={playback.status !== 'playing'}
        >
          Pause
        </button>
      </div>

      <div className="btn-row">
        <button type="button" data-testid="teach-stop" onClick={() => player.stop()}>
          Stop
        </button>
        <button type="button" data-testid="teach-clear" onClick={handleClear}>
          Clear
        </button>
        <button
          type="button"
          data-testid="teach-export"
          onClick={handleExport}
          disabled={stats.frames === 0}
        >
          Export
        </button>
        <button type="button" data-testid="teach-import" onClick={() => fileRef.current?.click()}>
          Import
        </button>
        <input
          ref={fileRef}
          type="file"
          accept="application/json,.json"
          style={{ display: 'none' }}
          onChange={(e) => {
            const file = e.target.files?.[0];
            if (file) handleImportFile(file);
            e.target.value = '';
          }}
        />
      </div>

      <div className="kv">
        <span>帧数</span>
        <span data-testid="teach-frames">
          {stats.frames}
          {recordFull ? `（已达上限 ${TEACH_MAX_FRAMES}）` : ''}
        </span>
      </div>
      <div className="kv">
        <span>时长</span>
        <span data-testid="teach-duration">{(durationMs / 1000).toFixed(2)} s</span>
      </div>
      <div className="kv">
        <span>状态</span>
        <span data-testid="teach-status">
          {STATUS_TEXT[playback.status]}
          {playback.durationMs > 0
            ? ` · ${(playback.tMs / 1000).toFixed(2)}/${(playback.durationMs / 1000).toFixed(2)} s`
            : ''}
        </span>
      </div>

      <div className="teach-progress" aria-hidden="true">
        <i style={{ width: `${playback.progress * 100}%` }} />
      </div>

      <div className="kv">
        <span>终点姿态</span>
        <span>
          {endJoints === null
            ? '—'
            : jointKeys
                .filter((k) => k in endJoints)
                .map((k) => `${jointLabel(k)} ${(endJoints[k] ?? 0).toFixed(1)}°`)
                .join(' · ')}
        </span>
      </div>

      <div className="teach-list" data-testid="teach-frames-list">
        {rows.length === 0 ? (
          <div className="dim">（空轨迹 —— 点 Record 然后用滑杆 / 拖动摆出动作）</div>
        ) : (
          <>
            {rows.map(({ i, frame }) => (
              <div className="teach-row" key={i}>
                <span className="idx">#{String(i + 1).padStart(4, '0')}</span>
                <span className="t">{(frame.t / 1000).toFixed(2)}s</span>
                <span className="vals">
                  {Object.entries(frame.joints)
                    .map(([k, v]) => `${jointLabel(k).split(' ').pop()} ${v.toFixed(1)}`)
                    .join('  ')}
                </span>
              </div>
            ))}
            {stats.frames > rows.length ? (
              <div className="dim">（显示 {rows.length} / {stats.frames} 帧，等距抽样）</div>
            ) : null}
          </>
        )}
      </div>

      {notice !== null ? <div className="teach-notice">{notice}</div> : null}

      <div className="notes" style={{ marginTop: 6 }}>
        录制的是**命令**（commandJoints），不是物理实际位置 —— 当前固件无位置回读。
        回放走既有命令路径，因此受**节流与安全门**约束：Simulation 模式下不会驱动真机。
        静止段与过密采样会被合并（阈值 {`${50}ms`} / 0.5°），故帧数少于操作次数是正常的。
      </div>
    </div>
  );
}
