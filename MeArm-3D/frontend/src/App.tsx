import { ConnectionControl } from '@/components/RobotControl/ConnectionControl';
import { JointControl } from '@/components/RobotControl/JointControl';
import { TargetControl } from '@/components/RobotControl/TargetControl';
import { RobotModelPanel } from '@/components/RobotConfig/RobotModelPanel';
import { RobotScene } from '@/components/RobotScene';
import { ViewportOverlay } from '@/components/RobotScene/ViewportOverlay';
import { LogPanel } from '@/components/StatusPanel/LogPanel';
import { StatusPanel } from '@/components/StatusPanel/StatusPanel';
import { useAutoConnect } from '@/hooks/useAutoConnect';

export default function App() {
  // 一键启动脚本（根目录 start.bat）会注入 VITE_AUTO_CONNECT，
  // 让页面挂载后自动连后端（可选自动切 Real Robot）。
  // 未注入时是 no-op —— `npm run dev` 的手动调试行为完全不变。
  useAutoConnect();

  return (
    <div className="app">
      <header className="header">
        <h1>ArmPilot</h1>
        <span className="sub">mARM 虚拟建模 + 真实机械臂同步控制</span>
        <span className="sub">Phase 1–7</span>
        <span className="spacer" />
        <span className="sub mono">
          RobotModel 唯一数据源 · 右手系 Z-up · mm / degree
        </span>
      </header>

      <div className="viewport">
        <RobotScene />
        <ViewportOverlay />
      </div>

      <aside className="sidebar">
        <JointControl />
        <TargetControl />
        <ConnectionControl />
        <StatusPanel />
        <RobotModelPanel />
        <LogPanel />
      </aside>
    </div>
  );
}
