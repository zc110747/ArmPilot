import { ConnectionControl } from '@/components/RobotControl/ConnectionControl';
import { JointControl } from '@/components/RobotControl/JointControl';
import { TargetControl } from '@/components/RobotControl/TargetControl';
import { RobotModelPanel } from '@/components/RobotConfig/RobotModelPanel';
import { RobotScene } from '@/components/RobotScene';
import { ViewportOverlay } from '@/components/RobotScene/ViewportOverlay';
import { LogPanel } from '@/components/StatusPanel/LogPanel';
import { StatusPanel } from '@/components/StatusPanel/StatusPanel';

export default function App() {
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
