/**
 * `src/robot` 统一出口。
 *
 * 分层（严格对应 spec §六）：
 *   model/        RobotModel / Link / Joint / Actuator / Pose / RobotState / RobotCommand
 *   kinematics/   coordinate（坐标系转换层）/ transform（矩阵）/ fk / ik
 *   calibration/  关节角 ↔ 舵机角 标定
 *   transport/    RobotTransport 抽象 + Mock / WebSocket 实现
 */
export * from './model/Pose';
export * from './model/Link';
export * from './model/Joint';
export * from './model/Actuator';
export * from './model/RobotModel';
export * from './model/RobotState';
export * from './model/RobotCommand';
export * from './model/loadRobotModel';
export * from './model/linkFeedback';

export * from './kinematics/transform';
export * from './kinematics/coordinate';
export * from './kinematics/fk';
export * from './kinematics/ik';

export * from './interaction/dragPlane';

export * from './calibration/calibration';

export * from './transport/RobotTransport';
export * from './transport/timer';
export * from './transport/socket';
export * from './transport/wsProtocol';
export * from './transport/MockTransport';
export * from './transport/WebSocketTransport';

export * from './teach/teachTrack';
export * from './teach/TeachPlayer';
