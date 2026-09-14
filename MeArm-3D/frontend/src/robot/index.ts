/**
 * `src/robot` 统一出口。
 *
 * 分层（严格对应 spec §六）：
 *   definition/   RobotDefinition（机器人定义的最小统一入口）
 *   model/        RobotModel / Link / Joint / Actuator / Pose / RobotState / RobotCommand
 *   kinematics/   coordinate（坐标系转换层）/ transform（矩阵）/ fk / ik
 *                 + KinematicsEngine（统一调用面）/ IKResult（统一结果）/ mearm/（实现）
 *   calibration/  关节角 ↔ 舵机角 标定
 *   transport/    RobotTransport 抽象 + Mock / WebSocket 实现
 */
export * from './definition/RobotDefinition';

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
// ⚠️ 注意命名：`IkResult`（ik.ts，MeArm 原生判别联合）与
//    `IKResult`（IKResult.ts，统一结果形状）**只差一个字母大小写**。
//    前者是算法实现细节，后者是给上层用的稳定契约；两者同时存在是刻意的
//    （替换原生返回会破坏现有 2000 组闭环验收）。新代码请用 `IKResult`。
export * from './kinematics/IKResult';
export * from './kinematics/KinematicsEngine';
export * from './kinematics/mearm/MeArmKinematics';

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
