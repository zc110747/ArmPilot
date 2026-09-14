/**
 * 从本地 `RobotModel` 生成一份"与后端一致"的模型元数据。
 *
 * 用途：模拟后端 `hello` 消息（后端读的是同一份真值 —— 路径由包 manifest 声明）。
 * 测试再按需把某个字段改坏，验证一致性校验真的能测出差异。
 *
 * `source` 字段（= 后端 `hello.model.source`，人读的诊断信息，**不参与任何比对**）
 * 现在从 `declaredPath()` 取，不再手写字符串 —— Phase 2 之前它写死 `config/robot.yaml`。
 */
import type { BackendModelInfo, RobotModel } from '@robot/index';
import { MEARM_V1_ROBOT_ID } from '@robot/model/robotIds';
import { declaredPath } from './robotPackage';

export function backendInfoFromLocal(
  model: RobotModel,
  robotId: string = MEARM_V1_ROBOT_ID,
): BackendModelInfo {
  // ⚠️ 判据是 `=== 'revolute'`，**不是** `!== 'fixed'`。
  // 后端 Go 的 `JointOrder()` 只收 revolute（`joints.tool` 是被动腕，没有独立输入、
  // 不进 JR 四元组），前端 `movableJoints()` / `isMovableJoint()` 同一条规则。
  // 若这里放行 passive，`describeModelMismatch` 会报"本地 4 / 后端 5" —— 那是**假警报**，
  // 但若反过来把被动关节真塞进 JR，才是真正会让位次错位的错。
  const order = model.joints.filter((j) => j.type === 'revolute').map((j) => j.id);
  return {
    id: model.id,
    name: model.name,
    source: declaredPath(robotId, 'model.config'),
    jointOrder: order,
    limits: order.map((id) => {
      const joint = model.joints.find((j) => j.id === id)!;
      return { id, role: joint.role ?? '', min: joint.limits.min, max: joint.limits.max };
    }),
    calibration: order.flatMap((id) =>
      model.actuators
        .filter((a) => a.jointId === id)
        .map((a) => ({
          jointId: id,
          servoId: a.id,
          channel: a.channel,
          offset: a.offset,
          scale: a.scale,
          reverse: a.reverse,
          servoLo: 0,
          servoHi: 0,
        })),
    ),
    homePose: { ...model.homePose },
  };
}
