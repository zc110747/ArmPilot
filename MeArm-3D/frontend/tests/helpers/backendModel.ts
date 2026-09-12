/**
 * 从本地 `RobotModel` 生成一份"与后端一致"的模型元数据。
 *
 * 用途：模拟后端 `hello` 消息（后端读的是同一份 `config/robot.yaml`）。
 * 测试再按需把某个字段改坏，验证一致性校验真的能测出差异。
 */
import type { BackendModelInfo, RobotModel } from '@robot/index';

export function backendInfoFromLocal(model: RobotModel): BackendModelInfo {
  const order = model.joints.filter((j) => j.type !== 'fixed').map((j) => j.id);
  return {
    id: model.id,
    name: model.name,
    source: 'config/robot.yaml',
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
