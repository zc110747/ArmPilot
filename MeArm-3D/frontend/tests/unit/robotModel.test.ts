/**
 * Phase 1 验收：`config/robot.yaml` → `RobotModel`（唯一数据源）
 *
 * 覆盖：
 *   - 内置配置可加载且模型自洽（无 error 级 issue）
 *   - 结构：根连杆 / 运动学链顺序 / 可动关节集合 / TCP
 *   - **改 yaml 即改模型**：link length、joint limit 单独一处修改就生效
 *   - homePose 与固件开机位（全部舵机 90°）一致
 *   - 非法配置能被校验拦截；多舵机关节（spec §二十五）天然支持
 */
import { describe, expect, it } from 'vitest';
import {
  BUNDLED_ROBOT_YAML,
  RobotModelError,
  homeJointState,
  jointById,
  jointIds,
  jointStateToServoAngles,
  kinematicChain,
  linkById,
  loadRobotModel,
  movableJoints,
  parseRobotModelYaml,
  rootLink,
  validateRobotModel,
} from '../../src/robot';

/** 便于在测试里做「只改一个字段」的定点替换 */
function replaceOnce(text: string, anchor: string, from: string, to: string): string {
  const index = text.indexOf(anchor);
  expect(index, `锚点未找到: ${anchor}`).toBeGreaterThanOrEqual(0);
  const head = text.slice(0, index);
  const tail = text.slice(index);
  const replacedTail = tail.replace(from, to);
  expect(replacedTail, `替换目标未找到: ${from}`).not.toBe(tail);
  return head + replacedTail;
}

describe('Phase 1 · RobotModel 加载与自洽性', () => {
  it('内置 config/robot.yaml 可加载且无 error 级校验问题', () => {
    const model = loadRobotModel();
    const issues = validateRobotModel(model);
    const errors = issues.filter((issue) => issue.level === 'error');
    expect(errors).toEqual([]);
    expect(model.id).toBe('mearm');
    expect(model.name).toBe('mARM');
    expect(model.units).toBe('mm');
  });

  it('结构正确：根连杆唯一，链序 root -> tip', () => {
    const model = loadRobotModel();

    expect(rootLink(model).id).toBe('base_link');
    expect(model.links.map((l) => l.id)).toEqual([
      'base_link',
      'column_link',
      'upper_arm_link',
      'forearm_link',
      'tool_link',
      'jaw_link',
    ]);
    expect(kinematicChain(model).map((j) => j.id)).toEqual([
      'base',
      'shoulder',
      'elbow',
      'tool',
      'gripper',
    ]);
    expect(movableJoints(model).map((j) => j.id)).toEqual(['base', 'shoulder', 'elbow', 'gripper']);
    expect(jointIds(model)).toEqual(['base', 'shoulder', 'elbow', 'gripper']);
  });

  it('每个关节的父/子连杆与轴向符合 Z-up 右手系约定', () => {
    const model = loadRobotModel();

    expect(jointById(model, 'base')!.axis).toEqual([0, 0, 1]); // 绕竖直轴偏航
    expect(jointById(model, 'shoulder')!.axis).toEqual([0, 1, 0]); // 绕 Y 俯仰
    expect(jointById(model, 'elbow')!.axis).toEqual([0, 1, 0]);
    expect(jointById(model, 'base')!.parentLink).toBe('base_link');
    expect(jointById(model, 'base')!.childLink).toBe('column_link');
    expect(jointById(model, 'gripper')!.parentLink).toBe('tool_link');

    // 固定关节不产生自由度
    expect(jointById(model, 'tool')!.type).toBe('fixed');
  });

  it('连杆长度是运动学唯一真值：改一个数字即改变机构尺寸', () => {
    const base = loadRobotModel();
    expect(linkById(base, 'upper_arm_link')!.length).toBe(80);

    const patched = replaceOnce(
      BUNDLED_ROBOT_YAML,
      '  - id: upper_arm_link',
      'length: 80',
      'length: 120',
    );
    const model = parseRobotModelYaml(patched);

    expect(linkById(model, 'upper_arm_link')!.length).toBe(120);
    // 未改动的连杆不受影响
    expect(linkById(model, 'forearm_link')!.length).toBe(80);
    // 依然自洽（无 error）
    expect(validateRobotModel(model).filter((i) => i.level === 'error')).toEqual([]);
  });

  it('关节限位可配置：改 limit 即生效', () => {
    const patched = replaceOnce(
      BUNDLED_ROBOT_YAML,
      '  - id: elbow',
      'max: 141.8582211436',
      'max: 130',
    );
    const model = parseRobotModelYaml(patched);

    expect(jointById(model, 'elbow')!.limits).toEqual({ min: 108.4414852068, max: 130 });
    // elbow: 108.44..130 → 舵机 100..72.33，仍在硬限位 20..100 内
    expect(validateRobotModel(model).filter((i) => i.level === 'error')).toEqual([]);
  });

  it('关节限位映射超出舵机硬限位时，校验必须报错', () => {
    // elbow: servo = -θ × 2.39401 + 359.61，硬限位 20..100。
    // 上限抬到 160° → 舵机 -23.4°，越界 ⇒ 必须报 ACTUATOR_REACH
    const patched = replaceOnce(
      BUNDLED_ROBOT_YAML,
      '  - id: elbow',
      'max: 141.8582211436',
      'max: 160',
    );
    expect(() => parseRobotModelYaml(patched)).toThrowError(RobotModelError);

    try {
      parseRobotModelYaml(patched);
    } catch (error) {
      const issues = (error as RobotModelError).issues;
      expect(issues.some((i) => i.code === 'ACTUATOR_REACH')).toBe(true);
    }
  });

  it('homePose 与固件开机位一致：全部舵机 90°', () => {
    const model = loadRobotModel();
    const home = homeJointState(model);
    const servo = jointStateToServoAngles(model, home);

    // HOME 由 2026-09-12 实拍反解得到（见 docs/hardware-measurement.md），
    // 与 actuators 的 offset/scale 严格自洽 ⇒ 四个舵机代入都是 90°（固件开机位）。
    expect(home).toEqual({
      base: 0,
      shoulder: 0.8498937633,
      elbow: 112.6185771989,
      gripper: 50,
    });
    expect(servo[9]).toBeCloseTo(90, 6); // base    :  0        × 1      + 90
    expect(servo[7]).toBeCloseTo(90, 6); // shoulder:  0.849894 × 1.44018 + 88.776
    expect(servo[8]).toBeCloseTo(90, 6); // elbow   : -112.618577 × 2.39401 + 359.61（reverse）
    expect(servo[6]).toBeCloseTo(90, 6); // gripper : 50       × 1      + 40
  });

  it('TCP 只由定位关节决定（不参考夹爪关节）', () => {
    const model = loadRobotModel();
    expect(model.tcp.joint).toBe('tool');
    expect(model.tcp.offset).toEqual([0, 0, 40]);
  });

  it('缺失字段 / 非法引用会被拦截', () => {
    expect(() => parseRobotModelYaml('version: 1\nrobot:\n  id: x\nlinks: []\njoints: []\n')).toThrow(
      /links 不能为空/,
    );

    const badRef = BUNDLED_ROBOT_YAML.replace('parentLink: column_link', 'parentLink: nope_link');
    expect(() => parseRobotModelYaml(badRef)).toThrowError(RobotModelError);
  });

  it('多舵机关节天然支持：同一 jointId 写两条 actuator', () => {
    const dual = `${BUNDLED_ROBOT_YAML}
  # 测试用：给 shoulder 增加第二个舵机（双舵机共同驱动一个机械关节）
  - id: servo_8b
    name: Shoulder Secondary
    jointId: shoulder
    channel: 5
    offset: 90
    scale: 1
    reverse: true
    limits:
      min: 10
      max: 110
`;
    const model = parseRobotModelYaml(dual);
    expect(validateRobotModel(model).filter((i) => i.level === 'error')).toEqual([]);

    const servo = jointStateToServoAngles(model, {
      base: 0,
      shoulder: 10,
      elbow: 112.6185771989,
      gripper: 50,
    });
    expect(servo[8]).toBeCloseTo(90, 6); // 主舵机 S8（elbow）
    expect(servo[5]).toBeCloseTo(80, 6); // 追加的 shoulder 副舵机：-10 × 1 + 90
  });
});
