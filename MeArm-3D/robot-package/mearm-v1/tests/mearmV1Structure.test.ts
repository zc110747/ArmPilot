/**
 * **MeArm-V1 的构造事实** —— 包内测试（Phase 2 步⑤）。
 *
 * ## 为什么这几条住在这里而不是 `frontend/tests/`
 *
 * 判据只有一条：**把这条断言放到另一台机器人上，它还是"对的说法"吗？**
 *
 * | 断言 | 换台机器人还成立吗 |
 * |---|---|
 * | `tcp.joint === 'tool'`（被动腕）⇒ 夹爪开合不影响 TCP | ✗ 这是 MeArm 的构造选择 |
 * | `tool` 是被动关节、不参与定位 | ✗ 同上 |
 * | `elbow.min > shoulder.max` ⇒ 合法域在局部角空间是**斜的** | ✗ 由平行四连杆耦合（gain = −1）决定 |
 * | 零位竖直段 = column + upper_arm + forearm | ✗ 依赖 MeArm 的连杆命名与被动腕 90° 锁定 |
 *
 * `frontend/tests/` 留下的都是**机制**：几何↔运动学解耦、标定往返、限位校验语义、
 * 纹理比例守卫、UV 布局基线… 它们换台机器人只是**数据**不同。
 *
 * ## 与 Python 侧的关系（刻意的双实现互证，不是重复）
 *
 * 同一批构造事实在 `robot-package/mearm-v1/tests/test_mearm_v1_structure.py` 里
 * 由 **MuJoCo + `fkref.py`** 核对；这里由**前端真实 FK（`engine.ts`）**核对。
 * 两侧用的是**完全不同的实现**，所以它们同时绿才是"两个独立裁判都同意"，
 * 一边绿一边红则是真信号。⇒ 刻意**不**把其中一侧删成"只留一份"。
 */
import { describe, expect, it } from 'vitest';
import { isMovableJoint, loadRobot } from '@robot/index';

/** 本文件所属的包 —— **从路径推**，不写死字符串（见下面的身份自检）。 */
const PACKAGE_ID = (() => {
  const p = decodeURIComponent(new URL('../', import.meta.url).pathname);
  return p.split('/').filter(Boolean).pop();
})();

/** 「本文件声明自己属于哪台机器人」—— 与 `PACKAGE_ID` 比对 */
const EXPECTED_PACKAGE_ID = 'mearm-v1';

const TOL = 1e-6;

describe('MeArm-V1 构造事实（包内）', () => {
  it('身份自检：本文件确实住在 mearm-v1 的包里', () => {
    // 这条防的是「整份拷到另一个包却忘了改」——那种情况下下面的断言会
    // 用 SO-101 的名义去测 MeArm，而且**全绿**。与 Python 侧
    // `test_package_identity` 是同一条纪律。
    expect(PACKAGE_ID).toBe(EXPECTED_PACKAGE_ID);
  });

  it('TCP 参考在被动腕上（`tcp.joint === tool`），不是夹爪', () => {
    const def = loadRobot(PACKAGE_ID!).definition;
    expect(def.tcp.joint).toBe('tool');
  });

  it('`tool` 是被动关节：在关节表里，但不参与定位（不在 movableJoints 里）', () => {
    const def = loadRobot(PACKAGE_ID!).definition;
    expect(def.joints.map((j) => j.id)).toContain('tool');
    expect(def.joints.filter(isMovableJoint).map((j) => j.id)).not.toContain('tool');
    // 定位关节恰好是那 4 个（多一个少一个都会改变协议帧宽度）
    expect(def.joints.filter(isMovableJoint).map((j) => j.id)).toEqual([
      'base',
      'shoulder',
      'elbow',
      'gripper',
    ]);
  });

  it('合法域在局部角空间是**斜的**：elbow 下限高于 shoulder 上限', () => {
    // 这是 MeArm 平行四连杆耦合（gain = −1，elbow 存**绝对角**）的直接后果：
    // 单一 hinge range 表达不了斜的合法域 ⇒ 限位一致性的唯一把关人只能是上层。
    // 细节论证见 Python 侧 `test_arithmetic_legal_region_is_oblique`。
    const def = loadRobot(PACKAGE_ID!).definition;
    const at = (id: string) => def.limits.find((l) => l.id === id)!.limits;
    expect(at('elbow').min).toBeGreaterThan(at('shoulder').max);
    // 内切区间必须是空集（否则"斜"这个前提变了）
    expect(at('elbow').min - at('shoulder').min).toBeGreaterThan(
      at('elbow').max - at('shoulder').max,
    );
  });

  it('零位 TCP：竖直段 = column + upper_arm + forearm，水平段 = tool_link 那一段', () => {
    const robot = loadRobot(PACKAGE_ID!);
    const len = (id: string): number => {
      const link = robot.definition.links.find((l) => l.id === id);
      expect(link, `模型里没有连杆 ${id}`).toBeDefined();
      return link!.length;
    };

    const zero: Record<string, number> = {};
    for (const joint of robot.definition.joints.filter(isMovableJoint)) zero[joint.id] = 0;
    const tcp = robot.kinematics.forward(zero).position;

    const wantZ = len('column_link') + len('upper_arm_link') + len('forearm_link');
    const wantX = len('tool_link');
    expect(tcp[2], '竖直段').toBeCloseTo(wantZ, 6);
    expect(tcp[0], '水平段').toBeCloseTo(wantX, 6);
    expect(Math.abs(tcp[1])).toBeLessThan(TOL);
  });

  it('夹爪开合**不影响** TCP（被动腕把它挡在 TCP 之外）', () => {
    const robot = loadRobot(PACKAGE_ID!);
    const at = (gripper: number): readonly number[] => {
      const js: Record<string, number> = {};
      for (const joint of robot.definition.joints.filter(isMovableJoint)) js[joint.id] = 0;
      js.base = 0;
      js.shoulder = 20;
      js.elbow = 125;
      js.gripper = gripper;
      return robot.kinematics.forward(js).position;
    };
    // 用**逐位相同**而不是容差：夹爪若真的参与了 TCP，偏差会是毫米级，
    // 而容差写法会让"只差一点点"这种真实耦合被放过。
    expect(at(90)).toEqual(at(0));
  });
});
