/**
 * `RobotRegistry` + `robotConfigRegistry` 的验收测试（单机器人：mearm-v1）。
 *
 * ## 这个文件要回答的事
 *
 * 1. **配置能选模型吗** —— `config/robots.yaml` 声明了哪些机器人、缺省是谁，
 *    且"配置里的 id"与"代码里的引擎"是否一一对应（防"加了配置忘了写引擎"）。
 * 2. **MeArm-V1 的 FK 是真的吗** —— 与参考实现交叉验证（见 `tests/sim`）。
 * 3. **注册表分派表是数据驱动的** —— 不靠 `if robot == …`（见 `RobotRegistry.ts`）：
 *    新增包只需往 `robot-package/<id>/` 丢一个 `kinematics/engine.ts`，
 *    `import.meta.glob` 自动发现，无需改本文件。
 */
import { describe, expect, it, beforeEach } from 'vitest';
import path from 'node:path';
import {
  MEARM_V1_ROBOT_ID,
  assertRegistryCoverage,
  defaultRobotId,
  forwardKinematics,
  listRegisteredRobotIds,
  listRobotIds,
  listRobots,
  loadRobot,
  loadRobotModel,
  loadRobotSelector,
  resetRobotRegistryCache,
} from '@robot/index';
import { REPO_ROOT, declaredPath } from '../helpers/robotPackage';

beforeEach(() => {
  resetRobotRegistryCache();
});

// ===========================================================================
// 1. 选择器（config/robots.yaml）
// ===========================================================================
describe('robotConfigRegistry · 模型选择器', () => {
  it('声明了 mearm-v1，缺省为 mearm-v1', () => {
    const selector = loadRobotSelector();
    expect(selector.robots.map((r) => r.id)).toEqual([MEARM_V1_ROBOT_ID]);
    expect(selector.default).toBe(MEARM_V1_ROBOT_ID);
    expect(defaultRobotId()).toBe(MEARM_V1_ROBOT_ID);
    expect(listRobotIds()).toEqual([MEARM_V1_ROBOT_ID]);
  });

  it('选择器只做选择：机器人的名字与路径都在，且路径指向真实存在的登记项', () => {
    const selector = loadRobotSelector();
    for (const robot of selector.robots) {
      expect(robot.name.length).toBeGreaterThan(0);
      // 路径必须是**仓库相对**路径（三端共用同一份解析约定）：
      // 不带盘符、不以 `/` 或 `..` 开头、分隔符统一为 `/`。
      expect(robot.config).not.toMatch(/^([A-Za-z]:|[\\/]|\.\.)/);
      expect(robot.config).not.toContain('\\');
      expect(robot.config.endsWith('.yaml')).toBe(true);
      // ★ 更强的一条：选择器声明的路径必须与该包 `manifest.yaml` 的
      //   `model.config` 声明**指向同一个文件** —— 否则"配置侧说 A、包侧说 B"
      //   会被不同读者各自读走。
      expect(path.resolve(REPO_ROOT, robot.config)).toBe(
        declaredPath(robot.id, 'model.config'),
      );
    }
  });

  it('listRobots() 与选择器同源（供 UI 列可选项）', () => {
    expect(listRobots()).toEqual([{ id: MEARM_V1_ROBOT_ID, name: 'MeArm-V1' }]);
  });

  it('未知 id 抛错且**不回退**到缺省（防止 id 写错时静默加载另一台机器人）', () => {
    expect(() => loadRobot('so-arm102')).toThrowError(/未知机器人 id/);
  });
});

// ===========================================================================
// 2. 注册表（配置 ↔ 代码 一致性）
// ===========================================================================
describe('RobotRegistry · 分派表', () => {
  it('配置里每一台机器人都有对应的引擎实现（防"加了配置忘了写引擎"）', () => {
    const coverage = assertRegistryCoverage();
    expect(coverage.missing).toEqual([]);
    expect(coverage.ok).toBe(true);
    // 反向：代码里也不该有配置里不存在的 id（防止残留死代码）
    expect([...listRegisteredRobotIds()].sort()).toEqual([...listRobotIds()].sort());
  });

  it('loadRobot() 省略 id 时读选择器的 default', () => {
    expect(loadRobot().id).toBe(defaultRobotId());
    expect(loadRobot().id).toBe(MEARM_V1_ROBOT_ID);
  });

  it('同一个 id 返回**同一个对象**（缓存），不同 id 各自独立', () => {
    const a1 = loadRobot(MEARM_V1_ROBOT_ID);
    const a2 = loadRobot(MEARM_V1_ROBOT_ID);
    expect(a1).toBe(a2);
  });

  it('复位缓存后重新构造（测试间不串味）', () => {
    const before = loadRobot(MEARM_V1_ROBOT_ID);
    resetRobotRegistryCache();
    const after = loadRobot(MEARM_V1_ROBOT_ID);
    expect(after).not.toBe(before);
    expect(after.definition.metadata.id).toBe(before.definition.metadata.id);
  });

  it('每台机器人都携带可用的 definition 与 kinematics', () => {
    for (const id of listRobotIds()) {
      const entry = loadRobot(id);
      expect(entry.definition.links.length).toBeGreaterThan(0);
      // 引擎与注册表共用**同一份** definition（引用相等）—— 工厂强制接收 definition 的意义所在
      expect(entry.kinematics.definition).toBe(entry.definition);
      expect(entry.definition.robotModel).toBe(loadRobotModel(id));
      expect(entry.definition.metadata.id.length).toBeGreaterThan(0);
    }
  });
});

// ===========================================================================
// 3. MeArm-V1 定义（RobotDefinition）
// ===========================================================================
describe('MeArm-V1 · RobotDefinition', () => {
  it('结构：6 连杆 / 4 可动(revolute)关节 + 1 被动腕(tool) / 4 执行器', () => {
    const { definition } = loadRobot(MEARM_V1_ROBOT_ID);
    expect(definition.links.length).toBe(6);
    // 可动（受控）关节 = revolute：base / shoulder / elbow / gripper
    const revolute = definition.joints.filter((j) => j.type === 'revolute');
    expect(revolute.map((j) => j.id)).toEqual(['base', 'shoulder', 'elbow', 'gripper']);
    // 被动腕 tool：爪锁水平，无独立自由度，不挂执行器（type 不是 revolute 也不是 fixed）
    // ⚠️ 用 `type === 'revolute'` 计可动关节：用 `!== 'fixed'` 会把 passive 也数进来（误得 5）
    const passive = definition.joints.filter((j) => j.type === 'passive');
    expect(passive.map((j) => j.id)).toEqual(['tool']);
    expect(definition.actuators.length).toBe(4);
  });

  it('TCP 参考被动腕关节 tool（爪锁水平），带 40mm 水平偏移', () => {
    const { definition } = loadRobot(MEARM_V1_ROBOT_ID);
    // mearm-v1 没有独立的 tcp_fixed 关节：TCP 直接挂在被动腕 tool 上，
    // offset [0,0,40] 即「腕枢轴 → 爪铰点」的水平前伸量（爪锁水平时完全落在 +X，
    // 故 IK 可先减这个常量偏移再退化成平面 2R）。
    expect(definition.tcp.joint).toBe('tool');
    expect(definition.tcp.offset).toEqual([0, 0, 40]);
    const toolJoint = definition.joints.find((j) => j.id === 'tool');
    expect(toolJoint?.type).toBe('passive');
  });

  it('defineRobot 零拷贝：links/joints/actuators/tcp/homePose 都是原对象引用', () => {
    const entry = loadRobot(MEARM_V1_ROBOT_ID);
    const model = entry.definition.robotModel;
    expect(entry.definition.links).toBe(model.links);
    expect(entry.definition.joints).toBe(model.joints);
    expect(entry.definition.actuators).toBe(model.actuators);
    expect(entry.definition.tcp).toBe(model.tcp);
    expect(entry.definition.homePose).toBe(model.homePose);
  });
});

// ===========================================================================
// 4. MeArm-V1 IK —— 真的在解
// ===========================================================================
describe('MeArm-V1 · IK', () => {
  it('能力声明：3 个定位自由度、无姿态接口、解析解', () => {
    const { kinematics } = loadRobot(MEARM_V1_ROBOT_ID);
    expect(kinematics.capability.positioningDof).toBe(3);
    expect(kinematics.capability.supportsOrientation).toBe(false);
    expect(kinematics.capability.solverKind).toBe('analytic');
  });

  it('MeArm 的 IK 仍然真的在解（与能力声明一致）', () => {
    const { definition, kinematics } = loadRobot(MEARM_V1_ROBOT_ID);
    // 目标由 FK 反推 ⇒ 天然可达，避免"测的是工作空间外"这种与被测行为无关的失败
    const target = forwardKinematics(definition.robotModel, definition.homePose).endEffector.position;

    const result = kinematics.inverse(target);
    expect(result.success).toBe(true);
    expect(result.positionError).not.toBeNull();
    expect(result.positionError!).toBeLessThan(1);
    expect(result.joints).not.toEqual({});
    // 有解就不该有失败码
    expect(result.error).toBeUndefined();
  });
});
