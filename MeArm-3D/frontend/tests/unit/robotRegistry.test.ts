/**
 * `RobotRegistry` + `robotConfigRegistry` + `SoArm101Kinematics` 的验收测试。
 *
 * ## 这个文件要回答的四件事
 *
 * 1. **配置能选模型吗** —— `config/robots.yaml` 声明了哪些机器人、缺省是谁，
 *    且"配置里的 id"与"代码里的引擎"是否一一对应（防"加了配置忘了写引擎"）。
 * 2. **SO-101 的 FK 是真的吗** —— 不是"跑出来了没报错"，而是与 **MuJoCo 官方 MJCF
 *    逐位交叉验证**。黄金值是 `mj_forward()` 读 `gripperframe` site 的世界位姿
 *    （见 `config/robots/so-arm101/robot.yaml` 的 homePose 注释与
 *    `assets/models/so-arm101/official/SOURCE.md`）。
 * 3. **IK 的留白是诚实的吗** —— `inverse()` 必须返回 `success:false` +
 *    `positionError === null` + `joints === {}`，**绝不能**是一个"看起来成功"的东西。
 * 4. **MeArm-V1 有没有被动到** —— 同一套抽象下 MeArm 的能力声明与加载结果逐值不变。
 *
 * ## 为什么姿态用旋转矩阵比而不是欧拉角
 *
 * FK 输出的是 intrinsic XYZ 欧拉角，穆 JoCo 给的是旋转矩阵。把矩阵反解成欧拉角再比，
 * 会在万向锁附近出现"其实完全等价却差了几十度"的假失败（本项目已踩过）。
 * ⇒ 一律把 FK 的欧拉角**正解成矩阵**再比，容差取 1e-9（量级上只受浮点误差影响）。
 */
import { describe, expect, it, beforeEach } from 'vitest';
import path from 'node:path';
// 资产登记表刻意**不**从 `@robot/index` 出口（与 textureRegistry 一致：它们是渲染层的
// URL 登记，不是模型层契约），故按路径直取。
import { listMeshKeys, resolveMeshUrl } from '@robot/model/meshRegistry';
import {
  SO_ARM101_ROBOT_ID,
  MEARM_V1_ROBOT_ID,
  assertRegistryCoverage,
  defaultRobotId,
  endEffectorMatrix,
  forwardKinematics,
  ikFailure,
  listRegisteredRobotIds,
  listRobotIds,
  listRobots,
  loadRobot,
  loadRobotModel,
  loadRobotSelector,
  mat4RotationMatrix3,
  resetRobotRegistryCache,
  rotationMatrixMaxAbsDiff,
  type JointState,
} from '@robot/index';
// SO-101 的引擎住在**包内**（Phase 2 步④）
import {
  SoArm101Kinematics,
  createSoArm101Kinematics,
  resetSoArm101IkWarning,
} from '../../../robot-package/so-arm101/kinematics/engine';
import { REPO_ROOT, declaredPath } from '../helpers/robotPackage';

// ---------------------------------------------------------------------------
// 黄金值：MuJoCo 官方 MJCF 的 gripperframe 世界位姿（mm / 行主序 3×3）
//
// 生成方式（可复现）：
//   m = mujoco.MjModel.from_xml_path('assets/models/so-arm101/official/so101_new_calib.xml')
//   d.qpos[:] = np.deg2rad([...]); mujoco.mj_forward(m, d)
//   d.site_xpos[gripperframe] * 1000 ,  d.site_xmat[gripperframe]
//
// ⚠️ 这不是"我们算出来的值"，是**引擎的作者（MuJoCo）给出的值** ——
//    用它当判据才构成"两个独立实现互证"，而不是自证。
// ---------------------------------------------------------------------------
const MUJOCO_GOLDEN: readonly {
  label: string;
  qposDeg: readonly [number, number, number, number, number, number];
  position: readonly [number, number, number];
  rotmat: readonly number[];
}[] = [
  {
    label: 'qpos 全零（= homePose）',
    qposDeg: [0, 0, 0, 0, 0, 0],
    position: [391.36190000000016, -0.011255287011580573, 226.46874461625617],
    rotmat: [1, 0, 0, -0, 0.99881539, -0.04866029, -0, 0.04866029, 0.99881539],
  },
  {
    label: 'qpos = [30, -20, 45, -15, 60, 0] deg',
    qposDeg: [30, -20, 45, -15, 60, 0],
    position: [294.1049760494343, -155.39042648162544, 147.97715429319035],
    rotmat: [
      0.85286853, 0.14435138, 0.5017748, -0.49240388, 0.54198532, 0.68102154, -0.17364818, -0.8278977,
      0.5333214,
    ],
  },
];

/** qpos（MuJoCo 顺序）→ JointState（本项目顺序与 qpos 同序，见 robot.yaml 的 joint 声明） */
function qposToJointState(qposDeg: readonly number[]): JointState {
  return {
    shoulder_pan: qposDeg[0]!,
    shoulder_lift: qposDeg[1]!,
    elbow_flex: qposDeg[2]!,
    wrist_flex: qposDeg[3]!,
    wrist_roll: qposDeg[4]!,
    gripper: qposDeg[5]!,
  };
}

beforeEach(() => {
  resetRobotRegistryCache();
  resetSoArm101IkWarning();
});

// ===========================================================================
// 1. 选择器（config/robots.yaml）
// ===========================================================================
describe('robotConfigRegistry · 模型选择器', () => {
  it('声明了两台机器人，缺省为 mearm-v1', () => {
    const selector = loadRobotSelector();
    expect(selector.robots.map((r) => r.id)).toEqual([MEARM_V1_ROBOT_ID, SO_ARM101_ROBOT_ID]);
    expect(selector.default).toBe(MEARM_V1_ROBOT_ID);
    expect(defaultRobotId()).toBe(MEARM_V1_ROBOT_ID);
    expect(listRobotIds()).toEqual([MEARM_V1_ROBOT_ID, SO_ARM101_ROBOT_ID]);
  });

  it('选择器只做选择：机器人的名字与路径都在，且路径指向真实存在的登记项', () => {
    const selector = loadRobotSelector();
    for (const robot of selector.robots) {
      expect(robot.name.length).toBeGreaterThan(0);
      // 路径必须是**仓库相对**路径（三端共用同一份解析约定）：
      // 不带盘符、不以 `/` 或 `..` 开头、分隔符统一为 `/`。
      //
      // ⚠️ Phase 2 之前这里断言的是 `startsWith('config/')` —— 那是**把当时的
      //    目录布局写进了期望值**。真值随包搬到 `robot-package/<id>/model/` 之后，
      //    它变红并**不是**回归，而是那条断言本来就问错了问题：
      //    "路径在不在 config/ 下" ≠ "路径是不是三端都能解析的仓库相对路径"。
      //    现在改成后者，并且**加**了一条更强的互检（见下）。
      expect(robot.config).not.toMatch(/^([A-Za-z]:|[\\/]|\.\.)/);
      expect(robot.config).not.toContain('\\');
      expect(robot.config.endsWith('.yaml')).toBe(true);
      // ★ 更强的一条：选择器声明的路径必须与该包 `manifest.yaml` 的
      //   `model.config` 声明**指向同一个文件** —— 否则"配置侧说 A、包侧说 B"
      //   会被不同读者各自读走，而这正是 Phase 2 要消灭的那类静默事故。
      expect(path.resolve(REPO_ROOT, robot.config)).toBe(
        declaredPath(robot.id, 'model.config'),
      );
    }
    // 两台机器人必须指向**不同**的配置文件 —— 否则"切换模型"是假的
    const paths = selector.robots.map((r) => r.config);
    expect(new Set(paths).size).toBe(paths.length);
  });

  it('listRobots() 与选择器同源（供 UI 列可选项）', () => {
    expect(listRobots()).toEqual([
      { id: MEARM_V1_ROBOT_ID, name: 'MeArm-V1' },
      { id: SO_ARM101_ROBOT_ID, name: 'SO-ARM101' },
    ]);
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
    const b = loadRobot(SO_ARM101_ROBOT_ID);
    expect(a1).toBe(a2);
    expect(a1).not.toBe(b);
    // 两台机器人的定义必须真的不同（否则"引入第二个模型"没发生）
    expect(a1.definition.metadata.id).not.toBe(b.definition.metadata.id);
  });

  it('复位缓存后重新构造（测试间不串味）', () => {
    const before = loadRobot(SO_ARM101_ROBOT_ID);
    resetRobotRegistryCache();
    const after = loadRobot(SO_ARM101_ROBOT_ID);
    expect(after).not.toBe(before);
    expect(after.definition.metadata.id).toBe(before.definition.metadata.id);
  });

  it('每台机器人都携带可用的 definition 与 kinematics', () => {
    for (const id of listRobotIds()) {
      const entry = loadRobot(id);
      expect(entry.definition.links.length).toBeGreaterThan(0);
      // 引擎与注册表共用**同一份** definition（引用相等）—— 工厂强制接收 definition 的意义所在
      expect(entry.kinematics.definition).toBe(entry.definition);
      // ⚠️ 这里**不能**断言 `definition.metadata.id === id`：
      //    选择器的 key（`mearm-v1`）是**注册表 id**，而 `robot.yaml → robot.id` 是
      //    **模型自己的 id**（`mearm`）—— 两者是两个概念，允许不同。
      //    真正该钉住的是「加载 id 得到的 definition 与 loadRobotModel(id) 同源」。
      expect(entry.definition.robotModel).toBe(loadRobotModel(id));
      expect(entry.definition.metadata.id.length).toBeGreaterThan(0);
    }
  });
});

// ===========================================================================
// 3. SO-ARM101 定义（RobotDefinition）
// ===========================================================================
describe('SO-ARM101 · RobotDefinition', () => {
  it('结构：8 连杆 / 7 关节（6 铰链 + 1 固定 TCP 帧）/ 6 执行器', () => {
    const { definition } = loadRobot(SO_ARM101_ROBOT_ID);
    expect(definition.links.length).toBe(8);
    expect(definition.joints.length).toBe(7);
    expect(definition.actuators.length).toBe(6);

    const movable = definition.joints.filter((j) => j.type !== 'fixed');
    expect(movable.map((j) => j.id)).toEqual([
      'shoulder_pan',
      'shoulder_lift',
      'elbow_flex',
      'wrist_flex',
      'wrist_roll',
      'gripper',
    ]);
    // 限位视图只含可动关节
    expect(definition.limits.map((l) => l.id)).toEqual(movable.map((j) => j.id));
  });

  it('TCP 帧是固定关节，且**声明在 gripper 之前**（kinematicChain 取第一个子关节）', () => {
    const { definition } = loadRobot(SO_ARM101_ROBOT_ID);
    const ids = definition.joints.map((j) => j.id);
    expect(ids.indexOf('gripper_frame')).toBeLessThan(ids.indexOf('gripper'));
    expect(definition.tcp.joint).toBe('gripper_frame');
    expect(definition.tcp.offset).toEqual([0, 0, 0]);
    expect(definition.joints.find((j) => j.id === 'gripper_frame')!.type).toBe('fixed');
  });

  it('限位取 MJCF 满精度（elbow_flex 的 ±1.69 rad = ±96.829867377°）', () => {
    const { definition } = loadRobot(SO_ARM101_ROBOT_ID);
    const elbow = definition.joints.find((j) => j.id === 'elbow_flex')!;
    expect(elbow.limits.min).toBeCloseTo(-96.829867377, 9);
    expect(elbow.limits.max).toBeCloseTo(96.829867377, 9);
  });

  it('执行器是关节空间（unit = joint），无 offset/scale/reverse 标定', () => {
    const { definition, kinematics } = loadRobot(SO_ARM101_ROBOT_ID);
    for (const actuator of definition.actuators) {
      expect(actuator.unit).toBe('joint');
      expect(actuator.offset).toBe(0);
      expect(actuator.scale).toBe(1);
      expect(actuator.reverse).toBe(false);
    }
    // 关节空间 ⇒ 不该被 0..180 的舵机行程规则约束（对齐 RobotModel 的 ACTUATOR_LIMIT_180）
    expect(kinematics.capability.solverKind).toBe('none');
  });

  it('defineRobot 零拷贝：links/joints/actuators/tcp/homePose 都是原对象引用', () => {
    const entry = loadRobot(SO_ARM101_ROBOT_ID);
    const model = entry.definition.robotModel;
    expect(entry.definition.links).toBe(model.links);
    expect(entry.definition.joints).toBe(model.joints);
    expect(entry.definition.actuators).toBe(model.actuators);
    expect(entry.definition.tcp).toBe(model.tcp);
    expect(entry.definition.homePose).toBe(model.homePose);
  });

  it('视觉件全部是 mesh，且声明了 rpy 约定（官方 URDF 原样承载）', () => {
    const { definition } = loadRobot(SO_ARM101_ROBOT_ID);
    let meshCount = 0;
    for (const link of definition.links) {
      if (link.geometry.type === 'mesh') {
        meshCount += 1;
        expect(link.geometry.rotationConvention).toBe('rpy');
        expect(link.geometry.file.startsWith('so-arm101/official/assets/')).toBe(true);
      }
      for (const detail of link.details ?? []) {
        if (detail.type === 'mesh') {
          meshCount += 1;
          expect(detail.rotationConvention).toBe('rpy');
        }
      }
    }
    // 官方 13 个 STL 全部登记（base_link 一个 mesh 被复用于多个 body，故按 geom 计数为 17 visual）
    expect(meshCount).toBe(17);
  });
});

// ===========================================================================
// 4. SO-ARM101 FK —— 与 MuJoCo 官方 MJCF 交叉验证（本文件的核心判据）
// ===========================================================================
describe('SO-ARM101 · FK ↔ MuJoCo 交叉验证', () => {
  /**
   * 容差的**实测依据**（不是拍的）。
   *
   * 官方两份文件对同一批旋转用了不同的表示与不同的截断精度：
   *
   * | 来源 | 表示 | 精度 |
   * |---|---|---|
   * | URDF `<origin rpy>` | fixed-axis XYZ，**弧度** | 截断到 6 位有效数字（`1.5708`、`3.14159`） |
   * | MJCF body `quat` | 四元数 | 6 位小数，但形如 `[0.707107,0,0.707107,0]` **归一化后恰好是 90°** |
   *
   * ⇒ `1.5708 − π/2 = 3.67e-6 rad`，逐关节累积到 TCP 上 = **实测 2.0~2.3 µm**。
   *   这是**官方文件自身的精度差**，不是我们的换算错误：
   *   robot.yaml 的 origin 逐值取自 URDF 原文（可逐个复核），故保留这微差。
   *
   * 上限取 10 µm / 1e-4 是**够松**（离实测有 4~10 倍余量）也**够紧**：
   * 欧拉角约定用反会差 90°（数十 mm）、origin 抄错会差 mm 级 —— 两者都会被这两条拦住。
   */
  const POS_TOL_MM = 0.01;
  const ROTMAT_TOL = 1e-4;

  for (const golden of MUJOCO_GOLDEN) {
    it(`${golden.label}：末端位置与 MuJoCo 一致（< ${POS_TOL_MM} mm）`, () => {
      const model = loadRobot(SO_ARM101_ROBOT_ID).definition.robotModel;
      const state = qposToJointState(golden.qposDeg);
      const pose = forwardKinematics(model, state).endEffector;

      const dpos = [0, 1, 2].map((axis) => pose.position[axis]! - golden.position[axis]!);
      const diff = Math.hypot(...dpos);
      if (diff >= POS_TOL_MM) {
        throw new Error(
          `末端位置与 MuJoCo 相差 ${(diff * 1000).toFixed(2)} µm（逐轴 µm: ` +
            `${dpos.map((d) => (d * 1000).toFixed(2)).join(', ')}），超出 ${POS_TOL_MM} mm。`,
        );
      }
      expect(diff).toBeLessThan(POS_TOL_MM);
    });

    it(`${golden.label}：末端姿态与 MuJoCo 一致（旋转矩阵最大绝对差 < ${ROTMAT_TOL}）`, () => {
      const model = loadRobot(SO_ARM101_ROBOT_ID).definition.robotModel;
      const state = qposToJointState(golden.qposDeg);
      const rotmat = mat4RotationMatrix3(endEffectorMatrix(model, state));

      // 用旋转矩阵比（而非欧拉角）以规避万向锁假失败 —— 见文件头
      expect(rotationMatrixMaxAbsDiff(rotmat, golden.rotmat)).toBeLessThan(ROTMAT_TOL);
    });
  }

  it('homePose 与机器人定义自洽：所有关节都在限位内，且 FK 落在第一象限远离原点', () => {
    const { definition } = loadRobot(SO_ARM101_ROBOT_ID);
    const pose = forwardKinematics(definition.robotModel, definition.homePose).endEffector;
    for (const limit of definition.limits) {
      const value = definition.homePose[limit.id]!;
      expect(value).toBeGreaterThanOrEqual(limit.limits.min);
      expect(value).toBeLessThanOrEqual(limit.limits.max);
    }
    expect(pose.position[0]!).toBeGreaterThan(100);
    expect(pose.position[2]!).toBeGreaterThan(100);
  });

  it('引擎层消费的是**同一份** RobotModel（引用相等），结果与直接调用逐位一致', () => {
    const entry = loadRobot(SO_ARM101_ROBOT_ID);
    const model = loadRobotModel(SO_ARM101_ROBOT_ID);
    const state = qposToJointState(MUJOCO_GOLDEN[1]!.qposDeg);

    // ① 数据同源：抽象层没有另建一份模型（否则会出现"抽象层看的是旧模型"的幽灵 bug）
    expect(entry.kinematics.definition.robotModel).toBe(model);

    // ② 行为逐位一致：`toStrictEqual` 而非容差逼近 ⇒ "抽象层没有顺手修正任何东西"
    expect(entry.kinematics.forward(state)).toStrictEqual(
      forwardKinematics(model, state).endEffector,
    );
    expect(entry.kinematics.forwardAll(state)).toStrictEqual(forwardKinematics(model, state));
  });

  it('forwardAll 覆盖全部 7 个关节坐标系（含固定 TCP 帧）', () => {
    const entry = loadRobot(SO_ARM101_ROBOT_ID);
    const poses = entry.kinematics.forwardAll(qposToJointState([0, 0, 0, 0, 0, 0]));
    expect(Object.keys(poses.joints).sort()).toEqual(
      ['elbow_flex', 'gripper', 'gripper_frame', 'shoulder_lift', 'shoulder_pan', 'wrist_flex', 'wrist_roll'].sort(),
    );
  });

  it('FK 不是退化的：动一个关节末端就会动（否则"通过"毫无意义）', () => {
    const entry = loadRobot(SO_ARM101_ROBOT_ID);
    const base = qposToJointState([0, 0, 0, 0, 0, 0]);
    const moved = qposToJointState([40, 0, 0, 0, 0, 0]);
    const a = entry.kinematics.forward(base).position;
    const b = entry.kinematics.forward(moved).position;
    const delta = Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    // shoulder_pan 绕基座 Z 转 40° ⇒ 末端在 XY 平面扫过一段明显弧长
    expect(delta).toBeGreaterThan(100);
  });
});

// ===========================================================================
// 5. 能力声明的**诚实性**（这是本阶段最容易被"做假"的地方）
// ===========================================================================
describe('SO-ARM101 · 能力声明与 IK 留白', () => {
  it('capability：5 个定位自由度、无姿态接口、无求解器', () => {
    const { kinematics } = loadRobot(SO_ARM101_ROBOT_ID);
    expect(kinematics.capability.positioningDof).toBe(5);
    expect(kinematics.capability.supportsOrientation).toBe(false);
    expect(kinematics.capability.solverKind).toBe('none');
  });

  it('inverse() 返回**诚实的失败**：不是假成功，也不是 0 误差', () => {
    const { kinematics } = loadRobot(SO_ARM101_ROBOT_ID);
    const result = kinematics.inverse([200, 0, 200]);

    expect(result.success).toBe(false);
    expect(result.error).toBe('NOT_IMPLEMENTED');
    expect(result.joints).toEqual({});
    // ★ 失败时两个误差必须都是 null，**不能**是 0（0 会被读成"完美命中"）
    expect(result.positionError).toBeNull();
    expect(result.orientationError).toBeNull();
    expect(result.constraintsSatisfied).toBe(false);
    expect(result.solutionType).toBe('none');
    expect(result.message).toContain('未实现');
  });

  it('inverse() 不依赖入参：任意目标都返回同一形状的失败（没有隐藏的魔数路径）', () => {
    const { kinematics } = loadRobot(SO_ARM101_ROBOT_ID);
    const a = kinematics.inverse([0, 0, 0]);
    const b = kinematics.inverse([999, -999, 999], { prefer: 'elbow-up', toleranceMm: 0.1 });
    expect(a).toEqual(b);
  });

  it('与 MeArm 的形状对齐：ikFailure 的单例语义逐字段相同', () => {
    const { kinematics } = loadRobot(SO_ARM101_ROBOT_ID);
    expect(kinematics.inverse([0, 0, 0])).toEqual(
      ikFailure(
        'NOT_IMPLEMENTED',
        kinematics.inverse([0, 0, 0]).message,
      ),
    );
  });

  it('createSoArm101Kinematics() 复用缓存的模型（不重复解析，也不另建一份定义视图）', () => {
    const fromRegistry = loadRobot(SO_ARM101_ROBOT_ID);
    const engine = createSoArm101Kinematics();
    expect(engine).toBeInstanceOf(SoArm101Kinematics);
    // `defineRobot` 是纯视图函数（每次都新建包装对象），所以比的是底层**模型**的引用
    expect(engine.definition.robotModel).toBe(fromRegistry.definition.robotModel);
    expect(engine.definition).toStrictEqual(fromRegistry.definition);
  });
});

// ===========================================================================
// 6. Golden Baseline 未被触及
// ===========================================================================
describe('MeArm-V1 · 抽象引入后行为不变', () => {
  it('能力声明与引入前逐值一致（3 DOF / 解析解 / 不支持姿态）', () => {
    const { kinematics } = loadRobot(MEARM_V1_ROBOT_ID);
    expect(kinematics.capability.positioningDof).toBe(3);
    expect(kinematics.capability.supportsOrientation).toBe(false);
    expect(kinematics.capability.solverKind).toBe('analytic');
  });

  it('MeArm 的 IK 仍然真的在解（与 SO-101 的留白形成对照）', () => {
    const { definition, kinematics } = loadRobot(MEARM_V1_ROBOT_ID);
    // 目标由 FK 反推 ⇒ 天然可达，避免"测的是工作空间外"这种与被测行为无关的失败
    const target = forwardKinematics(definition.robotModel, definition.homePose).endEffector.position;

    const result = kinematics.inverse(target);
    expect(result.success).toBe(true);
    expect(result.positionError).not.toBeNull();
    expect(result.positionError!).toBeLessThan(1);
    expect(result.joints).not.toEqual({});
    // 与 SO-101 的关键差别：这里 `error` **不出现**（有解就不该有失败码）
    expect(result.error).toBeUndefined();
  });

  it('两台机器人的几何尺度确实不同（引入的是"另一个模型"而不是同一份数据换个名字）', () => {
    const mearm = loadRobot(MEARM_V1_ROBOT_ID).definition;
    const so101 = loadRobot(SO_ARM101_ROBOT_ID).definition;
    const mearmGeom = mearm.links.map((l) => l.geometry.type).join(',');
    const so101Geom = so101.links.map((l) => l.geometry.type).join(',');
    expect(mearmGeom).not.toBe(so101Geom);
    expect(mearm.joints.length).not.toBe(so101.joints.length);
  });
});

// ===========================================================================
// 7. 交叉产物一致性：robot.yaml 引用的每个网格都必须能被渲染层解析到
// ===========================================================================
describe('SO-101 的网格引用可解析（robot.yaml ↔ meshRegistry）', () => {
  it('13 个官方 STL 全部登记，且 key 与 robot.yaml 的 file 逐字对应', () => {
    const { definition } = loadRobot(SO_ARM101_ROBOT_ID);
    const referenced = definition.links
      .flatMap((link) => [
        ...(link.geometry.type === 'mesh' ? [link.geometry.file] : []),
        ...(link.details ?? []).flatMap((d) => (d.type === 'mesh' ? [d.file] : [])),
      ])
      .filter((file, index, all) => all.indexOf(file) === index);

    expect(referenced.length).toBe(13);
    for (const file of referenced) {
      // resolveMeshUrl 对未登记的 key 返回 undefined —— 贴图/网格失败是**静默**的，
      // 所以这条断言是"再没有第二个地方会告诉你网格丢了"。
      expect(resolveMeshUrl(file), `robot.yaml 引用了未登记的网格: ${file}`).toBeDefined();
    }
    // 反向：官方目录里的 STL 数量与登记数一致（防止漏登记）
    expect(listMeshKeys().filter((key) => key.startsWith('so-arm101/')).length).toBe(13);
  });

  it('所有 mesh 都声明了 rpy 约定（官方 URDF 原样承载，未换算）', () => {
    const { definition } = loadRobot(SO_ARM101_ROBOT_ID);
    const meshes = definition.links.flatMap((link) => [
      ...(link.geometry.type === 'mesh' ? [link.geometry] : []),
      ...(link.details ?? []).flatMap((d) => (d.type === 'mesh' ? [d] : [])),
    ]);
    expect(meshes.length).toBe(17);
    for (const mesh of meshes) {
      expect(mesh.rotationConvention).toBe('rpy');
    }
  });
});
