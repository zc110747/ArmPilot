/**
 * Robot Package manifest ↔ Core 的**跨端一致性**测试（Phase 1）。
 *
 * ## 为什么必须有一个 TS 侧的测试
 *
 * `manifest.yaml` 的内容由 `core/python/robopkg/` 校验（Python 侧），但
 * **能力声明的另一半是 TS 的**：`KinematicsEngine.capability`
 * （`solverKind` / `supportsOrientation` / `positioningDof`）。
 *
 * Python 读不到 TS 的引擎，TS 也读不到 Python 的推导 ⇒ 于是出现一条**跨端缝**：
 * manifest 可以声明 `ik: true` 而引擎的 `solverKind === 'none'`，
 * 两边各自"自洽"，合起来是矛盾的。上层拿 manifest 判断能力、拿引擎去求解，
 * 就会去调一个不存在的求解器。
 *
 * ⇒ 本文件就是那条缝的守卫：**把两份声明对起来**。
 *
 * ## 顺带守住的三件事
 *
 * 1. **包集合 == 选择器集合 == 引擎工厂集合**（任何一边多/少都必须报错）；
 * 2. manifest 的**形状**（顶层键集合）—— 形状本身就是契约，
 *    少一个字段意味着某处调用会拿到 `undefined`；
 * 3. `ik: false` 的包**不许有 entry**（伪造 IK 的第二种形态：声明说没有，
 *    却留了一个"随时可以接上"的实现）。
 */
import { describe, expect, it } from 'vitest';
import { parse as parseYaml } from 'yaml';
import {
  MEARM_V1_ROBOT_ID,
  SO_ARM101_ROBOT_ID,
  isMovableJoint,
  listRegisteredRobotIds,
  listRobotIds,
  listRobots,
  loadRobot,
  loadRobotSelector,
} from '@robot/index';

/** `robot-package/<id>/manifest.yaml` 的形状（与 Python 侧 `parse_manifest` 对齐） */
interface ManifestLike {
  readonly id: string;
  readonly name: string;
  readonly package: { readonly format: number; readonly version: string };
  readonly robot: { readonly type: string };
  readonly model: { readonly config: string; readonly physics?: string };
  readonly simulation: { readonly tcp_site: string; readonly mjcf?: string };
  readonly kinematics: {
    readonly fk: { readonly type: string; readonly entry?: string };
    readonly ik: { readonly type: string; readonly entry?: string };
  };
  readonly capabilities: {
    readonly position: boolean;
    readonly orientation: boolean;
    readonly gripper: boolean;
    readonly ik: boolean;
    readonly simulation: boolean;
    readonly hardware: boolean;
  };
  readonly tests: { readonly cases: string };
}

const MANIFEST_TEXTS = import.meta.glob('../../../robot-package/*/manifest.yaml', {
  eager: true,
  query: '?raw',
  import: 'default',
}) as Record<string, string>;

/** manifest 顶层必须**恰好**是这些键（形状即契约） */
const MANIFEST_TOP_KEYS = [
  'capabilities',
  'id',
  'kinematics',
  'model',
  'name',
  'package',
  'robot',
  'simulation',
  'tests',
];

function packageIdOf(modulePath: string): string {
  // 用标记定位而不是切片：glob 返回的 key 是否带 `../` 前缀是 Vite 的实现细节
  const MARKER = 'robot-package/';
  const i = modulePath.lastIndexOf(MARKER);
  if (i < 0) throw new Error(`manifest glob 的 key 不符合预期：${modulePath}`);
  const rest = modulePath.slice(i + MARKER.length);
  const id = rest.replace(/\/manifest\.yaml$/, '');
  if (id === rest || id.length === 0) {
    throw new Error(`manifest glob 的 key 不符合预期：${modulePath}`);
  }
  return id;
}

const MANIFESTS: Record<string, ManifestLike> = Object.fromEntries(
  Object.entries(MANIFEST_TEXTS).map(([path, text]) => [
    packageIdOf(path),
    parseYaml(text) as ManifestLike,
  ]),
);

const PACKAGE_IDS = Object.keys(MANIFESTS).sort();

describe('Robot Package manifest 与选择器/引擎的三方一致性', () => {
  it('glob 确实扫到了包（否则下面所有断言都会"空集通过"）', () => {
    expect(PACKAGE_IDS).toEqual([MEARM_V1_ROBOT_ID, SO_ARM101_ROBOT_ID].sort());
  });

  it('包集合 == 选择器声明的集合', () => {
    expect(PACKAGE_IDS).toEqual([...listRobotIds()].sort());
  });

  it('包集合 == 引擎工厂注册的集合（配置有、代码没有 ⇒ 在这里就炸）', () => {
    expect(PACKAGE_IDS).toEqual([...listRegisteredRobotIds()].sort());
  });

  it('每台的 manifest.name 与选择器 name 一致', () => {
    const byId = new Map(listRobots().map((r) => [r.id, r.name]));
    for (const id of PACKAGE_IDS) {
      expect(MANIFESTS[id]!.name, `${id} 的 name`).toBe(byId.get(id));
    }
  });

  it('manifest 与选择器指向**同一批文件**（路径二义会让加载结果取决于读哪一份）', () => {
    const byId = new Map(loadRobotSelector().robots.map((r) => [r.id, r]));
    for (const id of PACKAGE_IDS) {
      const m = MANIFESTS[id]!;
      const sel = byId.get(id);
      expect(sel, `选择器里没有 ${id}`).toBeDefined();
      expect(m.model.config, `${id}.model.config`).toBe(sel!.config);
      expect(m.simulation.mjcf, `${id}.simulation.mjcf`).toBe(sel!.mjcf);
      expect(m.simulation.tcp_site, `${id}.simulation.tcp_site`).toBe(sel!.tcpSite);
    }
  });

  it('manifest 顶层键集合就是契约（多一个少一个都报错）', () => {
    for (const id of PACKAGE_IDS) {
      expect(Object.keys(MANIFESTS[id]!).sort(), `${id} 的顶层键`).toEqual(MANIFEST_TOP_KEYS);
    }
  });
});

describe('能力声明（manifest）必须与引擎的 capability 逐字段一致', () => {
  it('ik ⟺ 有求解器；position ⟺ 有求解器；orientation ⟺ 支持姿态', () => {
    for (const id of PACKAGE_IDS) {
      const caps = MANIFESTS[id]!.capabilities;
      const cap = loadRobot(id).kinematics.capability;
      const hasSolver = cap.solverKind !== 'none';

      expect(caps.ik, `${id}: manifest.ik vs solverKind=${cap.solverKind}`).toBe(hasSolver);
      expect(caps.position, `${id}: manifest.position vs solverKind=${cap.solverKind}`).toBe(
        hasSolver,
      );
      expect(
        caps.orientation,
        `${id}: manifest.orientation vs supportsOrientation`,
      ).toBe(cap.supportsOrientation);
    }
  });

  it('manifest.ik.type 与 solverKind 是同一件事的两种写法', () => {
    for (const id of PACKAGE_IDS) {
      const declared = MANIFESTS[id]!.kinematics.ik.type;
      const cap = loadRobot(id).kinematics.capability;
      const expected = declared === 'none' ? 'none' : cap.solverKind;
      expect(expected, `${id}: ik.type=${declared} vs solverKind=${cap.solverKind}`).toBe(
        cap.solverKind,
      );
      // 'none' 是唯一允许 solverKind 为 none 的声明方式
      if (cap.solverKind === 'none') expect(declared).toBe('none');
      if (declared === 'none') expect(cap.solverKind).toBe('none');
    }
  });

  it('gripper 能力与模型里真实存在的夹爪关节一致（声明不许撒谎）', () => {
    for (const id of PACKAGE_IDS) {
      const caps = MANIFESTS[id]!.capabilities;
      const joints = loadRobot(id).definition.joints;
      // ⚠️ 判据必须写成 `role ?? id`，与 Python 侧 `robotcfg.py` 的
      //    `role=str(j.get("role", j["id"]))` **同一条规则**。
      //
      //    起因是一处实测到的跨端分歧（本测试的首版就是被它顶红的）：
      //      · `config/robots/so-arm101/robot.yaml` 里关节**没有** `role` 字段
      //        （生成器只写 id / type —— 官方 URDF 里本来也没有"角色"这个概念）
      //      · Python：`role` 缺省 = **关节 id** ⇒ `gripper` 在里面，`has_gripper = true`
      //      · TS：`Joint.role` 可选，且 `JointRole` 是**封闭 5 值联合**（MeArm 专用）
      //        ⇒ `role === undefined` ⇒ 推导出 `has_gripper = false`
      //    两者对同一份文件给出相反结论 ⇒ 契约测试必须先用**同一条规则**判事实，
      //    否则它测的是"两端各自的实现"，而不是"这份 manifest 说得对不对"。
      //
      //    这条分歧本身已作为缺陷单独登记（前端 `jointByRole(model,'gripper')`
      //    对 SO-101 返回 `undefined`、`gripperJointId()` 返回 `null`），
      //    **不在本次重构里顺手修** —— 它要动 `JointRole` 这个公共类型，属于独立决策。
      const hasGripper = joints
        .filter(isMovableJoint)
        .some((j) => (j.role ?? j.id) === 'gripper');
      expect(caps.gripper, `${id}: manifest.gripper`).toBe(hasGripper);
    }
  });

  it('SO-101 的夹爪关节确实以 id 形式存在（契约测试的判据前提）', () => {
    const joints = loadRobot(SO_ARM101_ROBOT_ID).definition.joints.filter(isMovableJoint);
    expect(joints.map((j) => j.id)).toContain('gripper');
  });

  it('orientation: true 的包必须真的有姿态能力（否则上层会去下姿态断言）', () => {
    for (const id of PACKAGE_IDS) {
      const caps = MANIFESTS[id]!.capabilities;
      if (!caps.orientation) continue;
      expect(loadRobot(id).kinematics.capability.supportsOrientation).toBe(true);
    }
  });
});

describe('SO-ARM101：能力留白是诚实的（不伪造 IK）', () => {
  it('manifest 声明 position=false / ik=false，且 ik 实现类型是 none 且**没有 entry**', () => {
    const m = MANIFESTS[SO_ARM101_ROBOT_ID]!;
    expect(m.capabilities.ik).toBe(false);
    expect(m.capabilities.position).toBe(false);
    expect(m.kinematics.ik.type).toBe('none');
    expect(m.kinematics.ik.entry).toBeUndefined();
  });

  it('引擎的 capability 与此一致：solverKind=none / supportsOrientation=false', () => {
    const cap = loadRobot(SO_ARM101_ROBOT_ID).kinematics.capability;
    expect(cap.solverKind).toBe('none');
    expect(cap.supportsOrientation).toBe(false);
  });

  it('inverse() 的失败是"说得出为什么"的失败（NOT_IMPLEMENTED + 空解 + null 残差）', () => {
    const r = loadRobot(SO_ARM101_ROBOT_ID);
    const out = r.kinematics.inverse([200, 0, 100] as const);
    expect(out.success).toBe(false);
    expect(out.error).toBe('NOT_IMPLEMENTED');
    expect(out.joints).toEqual({});
    expect(out.positionError).toBeNull();
  });
});

describe('MeArm-V1：能力没有因为抽象而改变', () => {
  it('manifest 声明 position/ik/orientation 与引擎一致（analytic + 不支持姿态）', () => {
    const m = MANIFESTS[MEARM_V1_ROBOT_ID]!;
    expect(m.capabilities.ik).toBe(true);
    expect(m.capabilities.position).toBe(true);
    expect(m.capabilities.orientation).toBe(false);
    expect(m.kinematics.fk.type).toBe('engine');
    expect(m.kinematics.ik.type).toBe('custom');

    const cap = loadRobot(MEARM_V1_ROBOT_ID).kinematics.capability;
    expect(cap.solverKind).toBe('analytic');
    expect(cap.supportsOrientation).toBe(false);
    expect(cap.positioningDof).toBe(3);
  });

  it('fk 类型是 engine ⇒ 不允许再有 entry（两份 FK 会分家且不报错）', () => {
    for (const id of PACKAGE_IDS) {
      const fk = MANIFESTS[id]!.kinematics.fk;
      expect(fk.type).toBe('engine');
      expect(fk.entry, `${id}: type=engine 时不应有 entry`).toBeUndefined();
    }
  });
});
