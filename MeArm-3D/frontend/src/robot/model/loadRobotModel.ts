/**
 * `robot.yaml` → `RobotModel` 加载器。
 *
 * 机器人参数必须独立于代码：本文件是「配置 → 模型」的唯一入口，
 * 业务代码只消费 `RobotModel`，绝不直接读 yaml，也绝不硬编码尺寸/角度。
 *
 * ## 加载哪一台？（`loadRobotModel(robotId?)`）
 *
 * ```text
 *   loadRobotModel('mearm-v1')   ← 显式指定（推荐；所有既有调用点都已显式化）
 *   loadRobotModel('so-arm101')  ← 另一台机器人
 *   loadRobotModel()             ← 读 config/robots.yaml 的 `default`
 * ```
 *
 * 「用哪台机器人」这件事由 `config/robots.yaml` 声明（见 `robotConfigRegistry`），
 * 而"参数是什么"永远只在各自的 `robot.yaml` 里 —— 两件事分开，谁都不复制谁。
 *
 * ⚠️ 未知 id **抛错**而不是回退到缺省：回退会让"id 写错一个字"表现成
 * "静默加载了另一台机器人"，而模型不对时所有 FK / 限位判据都会失真却都能跑。
 *
 * 校验分两级：
 *   - 解析级：字段类型 / 必填项错误 → 抛 `RobotConfigError`
 *   - 模型级：拓扑 / 限位 / 标定自洽性 → 抛 `RobotModelError`（携带完整 issue 列表）
 */
import { parse as parseYaml } from 'yaml';
import type { Actuator, ActuatorLimits, ActuatorUnit } from './Actuator';
import { RobotConfigError } from './configError';
import type { Joint, JointCoupling, JointLimits, JointOrigin, JointRole, JointType } from './Joint';
import type {
  JawGeometry,
  Link,
  LinkGeometry,
  MeshGeometry,
  PlateGeometry,
  ServoGeometry,
} from './Link';
import {
  DEFAULT_JAW_SPEC,
  DEFAULT_PLATE_CORNER_RADIUS,
  DEFAULT_SERVO_SHAFT_LENGTH,
  DEFAULT_SERVO_SIZE,
} from './Link';
import type { EulerDeg, JointState, RotationConvention, Vec3 } from './Pose';
import { MEARM_V1_ROBOT_ID } from './robotIds';
import {
  defaultRobotId,
  resetRobotSelectorCache,
  robotYamlTextById,
} from './robotConfigRegistry';
import type { Appearance, ModelIssue, RobotModel, TcpSpec } from './RobotModel';
import {
  DEFAULT_APPEARANCE,
  EXPOSURE_EV_RANGE,
  RobotModelError,
  assertValidRobotModel,
  validateRobotModel,
} from './RobotModel';

/**
 * 随包内置的 **MeArm-V1**（Golden Baseline）配置原文。
 *
 * 用途：① 缺省加载路径的历史兼容；② `tests/unit/robotModel.test.ts` 对它做
 * 字符串级改写以构造"坏配置"用例。**它不是"通用内置配置"** ——
 * 要拿某一台机器人的原文，请用 `robotYamlTextById(id)`。
 *
 * ★ Phase 2：这里**不再有**自己的静态导入。原先写的是
 *   `import robotYamlText from '@config/robot.yaml?raw'`
 *   —— 真值随包搬走后那条路径已不存在，而 Vite 的 `?raw` 又**必须**是静态的，
 *   于是它成了唯一一个"绕开 Registry 自己拼路径"的读者。现在改为向
 *   `robotConfigRegistry` 要 mearm-v1 的原文（与全项目其余读者同一条链路）。
 *
 * ⚠️ 这仍是一个 **MeArm-specific 的过渡导出**（fixture 级）：按 Phase 2 步⑤
 *   "包内测试代码（期望值随包）" 的拆分，对它做字符串改写的用例会搬进
 *   `robot-package/mearm-v1/`，届时本常量一并随之走。
 */
export const BUNDLED_ROBOT_YAML: string = robotYamlTextById(MEARM_V1_ROBOT_ID);

export { RobotConfigError };

type Dict = Record<string, unknown>;

// ---------------------------------------------------------------------------
// 取值原语
// ---------------------------------------------------------------------------

function asDict(value: unknown, path: string): Dict {
  if (typeof value !== 'object' || value === null || Array.isArray(value)) {
    throw new RobotConfigError(`${path} 必须是对象`);
  }
  return value as Dict;
}

function asArray(value: unknown, path: string): unknown[] {
  if (!Array.isArray(value)) throw new RobotConfigError(`${path} 必须是数组`);
  return value;
}

function reqString(dict: Dict, key: string, path: string): string {
  const value = dict[key];
  if (typeof value !== 'string' || value.trim() === '') {
    throw new RobotConfigError(`${path}.${key} 必须是非空字符串`);
  }
  return value;
}

function optString(dict: Dict, key: string, path: string): string | undefined {
  const value = dict[key];
  if (value === undefined || value === null) return undefined;
  if (typeof value !== 'string') throw new RobotConfigError(`${path}.${key} 必须是字符串`);
  return value;
}

function reqNumber(dict: Dict, key: string, path: string): number {
  const value = dict[key];
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    throw new RobotConfigError(`${path}.${key} 必须是有限数值`);
  }
  return value;
}

function optNumber(dict: Dict, key: string, path: string, fallback: number): number {
  const value = dict[key];
  if (value === undefined || value === null) return fallback;
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    throw new RobotConfigError(`${path}.${key} 必须是有限数值`);
  }
  return value;
}

function optBool(dict: Dict, key: string, path: string, fallback: boolean): boolean {
  const value = dict[key];
  if (value === undefined || value === null) return fallback;
  if (typeof value !== 'boolean') throw new RobotConfigError(`${path}.${key} 必须是布尔值`);
  return value;
}

function numberVec(value: unknown, path: string, size: number): Vec3 {
  const arr = asArray(value, path);
  if (arr.length !== size) {
    throw new RobotConfigError(`${path} 必须是 ${size} 个数值的数组，实际 ${arr.length} 个`);
  }
  const out = arr.map((item, index) => {
    if (typeof item !== 'number' || !Number.isFinite(item)) {
      throw new RobotConfigError(`${path}[${index}] 必须是有限数值`);
    }
    return item;
  });
  return [out[0]!, out[1]!, out[2]!];
}

function optNumberVec(value: unknown, path: string, size: number, fallback: Vec3): Vec3 {
  if (value === undefined || value === null) return [...fallback];
  return numberVec(value, path, size);
}

function optEnum<T extends string>(
  dict: Dict,
  key: string,
  path: string,
  allowed: readonly T[],
  fallback?: T,
): T {
  const value = dict[key];
  if (value === undefined || value === null) {
    if (fallback !== undefined) return fallback;
    throw new RobotConfigError(`${path}.${key} 必填，可选值: ${allowed.join(' / ')}`);
  }
  if (typeof value !== 'string' || !allowed.includes(value as T)) {
    throw new RobotConfigError(`${path}.${key} 非法: ${String(value)}，可选值: ${allowed.join(' / ')}`);
  }
  return value as T;
}

// ---------------------------------------------------------------------------
// 各段落解析
// ---------------------------------------------------------------------------

function parseLimits(value: unknown, path: string, fallback?: JointLimits): JointLimits {
  if (value === undefined || value === null) {
    if (fallback) return { ...fallback };
    throw new RobotConfigError(`${path} 必填`);
  }
  const dict = asDict(value, path);
  return { min: reqNumber(dict, 'min', path), max: reqNumber(dict, 'max', path) };
}

/** 从字典按 key 取可选的 3 元数值向量（用于 geometry 的 position / rotation / size） */
function optKeyVec3(dict: Dict, key: string, path: string): Vec3 | undefined {
  return dict[key] === undefined ? undefined : numberVec(dict[key], `${path}.${key}`, 3);
}

const GEOMETRY_TYPES = [
  'none',
  'box',
  'plate',
  'cylinder',
  'sphere',
  'servo',
  'jaw',
  'mesh',
] as const;

/** 欧拉角约定（`origin.rotation` / `geometry.rotation` 共用），缺省为 config 的 'xyz' */
const ROTATION_CONVENTIONS = ['xyz', 'rpy'] as const;

/** 执行器角度空间，缺省 `'deg'`（舵机空间 0..180°） */
const ACTUATOR_UNITS = ['deg', 'joint'] as const;

function parseGeometry(value: unknown, path: string): LinkGeometry {
  if (value === undefined || value === null) return { type: 'none' };
  const dict = asDict(value, path);
  const type = optEnum(dict, 'type', path, GEOMETRY_TYPES, 'none');
  const color = optString(dict, 'color', path);
  const position = optKeyVec3(dict, 'position', path);
  const rotation = optKeyVec3(dict, 'rotation', path);
  // 欧拉角约定：缺省 'xyz'（= 既有语义）。写了 'rpy' 才按 URDF <origin rpy> 解释。
  const rotationConvention = optEnum(dict, 'rotationConvention', path, ROTATION_CONVENTIONS, 'xyz');
  // 只有非缺省时才写进对象 —— 这样 'xyz' 分支产出的对象与引入本字段前**逐字段相同**。
  const convField: { rotationConvention?: RotationConvention } =
    rotationConvention === 'xyz' ? {} : { rotationConvention };

  switch (type) {
    case 'none':
      return { type: 'none', ...(color ? { color } : {}) };
    case 'box': {
      const size = numberVec(dict['size'], `${path}.size`, 3);
      return {
        type: 'box',
        size,
        ...(position ? { position } : {}),
        ...(rotation ? { rotation } : {}),
        ...convField,
        ...(color ? { color } : {}),
      };
    }
    case 'plate': {
      const size = numberVec(dict['size'], `${path}.size`, 3);
      // 允许 radius 作为 cornerRadius 的别名（YAML 里更短更好读）
      const rawRadius = dict['cornerRadius'] ?? dict['radius'];
      const cornerRadius =
        rawRadius === undefined
          ? DEFAULT_PLATE_CORNER_RADIUS
          : (() => {
              if (typeof rawRadius !== 'number' || !Number.isFinite(rawRadius)) {
                throw new RobotConfigError(`${path}.cornerRadius 必须是有限数值`);
              }
              return rawRadius;
            })();
      const texture = optString(dict, 'texture', path);
      const flipU = optBool(dict, 'textureFlipU', path, false);
      const flipV = optBool(dict, 'textureFlipV', path, false);
      const plate: PlateGeometry = {
        type: 'plate',
        size,
        cornerRadius,
        ...(position ? { position } : {}),
        ...(rotation ? { rotation } : {}),
        ...convField,
        ...(color ? { color } : {}),
        ...(texture ? { texture } : {}),
        ...(flipU ? { textureFlipU: true } : {}),
        ...(flipV ? { textureFlipV: true } : {}),
      };
      return plate;
    }
    case 'servo': {
      const size = optKeyVec3(dict, 'size', path) ?? [...DEFAULT_SERVO_SIZE];
      const shaftLength = optNumber(dict, 'shaftLength', path, DEFAULT_SERVO_SHAFT_LENGTH);
      const ears = optBool(dict, 'ears', path, true);
      const shaftColor = optString(dict, 'shaftColor', path);
      const servo: ServoGeometry = {
        type: 'servo',
        size,
        shaftLength,
        ears,
        ...(position ? { position } : {}),
        ...(rotation ? { rotation } : {}),
        ...convField,
        ...(color ? { color } : {}),
        ...(shaftColor ? { shaftColor } : {}),
      };
      return servo;
    }
    case 'cylinder': {
      const radius = reqNumber(dict, 'radius', path);
      const height = reqNumber(dict, 'height', path);
      const radialSegments =
        dict['radialSegments'] === undefined
          ? undefined
          : optNumber(dict, 'radialSegments', path, 24);
      return {
        type: 'cylinder',
        radius,
        height,
        ...(radialSegments !== undefined ? { radialSegments } : {}),
        ...(position ? { position } : {}),
        ...(rotation ? { rotation } : {}),
        ...convField,
        ...(color ? { color } : {}),
      };
    }
    case 'sphere': {
      const radius = reqNumber(dict, 'radius', path);
      return {
        type: 'sphere',
        radius,
        ...(position ? { position } : {}),
        ...(rotation ? { rotation } : {}),
        ...convField,
        ...(color ? { color } : {}),
      };
    }
    case 'jaw': {
      // 爪型参数（字段含义见 Link.ts 的 JawGeometry）。除 length 外都可省略，
      // 缺省在 jawRenderSpec 里统一补齐；解析层只负「写错了要立刻报错」。
      const length = reqNumber(dict, 'length', path);
      const thickness = optNumber(dict, 'thickness', path, DEFAULT_JAW_SPEC.thickness);
      const width = optNumber(dict, 'width', path, DEFAULT_JAW_SPEC.width);
      const tipWidth = optNumber(dict, 'tipWidth', path, DEFAULT_JAW_SPEC.tipWidth);
      const gearRadius = optNumber(dict, 'gearRadius', path, DEFAULT_JAW_SPEC.gearRadius);
      const gearTeeth = optNumber(dict, 'gearTeeth', path, DEFAULT_JAW_SPEC.gearTeeth);
      const toothDepth = optNumber(dict, 'toothDepth', path, DEFAULT_JAW_SPEC.toothDepth);
      const serrations = optNumber(dict, 'serrations', path, DEFAULT_JAW_SPEC.serrations);
      const serrationDepth = optNumber(
        dict,
        'serrationDepth',
        path,
        DEFAULT_JAW_SPEC.serrationDepth,
      );
      const tipSkew = optNumber(dict, 'tipSkew', path, DEFAULT_JAW_SPEC.tipSkew);
      const neckInset = optNumber(dict, 'neckInset', path, DEFAULT_JAW_SPEC.neckInset);
      const neckAt = optNumber(dict, 'neckAt', path, DEFAULT_JAW_SPEC.neckAt);
      const holeRadius = optNumber(dict, 'holeRadius', path, DEFAULT_JAW_SPEC.holeRadius);

      const spanRaw = dict['serrationSpan'];
      const serrationSpan: [number, number] =
        spanRaw === undefined
          ? [DEFAULT_JAW_SPEC.serrationSpan[0], DEFAULT_JAW_SPEC.serrationSpan[1]]
          : (() => {
              const value = numberVec(spanRaw, `${path}.serrationSpan`, 2);
              return [value[0]!, value[1]!];
            })();

      // 自洽性前置检查：三条派生关系（见 jawRenderSpec）都以「分度圆半径 = gearRadius −
      // toothDepth/2 不小于爪指半宽」为前提。不满足时爪指中线会落到齿轮外侧，
      // θ=0 两爪也贴不上 —— 这种配置必须当场拦下，不能渲染出一个畸形的爪。
      const pitchRadius = gearRadius - toothDepth / 2;
      if (pitchRadius < width / 2) {
        throw new RobotConfigError(
          `${path}: 分度圆半径(= gearRadius − toothDepth/2 = ${pitchRadius}) 必须 ≥ width/2 ` +
            `(= ${width / 2})，否则爪指中线会落到齿轮外侧且 θ=0 两爪无法贴合`,
        );
      }

      const jaw: JawGeometry = {
        type: 'jaw',
        length,
        thickness,
        width,
        tipWidth,
        gearRadius,
        gearTeeth,
        toothDepth,
        serrations,
        serrationDepth,
        serrationSpan,
        tipSkew,
        neckInset,
        neckAt,
        holeRadius,
        ...(position ? { position } : {}),
        ...(rotation ? { rotation } : {}),
        ...convField,
        ...(color ? { color } : {}),
      };
      return jaw;
    }
    case 'mesh': {
      // 外部 CAD 网格（SO-ARM101 的官方 STL 走这条路）。`file` 相对 `assets/models/`。
      // 几何来自权威模型，**不重做、不猜尺寸** —— 故除 file 外全部可选。
      const file = reqString(dict, 'file', path);
      const scale = optKeyVec3(dict, 'scale', path);
      const metalness = dict['metalness'] === undefined ? undefined : reqNumber(dict, 'metalness', path);
      const roughness = dict['roughness'] === undefined ? undefined : reqNumber(dict, 'roughness', path);
      const mesh: MeshGeometry = {
        type: 'mesh',
        file,
        ...(scale ? { scale } : {}),
        ...(position ? { position } : {}),
        ...(rotation ? { rotation } : {}),
        ...convField,
        ...(color ? { color } : {}),
        ...(metalness !== undefined ? { metalness } : {}),
        ...(roughness !== undefined ? { roughness } : {}),
      };
      return mesh;
    }
  }
}

const JOINT_ROLES: readonly JointRole[] = ['base', 'shoulder', 'elbow', 'tool', 'gripper'];
const JOINT_TYPES: readonly JointType[] = ['revolute', 'fixed', 'passive'];

function parseLink(value: unknown, index: number): Link {
  const path = `links[${index}]`;
  const dict = asDict(value, path);
  const id = reqString(dict, 'id', path);
  const detailsRaw = dict['details'];
  const details =
    detailsRaw === undefined
      ? undefined
      : asArray(detailsRaw, `${path}.details`).map((item, detailIndex) =>
          parseGeometry(item, `${path}.details[${detailIndex}]`),
        );

  const link: Link = {
    id,
    name: optString(dict, 'name', path) ?? id,
    parent: dict['parent'] === undefined ? null : (dict['parent'] as string | null),
    length: optNumber(dict, 'length', path, 0),
    geometry: parseGeometry(dict['geometry'], `${path}.geometry`),
    ...(details ? { details } : {}),
  };
  if (link.parent !== null && typeof link.parent !== 'string') {
    throw new RobotConfigError(`${path}.parent 必须是字符串或 null`);
  }
  return link;
}

function parseOrigin(value: unknown, path: string): JointOrigin {
  if (value === undefined || value === null) {
    return { position: [0, 0, 0], rotation: [0, 0, 0] };
  }
  const dict = asDict(value, path);
  const convention = optEnum(dict, 'rotationConvention', path, ROTATION_CONVENTIONS, 'xyz');
  return {
    position: optNumberVec(dict['position'], `${path}.position`, 3, [0, 0, 0]),
    rotation: optNumberVec(dict['rotation'], `${path}.rotation`, 3, [0, 0, 0]) as EulerDeg,
    // 只在非缺省时写出 —— 保证 'xyz' 的 origin 与引入本字段前**逐字段相同**。
    ...(convention === 'xyz' ? {} : { rotationConvention: convention }),
  };
}

function parseJoint(value: unknown, index: number): Joint {
  const path = `joints[${index}]`;
  const dict = asDict(value, path);
  const id = reqString(dict, 'id', path);
  const type = optEnum(dict, 'type', path, JOINT_TYPES, 'revolute');
  const role = optString(dict, 'role', path);
  if (role !== undefined && !JOINT_ROLES.includes(role as JointRole)) {
    throw new RobotConfigError(`${path}.role 非法: ${role}，可选值: ${JOINT_ROLES.join(' / ')}`);
  }

  // ⚠️ 被动关节**必须显式写出 limit**（它的值恒取 limit.min = 锁定角）。
  //    若允许缺省，`parseLimits` 的默认 0 会让「锁在 0°」静默成立 ——
  //    那是"爪指向天顶"这种一眼假的姿态，却在模型层完全合法。宁可在这里炸。
  if (type === 'passive' && dict['limit'] === undefined && dict['limits'] === undefined) {
    throw new RobotConfigError(
      `${path}: 被动关节必须显式给出 limit（min 是它的锁定角，且必须 min === max）`,
    );
  }

  return {
    id,
    name: optString(dict, 'name', path) ?? id,
    ...(role ? { role: role as JointRole } : {}),
    parentLink: reqString(dict, 'parentLink', path),
    childLink: reqString(dict, 'childLink', path),
    type,
    axis: optNumberVec(dict['axis'], `${path}.axis`, 3, [0, 0, 1]),
    origin: parseOrigin(dict['origin'], `${path}.origin`),
    limits: parseLimits(dict['limit'] ?? dict['limits'], `${path}.limit`, {
      min: 0,
      max: 0,
    }),
    ...parseCoupling(dict['coupling'], `${path}.coupling`),
  };
}

/**
 * 解析关节耦合（平行四连杆）：
 *
 *   coupling:
 *     joint: shoulder     # 与之耦合的关节 id
 *     gain: -1            # 平行四连杆恒为 −1
 */
function parseCoupling(value: unknown, path: string): { coupling?: JointCoupling } {
  if (value === undefined || value === null) return {};
  const dict = asDict(value, path);
  return {
    coupling: {
      jointId: reqString(dict, 'joint', path),
      gain: optNumber(dict, 'gain', path, -1),
    },
  };
}

function parseActuator(value: unknown, index: number): Actuator {
  const path = `actuators[${index}]`;
  const dict = asDict(value, path);
  const id = reqString(dict, 'id', path);
  const name = optString(dict, 'name', path);
  const limitsValue = asDict(dict['limits'], `${path}.limits`);
  const limits: ActuatorLimits = {
    min: reqNumber(limitsValue, 'min', `${path}.limits`),
    max: reqNumber(limitsValue, 'max', `${path}.limits`),
  };
  const unit = optEnum(dict, 'unit', path, ACTUATOR_UNITS, 'deg') as ActuatorUnit;

  return {
    id,
    ...(name ? { name } : {}),
    jointId: reqString(dict, 'jointId', path),
    channel: dict['channel'] === undefined ? -1 : reqNumber(dict, 'channel', path),
    offset: optNumber(dict, 'offset', path, 0),
    scale: optNumber(dict, 'scale', path, 1),
    reverse: optBool(dict, 'reverse', path, false),
    limits,
    // 只在非缺省时写出 —— 保证 'deg'（缺省）的执行器与引入本字段前**逐字段相同**。
    ...(unit === 'deg' ? {} : { unit }),
  };
}

function parseHomePose(value: unknown): JointState {
  if (value === undefined || value === null) return {};
  const dict = asDict(value, 'robot.homePose');
  const out: JointState = {};
  for (const [key, raw] of Object.entries(dict)) {
    if (typeof raw !== 'number' || !Number.isFinite(raw)) {
      throw new RobotConfigError(`robot.homePose.${key} 必须是有限数值`);
    }
    out[key] = raw;
  }
  return out;
}

function parseTcp(value: unknown, joints: Joint[]): TcpSpec {
  if (value === undefined || value === null) {
    // 缺省：以链上最后一个关节为参考，零偏移
    const last = joints[joints.length - 1];
    return { joint: last ? last.id : '', offset: [0, 0, 0] };
  }
  const dict = asDict(value, 'robot.tcp');
  return {
    joint: reqString(dict, 'joint', 'robot.tcp'),
    offset: optNumberVec(dict['offset'], 'robot.tcp.offset', 3, [0, 0, 0]),
  };
}

/**
 * 解析 `appearance` 段（渲染参数）。
 *
 * 整段可选：缺省时返回 `DEFAULT_APPEARANCE`（= 与引入本特性前逐值一致的行为）。
 * 只有 `exposureEv` 做范围校验 —— 它是唯一一个填错会明显破坏画面的量
 * （超出 ±EV 区间只会把整机推爆，属于误填而非风格选择）。
 */
function parseAppearance(value: unknown): Appearance {
  if (value === undefined || value === null) return structuredClone(DEFAULT_APPEARANCE);

  const path = 'appearance';
  const root = asDict(value, path);
  const platePath = `${path}.texturedPlate`;
  const plateRaw = root['texturedPlate'];
  if (plateRaw === undefined || plateRaw === null) {
    return { texturedPlate: { ...DEFAULT_APPEARANCE.texturedPlate } };
  }

  const plate = asDict(plateRaw, platePath);
  const exposureEv = optNumber(
    plate,
    'exposureEv',
    platePath,
    DEFAULT_APPEARANCE.texturedPlate.exposureEv,
  );
  if (exposureEv < EXPOSURE_EV_RANGE.min || exposureEv > EXPOSURE_EV_RANGE.max) {
    throw new RobotConfigError(
      `${platePath}.exposureEv=${exposureEv} 超出合法区间 ` +
        `[${EXPOSURE_EV_RANGE.min}, ${EXPOSURE_EV_RANGE.max}] EV`,
    );
  }

  return {
    texturedPlate: {
      environmentIntensity: optNumber(
        plate,
        'environmentIntensity',
        platePath,
        DEFAULT_APPEARANCE.texturedPlate.environmentIntensity,
      ),
      exposureEv,
      roughness: optNumber(plate, 'roughness', platePath, DEFAULT_APPEARANCE.texturedPlate.roughness),
      metalness: optNumber(plate, 'metalness', platePath, DEFAULT_APPEARANCE.texturedPlate.metalness),
    },
  };
}

// ---------------------------------------------------------------------------
// 公开 API
// ---------------------------------------------------------------------------

export interface ParseRobotModelOptions {
  /** 是否做模型级校验（默认 true） */
  validate?: boolean;
  /** 收集 warning（error 仍会抛异常） */
  onIssue?: (issue: ModelIssue) => void;
}

/** 解析 YAML 文本为 RobotModel */
export function parseRobotModelYaml(text: string, options: ParseRobotModelOptions = {}): RobotModel {
  let raw: unknown;
  try {
    raw = parseYaml(text);
  } catch (error) {
    throw new RobotConfigError(`YAML 语法错误: ${(error as Error).message}`);
  }
  return parseRobotModel(raw, options);
}

/** 解析已反序列化的配置对象为 RobotModel */
export function parseRobotModel(raw: unknown, options: ParseRobotModelOptions = {}): RobotModel {
  const root = asDict(raw, 'root');
  const version = optNumber(root, 'version', 'root', 1);
  const robotDict = asDict(root['robot'], 'robot');
  const units = optEnum(robotDict, 'units', 'robot', ['mm'] as const, 'mm');

  const links = asArray(root['links'] ?? [], 'links').map(parseLink);
  const joints = asArray(root['joints'] ?? [], 'joints').map(parseJoint);
  const actuators = asArray(root['actuators'] ?? [], 'actuators').map(parseActuator);

  if (links.length === 0) throw new RobotConfigError('links 不能为空');
  if (joints.length === 0) throw new RobotConfigError('joints 不能为空');

  const model: RobotModel = {
    version,
    id: reqString(robotDict, 'id', 'robot'),
    name: optString(robotDict, 'name', 'robot') ?? reqString(robotDict, 'id', 'robot'),
    // 模型标识（只读元数据，不参与任何运动学计算）—— 缺省不影响行为。
    ...(optString(robotDict, 'model', 'robot')
      ? { model: optString(robotDict, 'model', 'robot')! }
      : {}),
    ...(optString(robotDict, 'version', 'robot')
      ? { modelVersion: optString(robotDict, 'version', 'robot')! }
      : {}),
    units,
    links,
    joints,
    actuators,
    homePose: parseHomePose(robotDict['homePose']),
    tcp: parseTcp(robotDict['tcp'], joints),
    appearance: parseAppearance(root['appearance']),
  };

  if (options.validate !== false) {
    const issues = validateRobotModel(model);
    if (options.onIssue) issues.forEach(options.onIssue);
    if (issues.some((issue) => issue.level === 'error')) throw new RobotModelError(issues);
    assertValidRobotModel(model);
  }

  return model;
}

/**
 * 按机器人 id 缓存已解析的模型。
 *
 * 每个 id 各一份 ⇒ 两台机器人可以**同时存在**（切换模型时旧的那份不会被顶掉，
 * 于是"A 的 FK 结果"与"B 的 FK 结果"可以在同一个进程里并存比对，
 * 这对 Phase 8 的切换压力回归是必需的）。
 */
const cache = new Map<string, RobotModel>();

/**
 * 加载机器人模型（结果按 id 缓存）。
 *
 * @param robotId 机器人 id（见 `config/robots.yaml`）。**省略时读选择器的 `default`**。
 *
 * ```ts
 * loadRobotModel('mearm-v1');   // Golden Baseline
 * loadRobotModel('so-arm101');  // 官方 SO-ARM101 派生模型
 * loadRobotModel();             // = loadRobotModel(defaultRobotId())
 * ```
 *
 * ⚠️ 未知 id 抛 `RobotConfigError`（**不回退**，理由见文件头）。
 */
export function loadRobotModel(robotId?: string): RobotModel {
  const id = robotId ?? defaultRobotId();
  const cached = cache.get(id);
  if (cached) return cached;

  const model = parseRobotModelYaml(robotYamlTextById(id));
  cache.set(id, model);
  return model;
}

/** 清空缓存（仅测试/热重载使用）。选择器缓存也一并清掉，避免两个缓存不一致 */
export function resetRobotModelCache(): void {
  cache.clear();
  resetRobotSelectorCache();
}
