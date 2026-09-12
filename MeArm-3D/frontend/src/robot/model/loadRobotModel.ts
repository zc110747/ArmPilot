/**
 * `config/robot.yaml` → `RobotModel` 加载器。
 *
 * 机器人参数必须独立于代码：本文件是「配置 → 模型」的唯一入口，
 * 业务代码只消费 `RobotModel`，绝不直接读 yaml，也绝不硬编码尺寸/角度。
 *
 * 校验分两级：
 *   - 解析级：字段类型 / 必填项错误 → 抛 `RobotConfigError`
 *   - 模型级：拓扑 / 限位 / 标定自洽性 → 抛 `RobotModelError`（携带完整 issue 列表）
 */
import { parse as parseYaml } from 'yaml';
import robotYamlText from '@config/robot.yaml?raw';
import type { Actuator, ActuatorLimits } from './Actuator';
import type { Joint, JointCoupling, JointLimits, JointOrigin, JointRole, JointType } from './Joint';
import type { Link, LinkGeometry, PlateGeometry, ServoGeometry } from './Link';
import {
  DEFAULT_PLATE_CORNER_RADIUS,
  DEFAULT_SERVO_SHAFT_LENGTH,
  DEFAULT_SERVO_SIZE,
} from './Link';
import type { EulerDeg, JointState, Vec3 } from './Pose';
import type { ModelIssue, RobotModel, TcpSpec } from './RobotModel';
import { RobotModelError, assertValidRobotModel, validateRobotModel } from './RobotModel';

/** 内置的 robot.yaml 原文（编译期内联，运行时无需读磁盘） */
export const BUNDLED_ROBOT_YAML: string = robotYamlText;

export class RobotConfigError extends Error {
  constructor(message: string) {
    super(`[robot.yaml] ${message}`);
    this.name = 'RobotConfigError';
  }
}

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

const GEOMETRY_TYPES = ['none', 'box', 'plate', 'cylinder', 'sphere', 'servo'] as const;

function parseGeometry(value: unknown, path: string): LinkGeometry {
  if (value === undefined || value === null) return { type: 'none' };
  const dict = asDict(value, path);
  const type = optEnum(dict, 'type', path, GEOMETRY_TYPES, 'none');
  const color = optString(dict, 'color', path);
  const position = optKeyVec3(dict, 'position', path);
  const rotation = optKeyVec3(dict, 'rotation', path);

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
      const plate: PlateGeometry = {
        type: 'plate',
        size,
        cornerRadius,
        ...(position ? { position } : {}),
        ...(rotation ? { rotation } : {}),
        ...(color ? { color } : {}),
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
        ...(color ? { color } : {}),
      };
    }
  }
}

const JOINT_ROLES: readonly JointRole[] = ['base', 'shoulder', 'elbow', 'tool', 'gripper'];
const JOINT_TYPES: readonly JointType[] = ['revolute', 'fixed'];

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
  return {
    position: optNumberVec(dict['position'], `${path}.position`, 3, [0, 0, 0]),
    rotation: optNumberVec(dict['rotation'], `${path}.rotation`, 3, [0, 0, 0]) as EulerDeg,
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
      max: type === 'revolute' ? 0 : 0,
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

  return {
    id,
    ...(name ? { name } : {}),
    jointId: reqString(dict, 'jointId', path),
    channel: dict['channel'] === undefined ? -1 : reqNumber(dict, 'channel', path),
    offset: optNumber(dict, 'offset', path, 0),
    scale: optNumber(dict, 'scale', path, 1),
    reverse: optBool(dict, 'reverse', path, false),
    limits,
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
    units,
    links,
    joints,
    actuators,
    homePose: parseHomePose(robotDict['homePose']),
    tcp: parseTcp(robotDict['tcp'], joints),
  };

  if (options.validate !== false) {
    const issues = validateRobotModel(model);
    if (options.onIssue) issues.forEach(options.onIssue);
    if (issues.some((issue) => issue.level === 'error')) throw new RobotModelError(issues);
    assertValidRobotModel(model);
  }

  return model;
}

let cached: RobotModel | undefined;

/** 加载随包内置的 `config/robot.yaml`（结果缓存） */
export function loadRobotModel(): RobotModel {
  if (!cached) cached = parseRobotModelYaml(BUNDLED_ROBOT_YAML);
  return cached;
}

/** 清空缓存（仅测试/热重载使用） */
export function resetRobotModelCache(): void {
  cached = undefined;
}
