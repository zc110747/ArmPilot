/**
 * MeArm-V1 黄金基线（Golden Test Dataset）的读取助手。
 *
 * 数据由 `tools/gen_mearm_v1_baseline.py` 采集，落在
 * `robot-package/mearm-v1/tests/cases/*.json`（Phase 2 起随包），
 * **前后端 + Python 三方共用同一份文件** —— 目录由该包的 manifest 声明，见
 * `helpers/robotPackage.ts`（TS 侧唯一解析处）/ `robopkg.declared_path`（Python 侧）。
 *
 * ⚠️ 这里**只读**，不生成、不修补。生成入口只有一个：
 * `tools/gen_mearm_v1_baseline.py`。手改这些 JSON 等于伪造基线。
 */
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { declaredPath } from './robotPackage';

/** 本门面**只**服务 MeArm-V1：显式写 id，不读选择器 `default`。 */
const MEARM_V1 = 'mearm-v1';

/** 用例目录 —— 取自 `robot-package/mearm-v1/manifest.yaml` 的 `tests.cases` */
export const BASELINE_DIR = declaredPath(MEARM_V1, 'tests.cases');

/** 所有基线文件共有的头信息 */
export interface BaselineMeta {
  model: string;
  modelVersion: string;
  robotId: string;
  robotName: string;
  seed: number;
  generator: string;
  note: string;
}

export interface JointCase {
  id: string;
  label: string;
  joints: Record<string, number>;
}

export interface FkCase extends JointCase {
  tcpFrontend: number[];
  tcpFrontendRotation: number[];
  framesFrontend: Record<string, number[]>;
  tcpRef: number[];
  tcpMujoco: number[];
  framesRef: Record<string, number[]>;
}

export interface IkExpectation {
  success: boolean;
  branch?: string;
  joints?: Record<string, number>;
  residual?: number;
  azimuth?: number;
  relativeAngle?: number;
  tcpFrontend?: number[];
  tcpMujoco?: number[];
  reason?: string;
  joint?: string | null;
  message?: string;
  candidateCount?: number;
}

export interface IkCase {
  id: string;
  label: string;
  kind: string;
  target: number[];
  expect: IkExpectation;
}

export interface WorkspaceCase {
  id: string;
  label: string;
  kind: string;
  target: number[];
  sagittalDistance: number;
  reach: number[];
  geometry: {
    pivotZ: number;
    pivotR: number;
    l1: number;
    l2: number;
    toolOffset: number[];
  };
  expect: { success: boolean; reason: string | null };
}

function read<T>(file: string): T {
  const full = path.join(BASELINE_DIR, file);
  return JSON.parse(readFileSync(full, 'utf8')) as T;
}

export function loadJointCases() {
  return read<BaselineMeta & { count: number; cases: JointCase[] }>('joint_cases.json');
}
export function loadFkCases() {
  return read<BaselineMeta & { tolerances: Record<string, number>; cases: FkCase[] }>(
    'fk_cases.json',
  );
}
export function loadIkCases() {
  return read<BaselineMeta & { tolerances: Record<string, number>; cases: IkCase[] }>(
    'ik_cases.json',
  );
}
export function loadWorkspaceCases() {
  return read<BaselineMeta & { cases: WorkspaceCase[] }>('workspace_cases.json');
}

/**
 * 逐位一致断言（`Object.is` 语义）。
 *
 * 为什么不用 `toBeCloseTo`：本阶段唯一的验收原则是「抽象前后**逐位**一致」
 * （spec §19）。用一个"接近"的判据去验一件声称"完全相同"的事，
 * 恰好会把要找的那类缺陷放过去（差 1e-10 的耦合项写错就是这么漏掉的）。
 */
export function expectBitIdentical(actual: readonly number[], expected: readonly number[]): void {
  if (actual.length !== expected.length) {
    throw new Error(`长度不同：${actual.length} vs ${expected.length}`);
  }
  for (let i = 0; i < actual.length; i += 1) {
    if (!Object.is(actual[i], expected[i])) {
      throw new Error(`第 ${i} 位不同：${actual[i]} vs ${expected[i]}`);
    }
  }
}
