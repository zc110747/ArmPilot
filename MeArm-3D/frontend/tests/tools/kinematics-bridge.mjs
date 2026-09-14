/**
 * 运动学桥：把**前端真实的** `ik.ts` / `fk.ts` 暴露成一个进程级 CLI，
 * 供前端之外的验收程序（`tests/sim/test_ik.py`）调用。
 *
 * ## 为什么需要它
 *
 * Phase 9 要回答的问题是「项目自己的 IK 解出来的关节角，在 MuJoCo 里跑出来
 * 到底落在哪」。如果验收程序用 Python **重写**一份 IK，那只能证明"我又写了一遍
 * 而且它自洽"——证明不了 `ik.ts` 是对的。所以必须加载**同一份源码**。
 *
 * 但 `ik.ts` 不能直接被 Node 加载，有三个硬障碍：
 *   1. `robotConfigRegistry.ts` / `robotPackage.ts` 用 **`import.meta.glob`** 在
 *      构建期登记包内真值与 manifest —— 那是 **Vite 专属**语法，Node 的解析器不认；
 *      （Phase 2 之前这里还有一条 `import ... from '@config/robot.yaml?raw'`，
 *       真值随包搬走后已改为"向 Registry 要原文"，见 `loadRobotModel.ts`。）
 *   2. 所有内部导入都是**无扩展名**的（`from './fk'`），Node ESM 无法解析；
 *   3. `?raw` 这类查询后缀同样是 Vite 专属。
 * 结论：只能用 Vite 自己的 SSR 加载器（`server.ssrLoadModule`）来加载 ——
 * 它复用 `frontend/vite.config.ts`，别名、`?raw`、TS 转译全部照旧，
 * 与浏览器里跑的**是同一套解析规则**。
 *
 * ## 为什么用 `--in` / `--out` 文件而不是 stdin/stdout
 *
 * Vite 与插件会往 stdout/stderr 打日志。把 JSON 写在 stdout 上，
 * 任何一行日志都会让对端解析失败 —— 而且失败信息通常很难定位。
 * 用文件传递结果、stdout 留给日志，这条路就没有污染面。
 *
 * ## 用法
 *
 * ```bash
 * cd frontend
 * node tests/tools/kinematics-bridge.mjs --in req.json --out res.json
 * node tests/tools/kinematics-bridge.mjs --info --out model.json     # 只导出模型元信息
 * node tests/tools/kinematics-bridge.mjs --robot mearm-v1 --info --out model.json
 * ```
 *
 * `--robot` 指定机器人 id（= `config/robots.yaml` 的 key）；省略时用选择器的 `default`。
 * 桥**不猜**任何能力：`model.capability.solverKind === 'none'` 的机器人，
 * 它的 IK 请求一律返回 `NOT_IMPLEMENTED`，绝不退化用另一台机器人的求解器。
 *
 * 请求格式：
 * ```json
 * {
 *   "cases": [ { "id": 0, "target": [120, 0, 90], "near": {...}, "prefer": "nearest" } ],
 *   "fk":    [ { "id": 0, "joints": { "base": 0, "shoulder": 30, ... } } ]
 * }
 * ```
 * `cases` = IK 批量求解（缺省空）；`fk` = FK 批量求值（缺省空）。
 * 两者互不影响，可同时给出 —— 响应里分别是 `results` 与 `fkResults`。
 *
 * 响应格式：见文件末尾 `run()` 的组装处。
 */
import { createServer } from 'vite';
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { parse as parseYaml } from 'yaml';

const HERE = path.dirname(fileURLToPath(import.meta.url));
/** `frontend/` 根 —— vite 的 root 与 configFile 都以它为基准 */
const FRONTEND_DIR = path.resolve(HERE, '..', '..');

/**
 * 仓库根（`ArmPilot/MeArm-3D`）—— 向上找"同时含 `core/` 与 `robot-package/`"的那层。
 *
 * 刻意不用 `path.resolve(HERE, '../../..')`：深度常量在本项目是被实测过会静默错位的写法
 * （见 `.workbuddy/memory/playbook.md` §8）。标记法在任意深度都成立。
 */
function findRepoRoot(start) {
  for (let d = start; ; d = path.dirname(d)) {
    if (existsSync(path.join(d, 'core')) && existsSync(path.join(d, 'robot-package'))) return d;
    if (path.dirname(d) === d) throw new Error(`无法从 ${start} 向上找到仓库根`);
  }
}
const REPO_DIR = findRepoRoot(FRONTEND_DIR);

/**
 * 读某台机器人的包 manifest（= 桥**唯一**的路径来源）。
 *
 * ⚠️ 这就是本轮改造的要点：桥原先直接写死 `/src/robot/kinematics/ik.ts`。
 * 那个文件 Phase 2 步④ 起住在**包内**，而"写死的读者"不会报错 ——
 * 它会以一个"模块找不到"的形式失败，看着像环境问题，实际是路径过期。
 * 现在桥问 manifest：**包在哪、IK 实现在哪**，由包自己声明。
 */
function loadManifest(robotId) {
  const rel = `robot-package/${robotId}/manifest.yaml`;
  const abs = path.join(REPO_DIR, rel);
  if (!existsSync(abs)) {
    throw new Error(`没有名为 ${robotId} 的 Robot Package（缺 ${rel}）`);
  }
  return parseYaml(readFileSync(abs, 'utf8'));
}

/** 把一个仓库相对路径变成 Vite 认得的 `/@fs/` URL（文件在 vite root 之外时的标准写法） */
function fsUrl(repoRel) {
  const abs = path.resolve(REPO_DIR, repoRel).split(path.sep).join('/');
  return `/@fs/${abs}`;
}

function parseArgs(argv) {
  const out = { in: null, out: null, info: false, robot: null };
  for (let i = 0; i < argv.length; i += 1) {
    const a = argv[i];
    if (a === '--in') out.in = argv[++i];
    else if (a === '--out') out.out = argv[++i];
    else if (a === '--info') out.info = true;
    else if (a === '--robot') out.robot = argv[++i];
    else if (a === '--help' || a === '-h') out.help = true;
    else throw new Error(`未知参数 ${a}`);
  }
  return out;
}

/** 把 JointState 里的非法数值挡在 JSON 之外（JSON 没有 NaN，会被写成 null） */
function sanitizeJoints(joints) {
  const out = {};
  for (const [k, v] of Object.entries(joints)) {
    if (typeof v !== 'number' || !Number.isFinite(v)) {
      throw new Error(`关节 ${k} 的值非有限数：${v}`);
    }
    out[k] = v;
  }
  return out;
}

/**
 * @param robotIdOrNull `--robot` 的值；`null` ⇒ 用选择器的 `default`
 *   （与 `loadRobot()` 的缺省语义一致 —— 桥不自己发明一个缺省机器人）。
 */
async function loadModules(robotIdOrNull) {
  // middlewareMode：不起监听端口，只借用 Vite 的模块图与转译管线。
  // logLevel 收到 'error'：正常路径上不允许有任何输出。
  const server = await createServer({
    configFile: path.join(FRONTEND_DIR, 'vite.config.ts'),
    root: FRONTEND_DIR,
    server: { middlewareMode: true },
    appType: 'custom',
    logLevel: 'error',
  });

  const load = async (rel) => {
    const mod = await server.ssrLoadModule(rel);
    return mod;
  };

  const fk = await load('/src/robot/kinematics/fk.ts');
  const robotModel = await load('/src/robot/model/RobotModel.ts');
  // 选择器 + 注册表：让桥能加载**任意**机器人，而不是只会 loadRobotModel() 的那一台。
  // 走注册表（而不是直接 loadRobotModel(id)）是为了让 `capability` 也一并拿到 ——
  // 「这台机器人有没有 IK」必须由数据声明，桥不猜。
  const registry = await load('/src/robot/registry/RobotRegistry.ts');
  const selectorMod = await load('/src/robot/model/robotConfigRegistry.ts');

  // id 必须在加载"包内 IK"之前定下来；缺省仍由选择器说了算。
  const robotId = robotIdOrNull ?? selectorMod.defaultRobotId();

  // IK 实现的位置**问包自己的 manifest**（`kinematics.ik.entry`）。
  // `type: none` 的包没有 entry ⇒ `ik = null`，而这不是错误：
  // 「没有逆解」是这台机器人**声明过**的能力事实，桥照实报告。
  const manifest = loadManifest(robotId);
  const ikEntry = manifest?.kinematics?.ik?.entry;
  const ik = typeof ikEntry === 'string' && ikEntry.length > 0 ? await load(fsUrl(ikEntry)) : null;

  return { server, ik, fk, robotModel, registry, manifest, robotId };
}

function describeModel(ik, model, entry) {
  // 有独立自由度的关节 —— 与前端 `movableJoints()` / Go `JointOrder()` /
  // Python `RobotCfg.movable_joints()` 是同一条规则（**只看 `type === 'revolute'`**）。
  // 刻意在桥里就地写出来而不是 import `RobotModel.ts`：桥是"一次性 CLI"，
  // 少一个模块就少一处 Node 解析失败的面；而这条规则本身只有一行，不会漂移。
  const movable = model.joints.filter((j) => j.type === 'revolute');
  const capability = entry.kinematics.capability;
  const hasIk = capability.solverKind !== 'none';
  return {
    id: entry.id,
    /** 模型自己的名字（`robot.yaml → robot.id`）—— 与注册表 id **不强制同名** */
    modelId: model.id,
    name: model.name,
    units: model.units,
    /** 模型标识（只读元数据）—— 让"基线属于哪个模型"可被机器检查 */
    model: model.model ?? null,
    modelVersion: model.modelVersion ?? null,
    /** 能力声明（数据，不是逻辑）：上层据此决定"要不要调用 IK" */
    capability: {
      positioningDof: capability.positioningDof,
      supportsOrientation: capability.supportsOrientation,
      solverKind: capability.solverKind,
    },
    homePose: { ...model.homePose },
    tcp: { joint: model.tcp.joint, offset: [...model.tcp.offset] },
    /** 按 `movableJoints` 的顺序（与 MuJoCo 的关节顺序无关，仅作对照） */
    joints: movable.map((j) => ({
      id: j.id,
      name: j.name,
      role: j.role ?? null,
      type: j.type,
      axis: [...j.axis],
      origin: { position: [...j.origin.position], rotation: [...j.origin.rotation] },
      limit: { min: j.limits.min, max: j.limits.max },
      coupling: j.coupling ? { jointId: j.coupling.jointId, gain: j.coupling.gain } : null,
      parentLink: j.parentLink,
      childLink: j.childLink,
    })),
    actuators: model.actuators.map((a) => ({
      id: a.id,
      jointId: a.jointId,
      channel: a.channel,
      offset: a.offset,
      scale: a.scale,
      reverse: a.reverse,
      limits: { min: a.limits.min, max: a.limits.max },
    })),
    /** 解析式 2R 从模型求导出来的几何量（IK 内部用的就是这几个数）。
     *  ⚠️ 只在**确实有解析 IK** 的机器人上才有值 —— 对 `solverKind:'none'` 的机器人，
     *  这里必须是 `null`，绝不为了"字段看起来完整"而编一组数出来。 */
    geometry: hasIk ? describeGeometry(requireIkModule(ik, entry), model) : null,
  };
}

/**
 * 取包的 IK 模块；**声明了有 IK 却加载不到**必须响亮失败。
 *
 * 这一条是本轮改造的护栏：桥现在按 manifest 的 `kinematics.ik.entry` 加载 IK。
 * 若有人只改了 `capability.solverKind` 却忘了在 manifest 里写 `entry`（或反过来），
 * 就会走到这里 —— 报错要指出**是声明与实现不一致**，而不是含糊的
 * "Cannot read properties of null"。
 */
function requireIkModule(ik, entry) {
  if (ik === null) {
    throw new Error(
      `${entry.id} 的引擎声明 solverKind='${entry.kinematics.capability.solverKind}'（= 有逆解），` +
        '但它的 manifest `kinematics.ik.entry` 是空的 ⇒ 声明与实现不一致。\n' +
        '  修法：要么在 manifest 里补 `kinematics.ik.entry`，要么把它改成 `type: none`。',
    );
  }
  return ik;
}

function describeGeometry(ik, model) {
  const g = ik.ikGeometry(model);
  return {
    baseId: g.baseId,
    shoulderId: g.shoulderId,
    elbowId: g.elbowId,
    /** TCP 参考关节（本机 = 被动腕 `tool`）；其坐标系原点 = 2R 子链末端的「腕枢轴」 */
    wristId: g.wristId,
    pivotZ: g.pivotZ,
    pivotR: g.pivotR,
    /** 肩枢轴 → 肘枢轴（mm） */
    l1: g.l1,
    /** 肘枢轴 → **腕枢轴**（mm）。⚠️ 不含腕→TCP 那一段，故不是「肘 → TCP」 */
    l2: g.l2,
    /** 腕枢轴 → TCP 的常量矢状面偏移 [径向, 竖直]（mm）。本机 = [40, 0] */
    toolOffset: [...g.toolOffset],
    /** 2R 子链的可达距离壳（mm）——约束的是**减去 toolOffset 后**的腕目标点 */
    reach: [...g.reach],
  };
}

async function run(opts) {
  const { server, ik, fk, registry, robotId } = await loadModules(opts.robot ?? null);
  try {
    // ⚠️ 一律走注册表（`loadRobot(id?)`）：省略 id 时它读选择器的 default，
    //    与 `loadRobotModel()` 的缺省行为一致 —— 但拿到的 definition/engine
    //    是**同一份对象**，于是"引擎算的"与"桥报告的"不可能错配。
    const entry = registry.loadRobot(robotId);
    const model = entry.definition.robotModel;
    const capability = entry.kinematics.capability;
    const hasIk = capability.solverKind !== 'none';

    const response = {
      ok: true,
      model: describeModel(ik, model, entry),
      results: [],
      fkResults: [],
    };

    if (opts.info) return response;

    const req = JSON.parse(readFileSync(opts.in, 'utf8'));

    // ---- FK 批量求值（MeArm-V1 黄金基线用）----------------------------------
    // 走的是**同一份** fk.ts 的 `forwardKinematics()` —— 与 Three.js 共用
    // `effectiveJointAngle()` 耦合语义，所以它同时是 "Joint → FK" 与
    // "Joint → Three.js" 两条判据的参考面。
    const fkCases = Array.isArray(req.fk) ? req.fk : [];
    for (const c of fkCases) {
      const id = c.id ?? response.fkResults.length;
      const joints = c.joints;
      if (!joints || typeof joints !== 'object') {
        response.fkResults.push({ id, success: false, message: 'joints 必须是对象' });
        continue;
      }
      try {
        const clean = sanitizeJoints(joints);
        const pose = fk.forwardKinematics(model, clean);
        const frames = {};
        for (const [jid, tf] of Object.entries(pose.joints)) {
          frames[jid] = { position: [...tf.position], rotation: [...tf.rotation] };
        }
        response.fkResults.push({
          id,
          success: true,
          tcp: [...pose.endEffector.position],
          tcpRotation: [...pose.endEffector.rotation],
          frames,
        });
      } catch (error) {
        response.fkResults.push({
          id,
          success: false,
          message: String(error && error.message ? error.message : error),
        });
      }
    }

    const cases = Array.isArray(req.cases) ? req.cases : [];
    const analytic = capability.solverKind === 'analytic';

    for (const c of cases) {
      const id = c.id ?? response.results.length;
      const target = c.target;
      if (!Array.isArray(target) || target.length !== 3) {
        response.results.push({
          id,
          success: false,
          reason: 'BAD_REQUEST',
          message: `target 必须是 3 个数值，收到 ${JSON.stringify(target)}`,
        });
        continue;
      }

      // ---- 有解析解的机器人（MeArm）：沿用原生 solveIk，输出最完整 ----
      if (!analytic) {
        // ⚠️ 这台机器人的 capability 声明"solverKind = none" ⇒ **不调用**任何求解器。
        //    绝不因为"看起来能忍"就退化成"用 A 的 IK 解 B 的机构"：
        //    解出来的角必然是错的，而 positionError 还会因为用自己的 FK 自证
        //    而显示成一个"很小的残差"。能诚实说"没实现"比伪造一个"看起来能用"的
        //    求解器重要（这是 spec「不伪造」那一条的直接后果）。
        response.results.push({
          id,
          success: false,
          reason: capability.solverKind === 'none' ? 'NOT_IMPLEMENTED' : 'UNSUPPORTED',
          joint: null,
          message:
            `${entry.id} 的运动学引擎声明 solverKind='${capability.solverKind}'，` +
            `未提供逆解 —— 桥不会替它编一个。`,
          candidates: [],
        });
        continue;
      }

      const options = {};
      if (c.near) options.near = c.near;
      if (c.seed) options.seed = c.seed;
      if (c.prefer) options.prefer = c.prefer;

      let result;
      try {
        result = requireIkModule(ik, entry).solveIk(model, target, options);
      } catch (error) {
        // `IkModelError` 属实现缺陷/配置错误，不是"目标不可达"，必须单独分类
        response.results.push({
          id,
          success: false,
          reason: 'MODEL_ERROR',
          message: String(error && error.message ? error.message : error),
        });
        continue;
      }

      if (!result.success) {
        response.results.push({
          id,
          success: false,
          reason: result.reason,
          joint: result.joint ?? null,
          message: result.message,
          candidates: result.candidates.map((cand) => ({
            branch: cand.branch,
            feasible: cand.feasible,
            violation: cand.violation,
            violatedJoint: cand.violatedJoint ?? null,
            joints: cand.joints,
          })),
        });
        continue;
      }

      // 前端自己的 FK 跑一遍解 —— 于是每个点都有**三方对照**：
      //   目标（MuJoCo 生成）· 前端 FK · MuJoCo FK
      const frontendTcp = fk.endEffectorPosition(model, result.joints);
      response.results.push({
        id,
        success: true,
        joints: sanitizeJoints(result.joints),
        branch: result.branch,
        /** 前端**自己算的**残差（自证，仅作参考） */
        residual: result.residual,
        azimuth: result.azimuth,
        relativeAngle: result.relativeAngle,
        /** 前端 FK 给出的 TCP —— 由验收程序拿去和 MuJoCo 对撞 */
        tcpFrontend: [...frontendTcp],
      });
    }

    return response;
  } finally {
    await server.close();
  }
}

async function main() {
  const opts = parseArgs(process.argv.slice(2));
  if (opts.help) {
    console.log(
      '用法: node tests/tools/kinematics-bridge.mjs [--robot <id>] [--info] [--in req.json] --out res.json',
    );
    return 0;
  }
  if (!opts.out) throw new Error('必须指定 --out（结果写文件，避免与 Vite 日志抢 stdout）');

  const response = await run(opts);
  writeFileSync(opts.out, JSON.stringify(response, null, 2), 'utf8');
  console.log(
    `[kinematics-bridge] ${opts.robot ?? '(default)'} · ` +
      `${opts.info ? '模型元信息' : `IK ${response.results.length} 个用例 · FK ${response.fkResults.length} 个用例`}` +
      ` → ${opts.out}`,
  );
  return 0;
}

main().then(
  (code) => process.exit(code ?? 0),
  (error) => {
    console.error('[kinematics-bridge] 失败:', error && error.stack ? error.stack : error);
    process.exit(1);
  },
);
