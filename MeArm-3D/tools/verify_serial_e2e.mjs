#!/usr/bin/env node
/**
 * verify_serial_e2e.mjs —— Phase 9 真机端到端验收：关节级命令 → **物理位置**
 *
 * 为什么必须请出相机
 * ------------------
 * 真机固件**没有位置反馈**（无编码器）：`arm_get_angle()` 返回的是固件内部记着的
 * **目标值**。于是 `OK SET` / `STATUS` / 后端的 `joint_state` 全都在说"我打算让它去哪"，
 * 没有任何一条能证明"它实际上在哪"。**串口回执原理上无法验证物理到位**——
 * 哪怕机械臂卡死在桌上，回执依然一字不差。
 *
 * 唯一的外部地面真值就是**相机**。本脚本把整条链路串成一个闭环：
 *
 *     verify_serial_e2e.mjs
 *          │  WebSocket（JSON，关节级）
 *          ▼
 *     armpilot-backend（serial 模式）── JR ──▶ AVR 固件 ──▶ 舵机 ──┐
 *          ▲                                                        │ 物理运动
 *          │  joint_state（开环目标值，只作链路自洽性参考）             ▼
 *          └──────────────────── ffmpeg 抓帧 ◀────────────────── 相机
 *                                     │
 *                                     ▼
 *                        tools/verify_pose.py（骨架拟合反解肩/肘绝对角）
 *                                     │
 *                                     ▼
 *                     与「本步意图关节角」比对 → PASS / FAIL
 *
 * 判定要点（详见 verify_pose.py 头注释的误差预算）
 * ----------------------------------------------
 *   1. 期望值取**量化后**的关节角：固件只吃整数舵机度，命令的物理落点是
 *      `servoToJoint(round(jointToServo(θ)))`，意图值与它差 0.347°(肩)/0.209°(肘)。
 *      拿意图值当期望 = 白送一份假误差。量化由 `verify_pose.py --quantize` 给出。
 *   2. 相机反解本身有**随轮廓厚度增长的系统偏置**（见 `--selftest` 自检 B），
 *      所以 **帧间差**比绝对误差更锐利：公共偏置在取差时被抵消。
 *   3. base 必须留在 0°：相机只能测矢状面（shoulder/elbow），base 离面即判 SKIP。
 *   4. 分割阈值默认 `auto`（全批 Otsu 中位数）、底座掩膜默认 `self`（本批自剔）。
 *      这两条都不是锦上添花 —— 写死阈值 / 用别批的掩膜，实测能让绝对角偏 5~6°。
 *
 * 2026-09-12 首次真机实测结论（Phase 9 验收）
 * ------------------------------------------
 *   ✅ 链路层 19/20 全绿，可复现：hello=serial、homePose 逐位一致、
 *      7 步回推 max|Δ| ≤ 0.004°、收尾复位到位、开机就绪门 2.7s。
 *   ✅ 相机**重复性**（同位姿两帧反解差）：肩 0.26~0.31°、肘 0.01~0.53°。
 *   ⚠️ 相机**跨批绝对角**：同台面同取景、相隔 2 分钟的两批，锚点偏置
 *      批一是 +2.69°、批二是 +7.75° —— **漂了 5°**，而两批 Otsu 都是 164。
 *      ⇒ 曝光/白平衡的自动漂移是绝对角的主导误差源，**锁死曝光是硬要求**。
 *   ⚠️ 单帧反解随机不确定度 ±2°（同一肩角在两个不同肘角下的反解差 2.0°，
 *      说明 2 参数联合拟合里肩/肘会互相补偿）。
 *   ⇒ 结论：相机当前**能**判「方向对不对、幅度大约对不对」，
 *      **不能**判「0.1° 级命令到位」。要提升必须先按 docs/hardware-measurement.md
 *      的 Phase 4.5 重布台面（白分割板 + 画面内标尺 + 正交侧视 + **锁死曝光**）。
 *
 * 用法
 * ----
 *     node tools/verify_serial_e2e.mjs                 # 完整跑（会真的驱动机械臂）
 *     node tools/verify_serial_e2e.mjs --dry-run       # 只打印动作计划，不碰硬件
 *     node tools/verify_serial_e2e.mjs --no-camera     # 只验链路（不抓帧、不反解）
 *     node tools/verify_serial_e2e.mjs --reuse-backend # 复用已在 8090 的实例
 *     node tools/verify_serial_e2e.mjs --only 01,05    # 只跑指定步骤
 *
 * 前置
 * ----
 *   * `backend/bin/armpilot-backend.exe` 已构建（`cd backend && go build -o bin/arpilot-backend.exe .`）
 *   * 机械臂接在 config.serial.yaml 写的串口上，相机取景已按 docs/hardware-measurement.md
 *     的 Phase 4.5 标准重布（白分割板 + 画面内标尺 + 正交侧视 + **锁死曝光**）
 *   * 反解依赖 numpy + PIL：脚本会优先用 $MEARM_PYTHON，其次探测托管 venv
 *
 * 退出码：0 = 全 PASS；1 = 有 FAIL/SKIP；2 = 前置缺失或脚本自身错误。
 */
import { spawn, spawnSync } from 'node:child_process';
import { existsSync, mkdirSync, readFileSync, statSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

// ---------------------------------------------------------------------------
// 路径 / 参数
// ---------------------------------------------------------------------------

const TOOLS_DIR = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(TOOLS_DIR, '..');
const BACKEND_DIR = path.join(ROOT, 'backend');
const BACKEND_EXE = path.join(BACKEND_DIR, 'bin', 'armpilot-backend.exe');
const SERIAL_CFG = 'config.serial.yaml';

const argv = process.argv.slice(2);
function opt(name, fallback) {
  const i = argv.indexOf(`--${name}`);
  return i >= 0 && argv[i + 1] && !argv[i + 1].startsWith('--') ? argv[i + 1] : fallback;
}
const flag = (name) => argv.includes(`--${name}`);

const A = {
  http: opt('http', process.env.BACKEND_HTTP ?? 'http://127.0.0.1:8090'),
  ws: opt('ws', process.env.BACKEND_WS ?? 'ws://127.0.0.1:8090/ws/joint'),
  cam: opt('cam', process.env.MEARM_CAM ?? 'Integrated Camera'),
  width: Number(opt('width', 1280)),
  height: Number(opt('height', 720)),
  outDir: opt('out', path.join(ROOT, '.workbuddy', 'captures', `e2e_${stamp()}`)),
  tol: Number(opt('tol', 5.0)),          // 绝对误差容差（含相机公共偏置）
  dtol: Number(opt('dtol', 1.5)),        // 帧间差容差（锐利判据）
  // 暗件分割阈值：**默认 auto**（全批逐帧 Otsu 取中位数）。
  // 写死阈值会被曝光坑：同一台面上午那批 Otsu≈112、下午那批 Otsu≈164，
  // 用固定 90 切下午那批会只剩立柱和底座，反解全错还不报错。
  thresh: opt('thresh', 'auto'),
  // 同位姿重复性容差：RESET 帧与收尾 RESET 帧的反解差（与期望值无关的噪声底）
  repeatTol: Number(opt('repeat-tol', 1.5)),
  // 底座/立柱掩膜的来源：**默认 self = 用本批照片自身**。
  // 立柱在同一批里恒定不动，取交集即可；跨批（换相机位/换曝光）掩膜会整体错位，
  // 实测把锚点绝对角带偏 5.8°（self 时只剩 2.7°）。
  baseRegion: opt('base-region', 'self'),
  ackTol: Number(opt('ack-tol', 0.4)),   // 回推值与期望的一致性（舵机取整上限 0.347°）
  settleMs: Number(opt('settle', 1200)), // 发完命令等舵机物理到位
  only: opt('only', null),
  dryRun: flag('dry-run'),
  noCamera: flag('no-camera'),
  reuseBackend: flag('reuse-backend'),
  keepBackend: flag('keep-backend'),
};

function stamp() {
  const d = new Date();
  const p = (n) => String(n).padStart(2, '0');
  return `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
}

// ---------------------------------------------------------------------------
// 结果收集（与 ui-smoke.mjs 同风格）
// ---------------------------------------------------------------------------

const results = [];
function check(name, ok, detail = '') {
  results.push({ name, ok, detail });
  console.log(`[${ok ? 'PASS' : 'FAIL'}] ${name}${detail ? `  — ${detail}` : ''}`);
}
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const info = (msg) => console.log(`  · ${msg}`);
const warn = (msg) => console.log(`  ! ${msg}`);
const phase = (msg) => console.log(`\n── ${msg} ${'─'.repeat(Math.max(0, 58 - msg.length))}`);

// ---------------------------------------------------------------------------
// 日志：每一步的时间戳 + 耗时，落盘到 outDir/run.log
// ---------------------------------------------------------------------------
// 真机验收的失败常常是「间歇的」——同一条命令这次挂下次过。
// 没有时间轴就无从判断「是哪一步慢/卡/重试了」，所以这里把每一步都打上
// 相对时间与增量耗时，并在结束时同步写一份 run.log 供事后比对。
const T0 = Date.now();
const logLines = [];
function log(tag, msg) {
  const rel = ((Date.now() - T0) / 1000).toFixed(2).padStart(7);
  const line = `[${rel}s] ${tag.padEnd(9)} ${msg}`;
  logLines.push(line);
  console.log(line);
}
/** 包住一个异步阶段：记录开始/结束/耗时，异常时记录并**继续抛出**（不吞错）。 */
async function step(tag, label, fn) {
  const t = Date.now();
  log(tag, `→ ${label}`);
  try {
    const out = await fn();
    log(tag, `✓ ${label}（${((Date.now() - t) / 1000).toFixed(2)}s）`);
    return out;
  } catch (err) {
    log(tag, `✗ ${label}（${((Date.now() - t) / 1000).toFixed(2)}s）抛出：${err?.message ?? err}`);
    throw err;
  }
}
function flushLog(outDir) {
  try {
    mkdirSync(outDir, { recursive: true });
    writeFileSync(path.join(outDir, 'run.log'), logLines.join('\n') + '\n', 'utf8');
    return path.join(outDir, 'run.log');
  } catch {
    return null;
  }
}

// ---------------------------------------------------------------------------
// 外部工具探测（不硬编码本机路径）
// ---------------------------------------------------------------------------

function findFfmpeg() {
  const cands = [process.env.FFMPEG, process.env.MEARM_FFMPEG, 'ffmpeg'];
  for (const c of cands) {
    if (!c) continue;
    const r = spawnSync(c, ['-version'], { encoding: 'utf8' });
    if (r.status === 0) return c;
  }
  return null;
}

/** 找一个能 `import numpy, PIL, serial` 的 Python —— 反解要用前两个。 */
function findPython() {
  const cands = [
    process.env.MEARM_PYTHON,
    process.env.PYTHON,
    process.env.USERPROFILE
      ? path.join(process.env.USERPROFILE, '.workbuddy', 'binaries', 'python', 'envs', 'default', 'Scripts', 'python.exe')
      : null,
    process.env.USERPROFILE
      ? path.join(process.env.USERPROFILE, '.workbuddy', 'binaries', 'python', 'envs', 'default', 'bin', 'python')
      : null,
    'python3',
    'python',
  ];
  for (const c of cands) {
    if (!c) continue;
    const r = spawnSync(c, ['-c', 'import numpy, PIL'], { encoding: 'utf8' });
    if (r.status === 0) return c;
  }
  return null;
}

// ---------------------------------------------------------------------------
// 后端进程
// ---------------------------------------------------------------------------

async function probeBackend(timeoutMs = 800) {
  const ac = new AbortController();
  const t = setTimeout(() => ac.abort(), timeoutMs);
  try {
    const res = await fetch(`${A.http}/healthz`, { signal: ac.signal });
    return res.ok ? await res.json() : null;
  } catch {
    return null;
  } finally {
    clearTimeout(t);
  }
}

let backendChild = null;

async function startBackend() {
  const existing = await probeBackend();
  if (existing) {
    if (existing.device !== 'serial') {
      throw new Error(
        `${A.http} 上已有实例但 device=${existing.device}（不是真机模式）。` +
          `先停掉它，或用 --http/--ws 换一组端口。`,
      );
    }
    info(`复用 ${A.http} 上的真机实例（device=${existing.device}）`);
    return { external: true };
  }
  if (!existsSync(BACKEND_EXE)) {
    throw new Error(`未找到 ${BACKEND_EXE}；先在 backend/ 下 go build -o bin/armpilot-backend.exe .`);
  }
  const child = spawn(BACKEND_EXE, ['-c', SERIAL_CFG], { cwd: BACKEND_DIR, stdio: 'ignore' });
  backendChild = child;
  child.on('exit', (code) => info(`后端进程退出 code=${code}`));

  const deadline = Date.now() + 15000;
  while (Date.now() < deadline) {
    const h = await probeBackend(1000);
    if (h) {
      info(`后端就绪 device=${h.device} linked=${h.linked} state=${JSON.stringify(h.state)}`);
      return { external: false };
    }
    await sleep(200);
  }
  throw new Error('后端 15s 内未就绪（检查 config.serial.yaml 的串口号与接线）');
}

function stopBackend() {
  if (backendChild && !A.keepBackend) {
    try {
      backendChild.kill();
    } catch {
      /* 已退出 */
    }
  }
}

// ---------------------------------------------------------------------------
// WebSocket（Node 22 自带全局 WebSocket）
// ---------------------------------------------------------------------------

class Link {
  constructor(ws) {
    this.ws = ws;
    this.lastState = null;
    this.stateAt = 0;
    this.errors = [];
    this.hello = null;
    ws.addEventListener('message', (ev) => {
      let env;
      try {
        env = JSON.parse(typeof ev.data === 'string' ? ev.data : ev.data.toString());
      } catch {
        return;
      }
      if (env.type === 'joint_state' && env.joints) {
        this.lastState = env.joints;
        this.stateAt = Date.now();
      } else if (env.type === 'hello') {
        this.hello = env;
      } else if (env.type === 'error') {
        this.errors.push(`${env.code}: ${env.message}`);
      }
    });
  }

  static async open(url) {
    return new Promise((resolve, reject) => {
      const ws = new WebSocket(url);
      const t = setTimeout(() => reject(new Error(`WebSocket 连接超时：${url}`)), 8000);
      ws.addEventListener('open', () => {
        clearTimeout(t);
        resolve(new Link(ws));
      });
      ws.addEventListener('error', (e) => {
        clearTimeout(t);
        reject(new Error(`WebSocket 连接失败：${url} (${e?.message ?? e?.type ?? 'error'})`));
      });
    });
  }

  /** 发一条 joint_command，等到回推的 joint_state 与期望一致（或超时）。返回差值。 */
  async drive(joints, expect, { tol = A.ackTol, timeoutMs = 4000 } = {}) {
    const sentAt = Date.now();
    this.ws.send(
      JSON.stringify({ version: 1, type: 'joint_command', timestamp: sentAt, joints }),
    );
    let last = null;
    while (Date.now() - sentAt < timeoutMs) {
      if (this.lastState && this.stateAt >= sentAt) {
        last = this.lastState;
        if (maxDiff(last, expect) <= tol) break;
      }
      await sleep(40);
    }
    return last ? maxDiff(last, expect) : Number.POSITIVE_INFINITY;
  }

  close() {
    try {
      this.ws.close();
    } catch {
      /* 已关闭 */
    }
  }
}

function maxDiff(a, b) {
  let m = 0;
  for (const k of Object.keys(b)) {
    if (typeof a[k] !== 'number') return Number.POSITIVE_INFINITY;
    m = Math.max(m, Math.abs(a[k] - b[k]));
  }
  return m;
}

// ---------------------------------------------------------------------------
// 相机
// ---------------------------------------------------------------------------

/**
 * 抓一帧。**必须带重试** —— dshow 首次打开设备时常因像素格式协商失败而报：
 *     Non full-range YUV is non-standard, set strict_std_compliance to at most unofficial
 *     [enc:mjpeg] Error while opening encoder - maybe incorrect parameters such as bit_rate...
 * 实测（2026-09-12）同一台机器上间歇出现，重试即可成功。
 * 第二道防线是显式指定 `-pixel_format yuyv422`（多数 USB 摄像头唯一可靠的 dshow 原生格式）。
 *
 * @returns {boolean} 是否成功落盘（失败已打印原因，调用方据此计 FAIL）
 */
function grab(outFile, ffmpeg) {
  const base = [
    '-hide_banner', '-loglevel', 'error',
    '-f', 'dshow', '-rtbufsize', '100M',
    '-video_size', `${A.width}x${A.height}`,
    '-i', `video=${A.cam}`,
    '-frames:v', '1', '-q:v', '2', '-update', '1', '-y', outFile,
  ];
  // 第一次不指定像素格式（兼容性好），失败后第二次强制 yuyv422（治上面那个协商错）
  const attempts = [
    { tag: '默认像素格式', args: base },
    { tag: 'yuyv422', args: [...base.slice(0, 7), '-pixel_format', 'yuyv422', ...base.slice(7)] },
  ];
  let lastErr = '';
  for (let i = 0; i < attempts.length; i++) {
    const { tag, args } = attempts[i];
    const r = spawnSync(ffmpeg, args, { encoding: 'utf8' });
    const ok = r.status === 0 && existsSync(outFile) && readSize(outFile) > 1024;
    if (ok) {
      if (i > 0) info(`抓帧 ${path.basename(outFile)} 在「${tag}」下重试成功`);
      return true;
    }
    lastErr = (r.stderr ?? '').trim().split('\n').filter(Boolean).slice(-2).join(' | ');
    if (i === 0) {
      warn(`抓帧 ${path.basename(outFile)} 首次失败（${tag}）：${lastErr || `exit=${r.status}`}；重试 yuyv422…`);
    }
  }
  info(`抓帧失败 ${path.basename(outFile)}: ${lastErr || '未知原因'}`);
  return false;
}

function readSize(p) {
  try {
    return statSync(p).size;
  } catch {
    return 0;
  }
}

// ---------------------------------------------------------------------------
// 反解（调 verify_pose.py）
// ---------------------------------------------------------------------------

/** 让 verify_pose.py 给出本次真值输入（homePose/限位）与量化后的落点。 */
function verifyPose(python, extraArgs, { allowVerdictFail = false } = {}) {
  const args = [path.join(TOOLS_DIR, 'verify_pose.py'), ...extraArgs];
  const r = spawnSync(python, args, { cwd: ROOT, encoding: 'utf8' });
  // verify_pose 的退出码语义：0 = 全 PASS，1 = 有 FAIL/SKIP（**不是**脚本崩溃）。
  // 只有 2 及以上的退出码、或完全没有 stdout，才算真出错。
  const bad = r.status !== 0 && !(allowVerdictFail && r.status === 1);
  if (bad || !r.stdout) {
    throw new Error(
      `verify_pose.py 执行失败（exit=${r.status}）：\n${(r.stderr ?? '').trim() || '(stderr 为空)'}`,
    );
  }
  return r.stdout;
}

function dumpInputs(python, intentSpec) {
  const args = ['--dump-inputs'];
  if (intentSpec) args.push('--quantize', intentSpec);
  return JSON.parse(verifyPose(python, args).trim().split('\n').pop());
}

// ---------------------------------------------------------------------------
// 动作计划
// ---------------------------------------------------------------------------

/**
 * 安全序列：**先单关节小步，再组合**（沿用 spec §Phase10 的保守流程）。
 * 步长挑在关节限位内且远离两端 —— 撞限位会让固件钳位，反解与期望比对就失去意义。
 */
function buildPlan(home) {
  const mv = (sh, el) => ({ base: home.base, shoulder: sh, elbow: el, gripper: home.gripper });
  return [
    { tag: '00_reset', label: 'RESET（锚点帧）', joints: mv(home.shoulder, home.elbow) },
    { tag: '01_sh_p5', label: '肩 +5°', joints: mv(home.shoulder + 5, home.elbow) },
    { tag: '02_sh_p15', label: '肩 +15°', joints: mv(home.shoulder + 15, home.elbow) },
    { tag: '03_el_p5', label: '肘 +5°（肩回 HOME）', joints: mv(home.shoulder, home.elbow + 5) },
    { tag: '04_el_p15', label: '肘 +15°', joints: mv(home.shoulder, home.elbow + 15) },
    { tag: '05_combo', label: '组合：肩 +15° 肘 +15°', joints: mv(home.shoulder + 15, home.elbow + 15) },
    { tag: '06_reset2', label: 'RESET（重复性复核）', joints: mv(home.shoulder, home.elbow) },
  ];
}

function checkLimits(steps, limits) {
  const bad = [];
  for (const s of steps) {
    for (const [id, v] of Object.entries(s.joints)) {
      const lim = limits[id];
      if (!lim) continue;
      if (v < lim[0] - 1e-9 || v > lim[1] + 1e-9) {
        bad.push(`${s.tag} ${id}=${v.toFixed(2)} 越出 [${lim[0].toFixed(2)}, ${lim[1].toFixed(2)}]`);
      }
    }
  }
  return bad;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

async function main() {
  console.log('==== ArmPilot 真机端到端验收（Phase 9 · 相机做外部地面真值）====');
  console.log(`# 工程根目录   ${ROOT}`);
  console.log(`# 输出目录     ${A.outDir}`);
  console.log(`# 容差         绝对 ${A.tol}° · 帧间差 ${A.dtol}° · 回推一致 ${A.ackTol}°`);

  // Python 是刚性依赖：即使 --no-camera，也要用它从 robot.yaml 解析 homePose/限位。
  const python = findPython();
  if (!python) {
    console.error('\n[fatal] 找不到带 numpy + PIL 的 Python（可用 MEARM_PYTHON 指定）。');
    return 2;
  }
  info(`Python: ${python}`);

  const ffmpeg = A.noCamera ? null : findFfmpeg();
  if (!A.noCamera && !ffmpeg) {
    console.error('\n[fatal] 找不到 ffmpeg（可用 FFMPEG 指定路径）。');
    return 2;
  }
  if (ffmpeg) info(`ffmpeg: ${ffmpeg}`);

  // ---- 真值输入（唯一来源 robot.yaml，经 verify_pose.py 解析） ----
  const inputs = dumpInputs(python);
  console.log(`# homePose     ${JSON.stringify(inputs.home)}`);
  console.log(`# 关节限位     ${JSON.stringify(inputs.limits)}`);

  const steps = buildPlan(inputs.home);
  const bad = checkLimits(steps, inputs.limits);
  if (bad.length) {
    console.error('\n[fatal] 动作计划越限，拒绝驱动真机：');
    for (const b of bad) console.error(`   ${b}`);
    return 2;
  }

  const selected = A.only
    ? steps.filter((s) => A.only.split(',').some((t) => s.tag.startsWith(t.trim())))
    : steps;
  // 锚点帧必须存在：--only 选了子集时要保留它
  if (!selected.some((s) => s.tag === '00_reset')) selected.unshift(steps[0]);

  console.log(`\n# 动作计划（共 ${selected.length} 步，base 恒为 0 —— 相机只能测矢状面）`);
  for (const s of selected) {
    const q = dumpInputs(python, `shoulder=${s.joints.shoulder},elbow=${s.joints.elbow}`);
    s.expect = q.quantized;
    s.servo = q.quantized_servo ?? {};
    console.log(
      `   ${s.tag.padEnd(12)} ${s.label.padEnd(22)} 意图 肩=${s.joints.shoulder.toFixed(2)} 肘=${s.joints.elbow.toFixed(2)}` +
        `  → 量化后 肩=${s.expect.shoulder.toFixed(3)} 肘=${s.expect.elbow.toFixed(3)}`,
    );
  }

  if (A.dryRun) {
    console.log('\n--dry-run：未连接任何硬件，退出。');
    return 0;
  }

  mkdirSync(A.outDir, { recursive: true });
  log('start', `工作目录 ${A.outDir}`);

  // ---- 起后端（会打开串口 → Uno 复位 → 舵机弹回 90°） ----
  phase('启动后端');
  const backend = await step('backend', '打开串口并等静默窗口 + 暖机探测', () => startBackend());

  const link = await step('link', '建立 WebSocket 连接', () => Link.open(A.ws));
  try {
    // hello：链路末端必须是真机
    const t0 = Date.now();
    while (!link.hello && Date.now() - t0 < 3000) await sleep(50);
    check('后端 hello 上报链路末端 = serial（真机）', link.hello?.device === 'serial',
      `device=${link.hello?.device ?? '(未收到 hello)'}`);

    // 等链路就绪：后端打开串口后要过 Uno 的**静默窗口 + 暖机探测**，
    // 这期间 `connected=false`，此时下发的命令会被 `DEVICE_UNAVAILABLE` 直接拒掉。
    // 首版没有这道门时，第一条 JR 的报错就藏在收尾日志里没人注意。
    phase('等待链路就绪');
    const readyDeadline = Date.now() + 10000;
    let linked = false;
    while (Date.now() < readyDeadline) {
      const h = await probeBackend(800);
      if (h?.linked) {
        linked = true;
        break;
      }
      log('ready', `linked=false，继续等（剩 ${((readyDeadline - Date.now()) / 1000).toFixed(1)}s）`);
      await sleep(200);
    }
    check('链路就绪（后端已过开机静默窗口并暖机成功）', linked,
      `healthz.linked=${linked}（等 ${((readyDeadline - Date.now()) / 1000 + 10).toFixed(1)}s）`);

    if (link.hello?.model?.homePose) {
      const hp = link.hello.model.homePose;
      const same = ['shoulder', 'elbow'].every(
        (k) => Math.abs((hp[k] ?? NaN) - inputs.home[k]) < 1e-9,
      );
      check('后端 homePose 与 robot.yaml 逐位一致（标定只有一份）', same,
        `后端 ${JSON.stringify(hp)}`);
    }

    // ---- 逐步执行 ----
    phase('执行动作序列（每步：发命令 → 稳定 → 抓帧）');
    const shotFiles = [];
    for (const s of selected) {
      const tStep = Date.now();
      const diff = await link.drive(s.joints, s.expect);
      const ackOk = diff <= A.ackTol;
      check(`链路回推 · ${s.tag} ${s.label}（≤${A.ackTol}°）`, ackOk,
        Number.isFinite(diff) ? `max|Δ|=${diff.toFixed(3)}°` : '未收到 joint_state');
      log(s.tag, `回推 max|Δ|=${Number.isFinite(diff) ? diff.toFixed(3) : 'NaN'}°  稳定等待 ${A.settleMs}ms`);

      await sleep(A.settleMs);

      if (!A.noCamera) {
        const f = path.join(A.outDir, `${s.tag}.jpg`);
        const ok = grab(f, ffmpeg);
        check(`相机抓帧 · ${s.tag}`, ok, path.relative(ROOT, f));
        if (ok) shotFiles.push(f);
        log(s.tag, `抓帧 ${ok ? '成功' : '失败'}  本步耗时 ${((Date.now() - tStep) / 1000).toFixed(2)}s`);
      } else {
        log(s.tag, `跳过抓帧（--no-camera）  本步耗时 ${((Date.now() - tStep) / 1000).toFixed(2)}s`);
      }
    }
    log('steps', `动作序列结束：${shotFiles.length}/${selected.length} 帧落盘`);

    // ---- 反解 ----
    if (!A.noCamera && shotFiles.length) {
      console.log('\n# 相机反解验收（verify_pose.py，锚点 = RESET 帧）');
      const planFile = path.join(A.outDir, 'plan.json');
      const plan = {};
      // `_servo` 是给「标定增益复核（Δ关节反解/Δ舵机）」用的自变量：
      // 走 --plan 时文件名不再带 S<ch>_<ang>，缺了它增益复核只能报「帧数不足」。
      for (const s of selected) plan[`${s.tag}.jpg`] = { ...s.expect, _servo: s.servo };
      writeFileSync(planFile, JSON.stringify(plan, null, 2), 'utf8');

      const anchor = path.join(A.outDir, `${selected[0].tag}.jpg`);
      const jsonOut = path.join(A.outDir, 'verify_pose.json');
      const overlay = path.join(A.outDir, 'overlay');
      const args = [
        A.outDir,
        '--anchor', anchor,
        '--plan', planFile,
        '--thresh', String(A.thresh),
        '--base-region', A.baseRegion,
        '--tol', String(A.tol),
        '--dtol', String(A.dtol),
        '--json', jsonOut,
        '--overlay', overlay,
      ];
      const out = verifyPose(python, args, { allowVerdictFail: true });
      console.log(out.split('\n').filter((l) => l.startsWith('#') || /PASS|FAIL|SKIP|实测/.test(l)).join('\n'));

      const report = JSON.parse(readFileSync(jsonOut, 'utf8'));
      const n = report.summary;
      check('相机反解：全部帧 PASS（绝对误差 + 帧间差）', n.fail === 0 && n.skip === 0,
        `PASS ${n.pass} / FAIL ${n.fail} / SKIP ${n.skip}`);
      for (const r of report.frames) {
        if (r.verdict !== 'PASS') {
          info(`${r.file}: ${r.verdict} — 肩差 ${r.err?.shoulder ?? '?'}° 肘差 ${r.err?.elbow ?? '?'}°`);
        }
      }
      // 增益复核：把「标定 scale 是否可信」变成一等输出
      for (const name of ['shoulder', 'elbow']) {
        const g = report.gain?.[name];
        if (g) info(`标定增益复核 ${name}: 实测 ${g.measured} vs yaml ${g.expected}` +
                    `（${g.delta_pct > 0 ? '+' : ''}${g.delta_pct.toFixed(1)}%）`);
      }

      // 重复性：`00_reset` 与 `06_reset2` 是**同一个物理位姿**（都回 RESET），
      // 两者反解之差就是「相机反解 + 机械回位」的总噪声底 —— **与任何期望值无关**，
      // 所以它比绝对误差更能说明这台相机在当前台面上到底能分辨多少度。
      const byName = Object.fromEntries(report.frames.map((r) => [r.file, r]));
      const r0 = byName[`${selected[0].tag}.jpg`];
      const r1 = byName[`${selected[selected.length - 1].tag}.jpg`];
      let repeat = null;
      if (r0?.fit && r1?.fit) {
        repeat = {
          shoulder: Math.abs(r1.fit.shoulder - r0.fit.shoulder),
          elbow: Math.abs(r1.fit.elbow - r0.fit.elbow),
        };
        check(
          `相机重复性（同位姿两帧 ${r0.file} vs ${r1.file}，≤${A.repeatTol}°）`,
          repeat.shoulder <= A.repeatTol && repeat.elbow <= A.repeatTol,
          `肩 ${repeat.shoulder.toFixed(2)}° 肘 ${repeat.elbow.toFixed(2)}°`,
        );
        info('⇒ 这条是本次测量的**噪声底**：它大于命令精度（0.1°）就说明相机在当前台面上' +
             '分辨不出「命令级」到位，只能判「方向对不对、幅度大约对不对」。');
      }
      console.log(`# 叠加图 -> ${path.relative(ROOT, overlay)}（逐帧目视复核：数字对不上眼睛时先信眼睛）`);
    }

    // ---- 收尾：回 RESET ----
    console.log('\n# 收尾：回到 RESET 位');
    await link.drive(steps[0].joints, steps[0].expect, { timeoutMs: 5000 });
    check('收尾复位到 HOME 位', maxDiff(link.lastState ?? {}, steps[0].expect) <= A.ackTol,
      JSON.stringify(link.lastState ?? null));

    if (link.errors.length) info(`链路错误消息：${link.errors.join('; ')}`);
  } finally {
    link.close();
    if (!backend.external) stopBackend();
  }

  // ---- 汇总 ----
  const npass = results.filter((r) => r.ok).length;
  const nfail = results.length - npass;
  console.log(`\n==== 真机端到端验收 PASS ${npass} / FAIL ${nfail}（共 ${results.length} 项）====`);
  console.log('# 读结果的口径（**两条独立的证据链，别混着读**）：');
  console.log('#   ① 链路项（回推一致 / 就绪 / 复位）PASS 只证明「指令确实送到了固件并被接受」，');
  console.log('#      **不证明机械臂物理到位** —— 真机固件无位置反馈，STATUS/JR 回执都是目标值。');
  console.log('#   ② 相机项才是物理证据。其中**重复性**（同位姿两帧反解差）是与期望值无关的噪声底，');
  console.log('#      **绝对误差**含反解公共偏置，台面未重布时不可当作机械精度读。');
  const summaryFile = path.join(A.outDir, 'summary.json');
  writeFileSync(summaryFile, JSON.stringify({ when: new Date().toISOString(), options: A, results }, null, 2), 'utf8');
  console.log(`# 结果 -> ${path.relative(ROOT, summaryFile)}`);
  if (nfail) {
    console.log(
      '# 排查顺序：① 台面是否按 docs/hardware-measurement.md 的 Phase 4.5 重布' +
        '（白分割板 + 画面内标尺 + 正交侧视 + 锁死曝光）；② 链路层 FAIL 才查固件/串口。',
    );
  }
  // ---- 日志落盘 ----
  // 真机验收的失败往往是间歇的，run.log 是事后唯一的时间轴证据（含每步耗时/重试）。
  const runLog = flushLog(A.outDir);
  if (runLog) console.log(`# 日志 -> ${path.relative(ROOT, runLog)}`);

  return nfail ? 1 : 0;
}

main()
  .then((code) => process.exit(code))
  .catch((err) => {
    // 走到这里说明是**脚本自身崩溃**（不是验收 FAIL，验收 FAIL 走 return 1）。
    // 崩溃也必须有据可查：打印完整堆栈 + 落盘 run.log，否则用户看到的只有一行
    // "[fatal] ..."，根本没法判断是卡在哪一步崩的。
    console.error('\n[fatal] 脚本执行中断（这是脚本崩溃，不是验收失败）');
    console.error(err?.stack ?? String(err));
    const runLog = flushLog(A.outDir);
    if (runLog) {
      console.error(`# 已把中断前的逐步日志写入 -> ${path.relative(ROOT, runLog)}`);
      console.error('# 看这份日志最后几行，即可知道崩在哪一步（✗ 标记的那条）。');
    } else {
      console.error(`# 日志落盘也失败了（输出目录不可写？）：${A.outDir}`);
    }
    stopBackend();
    process.exit(2);
  });
