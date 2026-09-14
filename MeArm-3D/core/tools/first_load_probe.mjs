#!/usr/bin/env node
/**
 * 首帧重影诊断（零依赖，直连 CDP）。
 *
 * 目的：抓「**只在首次加载出现、一交互就消失**」的渲染瞬态。
 *
 * 为什么不能用截图：`Page.captureScreenshot` 取到的永远是**已经稳定**的一帧。
 * 等页面能被截图时，这类现象通常已经自愈 —— 截 10 张也都是对的。
 * 所以本脚本读的是页面里 dev 探针装的**逐帧时序**（`window.__armPilotFrames`，
 * 见 `TestProbe.tsx` 的 `SceneFrameSample`），每一帧都有：
 *
 *   - `roots`          场景里 `robot:` 根节点数（**>1 ⇒ 同时画了多棵树 = 重影**）
 *   - `visibleRoots`   其中可见的数量
 *   - `arms[]`         每棵树的 是否幽灵 / 可见 / 世界原点 / tcp 世界坐标
 *   - `texturedWithEnv` 已挂 envMap 的贴图材质数（判断环境反射那轮重建发生没有）
 *
 * 读矩阵时**不调** `updateMatrixWorld`：本记录器在 `useFrame` 阶段跑，先于 `gl.render`，
 * 读到的正是上一帧真正画出去的矩阵；手动更新会把"矩阵陈旧"这类 bug 抹掉。
 *
 * 用法：
 *   node core/tools/first_load_probe.mjs [url] [debugPort]
 *   BACKEND_HTTP / BACKEND_WS 可覆盖后端地址（默认 8090）
 * 产物：`.workbuddy/captures/first-load-00-cold.png` / `-01-after-move.png`
 */
import { spawn } from 'node:child_process';
import { existsSync, mkdirSync, writeFileSync } from 'node:fs';
import { mkdtempSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const REPO_DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const CAPTURE_DIR = path.join(REPO_DIR, '.workbuddy', 'captures');

const TARGET_URL = process.argv[2] ?? 'http://127.0.0.1:5276/';
const DEBUG_PORT = Number(process.argv[3] ?? 9334);

const EDGE_CANDIDATES = [
  process.env.EDGE_PATH,
  'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe',
  'C:\\Program Files\\Microsoft\\Edge\\Application\\msedge.exe',
  'C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe',
  'C:\\Program Files (x86)\\Google\\Chrome\\Application\\chrome.exe',
].filter(Boolean);

const CHROME_FLAGS = [
  '--headless=new',
  '--no-sandbox',
  '--disable-gpu',
  '--disable-dev-shm-usage',
  '--no-first-run',
  '--no-default-browser-check',
  '--use-gl=angle',
  '--use-angle=swiftshader',
  '--enable-unsafe-swiftshader',
  '--hide-scrollbars',
  '--window-size=1400,1000',
  '--disable-sync',
  '--disable-features=msEdgeSyncConfirmationDialog,EdgeSyncPromo',
];

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

function findBrowser() {
  for (const candidate of EDGE_CANDIDATES) {
    if (candidate && existsSync(candidate)) return candidate;
  }
  return null;
}

class Cdp {
  constructor(socket) {
    this.socket = socket;
    this.nextId = 1;
    this.pending = new Map();
    this.events = [];
    socket.addEventListener('message', (event) => {
      const message = JSON.parse(event.data);
      if (message.id && this.pending.has(message.id)) {
        const { resolve, reject } = this.pending.get(message.id);
        this.pending.delete(message.id);
        if (message.error) reject(new Error(JSON.stringify(message.error)));
        else resolve(message.result);
      } else if (message.method) {
        this.events.push(message);
      }
    });
  }

  send(method, params = {}) {
    const id = this.nextId++;
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.socket.send(JSON.stringify({ id, method, params }));
      setTimeout(() => {
        if (this.pending.has(id)) {
          this.pending.delete(id);
          reject(new Error(`CDP 超时: ${method}`));
        }
      }, 20000);
    });
  }

  async evaluate(expression) {
    const result = await this.send('Runtime.evaluate', {
      expression,
      returnByValue: true,
      awaitPromise: true,
    });
    if (result.exceptionDetails) {
      throw new Error(`页面求值异常: ${JSON.stringify(result.exceptionDetails)}`);
    }
    return result.result?.value;
  }

  async screenshot() {
    const result = await this.send('Page.captureScreenshot', { format: 'png' });
    return result.data;
  }

  pageErrors() {
    const errors = [];
    for (const event of this.events) {
      if (event.method === 'Runtime.exceptionThrown') {
        const d = event.params?.exceptionDetails;
        errors.push(`exception: ${d?.exception?.description ?? d?.text ?? 'unknown'}`);
      }
      if (event.method === 'Runtime.consoleAPICalled' && event.params?.type === 'error') {
        errors.push(
          `console.error: ${(event.params.args ?? [])
            .map((a) => a.value ?? a.description ?? a.type)
            .join(' ')}`,
        );
      }
      if (event.method === 'Log.entryAdded' && event.params?.entry?.level === 'error') {
        errors.push(`log: ${event.params.entry.text}`);
      }
    }
    return errors;
  }
}

/** 取紧凑形式：`[f, roots, visibleRoots, texturedWithEnv, ...每棵树的读数]` */
const FETCH_FRAMES_JS = (from) => `
(() => {
  const all = window.__armPilotFrames;
  if (!all) return { ready: false };
  const rows = [];
  for (let i = ${from}; i < all.length; i += 1) {
    const s = all[i];
    const arms = s.arms.map((a) => [
      a.ghost ? 1 : 0,
      a.visible ? 1 : 0,
      ...a.origin.map((v) => Math.round(v * 1000) / 1000),
      ...(a.tcp ? a.tcp.map((v) => Math.round(v * 1000) / 1000) : [null, null, null]),
    ]);
    rows.push([s.f, s.roots, s.visibleRoots, s.texturedWithEnv, arms]);
  }
  return { ready: true, total: all.length, rows };
})()
`;

function fmtArm(arm) {
  const kind = arm[0] ? '幽灵' : '主臂';
  const vis = arm[1] ? '可见' : '隐藏';
  const tcp = arm.slice(5);
  const tcpText = tcp[0] === null ? '—' : `[${tcp.join(', ')}]`;
  return `${kind}/${vis} tcp=${tcpText}`;
}

function printRows(label, rows, limit) {
  console.log(`\n=== ${label}（共 ${rows.length} 帧，打印前 ${Math.min(limit, rows.length)} 帧）===`);
  for (const [f, roots, visibleRoots, envs, arms] of rows.slice(0, limit)) {
    console.log(
      `f${String(f).padStart(3)} roots=${roots} visible=${visibleRoots} envMapMat=${envs} | ` +
        arms.map(fmtArm).join('  ||  '),
    );
  }
}

/** 只在读数发生变化的帧上打印：瞬态往往只有两三帧，逐帧刷屏反而看不出来 */
function printTransitions(label, rows) {
  console.log(`\n=== ${label} · 状态跃迁（只打印发生变化的帧）===`);
  let prev = null;
  for (const row of rows) {
    const key = JSON.stringify([row[1], row[2], row[3], row[4]]);
    if (key !== prev) {
      const [f, roots, visibleRoots, envs, arms] = row;
      console.log(
        `f${String(f).padStart(3)} roots=${roots} visible=${visibleRoots} envMapMat=${envs} | ` +
          arms.map(fmtArm).join('  ||  '),
      );
      prev = key;
    }
  }
}

async function main() {
  const browser = findBrowser();
  if (!browser) throw new Error('未找到 Edge / Chrome 可执行文件，请设置 EDGE_PATH');
  mkdirSync(CAPTURE_DIR, { recursive: true });

  const userDataDir = mkdtempSync(path.join(tmpdir(), 'armpilot-probe-'));
  const child = spawn(
    browser,
    [...CHROME_FLAGS, `--user-data-dir=${userDataDir}`, `--remote-debugging-port=${DEBUG_PORT}`, TARGET_URL],
    { stdio: 'ignore' },
  );

  let cdp = null;
  try {
    // 等待调试端口 —— 优先选 URL 命中目标页的那个（全新 profile 首启可能先开别的内部页）
    let target = null;
    const deadline = Date.now() + 25000;
    while (Date.now() < deadline) {
      try {
        const response = await fetch(`http://127.0.0.1:${DEBUG_PORT}/json/list`);
        const targets = await response.json();
        const pages = targets.filter((t) => t.type === 'page' && t.webSocketDebuggerUrl);
        target = pages.find((t) => t.url.startsWith(TARGET_URL.replace(/\/$/, ''))) ?? pages[0];
        if (target) break;
      } catch {
        /* 端口还没起来 */
      }
      await sleep(300);
    }
    if (!target) throw new Error('未能连上调试端口');

    const socket = new WebSocket(target.webSocketDebuggerUrl);
    await new Promise((resolve, reject) => {
      socket.addEventListener('open', resolve, { once: true });
      socket.addEventListener('error', reject, { once: true });
    });
    cdp = new Cdp(socket);
    await cdp.send('Runtime.enable');
    await cdp.send('Log.enable');
    await cdp.send('Page.enable');

    // 等页面出现首批帧。**不重新导航**：导航会把已经积累的时序清掉，
    // 而目标恰恰是"从零开始的前若干帧"。
    console.log(`[probe] 目标 ${TARGET_URL}（debugPort=${DEBUG_PORT}）`);
    let first = { ready: false };
    const waitDeadline = Date.now() + 30000;
    while (Date.now() < waitDeadline) {
      first = await cdp.evaluate(FETCH_FRAMES_JS(0)).catch(() => ({ ready: false }));
      if (first.ready && first.total >= 30) break;
      await sleep(200);
    }
    if (!first.ready) throw new Error('页面未暴露 __armPilotFrames（dev 构建才挂探针）');

    const cold = first.rows;
    printTransitions('冷启动', cold);
    if (cold.length > 0) printRows('冷启动', cold, 12);

    const shot0 = await cdp.screenshot();
    writeFileSync(path.join(CAPTURE_DIR, 'first-load-00-cold.png'), Buffer.from(shot0, 'base64'));

    const state0 = await cdp.evaluate('JSON.stringify(window.__armPilot.state())');
    console.log(`\n[probe] 冷启动 state = ${state0}`);
    const ghost0 = await cdp.evaluate('JSON.stringify(window.__armPilotGhost ?? null)');
    console.log(`[probe] 冷启动 __armPilotGhost = ${ghost0}`);

    // ---- 做一次移动，看现象是否随之消失 ----
    const mark = cold.length;
    const moved = await cdp.evaluate(
      'JSON.stringify(window.__armPilot.moveTo([120, -60, 180]))',
    );
    console.log(`\n[probe] moveTo([120,-60,180]) = ${moved}`);
    await sleep(1500);

    const after = await cdp.evaluate(FETCH_FRAMES_JS(mark));
    printTransitions('移动后', after.rows);

    const shot1 = await cdp.screenshot();
    writeFileSync(path.join(CAPTURE_DIR, 'first-load-01-after-move.png'), Buffer.from(shot1, 'base64'));

    const errors = cdp.pageErrors();
    console.log(`\n[probe] 页面错误 = ${errors.length === 0 ? '无' : JSON.stringify(errors, null, 2)}`);
    console.log(`[probe] 截图 → ${path.join(CAPTURE_DIR, 'first-load-00-cold.png')} / -01-after-move.png`);
  } finally {
    try {
      cdp?.socket.close();
    } catch {
      /* 已关闭 */
    }
    child.kill();
  }
}

main().catch((error) => {
  console.error(`[probe] 失败：${error.message}`);
  process.exitCode = 1;
});
