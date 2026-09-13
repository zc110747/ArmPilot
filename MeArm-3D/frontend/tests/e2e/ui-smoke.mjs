#!/usr/bin/env node
/**
 * ArmPilot 前端 e2e 冒烟验收（零依赖：Node >= 22 自带 fetch / WebSocket，直连 CDP）。
 *
 * 验证内容（对应 Phase 2 / Phase 3 / Phase 4 / Phase 6 / Phase 7 / Phase 8 的"运行"环节）：
 *   1. 页面渲染无控制台错误 / 未捕获异常
 *   2. 3D 场景与外层面板都渲染出来（有 canvas 与外层读数）
 *   3. 拖动 J2/J3 滑杆后，末端 TCP 读数改变
 *   4. **改姿态后 FK ↔ Three.js 一致性仍 < 0.1mm**（运行态最强证据：
 *      屏幕上的模型与纯数学 FK 在任何位姿下都重合）
 *   5. 状态表 Command 列跟随滑杆
 *   6. 点击 HOME 能回到 HOME 位姿读数
 *   7. Phase 6：XYZ 直输 / 越界不钳位 / 真实鼠标拖拽把手
 *   8. Phase 7：MockTransport 闭环（Actual 滞后 → 收敛 → 断开复位）
 *   9. Phase 8：**真实 Go 后端**（`backend/bin/armpilot-backend.exe`，链路末端 = 内置
 *      sim 串口）—— 浏览器 → WebSocket → Go → 假固件 → 反算关节角 → 回推 Actual，
 *      并验证断线后自动重连 + 补发当前命令。后端起不来时这一段整体记 FAIL。
 *
 * 用法：
 *   node tests/e2e/ui-smoke.mjs [url] [debugPort] [screenshotPngPath]
 * 传了截图路径时，会在「HOME 复位」之后截一张 PNG —— 用于更新
 * docs/images/armpilot-console.png（HOME 位 = 固件 RESET 位，可与实拍照直接比对）。
 * 前置：前端开发服务器已启动（默认 http://127.0.0.1:5273/）；后端可执行文件已构建
 *       （`cd backend && go build -o bin/armpilot-backend.exe .`）。
 *       若 8090 上已有实例在跑，则复用该实例并跳过「断线重连」子项。
 */
import { spawn, spawnSync } from 'node:child_process';
import { existsSync, mkdirSync, mkdtempSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

// 位置参数：只有第 1 个是 url，第 2 个才是 debugPort。
// ⚠️ 只传一个数字（例如 `... 9334`）会被当成 url，浏览器就去开 "9334" 这个地址，
//    表现为「页面未在 30s 内完成渲染」——实测浪费过一轮排查。这里做形状校正：
//    纯数字的 url 视为想改 debugPort，url 退回默认值。
function parseArgs(argv) {
  let url = argv[2];
  let port = argv[3];
  if (url !== undefined && /^\d+$/.test(String(url)) && port === undefined) {
    port = url;
    url = undefined;
  }
  if (url !== undefined && !/^https?:\/\//i.test(String(url))) {
    console.warn(`[e2e] 警告: "${url}" 不是 http(s) URL，已回退到默认地址`);
    url = undefined;
  }
  const p = port === undefined ? 9333 : Number(port);
  if (port !== undefined && !Number.isInteger(p)) {
    console.warn(`[e2e] 警告: debugPort "${port}" 不是整数，回退到 9333`);
  }
  return {
    url: url ?? 'http://127.0.0.1:5273/',
    port: Number.isInteger(p) ? p : 9333,
  };
}

const { url: TARGET_URL, port: DEBUG_PORT } = parseArgs(process.argv);

// ---- Phase 8：真实 Go 后端（关节级 WebSocket 服务）----
const REPO_DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..', '..');
const BACKEND_DIR = path.join(REPO_DIR, 'backend');
const BACKEND_EXE = path.join(BACKEND_DIR, 'bin', 'armpilot-backend.exe');
const BACKEND_HTTP = process.env.BACKEND_HTTP ?? 'http://127.0.0.1:8090';
const BACKEND_WS = process.env.BACKEND_WS ?? 'ws://127.0.0.1:8090/ws/joint';
/** 已拉起的后端进程（正常退出时兜底清理，避免占用 8090 影响下一次运行） */
const backendChildren = [];
/**
 * Phase 9（opt-in）：真机段由 `tools/verify_serial_e2e.mjs` 自己拉起 serial 后端，
 * 本脚本只在 `finally` 里兜底清理 —— 它在自己的进程里，故本数组通常为空。
 */
const serialChildren = [];

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
  // 全新 profile 首启时 Edge 会弹出 edge://sync-confirmation-dialog/，
  // 它是 /json/list 里的**第一个 page**。若按序取首个 page 就会导航到弹窗上，
  // 表现为「页面未在 30s 内完成渲染」——实测踩过，且只在干净 profile 上复现。
  '--disable-sync',
  '--disable-features=msEdgeSyncConfirmationDialog,EdgeSyncPromo',
];

const results = [];
function check(name, ok, detail = '') {
  results.push({ name, ok, detail });
  const mark = ok ? 'PASS' : 'FAIL';
  console.log(`[${mark}] ${name}${detail ? `  — ${detail}` : ''}`);
}

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

/**
 * 反复求值直到满足条件（用于等待异步/节流产生的读数出现，而不是掩盖失败：
 * 条件不满足时会一直重试到超时，最终断言仍会失败）。
 */
async function poll(cdp, expression, predicate, timeoutMs = 10000, intervalMs = 250) {
  const deadline = Date.now() + timeoutMs;
  let last = null;
  while (Date.now() < deadline) {
    last = await cdp.evaluate(expression).catch(() => null);
    if (predicate(last)) return last;
    await sleep(intervalMs);
  }
  return last;
}

/**
 * 用 CDP 派发**真实鼠标事件**完成一次拖拽（按下 → 分步移动 → 抬起）。
 * 这是 Phase 6「鼠标拖动末端」唯一可信的证据：合成 DOM 事件绕不过 R3F 的射线拾取，
 * 只有走浏览器输入管线才能真正命中场景里的把手。
 */
async function dragMouse(cdp, from, to, steps = 10) {
  await cdp.send('Input.dispatchMouseEvent', {
    type: 'mousePressed',
    x: from.x,
    y: from.y,
    button: 'left',
    buttons: 1,
    clickCount: 1,
  });
  for (let i = 1; i <= steps; i += 1) {
    const t = i / steps;
    await cdp.send('Input.dispatchMouseEvent', {
      type: 'mouseMoved',
      x: from.x + (to.x - from.x) * t,
      y: from.y + (to.y - from.y) * t,
      button: 'left',
      buttons: 1,
    });
    await sleep(16);
  }
  await cdp.send('Input.dispatchMouseEvent', {
    type: 'mouseReleased',
    x: to.x,
    y: to.y,
    button: 'left',
    buttons: 0,
    clickCount: 1,
  });
}

function findBrowser() {
  for (const candidate of EDGE_CANDIDATES) {
    if (candidate && existsSync(candidate)) return candidate;
  }
  return null;
}

async function waitForDebuggerEndpoint(port, timeoutMs = 25000) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    try {
      const response = await fetch(`http://127.0.0.1:${port}/json/list`);
      const targets = await response.json();
      const pages = targets.filter((t) => t.type === 'page' && t.webSocketDebuggerUrl);
      // 优先选 URL 命中目标页的那个，别盲取首个：浏览器首启可能先开
      // 内部页（edge://sync-confirmation-dialog/ 等），取错就会一直等不到渲染。
      const wanted = pages.find((t) => typeof t.url === 'string' && t.url.startsWith(TARGET_URL));
      if (wanted) return wanted;
      // 兜底：忽略浏览器内部页，取第一个真实 http(s) 页面
      const httpPage = pages.find((t) => typeof t.url === 'string' && /^https?:/i.test(t.url));
      if (httpPage) return httpPage;
    } catch {
      /* 端口还没起来，继续等 */
    }
    await sleep(250);
  }
  throw new Error(`调试端口 ${port} 在 ${timeoutMs}ms 内未就绪`);
}

// ---------------------------------------------------------------------------
// Phase 8：后端进程生命周期
// ---------------------------------------------------------------------------

/** 探一次 /healthz；不可达或非 200 返回 null */
async function probeBackend(timeoutMs = 800) {
  const abort = new AbortController();
  const timer = setTimeout(() => abort.abort(), timeoutMs);
  try {
    const response = await fetch(`${BACKEND_HTTP}/healthz`, { signal: abort.signal });
    if (!response.ok) return null;
    return await response.json();
  } catch {
    return null;
  } finally {
    clearTimeout(timer);
  }
}

/**
 * 拉起后端并等到 /healthz 就绪。
 *
 * 返回 `{ child, external, health }`；可执行文件缺失时返回 `null`（此段整体判 FAIL）。
 * 若 8090 上已有实例（例如用户手工起的），复用并置 `external=true` ——
 * 这种情况下**不能**做断线重连子项：我们杀不掉别人的进程。
 */
async function startBackend() {
  const existing = await probeBackend();
  if (existing) {
    console.log(
      `[backend] ${BACKEND_HTTP} 已有实例（device=${existing.device}），复用之；跳过断线重连子项`,
    );
    return { child: null, external: true, health: existing };
  }
  if (!existsSync(BACKEND_EXE)) {
    console.log(`[backend] 未找到可执行文件 ${BACKEND_EXE}`);
    return null;
  }

  const child = spawn(BACKEND_EXE, ['-c', 'config.yaml'], { cwd: BACKEND_DIR, stdio: 'ignore' });
  backendChildren.push(child);
  child.on('exit', (code) => console.log(`[backend] 进程退出 code=${code}`));

  const deadline = Date.now() + 15000;
  while (Date.now() < deadline) {
    const health = await probeBackend(1000);
    if (health) {
      console.log(
        `[backend] 就绪 ${BACKEND_HTTP} device=${health.device} linked=${health.linked} state=${JSON.stringify(health.state)}`,
      );
      return { child, external: false, health };
    }
    await sleep(200);
  }
  stopBackend(child);
  return null;
}

function stopBackend(child) {
  if (!child) return;
  try {
    child.kill();
  } catch {
    /* 已退出 */
  }
}

// 兜底：无论正常结束还是断言抛错退出，都不留下占用 8090 的僵尸进程
process.on('exit', () => {
  for (const child of backendChildren) stopBackend(child);
});

/** 极简 CDP 客户端 */
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

  /** 在页面里求值，返回 JSON 化结果 */
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

  /** 截屏，返回 base64 PNG */
  async screenshot() {
    const result = await this.send('Page.captureScreenshot', { format: 'png' });
    return result.data;
  }

  /** 页面错误（控制台 error + 未捕获异常 + 日志 error） */
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
        const entry = event.params.entry;
        errors.push(`log: ${entry.text}${entry.url ? ` <${entry.url}>` : ''}`);
      }
    }
    return errors;
  }
}

// ---------------------------------------------------------------------------
// 页面内取值表达式
// ---------------------------------------------------------------------------

const READ_CHIPS = `Array.from(document.querySelectorAll('.overlay .chip')).map(e => e.textContent.replace(/\\s+/g,' ').trim())`;

/** 解析 "TCP X 111.96 Y 0.0 Z 93.84 mm" */
const PARSE_TCP = `(() => {
  const chip = Array.from(document.querySelectorAll('.overlay .chip')).find(e => e.textContent.includes('TCP'));
  if (!chip) return null;
  const m = chip.textContent.match(/X\\s*(-?[\\d.]+)\\s*Y\\s*(-?[\\d.]+)\\s*Z\\s*(-?[\\d.]+)/);
  return m ? [Number(m[1]), Number(m[2]), Number(m[3])] : null;
})()`;

/** 解析 "FK↔3D 0.00e+0 mm" */
const PARSE_ALIGNMENT = `(() => {
  const chip = Array.from(document.querySelectorAll('.overlay .chip')).find(e => e.textContent.includes('FK'));
  if (!chip) return null;
  const m = chip.textContent.match(/([\\d.]+e[+-]?\\d+|[\\d.]+)\\s*mm/);
  return m ? Number(m[1]) : null;
})()`;

/** 设置第 index 个 range 滑杆的值（走 React 认识的 input 事件） */
const setSlider = (index, value) => `(() => {
  const el = Array.from(document.querySelectorAll('input[type=range]'))[${index}];
  if (!el) return false;
  const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set;
  setter.call(el, '${value}');
  el.dispatchEvent(new Event('input', { bubbles: true }));
  return true;
})()`;

/** 读取状态表里某个关节的 Command 值 */
const readCommandCell = (rowIndex) => `(() => {
  const rows = document.querySelectorAll('.sidebar table.grid tbody tr');
  const row = rows[${rowIndex}];
  if (!row) return null;
  return row.children[1].textContent.trim();
})()`;

const CLICK_HOME = `(() => {
  const btn = Array.from(document.querySelectorAll('button')).find(b => b.textContent.trim() === 'HOME');
  if (!btn) return false;
  btn.click();
  return true;
})()`;

const READY_PROBE = `(() => {
  const overlay = document.querySelector('.overlay');
  const canvas = document.querySelector('.viewport canvas');
  return Boolean(overlay && canvas) && document.querySelectorAll('.overlay .chip').length >= 3;
})()`;

// ---- Phase 6：末端目标与拖动 ----

/** 写入第 index 个目标输入框（0=X / 1=Y / 2=Z），走 React 认识的 input 事件 */
const setTargetInput = (index, value) => `(() => {
  const el = Array.from(document.querySelectorAll('.target-row input[type=number]'))[${index}];
  if (!el) return false;
  const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set;
  setter.call(el, '${value}');
  el.dispatchEvent(new Event('input', { bubbles: true }));
  return true;
})()`;

/** 点击末端目标面板的 Move 按钮 */
const CLICK_MOVE = `(() => {
  const card = Array.from(document.querySelectorAll('.card')).find(c => c.textContent.includes('Target XYZ'));
  const btn = card ? Array.from(card.querySelectorAll('button')).find(b => b.textContent.trim() === 'Move') : null;
  if (!btn) return false;
  btn.click();
  return true;
})()`;

/** 读 IK 状态文本（成功分支 / 失败错误码） */
const READ_IK_STATUS = `(() => {
  const el = document.querySelector('.ik-status');
  return el ? el.textContent.replace(/\\s+/g, ' ').trim() : null;
})()`;

/** dev 探针：精确状态快照（TCP 未做小数截断，可用于 1e-6 级断言） */
const PROBE_STATE = `(() => {
  const p = window.__armPilot;
  if (!p) return null;
  const s = p.state();
  return {
    target: s.target,
    tcp: s.tcp,
    dragging: s.dragging,
    ok: s.ikStatus ? s.ikStatus.ok : null,
    reason: s.ikStatus ? s.ikStatus.reason : null,
  };
})()`;

/** dev 探针：末端 TCP 的屏幕像素坐标（= 拖动把手中心） */
const PROBE_TCP_SCREEN = `(() => { const p = window.__armPilot; return p ? p.tcpScreen() : null; })()`;

// ---------------------------------------------------------------------------
// Phase 12：实际臂幽灵（ADR D45）
// ---------------------------------------------------------------------------

/**
 * 幽灵**渲染后**的 TCP 世界坐标。
 *
 * 刻意取矩阵而不是再算一遍 FK：本功能的全部意义是"实际臂真的被画出来了"。
 * 若 e2e 用 FK(actualJoints) 当证据，那只是把数学又算了一遍 ——
 * 渲染树挂错、可见性被误关、材质全透明都会让画面空掉而断言依然全绿。
 */
const PROBE_GHOST = `(() => {
  const g = window.__armPilotGhost;
  return g && Array.isArray(g.tcp) ? { tcp: g.tcp, visible: g.visible } : null;
})()`;

/**
 * 幽灵 TCP 与「store 里 actualEndEffector（= FK of actualJoints）」的距离（mm）。
 *
 * 这是**时间无关**的不变量：无论链路是在滞后还是已收敛，幽灵都必须站在 actual 上。
 * 两条计算路径互相独立（three.js 对象图世界矩阵 vs 纯数学 FK），故不构成自证。
 * 若把幽灵接成 command，滞后期间这个值会等于滞后量（几十 mm），一眼可辨。
 */
const GHOST_GAP_TO_ACTUAL_MM = `(() => {
  const g = window.__armPilotGhost;
  const p = window.__armPilot;
  if (!g || !p || !Array.isArray(g.tcp)) return null;
  const a = p.state().actualTcp;
  return Math.hypot(g.tcp[0] - a[0], g.tcp[1] - a[1], g.tcp[2] - a[2]);
})()`;

/** 视口显示开关里是否存在「Actual Arm」勾选框 */
const HAS_ACTUAL_ARM_TOGGLE = `(() => {
  const labels = Array.from(document.querySelectorAll('.overlay .axis-row label.axis'));
  return labels.some(l => l.textContent.includes('Actual Arm'));
})()`;

/** 点一下「Actual Arm」勾选框（无条件下切换） */
const TOGGLE_ACTUAL_ARM = `(() => {
  const label = Array.from(document.querySelectorAll('.overlay .axis-row label.axis'))
    .find(l => l.textContent.includes('Actual Arm'));
  const input = label ? label.querySelector('input[type=checkbox]') : null;
  if (!input) return false;
  input.click();
  return true;
})()`;

/**
 * 同一次求值里同时取「J2 关节滞后」与「幽灵 ↔ 主臂分离量」。
 *
 * 为什么必须合并成一次求值：Mock 的 34° 跳变在约 150ms 内就收敛完了，
 * 分两次 CDP 往返（每次十几毫秒 + 中间还有别的 evaluate）会让第二次读到时
 * 误差已经归零 —— 那样断言会变成**间歇性**失败，而不是稳定复现。
 */
const READ_LAG_WITH_GHOST = `(() => {
  const rows = document.querySelectorAll('[data-testid="err-rows"] .err-row');
  const row = rows[1];
  const errEl = row ? row.querySelector('.err-val') : null;
  const p = window.__armPilot;
  const g = window.__armPilotGhost;
  let ghostGap = null;
  if (p && g && Array.isArray(g.tcp)) {
    const c = p.state().tcp;
    ghostGap = Math.hypot(g.tcp[0] - c[0], g.tcp[1] - c[1], g.tcp[2] - c[2]);
  }
  return {
    errDeg: errEl ? parseFloat(errEl.textContent.replace(/[+°]/g, '')) : null,
    ghostGap,
  };
})()`;

/** 主视口 canvas 的屏幕矩形 */
const CANVAS_RECT = `(() => {
  const c = document.querySelector('.viewport canvas');
  if (!c) return null;
  const r = c.getBoundingClientRect();
  return { left: r.left, top: r.top, width: r.width, height: r.height };
})()`;

// ---- Phase 7：MockTransport 闭环 ----

/** 按卡片标题定位，避免与 StatusPanel 的 .badge / .grid 混淆 */
const cardByTitle = (title) =>
  `Array.from(document.querySelectorAll('.card')).find(c => c.textContent.includes('${title}'))`;

const CLICK_CONNECT = `(() => {
  const card = ${cardByTitle('连接 · Transport')};
  const btn = card ? Array.from(card.querySelectorAll('button')).find(b => b.textContent.trim() === 'Connect Mock') : null;
  if (!btn) return false;
  btn.click();
  return true;
})()`;

const CLICK_DISCONNECT = `(() => {
  const card = ${cardByTitle('连接 · Transport')};
  const btn = card ? Array.from(card.querySelectorAll('button')).find(b => b.textContent.trim() === 'Disconnect') : null;
  if (!btn) return false;
  btn.click();
  return true;
})()`;

/** 连接面板里的 Connection 徽标文本 */
const READ_CONNECTION = `(() => {
  const card = ${cardByTitle('连接 · Transport')};
  const badge = card ? card.querySelector('.badge') : null;
  return badge ? badge.textContent.replace(/\\s+/g,' ').trim() : null;
})()`;

/** 日志区全文（用于确认 SEND / RECV 真的发生了） */
const READ_LOG = `(() => {
  const el = document.querySelector('.log');
  return el ? el.textContent.replace(/\\s+/g, ' ').trim() : '';
})()`;

/** 关节滑杆：限定在 Joint Control 卡片内，不受 Connection 面板新增滑杆的影响 */
const setJointSlider = (index, value) => `(() => {
  const card = ${cardByTitle('Joint Control')};
  const el = card ? card.querySelectorAll('input[type=range]')[${index}] : null;
  if (!el) return false;
  const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set;
  setter.call(el, '${value}');
  el.dispatchEvent(new Event('input', { bubbles: true }));
  return true;
})()`;

/** 状态表某行的 Command / Actual / 差值（表格显示保留 1 位小数） */
const readJointGap = (rowIndex) => `(() => {
  const card = ${cardByTitle('状态 · Status')};
  if (!card) return null;
  const row = card.querySelectorAll('table.grid tbody tr')[${rowIndex}];
  if (!row) return null;
  const command = parseFloat(row.children[1].textContent);
  const actual = parseFloat(row.children[2].textContent);
  if (!Number.isFinite(command) || !Number.isFinite(actual)) return null;
  return { command, actual, gap: Math.abs(actual - command) };
})()`;

// ---- Phase 8：真实后端（WebSocket）----

/** 选中「传输方式 = WebSocket」单选 */
const CLICK_WS_MODE = `(() => {
  const card = ${cardByTitle('连接 · Transport')};
  const radio = card ? card.querySelector('input[type=radio][value="websocket"]') : null;
  if (!radio) return false;
  radio.click();
  return true;
})()`;

/**
 * 切回 Mock 模式。
 *
 * 为什么必须有：连接面板的模式是 radio，**会跨 e2e 批次残留**（页面不刷新）。
 * 上一轮跑完停在 websocket 模式时，"Connect Mock" 按钮根本不存在，
 * `CLICK_CONNECT` 会静默返回 false → 状态灯停在 Disconnected → 假红。
 * 这类"依赖上一轮终态"的失败极难复现（单跑一次可能是绿的），必须在段前显式归位。
 */
const CLICK_MOCK_MODE = `(() => {
  const card = ${cardByTitle('连接 · Transport')};
  const radio = card ? card.querySelector('input[type=radio][value="mock"]') : null;
  if (!radio) return false;
  radio.click();
  return true;
})()`;

/** 写入后端地址（走 React 认识的 input 事件） */
const setWsUrl = (url) => `(() => {
  const el = document.querySelector('[data-testid="ws-url"]');
  if (!el) return false;
  const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set;
  setter.call(el, ${JSON.stringify(url)});
  el.dispatchEvent(new Event('input', { bubbles: true }));
  return true;
})()`;

const CLICK_CONNECT_WS = `(() => {
  const card = ${cardByTitle('连接 · Transport')};
  const btn = card ? Array.from(card.querySelectorAll('button')).find(b => b.textContent.trim() === 'Connect WS') : null;
  if (!btn) return false;
  btn.click();
  return true;
})()`;

/** 连接面板统计表里按行名取值（如「链路末端」「重连次数」） */
const readStatRow = (label) => `(() => {
  const card = ${cardByTitle('连接 · Transport')};
  if (!card) return null;
  const row = Array.from(card.querySelectorAll('table.grid tbody tr')).find(r => r.children[0].textContent.includes('${label}'));
  return row ? row.children[1].textContent.replace(/\\s+/g,' ').trim() : null;
})()`;

const READ_DEVICE = readStatRow('链路末端');

// ---------------------------------------------------------------------------
// Phase 11：链路误差反馈面板（ADR D44）
// ---------------------------------------------------------------------------

/**
 * 健康结论文本。判据是"面板说的是不是与链路事实一致"：
 * 滞后期间不得说"已到位"，收敛后必须说"已到位"，断开后必须说"未接入传输"。
 * 这三态互斥，任何一态错位都说明判定逻辑与真实链路脱钩了。
 */
const READ_LINK_HEALTH = `(() => {
  const el = document.querySelector('[data-testid="link-health"]');
  return el ? el.textContent.replace(/\\s+/g, ' ').trim() : null;
})()`;

/** TCP 位置误差（mm，数值） */
const READ_TCP_ERROR = `(() => {
  const el = document.querySelector('[data-testid="tcp-error"]');
  return el ? parseFloat(el.textContent) : null;
})()`;

/** 逐关节误差（带符号，deg）。all-zero 即 Actual ≡ Command */
const READ_ERR_VALUES = `(() => {
  const rows = document.querySelectorAll('[data-testid="err-rows"] .err-row .err-val');
  if (!rows.length) return null;
  return Array.from(rows).map(r => parseFloat(r.textContent.replace(/[+°]/g, '')));
})()`;

/** 偏差条里"超差"（> ERROR_TOL_DEG）的条数 —— 用 DOM class 而非重算，确保 UI 真的在标红 */
const COUNT_OVER_BARS = `document.querySelectorAll('[data-testid="err-rows"] .errbar .fill.over').length`;

// ---------------------------------------------------------------------------
// Phase 13：示教录制 / 回放
// ---------------------------------------------------------------------------

/** 示教面板是否齐备（面板 + 全部按钮 + 帧数读数） */
const HAS_TEACH_PANEL = `(() => {
  const card = ${cardByTitle('示教 · Teach')};
  if (!card) return false;
  const ids = ['teach-record','teach-play','teach-pause','teach-stop','teach-clear','teach-export','teach-import','teach-frames','teach-status','teach-frames-list'];
  return ids.every(id => card.querySelector('[data-testid="' + id + '"]') !== null);
})()`;

/** 点某个示教按钮（返回 false = 按钮不存在；按钮被禁用时 click 无副作用） */
const clickTeach = (id) => `(() => {
  const el = document.querySelector('[data-testid="${id}"]');
  if (!el) return false;
  el.click();
  return true;
})()`;

/** 导出按钮的禁用状态（空轨迹时必须禁用，否则是个能点但没用的死操作） */
const TEACH_EXPORT_DISABLED = `(() => {
  const b = document.querySelector('[data-testid="teach-export"]');
  return b ? b.disabled : null;
})()`;

/**
 * 示教快照。`lastJoints` 来自 dev 探针（store 里的原始数值），
 * 而不是面板里 1 位小数的文本 —— "回放终点逐值等于录制末帧"这条断言
 * 必须是 1e-9 级判定，四舍五入过的文本做不了。
 */
const READ_TEACH = `(() => {
  const p = window.__armPilot;
  if (!p) return null;
  return p.state().teach;
})()`;

/** 同一次求值里取「当前命令」「轨迹末帧」「播放状态文本」—— 三者在时间上必须同帧 */
const READ_TEACH_TERMINAL = `(() => {
  const p = window.__armPilot;
  if (!p) return null;
  const s = p.state();
  const status = document.querySelector('[data-testid="teach-status"]');
  return {
    cmd: s.commandJoints,
    last: s.teach.lastJoints,
    frames: s.teach.frames,
    durationMs: s.teach.durationMs,
    recording: s.teach.recording,
    statusText: status ? status.textContent.replace(/\\s+/g,' ').trim() : null,
  };
})()`;

/** 帧数读数（DOM 层，确认面板真的在刷新而不只是 store 变了） */
const READ_TEACH_FRAME_COUNT = `(() => {
  const el = document.querySelector('[data-testid="teach-frames"]');
  return el ? parseInt(el.textContent, 10) : null;
})()`;


// ---------------------------------------------------------------------------
// mode ↔ transport 联动（Phase 9 修正 · ADR D41）
// ---------------------------------------------------------------------------

/** 关节控制卡片里的「命令去向」提示文本 */
const READ_MODE_ROUTING = `(() => {
  const el = document.querySelector('[data-testid="mode-routing"]');
  return el ? el.textContent.replace(/\\s+/g, ' ').trim() : null;
})()`;

/** 点关节控制卡片里的 Real Robot / Simulation 按钮 */
const clickModeButton = (label) => `(() => {
  const card = ${cardByTitle('关节控制 · Joint Control')};
  const btn = card
    ? Array.from(card.querySelectorAll('.btn-row button')).find(b => b.textContent.trim() === '${label}')
    : null;
  if (!btn) return false;
  btn.click();
  return true;
})()`;

/** 日志区是否包含某段文本 */
const LOG_CONTAINS = (needle) => `(() => {
  const el = document.querySelector('.log');
  return el ? el.textContent.includes(${JSON.stringify(needle)}) : false;
})()`;

/** 日志区最后一段文本（用于断言"最新的那条说了什么"） */
const READ_LOG_TAIL = `(() => {
  const el = document.querySelector('.log');
  if (!el) return null;
  return el.textContent.replace(/\\s+/g, ' ').trim().slice(-400);
})()`;

/** 日志条目数（用于"只看点击之后新增的日志"，避免读到历史同类文本造成假绿） */
const READ_LOG_LENGTH = `(() => {
  const el = document.querySelector('.log');
  if (!el) return null;
  return el.children.length;
})()`;

/**
 * Real / Simulation 两个按钮的 active 状态。
 *
 * 这是断言"拒绝切换"的关键：修复前点 Real Robot 会让 realActive 变 true
 * （mode 被改掉），修复后必须仍停在 Simulation。
 */
const READ_MODE_BUTTONS = `(() => {
  const card = ${cardByTitle('关节控制 · Joint Control')};
  if (!card) return null;
  const btns = Array.from(card.querySelectorAll('.btn-row button'));
  const sim = btns.find(b => b.textContent.trim() === 'Simulation');
  const real = btns.find(b => b.textContent.trim() === 'Real Robot');
  if (!sim || !real) return null;
  return { simActive: sim.classList.contains('active'), realActive: real.classList.contains('active') };
})()`;

/** 点 Real Robot / Simulation（复用 clickModeButton 工厂） */
const CLICK_REAL_ROBOT = clickModeButton('Real Robot');
const CLICK_SIMULATION = clickModeButton('Simulation');

/** 重连次数（数字） */
const READ_RECONNECTS = `(() => {
  const raw = ${readStatRow('重连次数')};
  if (raw === null) return null;
  const n = Number.parseInt(raw, 10);
  return Number.isFinite(n) ? n : null;
})()`;

/** 心跳 RTT（ms）；null 表示尚未收到过 pong（界面显示 "—"） */
const READ_RTT = `(() => {
  const el = document.querySelector('[data-testid="stat-rtt"]');
  if (!el) return null;
  const m = el.textContent.match(/(\\d+)\\s*ms/);
  return m ? Number(m[1]) : null;
})()`;

/** WebSocket 专有统计行是否已渲染（心跳 RTT / 重连次数 / 链路末端） */
const READ_WS_ROWS = `(() => {
  const card = ${cardByTitle('连接 · Transport')};
  if (!card) return false;
  const text = card.textContent;
  return text.includes('心跳 RTT') && text.includes('重连次数') && text.includes('链路末端');
})()`;

/** 前后端模型一致性告警文本（null = 无告警） */
const READ_MODEL_WARN = `(() => {
  const card = ${cardByTitle('连接 · Transport')};
  if (!card) return null;
  const note = Array.from(card.querySelectorAll('.notes')).find(n => n.textContent.includes('模型/标定不一致'));
  return note ? note.textContent.replace(/\\s+/g,' ').trim() : null;
})()`;

/**
 * 点 Disconnect 并读回状态灯 —— **点击与读取放在同一次求值里，且可重复调用**。
 *
 * 为什么不能"先点一次、再去 poll 状态"：按钮在 `busy` 或**未连接**时是 `disabled`，
 * 而重连过程中连接态会短暂进入 `connecting`（界面映射为 "Disconnected"）。
 * 若那一次点击正好落在这个窗口里，`disabled` 按钮的 `click()` **不派发事件**、
 * 被静默吞掉，于是断言失败但真实原因是"没点到"而不是"断不开"。
 * 放在同一表达式里重试即可消除这个窗口（真断不开时依然会失败 —— 不掩盖问题）。
 */
const DISCONNECT_AND_READ = `(() => {
  const card = ${cardByTitle('连接 · Transport')};
  if (!card) return null;
  const btn = Array.from(card.querySelectorAll('button')).find(b => b.textContent.trim() === 'Disconnect');
  if (btn && !btn.disabled) btn.click();
  const badge = card.querySelector('.badge');
  return {
    badge: badge ? badge.textContent.replace(/\\s+/g,' ').trim() : null,
    disabled: btn ? btn.disabled : null,
  };
})()`;

// ---------------------------------------------------------------------------
// 主流程
// ---------------------------------------------------------------------------

async function main() {
  const browser = findBrowser();
  if (!browser) throw new Error('未找到 Edge / Chrome 可执行文件，请设置 EDGE_PATH');
  console.log(`浏览器: ${browser}`);
  console.log(`目标  : ${TARGET_URL}`);

  const userDataDir = mkdtempSync(path.join(tmpdir(), 'armpilot-e2e-'));
  const child = spawn(browser, [...CHROME_FLAGS, `--user-data-dir=${userDataDir}`, `--remote-debugging-port=${DEBUG_PORT}`, TARGET_URL], {
    stdio: 'ignore',
    detached: false,
  });

  let cdp;
  try {
    const page = await waitForDebuggerEndpoint(DEBUG_PORT);
    const socket = new WebSocket(page.webSocketDebuggerUrl);
    await new Promise((resolve, reject) => {
      socket.addEventListener('open', resolve, { once: true });
      socket.addEventListener('error', () => reject(new Error('WebSocket 连接失败')), { once: true });
    });
    cdp = new Cdp(socket);

    await cdp.send('Runtime.enable');
    await cdp.send('Log.enable');
    await cdp.send('Page.enable');

    // 1. 等待 React + R3F 挂载完成
    let ready = false;
    for (let i = 0; i < 60; i += 1) {
      try {
        ready = await cdp.evaluate(READY_PROBE);
      } catch {
        ready = false;
      }
      if (ready) break;
      await sleep(500);
    }
    check('页面渲染完成（canvas + 读数面板）', ready === true);

    if (!ready) throw new Error('页面未在 30s 内完成渲染');

    // 2. 初始读数（FK 一致性由场景按 250ms 节流回写，需等它出现）
    const initialTcp = await cdp.evaluate(PARSE_TCP);
    // 轮询到"收敛"而不是"出现数字"：读数被 250ms 节流回写，且首个非空值可能是
    // 关节状态尚未应用时的零位瞬态。超时仍不达标时下面的断言照样会失败（不掩盖问题）。
    const initialAlignment = await poll(
      cdp,
      PARSE_ALIGNMENT,
      (v) => typeof v === 'number' && v < 0.1,
    );
    // HOME 解析解（2026-09-12 实拍反解）：
    //   x = 80·sin(0.849894°) + 120·sin(112.618577°) ≈ 111.96
    //   z = 60 + 80·cos(0.849894°) + 120·cos(112.618577°) ≈ 93.84
    // 小臂存**绝对角**（平行四连杆解耦），所以不再叠加肩角。
    check(
      '初始 TCP 读数与解析解一致（X 111.96 / Y 0.0 / Z 93.84）',
      initialTcp !== null &&
        Math.abs(initialTcp[0] - 111.96) < 0.2 &&
        Math.abs(initialTcp[1] - 0) < 0.2 &&
        Math.abs(initialTcp[2] - 93.84) < 0.2,
      JSON.stringify(initialTcp),
    );
    check(
      '初始 FK↔Three.js 一致性 < 0.1mm',
      initialAlignment !== null && initialAlignment < 0.1,
      `${initialAlignment} mm`,
    );

    const chips = await cdp.evaluate(READ_CHIPS);
    check('外层读数含 TCP / Mode / FK 三项', Array.isArray(chips) && chips.length >= 3, `${chips?.length} chips`);

    // 3. 拖动 J2 (shoulder) 滑杆到 40
    const movedJ2 = await cdp.evaluate(setSlider(1, 40));
    await sleep(400);
    const tcpAfterJ2 = await cdp.evaluate(PARSE_TCP);
    const alignAfterJ2 = await cdp.evaluate(PARSE_ALIGNMENT);
    const cmdJ2 = await cdp.evaluate(readCommandCell(1));

    check('J2 滑杆可交互（input 事件生效）', movedJ2 === true);
    check(
      'J2=40° 后 TCP 随之变化且高度下降',
      tcpAfterJ2 !== null && tcpAfterJ2[2] < initialTcp[2] - 5,
      `Z ${initialTcp[2]} -> ${tcpAfterJ2?.[2]}`,
    );
    // 滑杆量程 = 关节限位（现在是 -6.09..49.45），步进量化后不一定精确落在 40.0
    const cmdJ2Value = Number.parseFloat(String(cmdJ2));
    check(
      '状态表 Command 列跟随滑杆（≈40°）',
      Number.isFinite(cmdJ2Value) && Math.abs(cmdJ2Value - 40) < 1,
      String(cmdJ2),
    );
    check(
      '**改姿态后 FK↔Three.js 仍 < 0.1mm**',
      alignAfterJ2 !== null && alignAfterJ2 < 0.1,
      `${alignAfterJ2} mm`,
    );

    // 4. 拖动 J3 (elbow) 滑杆到 60
    await cdp.evaluate(setSlider(2, 60));
    await sleep(400);
    const tcpAfterJ3 = await cdp.evaluate(PARSE_TCP);
    const alignAfterJ3 = await cdp.evaluate(PARSE_ALIGNMENT);
    check(
      'J3=60° 后 TCP 再次变化且 FK↔3D 仍 < 0.1mm',
      tcpAfterJ3 !== null &&
        Math.abs(tcpAfterJ3[2] - tcpAfterJ2[2]) > 1 &&
        alignAfterJ3 !== null &&
        alignAfterJ3 < 0.1,
      `Z ${tcpAfterJ2?.[2]} -> ${tcpAfterJ3?.[2]} / align ${alignAfterJ3} mm`,
    );

    // 5. 夹爪滑杆不改变末端定位
    await cdp.evaluate(setSlider(3, 90));
    await sleep(350);
    const tcpAfterGripper = await cdp.evaluate(PARSE_TCP);
    check(
      '夹爪开合不改变末端 TCP（IK 只解 J1/J2/J3）',
      tcpAfterGripper !== null &&
        Math.abs(tcpAfterGripper[2] - tcpAfterJ3[2]) < 0.05 &&
        Math.abs(tcpAfterGripper[0] - tcpAfterJ3[0]) < 0.05,
      JSON.stringify(tcpAfterGripper),
    );

    // 6. HOME 复位
    await cdp.evaluate(CLICK_HOME);
    await sleep(400);
    const tcpAfterHome = await cdp.evaluate(PARSE_TCP);
    check(
      'HOME 按钮恢复到 HOME 位姿读数',
      tcpAfterHome !== null &&
        Math.abs(tcpAfterHome[2] - 93.84) < 0.2 &&
        Math.abs(tcpAfterHome[0] - initialTcp[0]) < 0.2,
      JSON.stringify(tcpAfterHome),
    );

    // 6c. Phase 6：末端目标（XYZ 直输）+ 越界不钳位 + 真实鼠标拖动
    const probeReady = await cdp.evaluate(`Boolean(window.__armPilot && window.__armPilot.tcpScreen)`);
    check('dev 测试探针可用（供 e2e 精确命中拖动把手）', probeReady === true);

    // (a) XYZ 直输：X 111.96 → 135，Y=0，Z 保持 93.84（该点经机构约束验算可达）
    await cdp.evaluate(setTargetInput(0, '135'));
    await cdp.evaluate(setTargetInput(1, '0'));
    const clicked = await cdp.evaluate(CLICK_MOVE);
    await sleep(400);
    const tcpAfterMove = await cdp.evaluate(PARSE_TCP);
    const ikOk = await cdp.evaluate(READ_IK_STATUS);
    check('末端目标面板 Move 按钮可点击', clicked === true);
    check(
      'XYZ 直输 → 末端跟随目标（X ≈ 135）',
      tcpAfterMove !== null && Math.abs(tcpAfterMove[0] - 135) < 0.5,
      JSON.stringify(tcpAfterMove),
    );
    check(
      'IK 状态栏显示求解成功与分支',
      typeof ikOk === 'string' && ikOk.includes('elbow-up'),
      String(ikOk),
    );

    // (b) 目标越界：关节必须**纹丝不动**（不静默钳位）
    await cdp.evaluate(setTargetInput(0, '600'));
    await cdp.evaluate(CLICK_MOVE);
    await sleep(300);
    const tcpAfterBad = await cdp.evaluate(PARSE_TCP);
    const ikBad = await cdp.evaluate(READ_IK_STATUS);
    check(
      '越界目标 → 末端纹丝不动（关节不静默钳位）',
      tcpAfterBad !== null &&
        tcpAfterMove !== null &&
        Math.abs(tcpAfterBad[0] - tcpAfterMove[0]) < 0.05 &&
        Math.abs(tcpAfterBad[2] - tcpAfterMove[2]) < 0.05,
      JSON.stringify(tcpAfterBad),
    );
    check(
      '越界目标 → 状态栏报 OUT_OF_WORKSPACE',
      typeof ikBad === 'string' && ikBad.includes('OUT_OF_WORKSPACE'),
      String(ikBad),
    );

    // (c) 真实鼠标拖拽：只有走浏览器输入管线才能命中 R3F 场景里的把手
    await cdp.evaluate(CLICK_HOME);
    await sleep(400);
    const beforeState = await cdp.evaluate(PROBE_STATE);
    const dragStart = await cdp.evaluate(PROBE_TCP_SCREEN);
    const canvasRect = await cdp.evaluate(CANVAS_RECT);

    check(
      '探针给出把手的屏幕坐标且落在 canvas 内',
      dragStart !== null &&
        canvasRect !== null &&
        Number.isFinite(dragStart.x) &&
        dragStart.x > canvasRect.left &&
        dragStart.x < canvasRect.left + canvasRect.width,
      JSON.stringify(dragStart),
    );

    let dragState = null;
    if (dragStart && canvasRect) {
      // 向右拖 4% 视口宽：由 HOME 出发该方向落在可达区内（已用机构约束验算）
      await dragMouse(cdp, dragStart, {
        x: dragStart.x + canvasRect.width * 0.04,
        y: dragStart.y,
      });
      await sleep(400);
      dragState = await cdp.evaluate(PROBE_STATE);
    }

    const movedBy =
      dragState && beforeState
        ? Math.hypot(dragState.tcp[0] - beforeState.tcp[0], dragState.tcp[1] - beforeState.tcp[1])
        : 0;
    check('真实鼠标拖拽把手 → 末端跟随移动', movedBy > 2, `Δ ${movedBy.toFixed(2)} mm`);
    check(
      '拖动在水平面内进行：目标 Z 严格锁定在按下瞬间的 TCP Z',
      dragState !== null &&
        beforeState !== null &&
        Math.abs(dragState.target[2] - beforeState.tcp[2]) < 1e-9,
      `target Z ${dragState?.target?.[2]} vs tcp Z ${beforeState?.tcp?.[2]}`,
    );
    check(
      '拖拽结束后退出拖动状态（OrbitControls 已解锁）',
      dragState !== null && dragState.dragging === false,
    );

    // (d) 复位，保证后续截图与文档基线一致
    await cdp.evaluate(CLICK_HOME);
    await sleep(400);

    // ---- Phase 7：MockTransport 闭环 ----
    // 先显式切回 Mock 模式：连接面板的模式 radio 会跨批次残留，
    // 若上一轮停在 websocket，"Connect Mock" 按钮不存在 ⇒ CLICK_CONNECT 静默失效。
    await cdp.evaluate(CLICK_MOCK_MODE);
    await sleep(200);

    // (a) 连接：状态灯必须变 Connected
    await cdp.evaluate(CLICK_CONNECT);
    const connected = await poll(cdp, READ_CONNECTION, (v) => v === 'Connected', 8000);
    check('连接 MockTransport → 状态灯变 Connected', connected === 'Connected', String(connected));

    // (b) 瞬间跳到 40°：立刻读，Actual 还没跟上。
    //     这一条是「Mock 不是零延迟等值回显」的直接证据 —— 若把 Mock 做成理想回显，这里恒为 0。
    await cdp.evaluate(setJointSlider(1, 40));
    const immediate = await cdp.evaluate(readJointGap(1));
    check(
      '命令送达前 Actual 明显滞后（Mock 模拟延迟 + 有限角速度）',
      immediate !== null && immediate.gap > 1,
      immediate ? `cmd ${immediate.command}° / act ${immediate.actual}° / 差 ${immediate.gap.toFixed(1)}°` : 'null',
    );

    // (b1) Phase 12：滞后瞬间，实际臂幽灵必须已渲染**且与主臂分离** —— 分离量就是滞后量。
    //      与 (b) 的滞后读数取在同一次求值里：Mock 的 34° 跳合约 150ms 就收敛完，
    //      分两次往返会让第二次读到归零值，断言就变成间歇性失败而非稳定复现。
    const lagGhost = await cdp.evaluate(READ_LAG_WITH_GHOST);
    check(
      'Phase 12：滞后瞬间实际臂幽灵已渲染并与主臂分离（分离量=滞后量）',
      lagGhost !== null && lagGhost.ghostGap !== null && lagGhost.ghostGap > 0.5,
      lagGhost && lagGhost.ghostGap !== null
        ? `分离 ${Number(lagGhost.ghostGap).toFixed(1)} mm / 误差 ${lagGhost.errDeg}°`
        : 'null',
    );

    // (b2) Phase 11：滞后期间误差面板必须与链路事实一致 —— 不得说"已到位"
    const errsLag = await cdp.evaluate(READ_ERR_VALUES);
    check(
      'Phase 11：误差面板逐关节列出偏差（可动关节数一致）',
      Array.isArray(errsLag) && errsLag.length === 4 && errsLag.every((v) => Number.isFinite(v)),
      Array.isArray(errsLag) ? `${errsLag.length} 行` : 'null',
    );
    const healthLag = await cdp.evaluate(READ_LINK_HEALTH);
    check(
      'Phase 11：滞后期间健康结论**不是**"已到位"（滞后与到位必须分开）',
      typeof healthLag === 'string' && !healthLag.includes('已到位'),
      String(healthLag),
    );

    // (c) 松手后按有限角速度收敛到命令值
    const converged = await poll(cdp, readJointGap(1), (v) => v !== null && v.gap <= 0.05, 10000);
    check(
      '松手后 Actual 收敛到 Command（误差归零）',
      converged !== null && converged.gap <= 0.05,
      converged ? `cmd ${converged.command}° / act ${converged.actual}°` : 'null',
    );

    // (c2) Phase 11：收敛后必须说"已到位"，TCP 误差归零，且偏差条不再标红
    const healthSettled = await poll(
      cdp,
      READ_LINK_HEALTH,
      (v) => typeof v === 'string' && v.includes('已到位'),
      4000,
    );
    check(
      'Phase 11：收敛后健康结论变为"已到位"',
      typeof healthSettled === 'string' && healthSettled.includes('已到位'),
      String(healthSettled),
    );
    const tcpSettled = await cdp.evaluate(READ_TCP_ERROR);
    check(
      'Phase 11：TCP 位置误差归零（Actual ≡ Command ⇒ ≈0 mm）',
      typeof tcpSettled === 'number' && tcpSettled <= 0.05,
      String(tcpSettled),
    );
    const overBars = await cdp.evaluate(COUNT_OVER_BARS);
    check('Phase 11：收敛后无超差偏差条（DOM 层确认真在标红）', overBars === 0, String(overBars));

    // (c3) Phase 12：幽灵必须**始终**站在 actual 上 —— 这是时间无关的不变量，
    //      收敛与否都成立；若幽灵被接成 command，滞后期间该值会等于滞后量。
    const ghostVsActual = await cdp.evaluate(GHOST_GAP_TO_ACTUAL_MM);
    check(
      'Phase 12：幽灵逐值跟随 actualEndEffector（非 command）—— 渲染路径与纯数学 FK 互证',
      typeof ghostVsActual === 'number' && ghostVsActual <= 0.01,
      String(ghostVsActual),
    );
    const ghostShown = await cdp.evaluate(PROBE_GHOST);
    check(
      'Phase 12：幽灵确已渲染且可见',
      ghostShown !== null && ghostShown.visible === true,
      ghostShown ? 'visible' : 'null',
    );
    const hasGhostToggle = await cdp.evaluate(HAS_ACTUAL_ARM_TOGGLE);
    check('Phase 12：视口提供 Actual Arm 显示开关', hasGhostToggle === true, String(hasGhostToggle));

    // 开关必须真的接管渲染，而不是只改了一个布尔值（关掉后对象树仍在画就白搭）
    const clickedOff = await cdp.evaluate(TOGGLE_ACTUAL_ARM);
    const ghostGone = await poll(cdp, PROBE_GHOST, (v) => v === null, 2000);
    check(
      'Phase 12：关闭 Actual Arm 后幽灵确实停止渲染',
      clickedOff === true && ghostGone === null,
      String(ghostGone),
    );
    await cdp.evaluate(TOGGLE_ACTUAL_ARM);
    const ghostBack = await poll(cdp, PROBE_GHOST, (v) => v !== null, 2000);
    check('Phase 12：重新打开后幽灵恢复渲染', ghostBack !== null, ghostBack ? 'visible' : 'null');

    // ---- Phase 13：示教录制 / 回放 ----
    //
    // 这一段刻意放在「连着 Mock 链路」的语境里：回放命令要真的经过节流下发
    // （下面的日志断言会验证这一点）。若为回放另开直发通道，这条链就断了。
    check(
      'Phase 13：示教面板齐备（录制 / 回放 / 清空 / 导入导出）',
      (await cdp.evaluate(HAS_TEACH_PANEL)) === true,
    );
    check(
      'Phase 13：空轨迹时导出按钮禁用（不留"能点但没用"的死操作）',
      (await cdp.evaluate(TEACH_EXPORT_DISABLED)) === true,
    );

    // (a) 录制：先进入录制态，再连摆三个姿态（间隔 > 50ms 采样节流）
    check('Phase 13：可进入录制态', (await cdp.evaluate(clickTeach('teach-record'))) === true);
    await sleep(120);
    const recOn = await cdp.evaluate(READ_TEACH);
    check(
      'Phase 13：录制态已置位（读探针，不只看按钮高亮）',
      recOn !== null && recOn.recording === true,
      recOn === null ? 'null' : `recording=${recOn.recording}`,
    );

    for (const v of [8, 28, 46]) {
      await cdp.evaluate(setJointSlider(1, v));
      await sleep(130);
    }

    check('Phase 13：可退出录制态', (await cdp.evaluate(clickTeach('teach-record'))) === true);
    await sleep(150);
    const rec = await cdp.evaluate(READ_TEACH_TERMINAL);
    check(
      'Phase 13：轨迹记录了多帧且时长 > 0',
      rec !== null && rec.frames >= 3 && rec.durationMs > 0,
      rec === null ? 'null' : `${rec.frames} 帧 / ${rec.durationMs} ms`,
    );
    check(
      'Phase 13：退出录制后录制态复位',
      rec !== null && rec.recording === false,
      rec === null ? 'null' : String(rec.recording),
    );
    check(
      'Phase 13：末帧姿态已捕获（回放终点判定的基准）',
      rec !== null && rec.last !== null && Number.isFinite(rec.last.shoulder),
      rec === null || rec.last === null ? 'null' : `shoulder ${rec.last.shoulder}°`,
    );
    // 面板读数必须与 store 一致 —— 否则"UI 显示录上了"仍是幻觉
    check(
      'Phase 13：面板帧数读数与轨迹一致',
      (await cdp.evaluate(READ_TEACH_FRAME_COUNT)) === rec?.frames,
      `dom=${await cdp.evaluate(READ_TEACH_FRAME_COUNT)} / store=${rec?.frames}`,
    );

    // (b) 回放：先把臂挪到别处，否则"回放到位"是恒真的假断言
    await cdp.evaluate(setJointSlider(1, 5));
    await sleep(250);
    const away = await cdp.evaluate(READ_TEACH_TERMINAL);
    check(
      'Phase 13：回放前把臂挪开（脱离录制终点）',
      away !== null && Math.abs((away.cmd.shoulder ?? 0) - 5) < 1.5,
      away === null ? 'null' : `shoulder ${away.cmd.shoulder}°`,
    );

    const recTerminal = rec === null || rec.last === null ? null : rec.last.shoulder;
    check('Phase 13：可开始回放', (await cdp.evaluate(clickTeach('teach-play'))) === true);

    // 回放期间不得产新帧：录制与回放共用 commandJoints 通道，自录自放会让轨迹无限生长
    await sleep(130);
    const during = await cdp.evaluate(READ_TEACH_TERMINAL);
    check(
      'Phase 13：回放期间不产生新帧（回放不录自己）',
      during !== null && rec !== null && during.frames === rec.frames,
      during === null || rec === null ? 'null' : `${during.frames} vs ${rec.frames}`,
    );

    const ended = await poll(
      cdp,
      READ_TEACH_TERMINAL,
      (v) => v !== null && typeof v.statusText === 'string' && v.statusText.startsWith('回放完成'),
      Math.max(4000, (rec === null ? 0 : rec.durationMs) + 2500),
    );
    check(
      'Phase 13：回放走完时间轴并进入「回放完成」',
      ended !== null &&
        typeof ended.statusText === 'string' &&
        ended.statusText.startsWith('回放完成'),
      ended === null ? 'null' : String(ended.statusText),
    );

    const terminalMatches =
      ended !== null &&
      ended.last !== null &&
      rec !== null &&
      rec.last !== null &&
      Object.keys(rec.last).every(
        (k) => Math.abs((ended.cmd[k] ?? 0) - (rec.last[k] ?? 0)) < 1e-9,
      );
    check(
      'Phase 13：回放终点**逐值**等于录制末帧（不留插值残差）',
      terminalMatches,
      ended === null || ended.last === null
        ? 'null'
        : `cmd ${JSON.stringify(ended.cmd)} / rec ${JSON.stringify(ended.last)}`,
    );
    check(
      'Phase 13：终点确实回到录制姿态（且与"挪开处 5°"明显不同，证明回放真的驱动了关节）',
      ended !== null &&
        typeof recTerminal === 'number' &&
        Math.abs((ended.cmd.shoulder ?? 0) - recTerminal) < 1e-9 &&
        Math.abs(recTerminal - 5) > 5,
      ended === null ? 'null' : `终点 ${ended.cmd.shoulder}° / 挪开处 5°`,
    );

    // 回放同样走既有命令路径 ⇒ 节流下游必须看得到下发日志（这就是"没绕过安全门"的证据）
    const teachLog = await cdp.evaluate(READ_LOG);
    check(
      'Phase 13：回放经既有命令路径下发（日志可见 joint_command，未绕过节流与安全门）',
      typeof teachLog === 'string' && teachLog.includes('joint_command'),
      typeof teachLog === 'string' ? teachLog.slice(-80) : 'null',
    );

    // (c) 清空 → 帧数归零、导出重新禁用
    check('Phase 13：可清空轨迹', (await cdp.evaluate(clickTeach('teach-clear'))) === true);
    await sleep(150);
    const cleared = await cdp.evaluate(READ_TEACH_TERMINAL);
    check(
      'Phase 13：清空后帧数归零且回到 idle',
      cleared !== null && cleared.frames === 0 && cleared.statusText === '已就绪',
      cleared === null ? 'null' : `${cleared.frames} 帧 / ${cleared.statusText}`,
    );
    check(
      'Phase 13：清空后导出按钮重新禁用',
      (await cdp.evaluate(TEACH_EXPORT_DISABLED)) === true,
    );

    // (d) 日志同时出现下发与回推 —— 闭环真的走通了
    const logText = await cdp.evaluate(READ_LOG);
    check(
      '日志出现 SEND joint_command 与 RECV joint_state（双向链路）',
      typeof logText === 'string' &&
        logText.includes('joint_command') &&
        logText.includes('joint_state'),
      typeof logText === 'string' ? logText.slice(-80) : 'null',
    );

    // (e) 断开 → 状态复原，且 Actual 拉回 Command（不留假误差）
    await cdp.evaluate(CLICK_DISCONNECT);
    const disconnected = await poll(cdp, READ_CONNECTION, (v) => v === 'Disconnected', 8000);
    check('断开 → 状态灯复原为 Disconnected', disconnected === 'Disconnected', String(disconnected));

    const afterDisconnect = await cdp.evaluate(readJointGap(1));
    check(
      '断开后 Actual 拉回 Command（回到仿真立即跟随）',
      afterDisconnect !== null && afterDisconnect.gap <= 0.05,
      afterDisconnect ? `差 ${afterDisconnect.gap.toFixed(2)}°` : 'null',
    );

    // (e2) Phase 11：断开后必须回到"未接入传输"，且各关节误差归零 ——
    //      否则会出现"链路已断但面板还挂着最后那个滞后值"的假状态。
    const healthIdle = await poll(
      cdp,
      READ_LINK_HEALTH,
      (v) => typeof v === 'string' && v.includes('未接入传输'),
      4000,
    );
    check(
      'Phase 11：断开后健康结论回到"未接入传输"（不留假链接状态）',
      typeof healthIdle === 'string' && healthIdle.includes('未接入传输'),
      String(healthIdle),
    );
    const errsIdle = await cdp.evaluate(READ_ERR_VALUES);
    check(
      'Phase 11：断开后各关节误差归零',
      Array.isArray(errsIdle) && errsIdle.every((v) => Math.abs(v) <= 0.05),
      JSON.stringify(errsIdle),
    );

    // 复位，保证截图与文档基线一致
    await cdp.evaluate(CLICK_HOME);
    await sleep(400);

    // ---- Phase 8：真实 Go 后端（WebSocket 关节级服务，链路末端 = 内置假固件）----
    // 这是「不接真机也能跑通完整闭环」的最强证据：命令真的离开浏览器 → 经 Go 控制器
    // 编码成 `JR` 文本 → 由内置假固件按**真实标定表**换算成舵机角 → 回执 →
    // 再把舵机**实际角**反算回关节角上报。整条链上只有串口驱动是假的。
    const backend = await startBackend();
    check(
      'Phase 8：后端可执行文件已就绪（backend/bin/armpilot-backend.exe）',
      backend !== null,
      backend === null
        ? `未找到 ${BACKEND_EXE}`
        : `${BACKEND_HTTP} device=${backend.health.device} linked=${backend.health.linked}`,
    );

    if (backend !== null) {
      // 后端读的是同一份 config/robot.yaml：初始状态必须是解析解 HOME 位。
      //
      // ⚠️ 容差必须依**链路末端**切换，不能写死 1e-9：
      //    * sim  ：假固件内部是浮点，可以逐位一致 ⇒ 1e-9
      //    * serial：固件舵机角是**整数度**，命令的物理落点是
      //              servoToJoint(round(jointToServo(θ))) ⇒ 回读必然带
      //              量化残差（实测 shoulder 0.85 vs 0.8499，差 1.06e-4）。
      //              拿 1e-9 去比，真机模式下**原理上必然 FAIL**，是假红。
      const isSerialLink = backend.health.device === 'serial';
      const homeEps = isSerialLink ? 1e-2 : 1e-9;
      const backHome = backend.health.state ?? {};
      if (!isSerialLink && !backend.external) {
        // sim（**本脚本自己拉起的**后端）：假固件每次启动都从 robot.yaml 的 homePose 起算，
        // 可以逐位校验。
        //
        // ⚠️ 必须排除 backend.external（复用 8090 上的既有实例）。
        //    复用的实例可能刚跑完上一轮 e2e，state 停在上轮命令值（实测 shoulder=20.9），
        //    那时"HOME 位"断言必假红 —— 这是跨批次状态残留，不是产品缺陷。
        check(
          'Phase 8：后端初始状态 = HOME 位（同一份 robot.yaml 真值）',
          Math.abs((backHome.shoulder ?? NaN) - 0.8498937633) < homeEps &&
            Math.abs((backHome.elbow ?? NaN) - 112.6185771989) < homeEps &&
            Math.abs((backHome.base ?? NaN)) < homeEps &&
            Math.abs((backHome.gripper ?? NaN) - 50) < homeEps,
          JSON.stringify(backHome),
        );
      } else if (!isSerialLink) {
        // sim 但复用既有实例：位置取决于上一轮操作，无权假定 HOME。
        // 改为校验可判定的不变量：四轴都是有限数（state 确实来自一个活着的 sim 链路）。
        const allFinite =
          Number.isFinite(backHome.shoulder) &&
          Number.isFinite(backHome.elbow) &&
          Number.isFinite(backHome.base) &&
          Number.isFinite(backHome.gripper);
        check(
          'Phase 8：后端 state 为可读的有限值（复用实例，不假定 HOME）',
          allFinite,
          `${JSON.stringify(backHome)}（8090 为复用实例，跳过 HOME 逐位比对）`,
        );
      } else {
        // serial：**臂的物理位置取决于上一次操作**，测试无权假定它一定是 HOME
        //（真机后端是把当前物理位当作初始 state 上报的；用 1e-9 去比必然假红）。
        //
        // 改为校验一条真正可判定的不变量：**后端上报的 state 与串口 STATUS 一致**。
        // 这条比"等于 HOME"更有意义 —— 它证明 state 确实来自链路末端，
        // 而不是后端自己的默认值/缓存。（/healthz 不返回 limits，无法做范围校验，
        // 故不写"落在限位内"这种无数据的断言，避免变成恒真空断言。）
        const status = backend.health.status ?? null;
        // 若后端提供原始 STATUS 文本，就逐轴核对；否则退化为"必须是有限数且非默认零值"
        const allFinite =
          Number.isFinite(backHome.shoulder) &&
          Number.isFinite(backHome.elbow) &&
          Number.isFinite(backHome.base) &&
          Number.isFinite(backHome.gripper);
        const eq = (a, b, eps = 0.01) => Math.abs(a - b) <= eps;
        const consistent =
          status === null
            ? // 无 STATUS 可对照：至少要求四轴有限，且 shoulder 不为 NaN/未初始化
              allFinite
            : eq(backHome.shoulder, status.shoulder) &&
              eq(backHome.elbow, status.elbow) &&
              eq(backHome.base, status.base) &&
              eq(backHome.gripper, status.gripper);
        check(
          'Phase 8：后端 state 为可读的有限值（真机=当前物理位，与链路末端自洽）',
          allFinite && consistent,
          `${JSON.stringify(backHome)}${status === null ? '（/healthz 无 status 字段，仅校验有限性）' : ' vs STATUS ' + JSON.stringify(status)}`,
        );
      }

      // (a) 切到 WebSocket 模式并写入后端地址
      check(
        'Phase 8：连接面板可切到 WebSocket 传输方式',
        (await cdp.evaluate(CLICK_WS_MODE)) === true,
      );
      check('Phase 8：后端地址输入框可写入', (await cdp.evaluate(setWsUrl(BACKEND_WS))) === true);

      // (b) 连接（后端不可达时这一段整体会失败，不会静默跳过）
      check('Phase 8：Connect WS 按钮可点击', (await cdp.evaluate(CLICK_CONNECT_WS)) === true);
      const wsConnected = await poll(cdp, READ_CONNECTION, (v) => v === 'Connected', 10000);
      check(
        'Phase 8：连接 Go 后端 → 状态灯变 Connected',
        wsConnected === 'Connected',
        String(wsConnected),
      );

      const wsRows = await poll(cdp, READ_WS_ROWS, (v) => v === true, 6000);
      check('Phase 8：WebSocket 专有统计行已渲染', wsRows === true);
      const device = await poll(cdp, READ_DEVICE, (v) => typeof v === 'string' && v !== '—', 6000);
      // hello 上报的链路末端必须与后端实际一致 —— **不能写死 'sim'**：
      // 复用一个以 config.serial.yaml 启动的真机后端时，末端**就是** serial，
      // 写死会让每次真机联调都报假红。这里改为与 /healthz 的 device 对齐。
      const expectDevice = backend.health.device;
      check(
        `Phase 8：hello 上报链路末端与后端一致（${expectDevice}）`,
        device === expectDevice,
        `hello=${device} healthz=${expectDevice}`,
      );
      check('Phase 8：模型/标定一致性校验通过（无告警）', (await cdp.evaluate(READ_MODEL_WARN)) === null);
      const reconnects0 = await poll(cdp, READ_RECONNECTS, (v) => typeof v === 'number', 5000);
      check('Phase 8：初次连接重连次数为 0', reconnects0 === 0, `reconnects=${reconnects0}`);

      // (c) 拖动 J2：Actual 必须滞后（后端假固件有 15ms 送达延迟 + 240°/s 有限角速度）
      //
      // ⚠️ 真机特有 ①：**必须先把臂移到远离目标的位置**，否则"命令 = 当前位姿"
      //    时天然就没有滞后（实测复用的真机后端停在 30°，再命令 30° ⇒ 差 0.1°，假红）。
      //    这里先按当前位姿挑一个**相距 >10° 且在限位内**的目标。
      // ⚠️ 真机特有 ②：**必须先切到 Real Robot**。ADR D41 的安全门在
      //    `mode === 'simulation'` 时会拦截下发给真机链路的命令 —— 这是**正确行为**，
      //    但会让本段的"滞后/收敛"观察不到任何变化。而 mode 跨批次残留，
      //    所以这里显式归位，不依赖上一轮的终态。
      if (isSerialLink) {
        await cdp.evaluate(clickModeButton('Real Robot'));
        await sleep(300);
      }
      const J2_MIN = -6.093682734102679;
      const J2_MAX = 49.454929244955494;
      const curJ2 = await cdp.evaluate(readJointGap(1));
      const curVal = curJ2?.actual ?? 0;
      const target = Math.min(J2_MAX - 0.5, Math.max(J2_MIN + 0.5, curVal + (curVal > (J2_MIN + J2_MAX) / 2 ? -20 : 20)));
      await cdp.evaluate(setJointSlider(1, target));
      const wsImmediate = await cdp.evaluate(readJointGap(1));
      check(
        'Phase 8：命令经链路送达前 Actual 明显滞后',
        wsImmediate !== null && wsImmediate.gap > 1,
        wsImmediate
          ? `cmd ${wsImmediate.command}° / act ${wsImmediate.actual}° / 差 ${wsImmediate.gap.toFixed(1)}°`
          : 'null',
      );

      // (d) 收敛：Actual 只能来自「舵机实际角 → 反算关节角」，标定表写错这里必然不收敛。
      //     容差同理依链路末端：sim 浮点可到 0.05°；serial 的舵机角是整数度，
      //     取整误差上限 0.5°（实测常驻 ~0.1°），用 0.05° 会把量化噪声当失败。
      const convEps = isSerialLink ? 0.5 : 0.05;
      const wsConverged = await poll(cdp, readJointGap(1), (v) => v !== null && v.gap <= convEps, 12000);
      check(
        `Phase 8：Actual 收敛到 Command（标定往返自洽 · 容差 ${convEps}°）`,
        wsConverged !== null && wsConverged.gap <= convEps,
        wsConverged ? `cmd ${wsConverged.command}° / act ${wsConverged.actual}°` : 'null',
      );

      const READ_LAG = `document.querySelector('[data-testid="stat-lag"]')?.textContent?.trim() ?? null`;
      const lagText = await poll(cdp, READ_LAG, (v) => v === '0.00°', 8000);
      check('Phase 8：跟踪误差统计归零（stat-lag）', lagText === '0.00°', String(lagText));

      // (e) 应用层心跳：10s 后首个 ping 才发出，等到 pong 回来才有 RTT
      const rtt = await poll(cdp, READ_RTT, (v) => typeof v === 'number' && v >= 0, 14000, 300);
      check(
        'Phase 8：应用层心跳收到 pong 并算出 RTT',
        typeof rtt === 'number',
        rtt === null ? 'null（14s 内未收到 pong）' : `${rtt} ms`,
      );

      // (f) 双向日志
      const wsLog = await cdp.evaluate(READ_LOG);
      check(
        'Phase 8：日志同时出现下发与回推（真实双向链路）',
        typeof wsLog === 'string' && wsLog.includes('joint_command') && wsLog.includes('joint_state'),
        typeof wsLog === 'string' ? wsLog.slice(-90) : 'null',
      );

      // (h) mode ↔ transport 联动（ADR D41）—— 这是本次要修的核心缺口
      //
      // 修正前：`mode` 是纯 UI 状态，点 Real Robot 只改按钮样式，
      //         命令照样走当时连着的 transport ⇒ 真机不动 / 或误驱动真机。
      // 修正后：真机链路上，simulation 模式必须**拦截**命令、real 模式才放行，
      //         且"命令去向"必须显式可见。
      if (isSerialLink) {
        // h1. **显式**切到 Simulation（(c) 段为了跑通真机链路已切到 real）⇒ 提示应说明"已拦截"
        await cdp.evaluate(clickModeButton('Simulation'));
        await sleep(300);
        const routingSim = await cdp.evaluate(READ_MODE_ROUTING);
        check(
          'Phase 8：真机链路 + Simulation → 提示"已拦截"（命令不下发真机）',
          typeof routingSim === 'string' && routingSim.includes('拦截'),
          String(routingSim),
        );

        // h2. 切到 Real Robot ⇒ 提示应变成"正在驱动真实机械臂"
        await cdp.evaluate(clickModeButton('Real Robot'));
        await sleep(300);
        const routingReal = await cdp.evaluate(READ_MODE_ROUTING);
        check(
          'Phase 8：真机链路 + Real Robot → 提示"正在驱动真实机械臂"',
          typeof routingReal === 'string' && routingReal.includes('真实机械臂'),
          String(routingReal),
        );

        // h3. Real 模式下命令真的能出去（回守：别把真机路径一起拦死）
        const curJ2b = await cdp.evaluate(readJointGap(1));
        const altTarget = Math.min(
          J2_MAX - 0.5,
          Math.max(J2_MIN + 0.5, (curJ2b?.actual ?? 0) + ((curJ2b?.actual ?? 0) > (J2_MIN + J2_MAX) / 2 ? -8 : 8)),
        );
        await cdp.evaluate(setJointSlider(1, altTarget));
        const sentInReal = await poll(
          cdp,
          LOG_CONTAINS('joint_command'),
          (v) => v === true,
          6000,
        );
        check('Phase 8：Real Robot 模式下命令确认下发（未被安全门误拦）', sentInReal === true);

        // h4. 切回 Simulation ⇒ 日志出现"已拦截"（"以为在仿真其实在动真机"必须不可能）
        await sleep(600);
        await cdp.evaluate(clickModeButton('Simulation'));
        await sleep(300);
        await cdp.evaluate(setJointSlider(1, altTarget + 5));
        const blocked = await poll(cdp, LOG_CONTAINS('已拦截'), (v) => v === true, 8000);
        check(
          'Phase 8：切回 Simulation → 命令被拦截且日志可见（不留静默）',
          blocked === true,
        );

        // h5. 归位：后续段落默认"命令能下发"，不能把页面留在被拦截的 Simulation 态
        //     （同理，这也是"跨批次状态残留"防线的最后一步）
        await cdp.evaluate(clickModeButton('Real Robot'));
        await sleep(300);
      } else {
        // sim 末端：安全门**不应**触发，命令必须照常走（否则打死 Phase 8 仿真闭环）
        const routingSim = await cdp.evaluate(READ_MODE_ROUTING);
        check(
          'Phase 8：sim 末端下命令照常下发（安全门只针对真机链路）',
          typeof routingSim === 'string' && !routingSim.includes('拦截'),
          String(routingSim),
        );
      }

      // (h2) **用户报障场景的精确复现**：连着后端（末端=sim）时点 Real Robot。
      //
      // 报障原文：「前端显示 Real 模式后端末端是 sim，非真机」。
      // 根因：`setMode('real')` 把 `set({ mode })` 写在准入校验之前 ⇒ 校验只发日志、
      // mode 照样变 real。这里在**链路仍然连着**的时刻断言修复后的语义：
      // 按钮不得切到 Real，且日志必须说清"末端是 sim"。
      //
      // ⚠️ 入口/出口状态必须显式设定（跨批次残留，§9.10 第 7 例）：mode 可能被
      //    上一轮的 (c)/(h1) 段留在 real，先归位到 Simulation 再点，否则断言读到的是
      //    "上一轮的结果"，与本次点击无关。
      {
        const connNow = await cdp.evaluate(READ_CONNECTION);
        if (connNow === 'Connected') {
          await cdp.evaluate(CLICK_SIMULATION);
          await sleep(200);
          // 只比对"点击之后新增"的日志：历史里可能已有同类文本，读全量会假绿
          const logLenBefore = await cdp.evaluate(READ_LOG_LENGTH);
          const clicked = await cdp.evaluate(CLICK_REAL_ROBOT);
          await sleep(400);
          const btns = await cdp.evaluate(READ_MODE_BUTTONS);
          const tail = await cdp.evaluate(READ_LOG_TAIL);
          const logLenAfter = await cdp.evaluate(READ_LOG_LENGTH);
          check(
            'Phase 8：连着后端（末端非 serial）点 Real Robot → **拒绝切换**，按钮仍在 Simulation',
            clicked === true && btns !== null && btns.realActive === false && btns.simActive === true,
            btns ? `simActive=${btns.simActive} realActive=${btns.realActive}` : 'null',
          );
          check(
            'Phase 8：拒绝原因必须点名链路末端（不留静默）',
            typeof tail === 'string' &&
              /Real Robot 未启用/.test(tail) &&
              typeof logLenAfter === 'number' &&
              logLenAfter > logLenBefore,
            tail === null ? 'null' : tail.slice(-100),
          );
          // 收尾：确保停在 Simulation，不把状态泄漏给后续分段
          await cdp.evaluate(CLICK_SIMULATION);
          await sleep(200);
        } else {
          console.log('[phase8] 当前未连接，跳过「连着后端点 Real Robot」子项');
        }
      }

      if (backend.external) {
        console.log('[phase8] 后端为复用实例（非本脚本拉起），跳过断线重连子项');
      } else {
        // (g) 杀掉后端 → 前端应自行重连（指数退避）
        stopBackend(backend.child);
        const reconnects = await poll(
          cdp,
          READ_RECONNECTS,
          (v) => typeof v === 'number' && v >= 1,
          12000,
        );
        check(
          'Phase 8：链路断开后自动重连（重连次数 ≥ 1）',
          typeof reconnects === 'number' && reconnects >= 1,
          `reconnects=${reconnects}`,
        );

        // (h) 重启后端：新进程的 sim 从 HOME 起步，只有「重连后补发当前命令」
        //     才可能把新 sim 拉回**前端当前命令值** —— 没补发就会永久停在 HOME。
        //
        // ⚠️ 期望值必须取「重连前实测到的命令值」，**不能写死数字**。
        //    (c) 段的 target 是按当时位姿自适应挑的（curVal ± 20），
        //    写死比如 30 会随 HOME/前序步骤漂移而假红 —— 实测已踩。
        const cmdBeforeRestart = await cdp.evaluate(readJointGap(1));
        const expectCmd = cmdBeforeRestart?.command ?? null;
        const backend2 = await startBackend();
        check(
          'Phase 8：后端重启成功',
          backend2 !== null && backend2.health.device === 'sim',
          backend2 === null ? '启动失败' : `device=${backend2.health.device}`,
        );
        if (backend2 !== null) {
          const backAgain = await poll(cdp, READ_CONNECTION, (v) => v === 'Connected', 25000);
          check(
            'Phase 8：重连成功 → 状态灯回到 Connected',
            backAgain === 'Connected',
            String(backAgain),
          );
          const reConverged = await poll(cdp, readJointGap(1), (v) => v !== null && v.gap <= 0.05, 15000);
          // 两条判据同时成立才算补发生效：
          //   1) 新 sim 的 Actual 收敛到 Command（gap→0）
          //   2) 那个 Command 仍是重连前的那一个（没被 HOME 顶掉）
          const cmdHeld = expectCmd === null || reConverged === null
            ? false
            : Math.abs(reConverged.command - expectCmd) < 2;
          check(
            'Phase 8：重连后自动补发当前命令（新 sim 被重新驱动到命令值）',
            reConverged !== null && reConverged.gap <= 0.05 && cmdHeld,
            reConverged
              ? `cmd ${reConverged.command}° / act ${reConverged.actual}° ` +
                `(期望命令 ${expectCmd === null ? 'null' : `${expectCmd.toFixed(1)}°`})`
              : 'null',
          );
        }
      }

      // (i) 显式断开 → 状态复原，Actual 拉回 Command（不留假误差）
      const wsDisconnect = await poll(
        cdp,
        DISCONNECT_AND_READ,
        (v) => v !== null && v.badge === 'Disconnected',
        10000,
      );
      check(
        'Phase 8：断开 → 状态灯复原为 Disconnected',
        wsDisconnect !== null && wsDisconnect.badge === 'Disconnected',
        JSON.stringify(wsDisconnect),
      );
      const wsAfterDisconnect = await cdp.evaluate(readJointGap(1));
      check(
        'Phase 8：断开后 Actual 拉回 Command（回到纯仿真）',
        wsAfterDisconnect !== null && wsAfterDisconnect.gap <= 0.05,
        wsAfterDisconnect ? `差 ${wsAfterDisconnect.gap.toFixed(2)}°` : 'null',
      );
    }

    // (j) Phase 10.6：Real Robot 准入必须是**拒绝**，不能只是"发条日志"
    //
    // 用户报障原文：「前端显示 Real 模式后端末端是 sim，非真机」。
    // 根因是 `setMode('real')` 的 `set({ mode })` 写在准入校验**之前** ——
    // 校验只 pushLog，mode 照样变成 real，于是按钮显示 Real 而链路是 sim。
    // 这里断言修复后的语义：点 Real Robot **不切**，且给出拒绝原因。
    {
      // 前端此刻连着 sim 后端（或已断开）—— 两种情形都**不该**切得过去。
      const clicked = await cdp.evaluate(CLICK_REAL_ROBOT);
      await sleep(400);
      const after = await cdp.evaluate(READ_MODE_BUTTONS);
      const hint = await cdp.evaluate(READ_MODE_ROUTING);
      const tail = await cdp.evaluate(READ_LOG_TAIL);

      check('Phase 10.6：Real Robot 按钮可点击', clicked === true, String(clicked));
      check(
        'Phase 10.6：链路末端非 serial 时点 Real Robot → **拒绝切换**（按钮仍高亮 Simulation）',
        after !== null && after.realActive === false && after.simActive === true,
        after
          ? `simActive=${after.simActive} realActive=${after.realActive}`
          : 'null（按钮未找到）',
      );
      check(
        'Phase 10.6：拒绝时必须给出原因（不留静默）',
        typeof tail === 'string' && /Real Robot 未启用/.test(tail),
        tail === null ? 'null' : tail.slice(-90),
      );
      check(
        'Phase 10.6：提示条不得显示"正在驱动真实机械臂"',
        typeof hint === 'string' && !/正在驱动真实机械臂/.test(hint),
        hint === null ? 'null' : hint,
      );
      // 收尾：确保 mode 停在 Simulation，避免影响后续分段
      await cdp.evaluate(CLICK_SIMULATION);
      await sleep(200);
    }

    // 复位，保证截图与文档基线一致
    await cdp.evaluate(CLICK_HOME);
    await sleep(400);

    // 6b. 可选：在 HOME 位截屏（用于文档配图，与实拍 RESET 照比对）
    const shotPath = process.argv[4] ?? process.env.SHOT_PATH;
    if (shotPath) {
      const data = await cdp.screenshot();
      mkdirSync(path.dirname(path.resolve(shotPath)), { recursive: true });
      writeFileSync(shotPath, Buffer.from(data, 'base64'));
      console.log(`[SHOT] ${shotPath}`);
    }

    // 7. 控制台干净
    await sleep(600);
    const errors = cdp.pageErrors();
    check('页面无控制台错误 / 未捕获异常', errors.length === 0, errors.slice(0, 5).join(' ;; '));

    socket.close();
  } finally {
    // 兜底：若 Phase 9 段落的后端仍在跑，先收掉（否则会占住 8090 影响下一次运行）
    for (const proc of serialChildren) {
      try {
        proc.kill();
      } catch {
        /* ignore */
      }
    }
    try {
      child.kill();
    } catch {
      /* ignore */
    }
    await sleep(600);
    try {
      rmSync(userDataDir, { recursive: true, force: true });
    } catch {
      /* Windows 文件占用时忽略 */
    }
  }

  const passed = results.filter((r) => r.ok).length;
  const failed = results.length - passed;

  // 8. Phase 9（**opt-in**）：真机端到端 —— 把相机请出来当唯一地面真值
  //    ⚠️ 会真的驱动机械臂 + 调用相机，默认**不跑**。开启方式：
  //        ARM_E2E_SERIAL=1 node tests/e2e/ui-smoke.mjs
  //    或   node tests/e2e/ui-smoke.mjs --serial
  //    前置：机械臂接在 backend/config.serial.yaml 写的串口上；
  //          相机取景已按 docs/hardware-measurement.md 的 Phase 4.5 标准重布
  //          （白分割板 + 画面内标尺 + 正交侧视 + **锁死曝光**）。
  const wantSerial =
    process.env.ARM_E2E_SERIAL === '1' || process.argv.includes('--serial');
  if (wantSerial) {
    const driver = path.join(REPO_DIR, 'tools', 'verify_serial_e2e.mjs');
    if (!existsSync(driver)) {
      check('Phase 9：真机端到端脚本存在', false, driver);
    } else {
      // 先把本进程拉起的 sim 后端收掉，让位给 serial 后端（避免抢 8090）
      if (backendChildren.length > 0) {
        console.log('[e2e] 让位：先关闭本进程的 sim 后端，改由 verify_serial_e2e.mjs 拉起 serial 后端');
        for (const proc of backendChildren) {
          try {
            proc.kill();
          } catch {
            /* ignore */
          }
        }
        backendChildren.length = 0;
        await sleep(1500);
      }
      console.log('[e2e] 启动 Phase 9 真机端到端（会驱动物理机械臂 + 相机）…');
      const r = spawnSync(process.execPath, [driver], {
        cwd: REPO_DIR,
        stdio: 'inherit',
        env: process.env,
      });
      const ok = r.status === 0;
      check(
        'Phase 9：真机端到端（WebSocket → 串口 → 舵机 → 相机反解）',
        ok,
        `exit=${r.status}${ok ? '' : '（详见 tools/verify_serial_e2e.mjs 输出与 summary.json）'}`,
      );
    }
  } else {
    console.log('[e2e] 跳过 Phase 9 真机验收（opt-in：ARM_E2E_SERIAL=1 或 --serial 开启）');
  }

  const passedAll = results.filter((r) => r.ok).length;
  const failedAll = results.length - passedAll;
  console.log('');
  console.log(`===== e2e 结果: ${passedAll}/${results.length} PASS, ${failedAll} FAIL =====`);
  process.exit(failedAll === 0 ? 0 : 1);
}

main().catch((error) => {
  console.error(`[e2e] 运行失败: ${error.message}`);
  const passed = results.filter((r) => r.ok).length;
  console.log(`===== e2e 结果: ${passed}/${results.length} PASS（中断） =====`);
  process.exit(1);
});
