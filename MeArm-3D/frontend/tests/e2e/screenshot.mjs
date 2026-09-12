#!/usr/bin/env node
/**
 * ArmPilot 前端截图工具（零依赖：Node >= 22 自带 fetch / WebSocket，直连 CDP）。
 *
 * 用途：把当前 3D 场景渲染结果落成 PNG，用于**视觉迭代**（spec §35 的"快速视觉反馈"）。
 * 不是断言型测试——断言由 tests/e2e/ui-smoke.mjs 承担。
 *
 * 用法：
 *   node tests/e2e/screenshot.mjs [url] [debugPort] [outPath] [width] [height]
 * 前置：前端开发服务器已启动（默认 http://127.0.0.1:5273/）
 */
import { spawn } from 'node:child_process';
import { existsSync, mkdtempSync, rmSync, writeFileSync, mkdirSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';

const TARGET_URL = process.argv[2] ?? 'http://127.0.0.1:5273/';
const DEBUG_PORT = Number(process.argv[3] ?? 9345);
const OUT_PATH = process.argv[4] ?? path.resolve('docs/images/armpilot-console.png');
const WIDTH = Number(process.argv[5] ?? 1600);
const HEIGHT = Number(process.argv[6] ?? 1100);
/** 可选：截图前摆姿势，"j1,j2,j3,gripper"（按滑杆顺序 J1/J2/J3/Gripper） */
const POSE = process.argv[7] ?? '';

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
  `--window-size=${WIDTH},${HEIGHT}`,
];

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

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
      const page = targets.find((t) => t.type === 'page' && t.webSocketDebuggerUrl);
      if (page) return page;
    } catch {
      /* 端口还没起来，继续等 */
    }
    await sleep(250);
  }
  throw new Error(`调试端口 ${port} 在 ${timeoutMs}ms 内未就绪`);
}

class Cdp {
  constructor(socket) {
    this.socket = socket;
    this.nextId = 1;
    this.pending = new Map();
    socket.addEventListener('message', (event) => {
      const message = JSON.parse(event.data);
      if (message.id && this.pending.has(message.id)) {
        const { resolve, reject } = this.pending.get(message.id);
        this.pending.delete(message.id);
        if (message.error) reject(new Error(JSON.stringify(message.error)));
        else resolve(message.result);
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
      }, 30000);
    });
  }

  async evaluate(expression) {
    const result = await this.send('Runtime.evaluate', {
      expression,
      returnByValue: true,
      awaitPromise: true,
    });
    if (result.exceptionDetails) throw new Error(`页面求值异常: ${JSON.stringify(result.exceptionDetails)}`);
    return result.result?.value;
  }
}

const READY_PROBE = `(() => {
  const overlay = document.querySelector('.overlay');
  const canvas = document.querySelector('.viewport canvas');
  return Boolean(overlay && canvas) && document.querySelectorAll('.overlay .chip').length >= 3;
})()`;

async function main() {
  const browser = findBrowser();
  if (!browser) throw new Error('未找到 Edge / Chrome 可执行文件，请设置 EDGE_PATH');
  console.log(`浏览器: ${browser}`);
  console.log(`目标  : ${TARGET_URL}`);
  console.log(`输出  : ${OUT_PATH}  (${WIDTH}x${HEIGHT})`);

  const userDataDir = mkdtempSync(path.join(tmpdir(), 'armpilot-shot-'));
  const child = spawn(
    browser,
    [...CHROME_FLAGS, `--user-data-dir=${userDataDir}`, `--remote-debugging-port=${DEBUG_PORT}`, TARGET_URL],
    { stdio: 'ignore', detached: false },
  );

  let socket;
  try {
    const page = await waitForDebuggerEndpoint(DEBUG_PORT);
    socket = new WebSocket(page.webSocketDebuggerUrl);
    await new Promise((resolve, reject) => {
      socket.addEventListener('open', resolve, { once: true });
      socket.addEventListener('error', () => reject(new Error('WebSocket 连接失败')), { once: true });
    });
    const cdp = new Cdp(socket);

    await cdp.send('Runtime.enable');
    await cdp.send('Page.enable');
    await cdp.send('Emulation.setDeviceMetricsOverride', {
      width: WIDTH,
      height: HEIGHT,
      deviceScaleFactor: 1,
      mobile: false,
    });

    let ready = false;
    for (let i = 0; i < 60 && !ready; i += 1) {
      try {
        ready = await cdp.evaluate(READY_PROBE);
      } catch {
        ready = false;
      }
      if (!ready) await sleep(500);
    }
    if (!ready) throw new Error('页面未在 30s 内完成渲染');

    // 给 R3F 首帧 + 材质编译 + 相机稳定留时间（swiftshader 软渲染偏慢）
    await sleep(3500);

    if (POSE) {
      const values = POSE.split(',').map((v) => Number(v.trim()));
      for (let i = 0; i < values.length; i += 1) {
        if (!Number.isFinite(values[i])) continue;
        await cdp.evaluate(`(() => {
          const el = Array.from(document.querySelectorAll('input[type=range]'))[${i}];
          if (!el) return false;
          const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set;
          setter.call(el, '${values[i]}');
          el.dispatchEvent(new Event('input', { bubbles: true }));
          return true;
        })()`);
        await sleep(120);
      }
      await sleep(900);
      console.log(`已摆姿势: ${POSE}`);
    }

    const shot = await cdp.send('Page.captureScreenshot', { format: 'png', fromSurface: true });
    mkdirSync(path.dirname(OUT_PATH), { recursive: true });
    writeFileSync(OUT_PATH, Buffer.from(shot.data, 'base64'));
    console.log(`OK 截图已保存: ${OUT_PATH}`);
  } finally {
    try {
      socket?.close();
    } catch {
      /* ignore */
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
}

main().catch((error) => {
  console.error(`[screenshot] 失败: ${error.message}`);
  process.exit(1);
});
