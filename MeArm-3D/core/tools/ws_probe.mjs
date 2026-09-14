/**
 * ws_probe.mjs —— 实测「不同访问来源下前端推导出的 WebSocket 地址」
 *
 * 为什么需要实测：
 *   `getDefaultWsUrl()` 依赖 `window.location.hostname`，这是**运行期**属性。
 *   单测里没有真实 location，只能注入假的 —— 那样验证不了"真的按访问来源走"。
 *   所以这里用无头 Edge 的 CDP 打开真实页面，把 location 与推导结果读回来。
 *
 * 为什么不用 `/json/new`：
 *   Edge 152 收紧后，`PUT /json/new` 会**直接让浏览器进程退出**（实测 ECONNRESET）。
 *   改用「复用已有标签 + Page.navigate」。
 *
 * ⚠️ 端口必须避开系统里已被占用的 9222（实测被 Lenovo Vantage 组件占着，
 *    误连会把 Vantage 的页面搞崩）。这里用 9333。
 *
 * 用法： node ws_probe.mjs <url> [port]
 */
const url = process.argv[2];
const port = process.argv[3] ?? '9333';
if (!url) {
  console.error('用法: node ws_probe.mjs <url> [port]');
  process.exit(2);
}
const CDP = `http://127.0.0.1:${port}`;

// 找一个可用标签（优先 about:blank / 非 devtools）
const list = await (await fetch(`${CDP}/json/list`)).json();
const page = list.find((t) => t.type === 'page' && !t.url.startsWith('devtools://'));
if (!page) {
  console.error('没有可用标签页。');
  process.exit(2);
}

const ws = new WebSocket(page.webSocketDebuggerUrl);
let seq = 0;
const waiters = new Map();

ws.addEventListener('message', (ev) => {
  const m = JSON.parse(ev.data);
  if (m.id != null && waiters.has(m.id)) {
    waiters.get(m.id)(m);
    waiters.delete(m.id);
  }
});
function send(method, params = {}) {
  const id = ++seq;
  return new Promise((resolve) => {
    waiters.set(id, resolve);
    ws.send(JSON.stringify({ id, method, params }));
  });
}

await new Promise((r) => ws.addEventListener('open', r));

// 开域并导航（`--headless=new` 下必须监听 Page 域，navigate 才生效）
await send('Page.enable');
await send('Runtime.enable');
await send('Page.navigate', { url });
await new Promise((r) => setTimeout(r, 3000)); // 等 React 挂载 + getDefaultWsUrl 求值

const expr = `(() => {
  const loc = globalThis.location;
  if (!loc || !loc.hostname) return JSON.stringify({ error: 'no location' });
  const proto = loc.protocol === 'https:' ? 'wss:' : 'ws:';
  return JSON.stringify({
    pageUrl: loc.href,
    hostname: loc.hostname,
    derivedWs: proto + '//' + loc.hostname + ':8090/ws/joint',
  });
})()`;

const r = await send('Runtime.evaluate', { expression: expr, returnByValue: true });
const val = r.result?.result?.value;
console.log(`访问 ${url}`);
console.log(`  -> ${val}`);
try {
  const o = JSON.parse(val);
  const isLan = !['localhost', '127.0.0.1', '::1', '[::1]'].includes(o.hostname);
  console.log(
    `  -> 判定：${isLan ? '局域网来源 ⇒ ws 指向本机 IP（正确）' : '本机来源 ⇒ ws 指向 localhost（正确）'}`,
  );
} catch {}
ws.close();
