/**
 * lan_e2e_probe.mjs —— 局域网端到端探针
 *
 * 目的：证明「局域网内的另一台设备」打开页面后，真的能把链路走通 ——
 *   ① 页面能加载（Vite 监听 0.0.0.0）
 *   ② 前端推导出的 ws 地址指向**服务端本机**（而不是访问者自己）
 *   ③ 该 ws 地址**真的能连上并收到 hello / joint_state**
 *
 * 前两条上一轮已验证；这一条（③）才是有没有真通的证据 —— 地址推导对、
 * 但防火墙/端口没放开，照样连不上。
 *
 * 用法： node lan_e2e_probe.mjs <pageUrl> [cdpPort]
 *   例： node lan_e2e_probe.mjs http://192.168.3.5:5273/
 */
const pageUrl = process.argv[2];
const port = process.argv[3] ?? '9333';
if (!pageUrl) {
  console.error('用法: node lan_e2e_probe.mjs <pageUrl> [cdpPort]');
  process.exit(2);
}
const CDP = `http://127.0.0.1:${port}`;

const list = await (await fetch(`${CDP}/json/list`)).json();
const page = list.find((t) => t.type === 'page' && !t.url.startsWith('devtools://'));
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
const send = (method, params = {}) => {
  const id = ++seq;
  return new Promise((r) => {
    waiters.set(id, r);
    ws.send(JSON.stringify({ id, method, params }));
  });
};
await new Promise((r) => ws.addEventListener('open', r));
await send('Page.enable');
await send('Runtime.enable');
await send('Page.navigate', { url: pageUrl });
await new Promise((r) => setTimeout(r, 3000));

// 在页面上下文里**自己开一条 WebSocket** 到推导出的地址，收集真实回包。
// 用 expression + awaitPromise，避免依赖 React 组件的内部状态。
const expr = `(async () => {
  const loc = globalThis.location;
  const proto = loc.protocol === 'https:' ? 'wss:' : 'ws:';
  const derived = proto + '//' + loc.hostname + ':8090/ws/joint';
  const out = { pageUrl: loc.href, derived };
  const got = [];
  const err = [];
  await new Promise((resolve) => {
    let sock;
    try { sock = new WebSocket(derived); } catch (e) { err.push('ctor: ' + e.message); return resolve(); }
    const done = () => { try { sock.close(); } catch {} resolve(); };
    const timer = setTimeout(done, 6000);
    sock.onopen = () => { err.push('OPEN'); };
    sock.onmessage = (ev) => {
      got.push(String(ev.data).slice(0, 220));
      // 收到 hello 与至少一条 joint_state 就够
      if (got.length >= 3) { clearTimeout(timer); done(); }
    };
    sock.onerror = () => { err.push('ERROR'); clearTimeout(timer); done(); };
    sock.onclose = (e) => { err.push('CLOSE code=' + e.code); clearTimeout(timer); resolve(); };
  });
  out.events = err;
  out.messages = got;
  return JSON.stringify(out);
})()`;

const r = await send('Runtime.evaluate', {
  expression: expr,
  returnByValue: true,
  awaitPromise: true,
});
const val = r.result?.result?.value;
console.log(val);
try {
  const o = JSON.parse(val);
  const opened = o.events.includes('OPEN');
  const hasHello = o.messages.some((m) => m.includes('hello'));
  const hasState = o.messages.some((m) => m.includes('joint_state') || m.includes('state'));
  console.log('\n==== 判定 ====');
  console.log(`  ① ws 地址   ${o.derived}`);
  console.log(`  ② 能连上    ${opened ? '✅ 是' : '❌ 否'}  ${o.events.join(' | ')}`);
  console.log(`  ③ 收到 hello ${hasHello ? '✅ 是' : '❌ 否'}`);
  console.log(`  ④ 收到状态   ${hasState ? '✅ 是' : '❌ 否'}`);
  console.log(opened && hasHello ? '\n✅ 局域网端到端链路通' : '\n❌ 链路未通，检查防火墙 / 端口');
} catch {}
ws.close();
