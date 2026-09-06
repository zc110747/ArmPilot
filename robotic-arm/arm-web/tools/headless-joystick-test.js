// headless-joystick-test.js
// 用途：用 Chromium 内核(Edge) 无头模式真实加载 arm-web 的 3D 摇杆页面，
// 通过 CDP 计算摇杆 knob 的屏幕坐标并派发真实 Pointer 事件模拟拖拽，
// 同时钩住 WebSocket.send 捕获网页发出的 JOY 指令，验证
// "3D 摇杆拖拽 -> WS {t:joy} -> 服务端 JOY 指令生成" 全链路。
//
// 零外部依赖：仅用 Node 内置 http/crypto/net 自研最小 WebSocket 客户端对接 CDP。
// 用法：
//   先启动 arm-web.exe -c config.yaml (HTTP 默认 :8080)
//   再启动 Edge 无头调试：msedge --headless=new --remote-debugging-port=9333 --remote-allow-origins=* --user-data-dir=/tmp/edge_profile about:blank
//   node tools/headless-joystick-test.js [cdpPort=9333] [pageUrl=http://127.0.0.1:8080]
const http = require("http");
const crypto = require("crypto");

const CDP_PORT = parseInt(process.argv[2] || "9333", 10);
const PAGE_URL = process.argv[3] || "http://127.0.0.1:8080";

// ---------- 最小 WebSocket 客户端（对接 CDP） ----------
class WS {
  constructor(socket) {
    this.sock = socket;
    this.buf = Buffer.alloc(0);
    this.frag = null;
    this.onMessage = null;
    this.sock.on("data", (d) => this._onData(d));
  }
  _onData(d) {
    this.buf = Buffer.concat([this.buf, d]);
    while (true) {
      if (this.buf.length < 2) return;
      const b0 = this.buf[0], b1 = this.buf[1];
      const opcode = b0 & 0x0f;
      let len = b1 & 0x7f;
      let offset = 2;
      if (len === 126) {
        if (this.buf.length < offset + 2) return;
        len = this.buf.readUInt16BE(offset); offset += 2;
      } else if (len === 127) {
        if (this.buf.length < offset + 8) return;
        len = this.buf.readUInt32BE(offset) * 4294967296 + this.buf.readUInt32BE(offset + 4);
        offset += 8;
      }
      const masked = (b1 & 0x80) !== 0;
      if (masked) offset += 4;
      if (this.buf.length < offset + len) return;
      let payload = this.buf.slice(offset, offset + len);
      if (masked) {
        const m = this.buf.slice(offset - 4, offset);
        const out = Buffer.alloc(len);
        for (let i = 0; i < len; i++) out[i] = payload[i] ^ m[i & 3];
        payload = out;
      }
      this.buf = this.buf.slice(offset + len);
      if (opcode === 0x8) { try { this.sock.end(); } catch (e) {} return; }
      if (opcode === 0x0) {
        this.frag = Buffer.concat([this.frag || Buffer.alloc(0), payload]);
        if ((b0 & 0x80) !== 0) { const f = this.frag; this.frag = null; if (this.onMessage) this.onMessage(f.toString("utf8")); }
      } else if (opcode === 0x1 || opcode === 0x2) {
        if ((b0 & 0x80) !== 0) { if (this.onMessage) this.onMessage(payload.toString("utf8")); }
        else this.frag = payload;
      }
    }
  }
  sendText(str) {
    const data = Buffer.from(str, "utf8");
    const len = data.length;
    const mask = crypto.randomBytes(4);
    let header;
    if (len < 126) { header = Buffer.alloc(2); header[0] = 0x81; header[1] = 0x80 | len; }
    else if (len < 65536) { header = Buffer.alloc(4); header[0] = 0x81; header[1] = 0x80 | 126; header.writeUInt16BE(len, 2); }
    else { header = Buffer.alloc(10); header[0] = 0x81; header[1] = 0x80 | 127; header.writeUInt32BE(0, 2); header.writeUInt32BE(len, 6); }
    const masked = Buffer.alloc(len);
    for (let i = 0; i < len; i++) masked[i] = data[i] ^ mask[i & 3];
    this.sock.write(Buffer.concat([header, mask, masked]));
  }
}

function connectCDP(port) {
  return new Promise((resolve, reject) => {
    const req = http.request({
      hostname: "127.0.0.1", port, path: "/json/new?about:blank",
      headers: { Connection: "Upgrade", Upgrade: "websocket", "Sec-WebSocket-Key": crypto.randomBytes(16).toString("base64"), "Sec-WebSocket-Version": "13" },
    });
    req.on("upgrade", (res, socket) => resolve(new WS(socket)));
    req.on("error", reject);
    req.end();
  });
}

class CDP {
  constructor(ws) {
    this.ws = ws; this.id = 0; this.pending = new Map(); this.events = [];
    ws.onMessage = (m) => {
      let o; try { o = JSON.parse(m); } catch (e) { return; }
      if (o.id !== undefined && this.pending.has(o.id)) {
        const p = this.pending.get(o.id); this.pending.delete(o.id);
        o.error ? p.reject(new Error(o.error.message)) : p.resolve(o.result);
      } else if (o.method) this.events.push(o);
    };
  }
  send(method, params = {}) {
    const id = ++this.id;
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.ws.sendText(JSON.stringify({ id, method, params }));
    });
  }
  async eval(expr, awaitPromise = false, returnByValue = true, timeout = 15000) {
    const r = await this.send("Runtime.evaluate", { expression: expr, awaitPromise, returnByValue, timeout });
    if (r && r.exceptionDetails) throw new Error("eval exception: " + JSON.stringify(r.exceptionDetails));
    return r && r.result ? r.result.value : undefined;
  }
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function waitFor(cdp, expr, ms = 20000) {
  const t0 = Date.now();
  while (Date.now() - t0 < ms) {
    try { if (await cdp.eval(expr)) return true; } catch (e) {}
    await sleep(300);
  }
  return false;
}

(async () => {
  let exitCode = 0;
  let cdp;
  try {
    const ws = await connectCDP(CDP_PORT);
    cdp = new CDP(ws);
    await cdp.send("Runtime.enable");
    await cdp.send("Page.enable");

    // 注入 WebSocket.send 钩子，捕获网页发出的所有 WS 报文
    await cdp.send("Page.addScriptToEvaluateOnNewDocument", {
      source: `(function(){ window.__wsSent=[]; var o=WebSocket.prototype.send; WebSocket.prototype.send=function(d){ try{ window.__wsSent.push(String(d)); }catch(e){} return o.apply(this, arguments); }; })();`,
    });

    await cdp.send("Page.navigate", { url: PAGE_URL });

    // 等待 Three.js 加载、摇杆场景初始化
    const okScene = await waitFor(cdp, "!!(window.ArmScene && window.ArmWS && Array.isArray(window.__wsSent) && window.THREE)", 25000);
    console.log("[test] 场景/WS 初始化:", okScene ? "OK" : "FAIL");
    if (!okScene) { console.log("[test] 页面未能初始化（可能是 CDN/WebGL 问题）"); process.exit(1); }

    const threeVer = await cdp.eval("window.THREE && window.THREE.REVISION");
    const canvasInfo = await cdp.eval(`(function(){var c=document.querySelector('#joystick-container canvas');return c?{w:c.clientWidth,h:c.clientHeight}:null;})()`);
    console.log("[test] Three.js REVISION:", threeVer, " canvas:", JSON.stringify(canvasInfo));

    // 计算 knob 与拖拽目标点的屏幕坐标（世界坐标依据 joystick3d.js 几何）
    const drag = await cdp.eval(`(function(){
      var THREE=window.ArmScene.THREE, cam=window.ArmScene.camera;
      var canvas=document.querySelector('#joystick-container canvas');
      function scr(x,y,z){var v=new THREE.Vector3(x,y,z);v.project(cam);var r=canvas.getBoundingClientRect();return {x:(v.x*0.5+0.5)*r.width+r.left,y:(-v.y*0.5+0.5)*r.height+r.top};}
      // knob 世界坐标: joy(-2.2,0,0)+stick(0,0.35,0)+knob(0,1.5,0)
      var knob=scr(-2.2,1.85,0);
      // 目标平面 y=1.4, x≈+0.8 => hit.x=-2.2+0.8*1.15=-1.28 ; y≈+0.8 => hit.z=-0.8*1.15=-0.92
      var target=scr(-1.28,1.4,-0.92);
      var before=(window.__wsSent||[]).length;
      canvas.dispatchEvent(new PointerEvent('pointerdown',{clientX:knob.x,clientY:knob.y,bubbles:true,cancelable:true}));
      window.dispatchEvent(new PointerEvent('pointermove',{clientX:target.x,clientY:target.y,bubbles:true,cancelable:true}));
      return {knob:knob,target:target,before:before};
    })()`);
    console.log("[test] 派发拖拽: knob=", JSON.stringify(drag.knob), " target=", JSON.stringify(drag.target));

    await sleep(700); // 等 WS 报文发出 + 服务端处理

    const sent = await cdp.eval("window.__wsSent");
    // 解析 joy 报文
    const joys = sent.filter((s) => s.indexOf('"t":"joy"') >= 0).map((s) => { try { return JSON.parse(s); } catch (e) { return null; } }).filter(Boolean);
    const pings = sent.filter((s) => s.indexOf('"t":"ping"') >= 0).length;

    console.log("[test] 浏览器共发出 WS 报文:", sent.length, " (ping:" + pings + ", joy:" + joys.length + ")");
    console.log("[test] 摇杆拖拽产生的 joy 报文:");
    joys.forEach((j) => console.log("        " + JSON.stringify(j)));

    // 校验：应至少有一个非零摇杆坐标（朝目标方向）
    const tilted = joys.find((j) => Math.abs(j.x) > 0.3 || Math.abs(j.y) > 0.3);
    if (pings >= 1 && joys.length >= 1 && tilted) {
      console.log("[test] 结果: PASS —— 摇杆拖拽已生成归一化坐标并经 WS 下发（含非零方向）");
    } else {
      console.log("[test] 结果: WARN —— 未捕获到预期 joy（可能为无头环境 raycast 误差；见下方服务端日志交叉验证）");
      exitCode = 2;
    }

    // 直接调用 sendJoy 作为二次验证（不依赖 raycast 命中），确保协议/服务端链路
    const direct = await cdp.eval(`(function(){ if(!window.ArmWS) return false; window.ArmWS.sendJoy("L",0.9,-0.4); return true; })()`);
    await sleep(400);
    const sent2 = await cdp.eval("window.__wsSent");
    const joy2 = sent2.filter((s) => s.indexOf('"t":"joy"') >= 0);
    console.log("[test] 直接 sendJoy(0.9,-0.4) 触发报文数:", joy2.length, direct ? "(已调用)" : "(ArmWS 缺失)");

    await cdp.send("Page.stopLoading").catch(() => {});
  } catch (e) {
    console.log("[test] 异常:", e.message);
    exitCode = 1;
  } finally {
    if (cdp) try { await cdp.send("Browser.close"); } catch (e) {}
  }
  process.exit(exitCode);
})();
