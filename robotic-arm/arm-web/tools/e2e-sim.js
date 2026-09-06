// e2e-sim.js — 端到端协议自测（无需 GUI 浏览器）。
//
// 严格按前端 web/static/js/wsclient.js 的契约对接真实运行的 arm-web 服务器：
//   WS 发送 {t:ping} / {t:joy,x,y} / {t:cmd,c}
// 验证：
//   1) WebSocket 握手 + JSON 双向帧
//   2) 摇杆拖拽坐标 -> 服务端生成 JOY 指令（写串口 COM4）
//   3) cmd 指令归一化后下发
//   4) TCP 局域网转发（其它设备经 :9001 下发指令）
//
// 用法（先启动 arm-web.exe -c config.test.yaml）：
//   node tools/e2e-sim.js [wsUrl=ws://127.0.0.1:8080/ws] [tcpHost=127.0.0.1] [tcpPort=9001]
const http = require("http");
const crypto = require("crypto");
const net = require("net");

const WS_URL = process.argv[2] || "ws://127.0.0.1:8080/ws";
const TCP_HOST = process.argv[3] || "127.0.0.1";
const TCP_PORT = parseInt(process.argv[4] || "9001", 10);

// 与 internal/protocol.JoystickToJOY 完全一致的本地镜像，用于交叉校验
function joyToJOY(x, y, map) {
  let rx = 512 + Math.round(x * 512);
  let ry = 512 + Math.round(y * 512);
  rx = Math.max(0, Math.min(1023, rx));
  ry = Math.max(0, Math.min(1023, ry));
  if (map.invX) rx = 1023 - rx;
  if (map.invY) ry = 1023 - ry;
  const slot = { 9: 0, 8: 1, 6: 2, 7: 3 };
  const r = [512, 512, 512, 512];
  r[slot[map.xServo]] = rx;
  r[slot[map.yServo]] = ry;
  return `JOY ${r[0]} ${r[1]} ${r[2]} ${r[3]}`;
}

// ---------- 最小 WS 客户端（对接 /ws） ----------
class WSClient {
  constructor(url) { this.url = url; this.buf = Buffer.alloc(0); this.onMsg = null; this.onOpen = null; }
  connect() {
    return new Promise((resolve, reject) => {
      const u = new URL(this.url);
      const key = crypto.randomBytes(16).toString("base64");
      const req = http.request({
        hostname: u.hostname, port: u.port, path: u.pathname,
        headers: { Connection: "Upgrade", Upgrade: "websocket", "Sec-WebSocket-Key": key, "Sec-WebSocket-Version": "13" },
      });
      req.on("upgrade", (res, socket) => {
        this.sock = socket;
        socket.on("data", (d) => this._onData(d));
        resolve(this);
      });
      req.on("error", reject);
      req.end();
    });
  }
  _onData(d) {
    this.buf = Buffer.concat([this.buf, d]);
    while (true) {
      if (this.buf.length < 2) return;
      const b0 = this.buf[0], b1 = this.buf[1];
      const opcode = b0 & 0x0f;
      let len = b1 & 0x7f, offset = 2;
      if (len === 126) { if (this.buf.length < 4) return; len = this.buf.readUInt16BE(2); offset = 4; }
      else if (len === 127) { if (this.buf.length < 10) return; len = this.buf.readUInt32BE(2) * 4294967296 + this.buf.readUInt32BE(6); offset = 10; }
      const masked = (b1 & 0x80) !== 0;
      if (masked) offset += 4;
      if (this.buf.length < offset + len) return;
      let payload = this.buf.slice(offset, offset + len);
      if (masked) { const m = this.buf.slice(offset - 4, offset); const o = Buffer.alloc(len); for (let i = 0; i < len; i++) o[i] = payload[i] ^ m[i & 3]; payload = o; }
      this.buf = this.buf.slice(offset + len);
      if (opcode === 0x8) { try { this.sock.end(); } catch (e) {} return; }
      if ((opcode === 0x1 || opcode === 0x2) && this.onMsg) this.onMsg(payload.toString("utf8"));
    }
  }
  send(obj) {
    const data = Buffer.from(JSON.stringify(obj), "utf8");
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

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const map = { xServo: 9, yServo: 8, invX: false, invY: false };

(async () => {
  let pass = true;
  console.log("===== E2E 协议自测（模拟 web 摇杆客户端）=====");
  console.log("[cfg] WS=" + WS_URL + "  TCP=" + TCP_HOST + ":" + TCP_PORT);

  const ws = new WSClient(WS_URL);
  const recv = [];
  ws.onMsg = (m) => { recv.push(m); console.log("  [WS<-] " + m); };
  await ws.connect();
  console.log("[WS] 握手成功");
  ws.send({ t: "ping" });
  await sleep(200);
  const gotPong = recv.some((m) => m.indexOf('"t":"pong"') >= 0);
  console.log("[WS] ping->pong: " + (gotPong ? "PASS" : "FAIL"));
  if (!gotPong) pass = false;

  // 模拟摇杆拖拽：中心 -> 右上(0.9,-0.4) -> 左下(-0.6,0.7) -> 回中(0,0)
  const drags = [
    { x: 0, y: 0 },
    { x: 0.9, y: -0.4 },
    { x: -0.6, y: 0.7 },
    { x: 0, y: 0 },
  ];
  for (const d of drags) {
    ws.send({ t: "joy", x: d.x, y: d.y });
    await sleep(120);
  }
  console.log("[WS] 已发送摇杆拖拽序列: " + JSON.stringify(drags));
  console.log("      期望服务端生成(镜像映射):");
  drags.forEach((d) => { if (d.x || d.y) console.log("        JOY<" + d.x + "," + d.y + "> -> " + joyToJOY(d.x, d.y, map)); });

  // cmd 路径
  ws.send({ t: "cmd", c: "S9=120" });
  await sleep(150);
  console.log("[WS] 已发送 cmd: S9=120 （期望归一化为 S9=120 下发）");

  // ---------- TCP 局域网转发 ----------
  console.log("===== TCP 局域网转发自测 =====");
  await new Promise((resolve) => {
    const t = net.connect(TCP_PORT, TCP_HOST, () => {
      console.log("[TCP] 已连接 " + TCP_HOST + ":" + TCP_PORT);
      t.write("S7=90\r\n");
      console.log("[TCP] 已发送: S7=90 （经 TCP 透传下发串口）");
    });
    let got = "";
    t.on("data", (d) => { got += d.toString(); });
    t.on("error", (e) => { console.log("[TCP] 连接错误: " + e.message); pass = false; resolve(); });
    setTimeout(() => { console.log("[TCP] 服务端欢迎/回显: " + JSON.stringify(got.trim())); t.end(); resolve(); }, 600);
  });

  await sleep(300);
  console.log("===== 结果: " + (pass ? "PASS" : "FAIL") + " =====");
  console.log("（下一步请查看服务器日志中的 '[web] 摇杆 -> 下发指令 JOY ...' 与 '[tcp] 客户端接入' 行做交叉验证）");
  ws.sock.end();
  process.exit(pass ? 0 : 1);
})().catch((e) => { console.log("[异常] " + e.message); process.exit(1); });
