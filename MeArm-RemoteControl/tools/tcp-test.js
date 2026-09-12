// tools/tcp-test.js — 模拟局域网 TCP 客户端，验证 TCP -> 服务器 -> 串口 转发链路。
//
// 用法: node tools/tcp-test.js [host] [port]
// 默认 host=127.0.0.1 port=9001
//
// 不依赖任何第三方库（仅用 Node 内置 net）。启动 arm-web 后用此脚本即可验证：
//   1) TCP 收到欢迎语；
//   2) 逐条 arm-device 指令被正确透传并写串口（服务器 debug 日志 [serial] TX ...）；
//   3) 高频 burst 时服务器按“命令-应答门控”只下发最新值、且不出现“命令队列满，丢弃”。
//
// 注意：命令-应答门控逻辑（等待下位机应答、超时通讯失败、latest-wins 合并）由
// internal/serial/ack_test.go 单测覆盖；本脚本主要用于在真机环境确认转发内容正确。
//
// 服务器侧需以 log_level: debug 启动才能看到 TX 日志。

const net = require("net");
const host = process.argv[2] || "127.0.0.1";
const port = parseInt(process.argv[3] || "9001", 10);

const sock = net.connect(port, host, () => {
  console.log(`[tcp-test] 已连接 ${host}:${port}`);

  // 1) 离散指令（真实控制场景）
  const discrete = [
    "STATUS",          // 查询状态
    "JOY 7 29 512 512", // 摇杆极端位置（底座/左舵各走一端）
    "S9=120",          // 单舵设置
    "JOYHW OFF",       // 关闭硬件摇杆扫描
    "RESET",           // 复位
  ];
  let i = 0;
  const sendDiscrete = () => {
    if (i >= discrete.length) {
      // 2) 高频 burst：连续 25 条 JOY，验证节流（不应出现队列满丢弃）
      console.log("[tcp-test] 开始高频 burst (25 条 JOY，间隔 ~1ms)");
      for (let k = 0; k < 25; k++) {
        const x = ((k % 5) - 2); // -2..2
        sock.write(`JOY ${512 + x * 100} 512 512 512\r\n`);
      }
      // 3) 回中
      setTimeout(() => sock.write("JOY 512 512 512 512\r\n"), 200);
      setTimeout(() => { console.log("[tcp-test] 完成，关闭连接"); sock.end(); }, 1200);
      return;
    }
    sock.write(discrete[i] + "\r\n");
    i++;
    setTimeout(sendDiscrete, 80);
  };
  setTimeout(sendDiscrete, 100);
});

sock.setEncoding("utf8");
sock.on("data", (chunk) => {
  // 服务器把串口回显广播回 TCP 客户端；无设备时通常无内容，但欢迎语/错误会有
  process.stdout.write("[tcp<-server] " + chunk);
});
sock.on("error", (e) => {
  console.error("[tcp-test] 连接错误:", e.message);
  process.exit(1);
});
sock.on("close", () => {
  console.log("[tcp-test] 连接关闭");
  process.exit(0);
});
