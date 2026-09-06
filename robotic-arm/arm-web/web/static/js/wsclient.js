// wsclient.js — 浏览器端 WebSocket 封装。
// 对外暴露 window.ArmWS：on(type, fn) / sendJoy(x,y) / sendCmd(text) / isReady()
// 消息类型（服务器->浏览器，JSON）：
//   {t:"serial", line:"...", angles?:{s6,s7,s8,s9,ok}}
//   {t:"err", msg:"..."}   {t:"pong"}
(function () {
  var url = (location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws";
  var ws = null;
  var handlers = { open: [], close: [], serial: [], err: [] };

  function on(type, fn) {
    (handlers[type] || (handlers[type] = [])).push(fn);
  }
  function emit(type, data) {
    (handlers[type] || []).forEach(function (f) { f(data); });
  }

  function connect() {
    try { ws = new WebSocket(url); }
    catch (e) { setTimeout(connect, 1500); return; }

    ws.onopen = function () {
      emit("open");
      send({ t: "ping" });
    };
    ws.onclose = function () {
      emit("close");
      setTimeout(connect, 1500);
    };
    ws.onerror = function () {};
    ws.onmessage = function (ev) {
      var m;
      try { m = JSON.parse(ev.data); } catch (e) { return; }
      if (!m || !m.t) return;
      if (m.t === "serial") {
        emit("serial", m); // 完整对象：main.js 统一解析 line / angles
      } else if (m.t === "serial_status") {
        emit("serial_status", m);
      } else if (m.t === "err") {
        emit("err", m.msg);
      } else if (m.t === "pong") {
        emit("pong", m);
      }
    };
  }

  function send(obj) {
    if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
  }

  window.ArmWS = {
    on: on,
    sendJoy: function (x, y) { send({ t: "joy", x: x, y: y }); },
    sendCmd: function (c) { send({ t: "cmd", c: c }); },
    isReady: function () { return !!(ws && ws.readyState === 1); },
  };

  connect();
})();
