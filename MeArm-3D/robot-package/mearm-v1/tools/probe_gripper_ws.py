#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""直连后端 WebSocket，实时记录 gripper 命令/状态往返（定位"偶发不顺畅"）。

## 为什么要走 WS 而不是轮询 /healthz

`/healthz` 只暴露 `lastState`（设备回读、由舵机角反算）。而"指令有没有真的下发成功"
要看**下行命令** vs **上行状态**的对应关系 —— 只有 WS 流量里有这个信息：

    下行  joint_command { joints: {..., gripper: X} }
    上行  joint_state   { joints: {..., gripper: Y} }   ← Y 由舵机角反算
    上行  error         { code..., message }             ← 越界/超时在这里暴露

## 判据（三条，互相独立）

  1. **下发是否被拒**：出现 `error` 且 code=JOINT_LIMIT ⇒ 命令没穿过 controller
  2. **命令-状态分叉**：某次 joint_command 的 gripper=X 之后，**再没有**任何
     joint_state 报出 ≈X ⇒ 设备侧没接受（链路丢 或 固件没改 target）
  3. **往返时延**：command 发出 → 首个带该值的 state 到达的毫秒数

用法：
    <python> robot-package/mearm-v1/tools/probe_gripper_ws.py --seconds 45
"""
from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import time


def ws_connect(host: str, port: int, path: str, timeout: float = 5.0):
    s = socket.create_connection((host, port), timeout=timeout)
    key = "dGhlIHNhbXBsZSBub25jZQ=="
    req = (
        f"GET {path} HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n"
    )
    s.sendall(req.encode())
    # 读握手响应头
    buf = b""
    while b"\r\n\r\n" not in buf:
        chunk = s.recv(4096)
        if not chunk:
            raise RuntimeError("握手期间连接关闭")
        buf += chunk
    if b"101" not in buf.split(b"\r\n")[0]:
        raise RuntimeError(f"握手失败: {buf[:200]!r}")
    s.settimeout(0.2)
    return s


def ws_send(s, obj) -> None:
    data = json.dumps(obj).encode()
    mask = b"\x00\x00\x00\x00"
    n = len(data)
    header = bytearray([0x81])
    if n < 126:
        header.append(0x80 | n)
    elif n < 65536:
        header.append(0x80 | 126)
        header += struct.pack(">H", n)
    else:
        header.append(0x80 | 127)
        header += struct.pack(">Q", n)
    s.sendall(bytes(header) + mask + data)


def ws_recv(s):
    """返回 (opcode, payload) 或 None。"""
    try:
        hdr = s.recv(2)
    except (socket.timeout, BlockingIOError):
        return None
    if len(hdr) < 2:
        return None
    opcode = hdr[0] & 0x0F
    length = hdr[1] & 0x7F
    if length == 126:
        length = struct.unpack(">H", s.recv(2))[0]
    elif length == 127:
        length = struct.unpack(">Q", s.recv(8))[0]
    payload = b""
    while len(payload) < length:
        chunk = s.recv(length - len(payload))
        if not chunk:
            break
        payload += chunk
    return opcode, payload


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="WS 直连记录 gripper 往返")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--path", default="/ws/joint")
    ap.add_argument("--seconds", type=float, default=45.0)
    ap.add_argument("--profile", default="",
                    help="空=纯旁听（不动硬件）；normal=温和往返；fast=高频往返")
    ap.add_argument("--cycles", type=int, default=6)
    args = ap.parse_args(argv)

    s = ws_connect(args.host, args.port, args.path)
    print(f"[ws] 已连接 ws://{args.host}:{args.port}{args.path}")
    print(f"[profile] {args.profile or 'listen-only'} · {args.seconds:.0f}s")
    print()

    t0 = time.monotonic()
    end = t0 + args.seconds
    seq = 0
    pending: dict[float, float] = {}     # 目标 grip 值 -> 发出时刻
    cmds: list[tuple[float, float]] = []
    states: list[tuple[float, float]] = []
    errors: list[tuple[float, str]] = []
    last_cmd = None

    # 驱动器（仅当指定 profile 时才发命令）
    next_drive = t0 + 1.0
    cyc = 0

    while time.monotonic() < end:
        # ---- 收 ----
        got = ws_recv(s)
        if got:
            opcode, payload = got
            if opcode == 0x1:
                try:
                    msg = json.loads(payload.decode())
                except Exception:
                    continue
                t = time.monotonic() - t0
                mtype = msg.get("type")
                if mtype == "joint_state":
                    g = msg.get("joints", {}).get("gripper")
                    if g is not None:
                        states.append((t, g))
                        if last_cmd is not None and abs(g - last_cmd) < 1.0:
                            dt = (time.monotonic() - pending.get(last_cmd, t0)) * 1000
                            print(f"  {t:6.2f}s  ✅ state grip={g:6.2f}  "
                                  f"（对应 command {last_cmd:.2f}，往返 {dt:.0f}ms）")
                            last_cmd = None
                elif mtype == "joint_command":
                    g = msg.get("joints", {}).get("gripper")
                    if g is not None:
                        cmds.append((t, g))
                elif mtype == "error":
                    errors.append((t, msg.get("message", "")))
                    print(f"  {t:6.2f}s  ❌ ERROR {msg.get('code')}: {msg.get('message')}")
            elif opcode == 0x8:
                print("[ws] 服务端关闭连接")
                break

        # ---- 发（驱动模式）----
        if args.profile and time.monotonic() >= next_drive and cyc < args.cycles:
            target = 10.0 if cyc % 2 == 0 else 100.0
            seq += 1
            ws_send(s, {"version": 1, "type": "joint_command",
                        "timestamp": int(time.time() * 1000), "seq": seq,
                        "joints": {"base": 0, "shoulder": 1,
                                   "elbow": 112.7, "gripper": target}})
            pending[target] = time.monotonic()
            last_cmd = target
            t = time.monotonic() - t0
            print(f"  {t:6.2f}s  → command grip={target:6.2f}  （seq={seq}）")
            cyc += 1
            next_drive = time.monotonic() + (0.35 if args.profile == "normal" else 0.06)

    s.close()

    print()
    print("=" * 68)
    print("汇总")
    print("=" * 68)
    print(f"下行 joint_command : {len(cmds)}")
    print(f"上行 joint_state   : {len(states)}")
    print(f"error 事件         : {len(errors)}")
    if cmds:
        gc = [g for _, g in cmds]
        print(f"命令 gripper 范围  : {min(gc):.1f} .. {max(gc):.1f}")
    if states:
        gs = [g for _, g in states]
        print(f"状态 gripper 范围  : {min(gs):.1f} .. {max(gs):.1f}")
    if not cmds and args.profile:
        print("❌ 没有收到任何 joint_command 回显 —— 本机可能没连上浏览器（clients 计数）")
    elif cmds and states:
        print("ℹ️  命令与状态均在流动。若真机仍顿挫 ⇒ 属物理层/固件斜坡，"
              "joint_state 是开环值，证明不了到位")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
