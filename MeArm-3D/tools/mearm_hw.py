#!/usr/bin/env python3
"""MeArm 硬件实测工具：单进程内完成「串口控制 + 相机抓拍」。

为什么必须单进程
----------------
打开 COM16 会拉低 DTR -> ATmega328P 复位 -> 固件把 6/7/8/9 全部驱到 90°。
所以「发一条指令就重开一次串口」会让机械臂每次都弹回 Reset 位。
本脚本一次连接内完成整段 [set -> 稳定 -> 抓拍] 序列。

固件事实（MeArm-Device/bsp/servo.h，权威）
-----------------------------------------
    id 9  SERVO_BASE   D9  底座旋转  硬限位 30..150
    id 8  SERVO_LEFT   D8  左舵(肩)  硬限位 20..100
    id 7  SERVO_RIGHT  D7  右舵(肘)  硬限位 80..160
    id 6  SERVO_GRIP   D6  夹取      硬限位 40..130
开机全部 -> 90°。串口 115200 8N1。

用法
----
    python mearm_hw.py status
    python mearm_hw.py reset
    python mearm_hw.py set 9 60
    python mearm_hw.py shot out.jpg
    python mearm_hw.py sweep 9 30 60 90 120 150 --out-dir caps/sweep_S9
    python mearm_hw.py trace --out-dir caps/trace          # 四舵机 ±delta 方向测试
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import serial

# ---------------------------------------------------------------- 常量

DEFAULT_PORT = "COM16"
DEFAULT_BAUD = 115200
DEFAULT_CAM = "Integrated Camera"
FFMPEG_FALLBACK = r"E:\agent-tools\ffmpeg-master-latest-win64-gpl-shared\bin\ffmpeg.exe"

# id -> (角色, 硬限位下限, 硬限位上限)，取自 MeArm-Device/bsp/servo.h
# 舵机 id -> (角色, 固件硬限位 min, max)
# ⚠️ 角色以 2026-09-12 实拍为准（见 docs/hardware-measurement.md）：
#    S7 = 肩(shoulder)，S8 = 肘(elbow)。固件里的 "left/right" 只是**安装位**称呼，
#    按它分配肩/肘会得到反向的机构 —— 这是本次实测纠正的一处真实缺陷。
SERVO_SPEC = {
    9: ("base", 30, 150),
    7: ("shoulder", 80, 160),
    8: ("elbow", 20, 100),
    6: ("gripper", 40, 130),
}
ALL_IDS = (9, 8, 7, 6)

# 打开串口后等待固件启动、舵机走到 90° 的时间
BOOT_WAIT = 2.2
# 发出 SET 之后等舵机物理到位的时间
SETTLE = 0.85


# ---------------------------------------------------------------- ffmpeg

def find_ffmpeg() -> str:
    exe = shutil.which("ffmpeg")
    if exe:
        return exe
    if Path(FFMPEG_FALLBACK).exists():
        return FFMPEG_FALLBACK
    raise SystemExit("找不到 ffmpeg，请用 --ffmpeg 指定路径")


def grab(cam: str, out: Path, width: int, height: int, ffmpeg: str, retries: int = 3) -> bool:
    """抓一帧存到 out。返回是否成功。"""
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        ffmpeg, "-hide_banner", "-loglevel", "error",
        "-f", "dshow", "-rtbufsize", "100M",
        "-video_size", f"{width}x{height}",
        "-i", f"video={cam}",
        "-frames:v", "1", "-q:v", "2", "-y", str(out),
    ]
    for attempt in range(1, retries + 1):
        p = subprocess.run(cmd, capture_output=True)
        if p.returncode == 0 and out.exists() and out.stat().st_size > 1024:
            return True
        if attempt < retries:
            time.sleep(0.6)
    err = p.stderr.decode("utf-8", "replace").strip()[:300]
    print(f"  [warn] 抓拍失败 ({out.name}): {err}", file=sys.stderr)
    return False


# ---------------------------------------------------------------- 串口

class Arm:
    """串口封装。

    打开 COM 口会拉低 DTR -> Uno 复位，带来两个必须同时处理的时序陷阱：
      坑 1  复位后 optiboot 有约 2.5s 引导窗口，窗口内下发的指令被丢弃
            -> 必须等到开机横幅 "[meArm] ... ready" 出现再发指令
      坑 2  引导交接后链路上的首个数据包仍会被静默吞掉
            -> 先写一个无害的 "\\n" 当暖机包，让真正的指令成为第 2 个包
    只做其中一条都不够。
    """

    def __init__(self, port: str, baud: int, boot_wait: float = BOOT_WAIT):
        self.port = port
        self.ser = serial.Serial(port, baud, timeout=0.35)
        # 显式拉低再回收 DTR/RTS，保证每次连接都产生一次可复现的复位
        self.ser.dtr = False
        self.ser.rts = False
        time.sleep(0.1)
        self.ser.dtr = True
        self.ser.rts = True
        self.banner = self._wait_banner(max(boot_wait, 3.2))
        self.warm = self._warmup()

    def _drain(self, wait: float) -> str:
        time.sleep(wait)
        return self.ser.read(8192).decode("ascii", "replace")

    def _wait_banner(self, timeout: float) -> str:
        """读到开机横幅（含 'ready'）为止；超时也返回已收内容。"""
        t0 = time.time()
        buf = b""
        while time.time() - t0 < timeout:
            chunk = self.ser.read(4096)
            if chunk:
                buf += chunk
                if b"ready" in buf:
                    break
            else:
                time.sleep(0.05)
        self.ser.reset_input_buffer()
        return buf.decode("ascii", "replace")

    def _warmup(self) -> bool:
        """坑 2：先投一个会被吞掉的换行，再确认链路已能应答。"""
        self.ser.write(b"\n")
        self.ser.flush()
        time.sleep(0.15)
        self.ser.reset_input_buffer()
        for _ in range(4):
            self.ser.write(b"STATUS\n")
            self.ser.flush()
            r = self._drain(0.5)
            if "S9=" in r:
                self.ser.reset_input_buffer()
                return True
        return False

    def cmd(self, line: str, settle: float = SETTLE, retry: int = 2) -> str:
        """发一条指令，等舵机到位，返回固件回显。

        SET 是幂等的（同一舵机同角度），所以空回显时重试是安全的；
        重试前补一个暖机包，规避偶发的首包丢失。
        """
        for attempt in range(retry + 1):
            self.ser.reset_input_buffer()
            self.ser.write((line + "\n").encode("ascii"))
            self.ser.flush()
            r = self._drain(settle)
            if r.strip():
                return r
            if attempt < retry:
                self.ser.write(b"\n")
                self.ser.flush()
                time.sleep(0.12)
        return ""

    def status(self) -> str:
        return self.cmd("STATUS", 0.5).strip()

    def set_servo(self, sid: int, angle: int) -> str:
        if sid not in SERVO_SPEC:
            raise SystemExit(f"未知舵机 id {sid}（应为 {sorted(SERVO_SPEC)}）")
        role, lo, hi = SERVO_SPEC[sid]
        if not (lo <= angle <= hi):
            print(f"  [warn] {role} 硬限位 {lo}..{hi}，{angle} 会被固件钳位", file=sys.stderr)
        return self.cmd(f"SET {sid} {angle}").strip()

    def reset(self) -> str:
        return self.cmd("RESET").strip()

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass


# ---------------------------------------------------------------- 子命令

def cmd_status(a: argparse.Namespace) -> int:
    arm = Arm(a.port, a.baud)
    try:
        print(f"banner: {arm.banner.strip() or '(none)'}")
        print(f"warmup: {'OK' if arm.warm else 'FAIL（链路未应答）'}")
        print(f"STATUS: {arm.status() or '(empty)'}")
    finally:
        arm.close()
    return 0


def cmd_reset(a: argparse.Namespace) -> int:
    arm = Arm(a.port, a.baud)
    try:
        print("->", arm.reset() or "(no echo)")
        print("STATUS:", arm.status())
    finally:
        arm.close()
    return 0


def cmd_set(a: argparse.Namespace) -> int:
    arm = Arm(a.port, a.baud)
    try:
        print("->", arm.set_servo(a.id, a.angle) or "(no echo)")
        print("STATUS:", arm.status())
    finally:
        arm.close()
    return 0


def cmd_shot(a: argparse.Namespace) -> int:
    ok = grab(a.cam, Path(a.out), a.width, a.height, find_ffmpeg())
    print(f"shot {'OK' if ok else 'FAIL'} -> {a.out}")
    return 0 if ok else 1


def cmd_sweep(a: argparse.Namespace) -> int:
    """把某个舵机依次置到给定角度，每个角度抓一张图。"""
    ff = find_ffmpeg()
    out_dir = Path(a.out_dir)
    arm = Arm(a.port, a.baud)
    role, lo, hi = SERVO_SPEC[a.id]
    try:
        print(f"sweep S{a.id} ({role}, 硬限位 {lo}..{hi}) -> {out_dir}")
        for ang in a.angles:
            echo = arm.set_servo(a.id, ang)
            tag = f"S{a.id}_{ang:03d}"
            path = out_dir / f"{tag}.jpg"
            ok = grab(a.cam, path, a.width, a.height, ff)
            print(f"  S{a.id}={ang:3d}  {'OK ' if ok else 'FAIL'} {path.name}   echo={echo or '-'}")
        arm.reset()
        print("  已回 RESET(90)")
    finally:
        arm.close()
    return 0


def cmd_trace(a: argparse.Namespace) -> int:
    """方向测试：每个舵机单独 90 -> 90-delta -> 90+delta，各抓一图。

    这是 spec §Phase10 要求的「先单关节 ±5°，再 ±10°」安全流程。
    图像里只要看某个连杆朝哪边动，就能定出 reverse 的符号。
    """
    ff = find_ffmpeg()
    out_dir = Path(a.out_dir)
    d = a.delta
    arm = Arm(a.port, a.baud)
    try:
        arm.reset()
        base = out_dir / "reset_90.jpg"
        grab(a.cam, base, a.width, a.height, ff)
        print(f"reset  90/90/90/90  -> {base.name}")
        for sid in ALL_IDS:
            if a.only and sid != a.only:
                continue
            role = SERVO_SPEC[sid][0]
            for ang, tag in ((90 - d, "minus"), (90 + d, "plus"), (90, "back")):
                arm.set_servo(sid, ang)
                path = out_dir / f"S{sid}_{tag}_{ang:03d}.jpg"
                ok = grab(a.cam, path, a.width, a.height, ff)
                print(f"  S{sid} {role:14s} {tag:5s} = {ang:3d}  {'OK ' if ok else 'FAIL'} {path.name}")
        arm.reset()
        print("已回 RESET(90)")
    finally:
        arm.close()
    return 0


# ---------------------------------------------------------------- main

def main() -> int:
    ap = argparse.ArgumentParser(description="MeArm 硬件实测（串口控制 + 相机抓拍）")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--cam", default=DEFAULT_CAM)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    sub = ap.add_subparsers(dest="op", required=True)

    sub.add_parser("status").set_defaults(fn=cmd_status)
    sub.add_parser("reset").set_defaults(fn=cmd_reset)

    p = sub.add_parser("set");  p.add_argument("id", type=int); p.add_argument("angle", type=int)
    p.set_defaults(fn=cmd_set)

    p = sub.add_parser("shot"); p.add_argument("out")
    p.set_defaults(fn=cmd_shot)

    p = sub.add_parser("sweep")
    p.add_argument("id", type=int)
    p.add_argument("angles", type=int, nargs="+")
    p.add_argument("--out-dir", required=True)
    p.set_defaults(fn=cmd_sweep)

    p = sub.add_parser("trace")
    p.add_argument("--out-dir", required=True)
    p.add_argument("--delta", type=int, default=10)
    p.add_argument("--only", type=int, default=0, help="只测某个舵机 id")
    p.set_defaults(fn=cmd_trace)

    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    raise SystemExit(main())
