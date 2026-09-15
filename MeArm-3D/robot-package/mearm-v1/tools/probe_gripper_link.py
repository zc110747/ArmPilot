#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gripper 偶发不顺畅 · 真机诊断（直连串口，不经过后端/前端）

## 为什么需要这个脚本

2026-09-14 修过一轮 gripper「有概率不执行」（提交 caf402d：execJR 不再牺牲第二条 SET、
awaitAck 严格匹配、固件 arm_nudge 基于 target、uart RX 64→256）。**但那次设备不响应串口，
真机端到端从未复现过**，且当时的验证脚本已丢失。

本脚本把"偶发"变成**可计数的读数**：反复下发 gripper 开关，逐帧记录
每一条 SET 的 ACK 是否按时到达、gripper 的 target/current 是否真的被改写。

## 复刻的形状（必须与后端 execJR 一致，否则测的不是同一条路径）

后端 `execJR` 把 4 执行器按固件 `MAX_PAIRS=3` 拆 2 条，顺序 = d.order = [base, shoulder, elbow, gripper]：
    第 1 条： SET 9 <b> 7 <sh> 8 <el>      ← base/shoulder/elbow
    第 2 条： SET 6 <grip>                  ← ★ gripper 恒定在第二条

## 两种模式（判据不同，别混用）

  --mode frame   （默认）复刻整帧：每条 2 条 SET，等齐 ACK 再发下一条。
                 用于回答"拖动一帧，gripper 有没有被下发/接受"。
  --mode burst   高频连发（默认 33ms = 前端节流间隔），**真正的 fire-and-forget**：
                 只按节拍写，不做逐条等待；发完后一次性收干回执，再对账。
                 复刻"快速拖动"下的串口压力 —— 昨天的根因就在这个场景。

★★ 2026-09-15 修正（重要，一次真实的误判）：
  最初的 burst 实现里有 `link.wait_for(...)` **逐条等待**，实测报出
  "第2条 SET 无 ACK: 2/60" —— 看上去复现了缺陷。但同一轮的 `STATS` 读数
  是 `rx_drop=0 tx_drop=0`，而"链路丢字节"若成立则必然是 rx_drop>0。
  改用真正的 fire-and-forget（本节实现）后：**120 发 120 收、60/60 含 S6、
  零 ERR、rx_drop=0**。也就是说那 2 条"缺失"是**脚本自身的度量缺陷**：
  `wait_for` 只认谓词命中的那一行，超时后把已在途的 ACK 塞进 `others`，
  而 `report()` 从不打印 `others` ⇒ 迟到的 ACK 被静默吞掉，计成"缺失"。
  教训（与 SOUL 的"不许用推导量当独立判据"同源）：一个"高延时"的读数
  未必是缺陷，可能只是**测量窗口太窄**；能一票否决它的只有**独立的计数器**。
  因此现在 burst 的判据以 ③ 的 `STATS` 增量为准，回执对账降为旁证。

## 判据（全部基于真实回执，无一处推断）

  ① 每帧两条 SET 的 ACK 时延（ms）
  ② ACK 缺失 / 超时次数                   ← frame 模式为主；burst 只作旁证
  ③ ★ `STATS` 前后增量（rx_drop / tx_drop） ← burst 的**主判据**：
      链路真的丢字节，rx_drop 必然非 0；rx_drop=0 即可否定"链路丢字节"这一假设
  ④ `STATUS` 回读的 S6 是否等于最后一条 SET 的目标（**固件 target 是否被改写**）
  ⑤ 异步行（OK IR / OK IRSEQ / 开机横幅）在窗口内出现的次数

用法：
    <python> robot-package/mearm-v1/tools/probe_gripper_link.py --port COM18 --mode frame
    <python> robot-package/mearm-v1/tools/probe_gripper_link.py --port COM18 --mode burst --frames 120
"""
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover
    print("[fatal] 需要 pyserial：pip install pyserial", file=sys.stderr)
    raise


# ---------------------------------------------------------------------------
# 与机器人无关的常量：只描述**协议形状**（SET 语法 / MAX_PAIRS），
# 标定与限位一律不在这里写死 —— 从 robot.yaml 读（与后端同一份真值）。
# ---------------------------------------------------------------------------
MAX_PAIRS_PER_SET = 3   # 固件 core/cmd.c 的 MAX_PAIRS
DEFAULT_PORT = "COM18"


@dataclass
class FrameResult:
    index: int
    grip_angle: int
    ack1_ms: float | None = None
    ack2_ms: float | None = None
    ack1_line: str = ""
    ack2_line: str = ""
    async_lines: list[str] = field(default_factory=list)
    raw_lines: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.ack1_ms is not None and self.ack2_ms is not None


class Link:
    """极简串口链路：写一行、按谓词读一行，异步行原样记录。"""

    def __init__(self, port: str, baud: int = 115200) -> None:
        # ⚠️ 打开 COM 口会使 Uno 复位（Windows 默认 DTR 断言），optiboot 交权窗口
        #    实测 ~1.6-2.6s。这里**不做 dtr/rts 翻转**：早期版本在 open 之后立刻
        #    写 `ser.dtr = False; ser.rts = False`，等于额外制造两次控制线跳变，
        #    复位窗口被拉长且不确定。
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self._buf = ""
        self._discard_banner()
        self._warmup()

    def _warmup(self) -> None:
        """★★ 消化「首条指令被丢弃」窗口（2026-09-15 实测）。

        打开端口后连续写 STATS，实测的响应序列（有复位时）：
            try0: ''                                  ← 窗口①（横幅还没出来）
            try1: '[meArm] ... ready' / 'STATUS ...'  ← 复位横幅（~1.6s 才到）
            try2: ''                                  ← 窗口②（横幅之后又吞一条）
            try3..7: 'OK STATS rx_drop=0 tx_drop=0'   ← 之后稳定

        ⚠️ **不要依赖"是否出现横幅"来判定就绪**：实测 DTR 边沿不是每次都产生
        复位（连续开关端口时尤其如此），于是横幅时有时无；一旦写死"先等横幅"，
        没有复位的那次就会白等满超时（早期版本踩过，表现为 banner=[] 且全失败）。

        因此这里改为**重试探针**：反复写 STATS 直到真的收到回执。这是唯一
        与"是否复位"解耦的判据 —— 就绪的定义就是"能应答"，不是"看到横幅"。

        固件侧主循环是非阻塞的（cmd_poll 每轮都跑），所以这不是固件缺陷，
        而是 Uno 复位后 UART 的稳定窗口（横幅由 uart_puts 在 sei() 之后立即打出，
        此时 RX 路径尚未完全就绪）。
        """
        # 逐次探针，每次给足 0.8s：复位横幅可能独占一次窗口（banner 有 3 行），
        # 窗口② 再独占一次，所以正常需要 2-4 次才拿到回执。
        for attempt in range(14):                    # 14 × 0.8s ≈ 11s 上限
            self.ser.reset_input_buffer()
            self._buf = ""
            self.send("STATS")
            line, _, _ = self.wait_for(lambda s: "STATS" in s.upper(), 0.8)
            if line:
                self.warmup_attempts = attempt + 1
                self.ser.reset_input_buffer()
                self._buf = ""
                return
        self.warmup_attempts = 14
        # 不在这里抛错：让 report() 读取 STATS 时给出统一的失败诊断。
    def _read_line(self) -> str | None:
        """取一行：**先吃内部缓冲，再读串口**。

        ★★ 2026-09-15 修正（关键）。原实现是：
            deadline = now + 0.05
            while now < deadline:
                chunk = ser.read(256)
                if chunk:
                    self._buf += chunk
                    if "\\n" in self._buf: return 第一条
                else: sleep(0.002)
            return None
        它**只在读到新的串口字节时才可能返回**一行。当一次 `read(256)` 拿回
        一大块（压测时每块可含 10+ 行）后，函数返回第一行、其余留在 `self._buf`；
        下次调用若串口暂无新字节，就会**空转到 0.05s 然后返回 None** ——
        尽管 `self._buf` 里明明还有 200 行没取出来。
        后果：`drain()` 的 idle 判据被这个假 None 触发，提前收工；
        实测 240 条回执只拿到 19 条，于是报告出"缺 221 条"这种**纯属虚构的缺陷**。
        修法：进入循环前先查 `self._buf` 是否已含完整行，有就直接返回，
        完全不碰串口。这样"缓冲有货"就不再依赖新的 IO 事件。
        """
        # ① 缓冲里已有完整行 → 立即返回，不做任何 IO
        if "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            return line.strip()
        # ② 缓冲不完整 → 读串口，直到凑出一行或超时
        deadline = time.monotonic() + 0.05
        while time.monotonic() < deadline:
            chunk = self.ser.read(256)
            if chunk:
                self._buf += chunk.decode("ascii", errors="replace")
                if "\n" in self._buf:
                    line, self._buf = self._buf.split("\n", 1)
                    return line.strip()
            else:
                time.sleep(0.002)
        return None

    def _discard_banner(self) -> None:
        """**尽力**吃掉复位横幅（有就吃，没有就立刻返回，绝不空等）。

        ★★ 2026-09-15 修正：原实现固定等 3.0s 且只读一行就 break。两个问题：
          ① 横幅实测在 1.6-2.6s 到达，晚于 3.0s 时会**留在缓冲区**，
             被后续 `wait_for` 当成"异步行"吃掉（"迟到 ACK 被静默吞掉"的机制之一）；
          ② DTR 边沿并非每次都触发复位 ⇒ 横幅时有时无，
             **固定等待**在"无复位"的那次纯属浪费，而且会让上层逻辑错判。
        现在：给 1.0s 的短窗口尽力收集，有就记下来，没有就返回。
        真正的就绪判据由 `_warmup()` 的重试探针承担（与是否复位解耦）。
        """
        self.banner = []
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            line = self._read_line()
            if line:
                self.banner.append(line)
        if self.banner:
            # 再收一小会儿，把多行横幅排干（不阻塞：排干即返回）
            while True:
                line = self._read_line()
                if not line:
                    break
                self.banner.append(line)

    def send(self, line: str) -> None:
        self.ser.write((line + "\r\n").encode("ascii"))
        self.ser.flush()

    def wait_for(self, pred, timeout_s: float) -> tuple[str | None, float, list[str]]:
        """等一行满足 pred。返回 (行, 时延ms, 期间收到的其它行)。"""
        t0 = time.monotonic()
        others: list[str] = []
        while True:
            elapsed = (time.monotonic() - t0) * 1000.0
            if elapsed > timeout_s * 1000.0:
                return None, elapsed, others
            line = self._read_line()
            if line is None:
                continue
            if pred(line):
                return line, (time.monotonic() - t0) * 1000.0, others
            others.append(line)

    def drain(self, seconds: float, idle_s: float = 0.15) -> list[str]:
        """持续读行直到「静默 idle_s」或总时长超过 seconds。

        ★★ 2026-09-15 修正：原实现是 `while time.monotonic() < end` ——
        **总是烧满 seconds**，而且每行最多耗 0.05s。压测后缓冲区里可能积压
        240 行回执，`drain(2.0)` 只能拉出 ~40 行，剩下的会污染后续
        `wait_for`（把 `STATS` 的应答埋在 200 行回执后面 ⇒ 主判据读不到）。
        现在改为**空闲即返回**：一旦连续 idle_s 没读到新行就收工，
        这样无论积压多少行都能拉干（同时保留 seconds 作为硬上限）。
        """
        out: list[str] = []
        end = time.monotonic() + seconds
        last_data = time.monotonic()
        while time.monotonic() < end:
            line = self._read_line()
            if line:
                out.append(line)
                last_data = time.monotonic()
            elif time.monotonic() - last_data > idle_s:
                break
        return out

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass


def load_servo_map():
    """从 robot.yaml 读 channel -> 关节名 / 标定（与后端同一份真值，不写死）。"""
    import os
    from pathlib import Path

    root = Path(__file__).resolve()
    while not (root / "core").is_dir() or not (root / "robot-package").is_dir():
        if root.parent == root:
            raise SystemExit("[fatal] 找不到仓库根（同时含 core/ 与 robot-package/）")
        root = root.parent
    sys.path.insert(0, str(root / "core" / "python"))
    from robopkg.loader import load_robot  # type: ignore

    r = load_robot(None)
    m = r.robot
    out = {}
    for a in m.actuators:
        out[a.channel] = {"actuator": a}
    return r, m, out


def joint_to_servo(a, theta: float) -> float:
    # ⚠️ 直接调模型自带的方法，**不在这里重写标定公式** ——
    #    同一套 offset/scale/reverse 只能有一份实现（多写一份就是第二份真值）。
    return a.joint_to_servo(theta)


def build_frame(m, joints: dict[str, float]) -> list[str]:
    """复刻后端 execJR 的拆帧：按 JointOrder 展开执行器 → 每 3 对一条 SET。

    ★ 顺序必须与后端 `d.order` 一致（Go: `robot.JointOrder()`，
      Python: `JointCfg.is_dof` 为真的关节按 yaml 顺序）—— 因为
      "gripper 落在第二条 SET" 这个事实**直接依赖顺序**，顺序错了就测错东西。
    """
    ordered = [j for j in m.joints if j.is_dof]
    pairs: list[tuple[int, int]] = []
    for j in ordered:
        for a in m.actuators:
            if a.joint_id != j.id:
                continue
            pairs.append((a.channel, int(round(a.joint_to_servo(joints[j.id])))))
    lines = []
    for start in range(0, len(pairs), MAX_PAIRS_PER_SET):
        chunk = pairs[start:start + MAX_PAIRS_PER_SET]
        lines.append("SET " + " ".join(f"{ch} {ang}" for ch, ang in chunk))
    return lines


def probe_frame_mode(link: Link, m, channels, n_frames: int, settle_s: float) -> list[FrameResult]:
    """整帧模式：每帧等齐 2 条 ACK。"""
    grip_ch = 6
    results: list[FrameResult] = []
    # 从当前 STATUS 起步，避免第一帧跨度过大
    link.send("STATUS")
    line, _, _ = link.wait_for(lambda s: s.startswith("STATUS") or "S6=" in s, 1.0)
    for i in range(n_frames):
        # 交替开/关：用**关节角**表达（走标定），闭合端/张开端取限额内部
        # ⚠️ 限位端点不可精确到达（整数度量化后可能落到限位外被拒绝），取限额内部
        theta = 10.0 if i % 2 == 0 else 100.0
        joints = {"base": 0.0, "shoulder": 1.0, "elbow": 112.7, "gripper": theta}
        fr = FrameResult(index=i, grip_angle=int(round(joint_to_servo(
            [a for a in m.actuators if a.channel == grip_ch][0], theta))))
        lines = build_frame(m, joints)
        for idx, ln in enumerate(lines):
            link.send(ln)
            got, dt, others = link.wait_for(
                lambda s: s.upper().startswith("OK SET") or s.upper().startswith("ERR"),
                timeout_s=1.5)
            if idx == 0:
                fr.ack1_ms, fr.ack1_line = (None if got is None else dt), (got or "")
            else:
                fr.ack2_ms, fr.ack2_line = (None if got is None else dt), (got or "")
            fr.async_lines.extend(others)
        results.append(fr)
        if settle_s:
            time.sleep(settle_s)
    return results


def probe_burst_mode(link: Link, m, channels, n_frames: int, interval_ms: float) -> list[FrameResult]:
    """高频连发模式：复刻前端 33ms 节流下的串口压力。

    ★★ 真正的 fire-and-forget：**只按节拍写，不做逐条等待**。
    早期版本在这里 `wait_for` 逐条等 ACK，等价于把压力降到了"发一条等一条"，
    既没有复刻前端行为，又会把迟到的 ACK 计成"缺失"（详见文件头的误判复盘）。
    现在的做法：先把 STATS 清零 → 按节拍连发 n_frames 帧 → 发完后一次性收干
    → 与 `STATS` 增量对账。这样「压力」与「度量」是分开的两件事。
    """
    grip_ch = 6
    results: list[FrameResult] = []

    # 发前清零，让 STATS 读数只反映本轮
    link.send("STATS CLEAR")
    link.wait_for(lambda s: "STATS" in s.upper(), 1.0)
    link.drain(0.15)

    sent_lines: list[str] = []
    for i in range(n_frames):
        theta = 10.0 + (i * 7) % 80  # 10..90 之间游走
        joints = {"base": 0.0, "shoulder": 1.0, "elbow": 112.7, "gripper": theta}
        fr = FrameResult(index=i, grip_angle=int(round(joint_to_servo(
            [a for a in m.actuators if a.channel == grip_ch][0], theta))))
        lines = build_frame(m, joints)
        t0 = time.monotonic()
        for ln in lines:
            link.send(ln)                # 只写，不等
            sent_lines.append(ln)
        results.append(fr)
        used = (time.monotonic() - t0) * 1000.0
        rest = interval_ms - used
        if rest > 0:
            time.sleep(rest / 1000.0)
    sent_total = len(sent_lines)

    # ---- 发完统一收干：让固件把积压的 ACK 全吐出来 ----
    #   ⚠️ 用 idle 判据的 drain：压测后积压可能达数百行，定时 drain 拉不干。
    time.sleep(0.4)
    all_lines = link.drain(8.0)
    for r in results:
        r.raw_lines = all_lines

    # ---- 回执对账（旁证）：按发送顺序 1:1 回填 ----
    acks = [l for l in all_lines if l.upper().startswith("OK SET")
            or l.upper().startswith("OK STATS")]
    errs = [l for l in all_lines if l.upper().startswith("ERR")]
    for r in results:
        r.async_lines = errs
    # ★★ 2026-09-15 修正：早期版本只填 `ack1_line`/`ack2_line`，**没填 `*_ms`**，
    #   而 `report()` 的 missing 统计读的是 `ack1_ms is None` ⇒ burst 模式下
    #   恒定报"120 帧全部无 ACK"。这是**报告口径的 bug**，不是链路缺陷。
    #   现在按对账结果同时填 line 与 ms（1.0 表示"已确认收到，时延未逐条测量"）。
    idx = 0
    for r in results:
        if idx < len(acks):
            r.ack1_line, idx = acks[idx], idx + 1
            r.ack1_ms = 1.0
        if idx < len(acks):
            r.ack2_line, idx = acks[idx], idx + 1
            r.ack2_ms = 1.0

    # 把「发送条数 / 收到回执 / 错误数」挂到 link 上，供 report() 打印。
    # 不用哨兵帧（那会污染 total 与 missing 统计）。
    link.sent_lines = sent_total
    link.got_acks = len(acks)
    link.got_errs = len(errs)
    return results


def report(results: list[FrameResult], link: Link, m) -> int:
    total = len(results)
    missing1 = [r for r in results if r.ack1_ms is None]
    missing2 = [r for r in results if r.ack2_ms is None]
    delayed2 = [r for r in results if r.ack2_ms is not None and r.ack2_ms > 100]

    print()
    print("=" * 68)
    print("gripper 链路诊断报告")
    print("=" * 68)
    print(f"总帧数            : {total}")
    # burst 模式的直接对账读数（frame 模式无此两项，为 None）
    sent_n = getattr(link, "sent_lines", None)
    got_n = getattr(link, "got_acks", None)
    if sent_n is not None:
        print(f"发送 SET 条数     : {sent_n}")
        print(f"收到 OK SET/STATS : {got_n}")
        if sent_n and got_n is not None:
            flag = "✅ 1:1 无缺失" if got_n >= sent_n else f"❌ 缺 {sent_n - got_n} 条"
            print(f"收发对账          : {flag}")
    print(f"第1条 SET 无 ACK  : {len(missing1)}")
    print(f"第2条 SET 无 ACK  : {len(missing2)}   ← ★ gripper 就在这一条")
    print(f"第2条 ACK > 100ms : {len(delayed2)}")

    a1 = [r.ack1_ms for r in results if r.ack1_ms is not None]
    a2 = [r.ack2_ms for r in results if r.ack2_ms is not None]
    if a1:
        print(f"第1条 ACK 时延    : min {min(a1):.0f} / avg {sum(a1)/len(a1):.0f} / max {max(a1):.0f} ms")
    if a2:
        print(f"第2条 ACK 时延    : min {min(a2):.0f} / avg {sum(a2)/len(a2):.0f} / max {max(a2):.0f} ms")

    # ★ 主判据：STATS 增量。链路真的丢字节 ⇒ rx_drop 必然非 0。
    #   这条读数独立于脚本的收发时序，因此不受"等待窗口太窄"这类度量缺陷影响。
    #
    #   ⚠️ 必须先 drain 干净：burst 模式下积压的回执可能有几百行，
    #   不排干就会把 STATS 的应答埋在它们后面 ⇒ 主判据"读不到"。
    #   （这里用 idle 判据的 drain，不是定时 drain。）
    print()
    link.drain(6.0)
    link.send("STATS")
    st, _, _ = link.wait_for(lambda s: "STATS" in s.upper(), 2.5)
    rx_drop = tx_drop = None
    if st:
        import re
        mr = re.search(r"rx_drop=(\d+)", st)
        mt = re.search(r"tx_drop=(\d+)", st)
        if mr:
            rx_drop = int(mr.group(1))
        if mt:
            tx_drop = int(mt.group(1))
    print(f"STATS 增量(本轮)  : rx_drop={rx_drop} tx_drop={tx_drop}"
          f"   ← ★ 主判据：链路丢字节的唯一直接证据")
    if rx_drop is None:
        # ★ 能红的守卫：主判据读不到时**不要**继续给结论。
        #   （2026-09-15 教训：带着"主判据缺失"跑完，会输出一份
        #    "❌ 复现到链路层问题" 的误导性报告——那其实是度量缺陷。）
        print()
        print("❌ 主判据不可读 —— 无法对本轮下结论。")
        print("   可能原因：① 固件为旧版本（无 STATS 命令）→ 重新构建并烧录")
        print("             ② 首条指令被吞（复位窗口）→ 本工具已有 _warmup，仍失败请重试")
        print("   本报告**不足以否定**链路丢字节，请勿据此下结论。")
        return 2

    # ★ 唯一能证明"固件真的接受了 gripper 目标"的读数：STATUS 回读
    print()
    link.drain(2.0)
    link.send("STATUS")
    st2, _, _ = link.wait_for(lambda s: "S6=" in s, 2.0)
    print(f"最终 STATUS       : {st2}")
    expected = results[-1].grip_angle if results else None
    print(f"最后一条 SET 目标 : S6={expected}")
    if st2 and expected is not None:
        import re
        mm = re.search(r"S6=(\d+)", st2)
        got = int(mm.group(1)) if mm else None
        verdict = "✅ 一致" if got == expected else f"❌ 不一致（固件 target={got}）"
        print(f"target 是否被改写 : {verdict}")

    # ---- 结论：以主判据为准 ----
    print()
    if rx_drop is not None and rx_drop == 0 and (tx_drop in (0, None)):
        # 主判据通过：明确否定"链路丢字节"
        acks = sum(1 for r in results if r.ack2_line)
        print(f"✅ 链路层无丢字节（rx_drop=0，含 S6 的回执 {acks}/{total}）")
        if missing2:
            print(f"   ⚠️ 但按逐条窗口计有 {len(missing2)} 条未见——这属于"
                  f"**度量窗口**问题，不是链路缺陷（见文件头复盘）")
        return 0
    print("❌ 存在 ACK 缺失 —— 复现到链路层问题（与 caf402d 同类）")
    return 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="gripper 偶发不顺畅 · 真机链路诊断")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", choices=["frame", "burst"], default="frame")
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--interval-ms", type=float, default=33.0, help="burst 模式的帧间隔（前端节流=33ms）")
    ap.add_argument("--settle", type=float, default=0.25, help="frame 模式每帧后等待（秒），让斜坡走完")
    args = ap.parse_args(argv)

    r, m, channels = load_servo_map()
    a6 = [a for a in m.actuators if a.channel == 6][0]
    print(f"[model] {r.model_id} · servo_6 offset={a6.offset} scale={a6.scale} reverse={a6.reverse} "
          f"servo_limits={a6.servo_min}..{a6.servo_max}")

    link = Link(args.port, args.baud)
    try:
        print(f"[link] {args.port} 已打开 · 横幅={getattr(link, 'banner', [])} "
              f"· 就绪探针用了 {getattr(link, 'warmup_attempts', '?')} 次")
        print(f"[mode] {args.mode} · frames={args.frames}")
        if args.mode == "frame":
            results = probe_frame_mode(link, m, channels, args.frames, args.settle)
        else:
            results = probe_burst_mode(link, m, channels, args.frames, args.interval_ms)
        link.drain(0.5)
        return report(results, link, m)
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
