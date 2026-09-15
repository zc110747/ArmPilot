#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""拖动时实时采样后端状态：把"偶发不顺畅"变成可计数的时间序列。

## 为什么

`gripper 偶发不顺畅` 有两种彼此互斥的成因，`joint_state` 单点读数**区分不了**：
  (A) 链路层：指令没送到 / ACK 丢了  ⇒ 后端记的 target 会**跳变或停滞**
  (B) 物理层：指令送到了、舵机也被命令了，但**机构卡住**（顶桌面 / 夹到东西）
      ⇒ 后端 target 正常变化，而**实际位置**（相机才能看）不动

后端 `joint_state` 是**开环目标值**（无位置反馈），所以它只能证伪 (A)、不能证伪 (B)。

## 判据

采样 `/healthz` 的 `state`，观察 gripper：
  - 你拖动时 gripper 值**完全不变** ⇒ 命令没穿过链路（(A) 类，昨天修的那条）
  - 值**在变但真机不动/顿挫** ⇒ 命令到了，问题在物理或固件斜坡（(B) 类）

用法：
    <python> robot-package/mearm-v1/tools/watch_gripper_live.py --seconds 40 --hz 10
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request


def fetch(base: str) -> dict | None:
    try:
        with urllib.request.urlopen(base + "/healthz", timeout=0.5) as r:
            return json.loads(r.read().decode())
    except Exception:
        return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="拖动时实时采样 gripper 状态")
    ap.add_argument("--base", default="http://127.0.0.1:8090")
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--hz", type=float, default=10.0)
    args = ap.parse_args(argv)

    period = 1.0 / args.hz
    end = time.monotonic() + args.seconds
    prev = None
    changes = 0
    frozen_streak = 0
    max_frozen = 0
    samples = 0
    t0 = time.monotonic()

    print(f"[watch] {args.base} · {args.seconds:.0f}s · {args.hz:.0f}Hz")
    print("    时间      base    sh      el      grip    变化")
    print("-" * 62)
    while time.monotonic() < end:
        d = fetch(args.base)
        if d is None:
            time.sleep(period)
            continue
        s = d["state"]
        grip = s["gripper"]
        mark = ""
        if prev is not None:
            if abs(grip - prev["gripper"]) > 0.01:
                changes += 1
                mark = f"grip {prev['gripper']:.0f}→{grip:.0f}"
                frozen_streak = 0
            else:
                frozen_streak += 1
                max_frozen = max(max_frozen, frozen_streak)
                if frozen_streak in (10, 25, 50):
                    mark = f"grip 停在 {grip:.0f}（{frozen_streak} 拍）"
        samples += 1
        if mark:
            el = time.monotonic() - t0
            print(f"  {el:6.1f}s  {s['base']:7.2f} {s['shoulder']:7.2f} {s['elbow']:7.2f} "
                  f"{grip:7.2f}  {mark}")
        prev = s
        time.sleep(period)

    print("-" * 62)
    print(f"采样 {samples} 次 · gripper 变化 {changes} 次 · 最长静默 {max_frozen} 拍"
          f"（≈{max_frozen/args.hz:.1f}s）")
    if changes == 0:
        print("❌ 整个窗口内 gripper 命令值从未变化 —— 若你确实在拖动，属 (A) 链路层问题")
        return 1
    print("ℹ️  命令值在变化。若真机仍不动/顿挫，问题在 (B) 物理层或固件斜坡，"
          "需相机证据（joint_state 是开环目标值，证明不了到位）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
