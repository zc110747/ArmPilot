#!/usr/bin/env python3
"""跨批可复现性矩阵：标定增益在「批次 × 底座掩膜 × 骨架模型」下的散布。

为什么需要
----------
D37 已确立纪律：**单批「最优」是过拟合**（同一批照片里把锚点偏置从 −2.20° 调到
−0.45°，换一批反而从 6.29° 恶化到 12.17°）。但「跑两批」此前是人肉操作 ——
于是很容易只跑一批，或者换了个 `--base-anywhere` 就把结果当成新证据。

本脚本把这件事**固化成一次命令**：固定若干配置组合，逐一反解 S7 / S8 的增益，
直接打印「实测增益 / yaml 期望 / 偏差%」的散布表。

判据（写死在这里，避免事后解释）
--------------------------------
* 每通道取所有组合的**偏差极差**（max − min）：
    - `极差 <= 5%`  ⇒ 测量可复现，此时才**允许**据"偏差"去改模型或标定；
    - `极差 >  5%`  ⇒ 台面 / 相机 / 曝光 / 底座掩膜里至少有一项没锁住，
                        先锁台面再谈精度 —— 此时任何"改模型"的结论都不成立。
* 另有一条否决：任一组合出现 `|偏差| > 40%` ⇒ 该配置下的拟合**退化**了
  （实测 `--rod-gap 10` 在 dir_S7 上把肩增益拉到 0.3727 / −46.3%），
  这种点不能拿来论证趋势。

用法
----
    python tools/verify_calib_repro.py                     # 默认配置集
    python tools/verify_calib_repro.py --rod-gaps 0 4 10   # 指定骨架杆距
    python tools/verify_calib_repro.py --only w2           # 只跑某批
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CAP = ROOT / ".workbuddy" / "captures"
VERIFY = ROOT / "tools" / "verify_pose.py"

# 每批 = 一个「相机位 / 曝光 / 台面」组合。锚点帧一律取该批的 RESET 位（四舵机 90°）。
# base_region: None ⇒ 沿用 verify_pose 默认（w2_S7）；"self" ⇒ 用该批自身（换相机位后必用）
BATCHES = [
    {"key": "w2", "label": "w2（台面 A · 2026-09-12 上午）",
     "s7": "w2_S7", "s8": "w2_S8", "anchor": "w2_S8/S8_090.jpg", "base_region": None},
    {"key": "dir", "label": "dir（台面 B · 另一相机位）",
     "s7": "dir_S7", "s8": "dir_S8", "anchor": "dir_S8/S8_090.jpg", "base_region": "self"},
]

GAIN_RE = re.compile(r"^\s+S(?P<ch>\d)\((?P<joint>\w+)\s*\)\s*实测\s*(?P<meas>[-+][\d.]+)\s+"
                     r"yaml\s*(?P<exp>[-+][\d.]+)\s+偏差\s*(?P<delta>[-+][\d.]+)%", re.M)


def run_one(target: str, anchor: str, base_region: str | None,
            base_anywhere: bool, rod_gap: float) -> dict[str, dict]:
    """跑一次反解，返回 {关节: {measured, expected, delta_pct}}。"""
    cmd = [sys.executable, str(VERIFY), str(CAP / target), "--anchor", str(CAP / anchor),
           "--rod-gap", f"{rod_gap:g}"]
    if base_region:
        cmd += ["--base-region", base_region]
    if base_anywhere:
        cmd += ["--base-anywhere"]
    proc = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", cwd=str(ROOT))
    if proc.returncode not in (0, 1):          # 1 = 有 FAIL/SKIP，增益表仍然有效
        return {"__error__": {"stderr": (proc.stderr or "")[-300:]}}
    out = {}
    for m in GAIN_RE.finditer(proc.stdout or ""):
        out[m.group("joint")] = {
            "measured": float(m.group("meas")),
            "expected": float(m.group("exp")),
            "delta_pct": float(m.group("delta")),
            "channel": m.group("ch"),
        }
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="标定增益跨批可复现性矩阵")
    ap.add_argument("--rod-gaps", type=float, nargs="+", default=[0.0],
                    help="骨架小臂平行杆对间距（mm）列表，默认仅 0（单折线模型）")
    ap.add_argument("--only", default=None, help="只跑 key 匹配的批次")
    ap.add_argument("--no-anywhere", dest="anywhere", action="store_false",
                    help="不跑 --base-anywhere 变体")
    ap.add_argument("--spread-tol", type=float, default=5.0,
                    help="偏差极差容差（百分点），超过即判测量不可复现，默认 5.0")
    a = ap.parse_args()

    rows: list[tuple[str, str, float, dict[str, dict]]] = []
    for b in BATCHES:
        if a.only and a.only != b["key"]:
            continue
        if not (CAP / b["s7"]).is_dir() or not (CAP / b["s8"]).is_dir():
            print(f"!! 跳过 {b['key']}：照片目录不存在")
            continue
        variants = [("默认底座掩膜", False)]
        if a.anywhere:
            variants.append(("--base-anywhere", True))
        for label, anywhere in variants:
            for gap in a.rod_gaps:
                for target in (b["s7"], b["s8"]):
                    got = run_one(target, b["anchor"], b["base_region"], anywhere, gap)
                    rows.append((b["key"], f"{label} / gap={gap:g}", gap, got))

    if not rows:
        print("没有可跑的组合。")
        return 1

    print("\n================ 标定增益跨批可复现性矩阵 ================")
    print(f"{'批次':<6}{'配置':<30}{'通道':<10}{'实测':>10}{'yaml':>10}{'偏差%':>10}")
    print("-" * 78)
    per_joint: dict[str, list[float]] = {}
    degenerate = 0
    for batch, cfg, _gap, got in rows:
        if "__error__" in got:
            print(f"{batch:<6}{cfg:<30}{'!! 反解失败':<10}  {got['__error__']['stderr'].strip()[:40]}")
            continue
        for joint, v in sorted(got.items()):
            per_joint.setdefault(joint, []).append(v["delta_pct"])
            if abs(v["delta_pct"]) > 40.0:
                degenerate += 1
            tag = f"S{v['channel']}({joint})"
            print(f"{batch:<6}{cfg:<30}{tag:<10}"
                  f"{v['measured']:>+10.4f}{v['expected']:>+10.4f}{v['delta_pct']:>+10.1f}")

    print("-" * 78)
    verdict = 0
    for joint, deltas in sorted(per_joint.items()):
        lo, hi = min(deltas), max(deltas)
        spread = hi - lo
        ok = spread <= a.spread_tol
        verdict += 0 if ok else 1
        print(f"{joint:<10} 偏差范围 [{lo:+.1f}%, {hi:+.1f}%]  极差 {spread:.1f}%  "
              f"(容差 {a.spread_tol:g}%)  ⇒ {'可复现':<8}  "  # noqa: E501
              f"{'可按偏差改模型/标定' if ok else '先锁台面，禁止据单点结论改模型'}")
    if degenerate:
        print(f"\n⚠️ 有 {degenerate} 个组合的 |偏差| > 40%：拟合在该配置下**退化**，"
              f"不能拿来论证趋势（实测 --rod-gap 10 会让 dir_S7 的肩增益崩到 0.3727）。")
    if verdict:
        print("\n结论：至少一个通道的跨配置散布超出容差 ⇒ **测量本身不可复现**，"
              "此时把偏差归因给骨架模型或标定表都是不成立的。")
    else:
        print("\n结论：各通道跨配置一致 ⇒ 测量可复现，偏差可以进一步归因。")
    return 1 if verdict else 0


if __name__ == "__main__":
    sys.exit(main())
