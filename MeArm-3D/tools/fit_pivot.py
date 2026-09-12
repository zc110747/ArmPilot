#!/usr/bin/env python3
"""从扫描图**圆拟合**枢轴位置与关节转角（把标定钉死在实测上）。

原理
----
单舵机扫描时，被驱动的那段结构绕**它的枢轴**刚性转动 —— 因此爪尖（离枢轴
最远的暗像素）在画面里走过一段**圆弧**，圆心即该关节枢轴在图像上的投影。

  * 最小二乘（Kasa）拟合圆 -> 圆心 (cx, cy) + 半径 r
  * 各帧爪尖相对圆心的极角 -> 直接读出**关节实际转角**
  * 转角 / 舵机行程 = 该关节的标定增益（关节度/舵机度）

判据（区分肩/肘）
----------------
  * 扫 S7 若圆心落在「塔基」附近 -> S7 驱动整条臂 => shoulder
  * 扫 S8 若圆心落在「大臂末端」附近 -> S8 只驱动前臂 => elbow
  同时输出「公共不动点」：所有帧掩膜求交集，其质心即零位移区，
  应与拟合圆心落在同一区域（两个独立方法互相印证）。

用法
----
    python tools/fit_pivot.py .workbuddy/captures/w2_S7
    python tools/fit_pivot.py .workbuddy/captures/w2_S8
    python tools/fit_pivot.py .workbuddy/captures/w2_S7 --pivot 600 420 --out fit.png
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_sweep import DEFAULT_PIVOT, DEFAULT_ROI, DEFAULT_THRESH, load_frames, mask_of  # noqa: E402


def fit_circle(xs: np.ndarray, ys: np.ndarray) -> tuple[float, float, float]:
    """Kasa 代数圆拟合：最小化 (x²+y² + Dx + Ey + F)²。"""
    a = np.column_stack([xs, ys, np.ones_like(xs)])
    b = -(xs**2 + ys**2)
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    d, e, f = sol
    cx, cy = -d / 2.0, -e / 2.0
    r = float(np.sqrt(max(cx * cx + cy * cy - f, 0.0)))
    return float(cx), float(cy), r


def extreme_point(mask: np.ndarray, l: int, t: int, pivot: tuple[int, int]) -> tuple[int, int]:
    """爪尖 = 离参考点最远的暗像素，但排除左下角底座总成（舵机体+杜邦线）。"""
    m = mask.copy()
    m[max(0, 390 - t):, : max(0, 650 - l)] = False
    ys, xs = np.nonzero(m)
    if xs.size == 0:
        ys, xs = np.nonzero(mask)
    ax, ay = xs + l, ys + t
    d2 = (ax - pivot[0]) ** 2 + (ay - pivot[1]) ** 2
    i = int(np.argmax(d2))
    return int(ax[i]), int(ay[i])


def main() -> int:
    ap = argparse.ArgumentParser(description="扫描图枢轴圆拟合 + 转角/增益")
    ap.add_argument("dir")
    ap.add_argument("--roi", type=int, nargs=4, default=list(DEFAULT_ROI), metavar=("L", "T", "R", "B"))
    ap.add_argument("--thresh", type=int, default=DEFAULT_THRESH)
    ap.add_argument("--pivot", type=int, nargs=2, default=list(DEFAULT_PIVOT), metavar=("X", "Y"),
                    help="仅用于挑「最远点」的粗参考点，不是拟合结果")
    ap.add_argument("--scale", type=float, default=0.75)
    ap.add_argument("--out", default=None, help="输出标注图路径")
    a = ap.parse_args()

    d = Path(a.dir)
    frames = load_frames(d)
    if not frames:
        raise SystemExit(f"{d} 下没找到 S<id>_<angle>.jpg")

    servo = re.sub(r"^w?\d*_?", "", d.name)
    roi, pivot = tuple(a.roi), tuple(a.pivot)
    print(f"# {d}   舵机={servo}   ROI={roi}   阈值<{a.thresh}   粗参考点={pivot}")

    angs, pts, masks = [], [], []
    for ang, p in frames:
        im = Image.open(p)
        m, l, t = mask_of(im, roi, a.thresh)
        pts.append(extreme_point(m, l, t, pivot))
        angs.append(ang)
        masks.append(m)

    xs = np.array([q[0] for q in pts], dtype=float)
    ys = np.array([q[1] for q in pts], dtype=float)
    cx, cy, r = fit_circle(xs, ys)

    # 残差 = 圆心拟合的可信度；> 4px 说明「爪尖」在帧间换了身份，别信转角
    resid = float(np.max(np.abs(np.hypot(xs - cx, ys - cy) - r)))

    theta = np.degrees(np.arctan2(-(ys - cy), xs - cx))  # y 向下 -> 取负得数学仰角
    unwrapped = np.unwrap(np.radians(theta))
    rel = np.degrees(unwrapped - unwrapped[0])

    print(f"\n{'角度':>5} {'爪尖x':>7} {'爪尖y':>7} {'半径px':>8} {'转角°':>8} {'相对首帧°':>10}")
    for i, ang in enumerate(angs):
        print(f"{ang:>5} {pts[i][0]:>7} {pts[i][1]:>7} {r:>7.1f}"
              f" {theta[i]:>8.2f} {rel[i]:>10.2f}")

    print(f"\n拟合圆心 = ({cx:.1f}, {cy:.1f})　半径 r = {r:.1f} px　最大半径残差 = {resid:.2f} px")
    if resid > 4.0:
        print("  ⚠️ 残差偏大：爪尖在帧间可能换了身份，转角**不可用**，请只看圆心是否合理")

    total = rel[-1]
    sweep = abs(angs[-1] - angs[0])
    print(f"总转角 = {total:+.2f}°　舵机行程 = {sweep}°　"
          f"⇒ 增益 = {abs(total)/sweep:.4f} 关节度/舵机度")

    # ---- 独立印证：所有帧掩膜的交集 = 零位移区（不动点集合）
    inter = masks[0].copy()
    for m in masks[1:]:
        inter &= m
    ys_i, xs_i = np.nonzero(inter)
    if xs_i.size > 0:
        fx = float(xs_i.mean()) + roi[0]
        fy = float(ys_i.mean()) + roi[1]
        print(f"公共不动区质心 = ({fx:.0f}, {fy:.0f})（{xs_i.size} px）　"
              f"与拟合圆心距离 = {np.hypot(fx - cx, fy - cy):.0f} px")
    else:
        print("公共不动区为空（所有像素都动过）—— 说明扫描幅度大或阈值偏松")

    if a.out:
        tiles = [Image.open(p).crop(roi) for _a, p in frames]
        w, h = tiles[0].size
        sheet = Image.new("RGB", (w, h), (22, 24, 30))
        sheet.paste(tiles[0], (0, 0))
        dr = ImageDraw.Draw(sheet)
        for x, y in pts:
            dr.ellipse([x - roi[0] - 3, y - roi[1] - 3, x - roi[0] + 3, y - roi[1] + 3],
                       outline=(255, 90, 90), width=2)
        dr.polygon([(x - roi[0], y - roi[1]) for x, y in pts], outline=(255, 200, 80))
        dr.line([cx - roi[0] - 14, cy - roi[1], cx - roi[0] + 14, cy - roi[1]], fill=(80, 255, 160), width=3)
        dr.line([cx - roi[0], cy - roi[1] - 14, cx - roi[0], cy - roi[1] + 14], fill=(80, 255, 160), width=3)
        sheet = sheet.resize((int(w * a.scale), int(h * a.scale)), Image.LANCZOS)
        sheet.save(a.out)
        print(f"标注图 -> {a.out}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
