#!/usr/bin/env python3
"""把机械臂按「哪次扫描时它不动」自动分离成 底座 / 大臂 / 小臂，并 PCA 定方向。

思想（不需要人肉认枢轴）
----------------------
  inter_S7 = 扫 S7 时**始终为暗**的像素 = 不随肩关节运动的部分 = 底座+立柱（含线材）
  inter_S8 = 扫 S8 时**始终为暗**的像素 = 不随肘关节运动的部分 = 底座+立柱+**大臂**

于是：
  大臂 ≈ inter_S8 − dilate(inter_S7)          （在 S7=基准角 时的位置）
  小臂+爪 ≈ mask(某帧) − dilate(inter_S8)      （该帧的小臂位置）

对每个区域做 PCA，主方向即该连杆在画面中的方向；配合「方向 vs 舵机角」
就能同时读出 **转角增益** 与 **该连杆是绝对角还是相对角**。

用法
----
    python tools/segment_arm.py
    python tools/segment_arm.py --out .workbuddy/analysis/segments.png
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_sweep import DEFAULT_ROI, DEFAULT_THRESH, load_frames, mask_of  # noqa: E402

CAP = Path(".workbuddy/captures")


def stack_masks(d: str, roi, thresh: int):
    frames = load_frames(CAP / d)
    out = []
    for ang, p in frames:
        m, _, _ = mask_of(Image.open(p), roi, thresh)
        out.append((ang, m))
    return out


def dilate(m: np.ndarray, k: int = 2) -> np.ndarray:
    """k 邻域膨胀（无 scipy 依赖，用位移或运算）。"""
    out = m.copy()
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            out |= np.roll(np.roll(m, dy, 0), dx, 1)
    return out


def pca_axis(m: np.ndarray, l: int, t: int) -> dict | None:
    ys, xs = np.nonzero(m)
    if xs.size < 60:
        return None
    ax, ay = xs.astype(float) + l, ys.astype(float) + t
    ax -= ax.mean()
    ay -= ay.mean()
    cov = np.array([[ax @ ax, ax @ ay], [ax @ ay, ay @ ay]]) / ax.size
    w, v = np.linalg.eigh(cov)
    d = v[:, int(np.argmax(w))]
    # 统一朝「+x（画面右侧）」；屏幕 y 向下，取负换成数学仰角
    if d[0] < 0:
        d = -d
    ang = float(np.degrees(np.arctan2(-d[1], d[0])))
    elong = float(np.sqrt(max(w) / max(min(w), 1e-9)))
    return dict(angle=ang, elong=elong, n=int(xs.size),
                bbox=(int(ax.min() + 0), int(ay.min() + 0), int(ax.max() + 0), int(ay.max() + 0)))


def main() -> int:
    ap = argparse.ArgumentParser(description="机械臂分段（底座/大臂/小臂）+ 方向")
    ap.add_argument("--roi", type=int, nargs=4, default=list(DEFAULT_ROI))
    ap.add_argument("--thresh", type=int, default=DEFAULT_THRESH)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    roi, thresh = tuple(a.roi), a.thresh
    l, t = roi[0], roi[1]

    s7 = stack_masks("w2_S7", roi, thresh)
    s8 = stack_masks("w2_S8", roi, thresh)

    inter7 = s7[0][1].copy()
    for _ang, m in s7[1:]:
        inter7 &= m
    inter8 = s8[0][1].copy()
    for _ang, m in s8[1:]:
        inter8 &= m

    base = dilate(inter7, 3)
    upper = inter8 & ~base
    print(f"inter_S7 像素={inter7.sum()}   inter_S8 像素={inter8.sum()}   大臂像素={upper.sum()}")

    # 肩枢轴 ≈ inter_S7 最高点（底座/立柱顶）
    ys, xs = np.nonzero(inter7)
    ay, ax = ys + t, xs + l
    k = int(np.argmin(ay))
    print(f"inter_S7 最高点 = ({ax[k]}, {ay[k]})   ← 肩枢轴的观察上界")
    ys, xs = np.nonzero(inter8)
    ay2, ax2 = ys + t, xs + l
    k = int(np.argmin(ay2))
    print(f"inter_S8 最高点 = ({ax2[k]}, {ay2[k]})   ← 大臂顶（肘附近）")

    up = pca_axis(upper, l, t)
    print(f"\n大臂（S7 基准位）PCA 方向 = {up['angle']:+.2f}° (仰角)  细长比={up['elong']:.1f} "
          f"bbox={up['bbox']}  n={up['n']}" if up else "\n大臂区域太小")

    print(f"\n{'S8':>4} {'小臂PCA仰角°':>12} {'细长比':>7} {'像素':>7} {'bbox':>28}")
    angles = []
    for ang, m in s8:
        fore = m & ~dilate(inter8, 3)
        r = pca_axis(fore, l, t)
        if r is None:
            print(f"{ang:>4}  (区域太小)")
            continue
        angles.append((ang, r["angle"]))
        print(f"{ang:>4} {r['angle']:>12.2f} {r['elong']:>7.1f} {r['n']:>7} {str(r['bbox']):>28}")

    if len(angles) >= 2:
        (a0, f0), (a1, f1) = angles[0], angles[-1]
        print(f"\n小臂仰角 {a0}°->{a1}° : {f0:+.2f}° -> {f1:+.2f}°  "
              f"Δ={f1 - f0:+.2f}° / 舵机 {a1 - a0}°  ⇒ 增益 {abs(f1 - f0) / abs(a1 - a0):.3f}")

    # 同一逻辑对 S7：小臂区域 = mask − inter_S8（但 S7 扫描时缺 inter_S7 之外的公共区）
    print(f"\n{'S7':>4} {'动区PCA仰角°':>12} {'细长比':>7} {'像素':>7}")
    for ang, m in s7:
        moving = m & ~dilate(inter7, 3)
        r = pca_axis(moving, l, t)
        if r:
            print(f"{ang:>4} {r['angle']:>12.2f} {r['elong']:>7.1f} {r['n']:>7}")

    if a.out:
        h, w = inter7.shape
        rgb = np.full((h, w, 3), 20, dtype=np.uint8)
        rgb[base] = (52, 78, 132)          # 蓝：底座/立柱（S7 不动）
        rgb[upper] = (240, 176, 60)        # 黄：大臂（S8 不动但 S7 动）
        fore = s8[-1][1] & ~dilate(inter8, 3)
        rgb[fore] = (70, 200, 120)         # 绿：小臂+爪（S8 最大角）
        fore0 = s8[0][1] & ~dilate(inter8, 3)
        rgb[fore0 & ~fore] = (225, 80, 80)  # 红：小臂（S8 最小角）
        Image.fromarray(rgb).resize((w * 2, h * 2), Image.NEAREST).save(a.out)
        print(f"\n分段图 -> {a.out}  蓝=底座 黄=大臂 绿=S8最大 红=S8最小")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
