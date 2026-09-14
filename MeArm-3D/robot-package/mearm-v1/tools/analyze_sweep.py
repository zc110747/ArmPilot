#!/usr/bin/env python3
"""扫描图分析：从一串「舵机角度 -> 照片」中量化机械臂在画面里的运动。

背景
----
实测台已把机械臂与深色桌面之间加了**白色分割板**，因此可以直接用
「暗像素 = 机械臂」做分割，不再被桌沿干扰。

输出
----
  area      掩膜面积            -> 手臂转进/转出画面（底座旋转的主要信号）
  cx, cy    掩膜重心            -> 整体位移
  tip       离枢轴最远的暗像素  -> 夹爪尖端轨迹（半径 r、仰角 elev）
  top_y     掩膜最高点          -> 手臂抬升
  bbox

另有 --overlay：把最小角(红)与最大角(绿)的掩膜叠在一起，
一动就立刻看出来（两图重合处为黄色）。

用法
----
    python tools/analyze_sweep.py .workbuddy/captures/w2_S8
    python tools/analyze_sweep.py .workbuddy/captures/w2_S8 --overlay out.png
    python tools/analyze_sweep.py .workbuddy/captures/w2_S8 --pivot 500 470
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

# 1280x720 实测台 ROI：上边界留白，下边界卡在桌沿之上（y>=580 是黑桌面）
# 右边界要放到 1040 —— S7 高角时前臂会伸到 x>960，截断会让「最右点」失去意义
DEFAULT_ROI = (330, 150, 1040, 530)
DEFAULT_THRESH = 90        # 灰度 < thresh 判为「机械臂」（黑件）
# 底座旋转轴在画面中的投影（用于极坐标读数），可按需 --pivot 覆盖
DEFAULT_PIVOT = (505, 500)


def load_frames(d: Path) -> list[tuple[int, Path]]:
    out = []
    for p in sorted(d.glob("*.jpg")):
        m = re.search(r"_(\d{3})\.jpg$", p.name)
        if m:
            out.append((int(m.group(1)), p))
    return sorted(out)


def mask_of(im: Image.Image, roi, thresh: int) -> tuple[np.ndarray, int, int]:
    l, t, r, b = roi
    a = np.asarray(im.convert("L").crop((l, t, r, b)), dtype=np.uint8)
    return a < thresh, l, t


def stats(mask: np.ndarray, l: int, t: int, pivot: tuple[int, int]):
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return None
    ax = xs + l
    ay = ys + t
    cx, cy = float(ax.mean()), float(ay.mean())
    px, py = pivot
    d2 = (ax - px) ** 2 + (ay - py) ** 2
    i = int(np.argmax(d2))
    tipx, tipy = int(ax[i]), int(ay[i])
    r = float(np.sqrt(d2[i]))
    # 仰角：以枢轴为原点，指向右方为 0°，向上为正
    elev = float(np.degrees(np.arctan2(py - tipy, tipx - px)))

    # 塔顶 = 掩膜最高点
    j = int(np.argmin(ay))
    topx, topy = int(ax[j]), int(ay[j])

    # 远端点 = 离塔顶最远的暗像素（= 爪尖）。
    # 约束：排除左下角的底座总成（舵机体 + 杜邦线，原图 x<650 且 y>390），
    # 否则手臂压低时它离塔顶更远，会抢走 argmax。
    # 注意 mask 是 ROI 局部坐标，需减去 ROI 左上角 (l, t)。
    arm_only = mask.copy()
    arm_only[max(0, 390 - t):, :max(0, 650 - l)] = False
    ys2, xs2 = np.nonzero(arm_only)
    if xs2.size == 0:
        ys2, xs2 = ys, xs
    bx = xs2 + l
    by = ys2 + t
    d2t = (bx - topx) ** 2 + (by - topy) ** 2
    k = int(np.argmax(d2t))
    distalx, distaly = int(bx[k]), int(by[k])
    arm_len = float(np.sqrt(d2t[k]))
    arm_ang = float(np.degrees(np.arctan2(distaly - topy, distalx - topx)))

    return dict(
        n=int(xs.size), cx=cx, cy=cy,
        tip=(tipx, tipy), r=r, elev=elev,
        top=topy, topx=topx,
        right=int(ax.max()), righty=int(ay[int(np.argmax(ax))]),
        distal=(distalx, distaly), arm_len=arm_len, arm_ang=arm_ang,
        bbox=(int(ax.min()), int(ay.min()), int(ax.max()), int(ay.max())),
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="扫描图运动分析（白底暗件分割）")
    ap.add_argument("dir")
    ap.add_argument("--roi", type=int, nargs=4, default=list(DEFAULT_ROI),
                    metavar=("L", "T", "R", "B"))
    ap.add_argument("--thresh", type=int, default=DEFAULT_THRESH)
    ap.add_argument("--pivot", type=int, nargs=2, default=list(DEFAULT_PIVOT),
                    metavar=("X", "Y"))
    ap.add_argument("--montage", default=None)
    ap.add_argument("--overlay", default=None, help="红绿差分叠加输出路径")
    ap.add_argument("--scale", type=float, default=1.0)
    a = ap.parse_args()

    d = Path(a.dir)
    frames = load_frames(d)
    if not frames:
        raise SystemExit(f"{d} 下没找到 S<id>_<angle>.jpg")

    servo = re.sub(r"^w?\d*_?", "", d.name)
    roi = tuple(a.roi)
    pivot = tuple(a.pivot)
    print(f"# {d}   舵机={servo}   ROI={roi}   阈值<{a.thresh}   枢轴={pivot}")
    print(f"{'角度':>5} {'暗像素':>7} {'重心x':>7} {'重心y':>7} "
          f"{'塔顶x':>6} {'塔顶y':>6} {'爪尖x':>6} {'爪尖y':>6} "
          f"{'臂长px':>7} {'臂倾角°':>8} {'面积Δ':>7}")

    rows = []
    base_n = None
    for ang, p in frames:
        im = Image.open(p)
        m, l, t = mask_of(im, roi, a.thresh)
        s = stats(m, l, t, pivot)
        rows.append((ang, p, im, s))
        if s is None:
            print(f"{ang:>5} {0:>7}   (无暗像素，检查 ROI/阈值)")
            continue
        if base_n is None:
            base_n = s["n"]
        # 臂长/臂倾角 = 塔顶 -> 爪尖。若机构是平行四边形（前臂绝对角只由 S8 决定），
        # 扫 S7 时 arm_ang 应基本恒定；扫 S8 时 arm_ang 应显著变化。
        print(f"{ang:>5} {s['n']:>7} {s['cx']:>7.1f} {s['cy']:>7.1f} "
              f"{s['topx']:>6} {s['top']:>6} {s['distal'][0]:>6} {s['distal'][1]:>6} "
              f"{s['arm_len']:>7.1f} {s['arm_ang']:>8.1f} {s['n']-base_n:>+7d}")

    valid = [r for r in rows if r[3] is not None]
    if len(valid) >= 2:
        a0, a1 = valid[0], valid[-1]
        s0, s1 = a0[3], a1[3]
        print(f"\n角度 {a0[0]} -> {a1[0]}（屏幕坐标，y 向下为正）")
        print(f"  重心    dx={s1['cx']-s0['cx']:+7.1f}  dy={s1['cy']-s0['cy']:+7.1f}"
              f"   -> {'右' if s1['cx']>s0['cx'] else '左'}/{'下' if s1['cy']>s0['cy'] else '上'}")
        print(f"  尖端    dx={s1['tip'][0]-s0['tip'][0]:+7d}  dy={s1['tip'][1]-s0['tip'][1]:+7d}"
              f"   半径 {s0['r']:.1f} -> {s1['r']:.1f} ({s1['r']-s0['r']:+.1f})")
        print(f"  仰角    {s0['elev']:+.1f}° -> {s1['elev']:+.1f}° ({s1['elev']-s0['elev']:+.1f}°)")
        print(f"  面积    {s0['n']} -> {s1['n']} ({s1['n']-s0['n']:+d})")
        # 单调性
        rs = [r[3]['elev'] for r in valid]
        mono = all(rs[i] <= rs[i+1] for i in range(len(rs)-1)) or \
               all(rs[i] >= rs[i+1] for i in range(len(rs)-1))
        print(f"  仰角单调: {'是' if mono else '否（有反复，注意看叠加图）'}")

    # ---- 可视化
    if a.montage or a.overlay:
        box = roi
        if a.montage:
            tiles = []
            for _ang, _p, im, _s in rows:
                c = im.crop(box)
                w, h = c.size
                tiles.append((_ang, c.resize((int(w*a.scale), int(h*a.scale)), Image.LANCZOS)))
            w, h = tiles[0][1].size
            n = len(tiles)
            sheet = Image.new("RGB", (w, h*n + 4*(n-1)), (22, 24, 30))
            dr = ImageDraw.Draw(sheet)
            for i, (ang, im) in enumerate(tiles):
                y = i*(h+4)
                dr.text((6, y+4), f"{servo} = {ang}", fill=(255, 220, 120))
                sheet.paste(im, (0, y))
            sheet.save(a.montage)
            print(f"\n拼图 -> {a.montage}  {sheet.size}")

        if a.overlay and len(valid) >= 2:
            angA, _pA, _imA, _sA = valid[0]
            angB, _pB, _imB, _sB = valid[-1]
            mA, l, t = mask_of(Image.open(valid[0][1]), box, a.thresh)
            mB, _, _ = mask_of(Image.open(valid[-1][1]), box, a.thresh)
            h, w = mA.shape
            rgb = np.zeros((h, w, 3), dtype=np.uint8)
            rgb[..., 0] = np.where(mA, 235, 26)     # 红：最小角
            rgb[..., 1] = np.where(mB, 235, 26)     # 绿：最大角
            rgb[..., 2] = np.where(mA & mB, 90, 30)  # 重合偏黄
            img = Image.fromarray(rgb).resize(
                (int(w*a.scale), int(h*a.scale)), Image.NEAREST)
            dr = ImageDraw.Draw(img)
            # 画枢轴
            px, py = pivot
            PX, PY = (px-box[0])*a.scale, (py-box[1])*a.scale
            dr.line([(PX-12, PY), (PX+12, PY)], fill=(0, 200, 255), width=2)
            dr.line([(PX, PY-12), (PX, PY+12)], fill=(0, 200, 255), width=2)
            dr.text((8, 6), f"RED = {angA}   GREEN = {angB}", fill=(255, 255, 255))
            img.save(a.overlay)
            print(f"叠加图 -> {a.overlay}  {img.size}  (红={angA}  绿={angB}  黄=重合)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
