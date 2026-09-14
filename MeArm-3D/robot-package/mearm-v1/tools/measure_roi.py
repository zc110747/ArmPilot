#!/usr/bin/env python3
"""
measure_roi.py —— 从实拍照片量出「暗件（机械臂）紧包围盒」，据此给出 / 复核 ROI 建议值。

为什么需要它
------------
`verify_pose.py` / `analyze_sweep.py` 里的 ROI（region of interest）是硬编码常量。
台面/相机一挪，旧 ROI 就不覆盖机械臂了 —— 表现是「反解莫名全错但不报错」，
因为 ROI 外的臂被裁掉，掩膜退化成一小块，拟合仍能收敛到一个荒谬的解。

**ROI 不能靠人肉框选**：臂伸到哪就脏哪，肉眼看「背景区」估不出臂的范围。
必须**先分割、再从分割结果取紧包围盒**，并剔除线缆/底座这类不该进 ROI 判断的连通块。

做法
----
1. 用与 verify_pose.py 同一套分割（Otsu / 固定阈值 + 地基掩膜剔除）；
2. 取最大若干连通块（暗件），合并求包围盒；
3. 用**面积 + 长宽比**把线缆（细长低面积）、噪点（小面积）剔掉；
4. 给出建议 ROI = 包围盒 + 边距，并和当前 ROI 对比覆盖情况。

输出建议值后**由人确认**再写回常量 —— 脚本只负责量，不自动改代码。

⚠️ 两条实测教训（2026-09-12 踩过）
--------------------------------
1. **量出的 bbox 会被线缆/桌沿污染**：本机实测紧包围盒是 `x0=0, x1=1279`（左右都贴边），
   看着像"臂有整幅宽"，实际是 **USB 线缆横穿 + 桌沿暗带**被当成前景。**必须目视复核**
   （`--view` 导图）再采信，不能只看数字。
2. **单批结果不足以定 ROI，必须跨批验证**：实验里某个新 ROI 在 `e2e_191514` 批
   让各帧误差均值从 6.29 涨到 12.17（变差），却在 `e2e_192500` 批从 4.14 微降到 4.48 附近
   （看起来变好）—— **单批"变好"是对该批的过拟合**。定 ROI 前至少跨 2 个批次的
   `verify_pose.py --json` 结果一起看（比锚点偏置 + 各帧 |err| 均值），再决定。
   > 该轮最终结论：**ROI 不是当前限制因素，维持旧值**；真正的偏差来自
   > 平行四连杆耦合增益（见 `docs/hardware-measurement.md` §2）。

用法
----
    python tools/measure_roi.py IMG [IMG ...]
    python tools/measure_roi.py IMG --thresh auto            # 默认
    python tools/measure_roi.py IMG --thresh 90 --current 330,150,1040,530
    python tools/measure_roi.py IMG --margin 20 --json out.json
    python tools/measure_roi.py IMG --view roi_check.png     # 导出可视化

退出码：0 = 给出建议；2 = 输入不可用。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

try:
    import numpy as np
    from PIL import Image
except ImportError as e:  # pragma: no cover
    print(f"[fatal] 需要 numpy + PIL：{e}", file=sys.stderr)
    sys.exit(2)


# ---------------------------------------------------------------------------
# 分割（与 verify_pose.py 保持同一套口径，避免两处阈值/掩膜不一致）
# ---------------------------------------------------------------------------

def otsu(gray: np.ndarray) -> int:
    """标准 Otsu 阈值（灰度直方图最大化类间方差）。"""
    hist = np.bincount(gray.ravel(), minlength=256).astype(np.float64)
    total = hist.sum()
    if total == 0:
        return 0
    omega = np.cumsum(hist) / total
    mu = np.cumsum(hist * np.arange(256)) / total
    mu_t = mu[-1]
    denom = omega * (1.0 - omega)
    denom[denom <= 0] = 1e-12
    sigma_b = (mu_t * omega - mu) ** 2 / denom
    return int(np.argmax(sigma_b))


def make_mask(gray: np.ndarray, thresh: "int | str") -> "tuple[np.ndarray, int]":
    """暗件掩膜：灰度 < 阈值 即视为前景。返回 (mask, 实际阈值)。"""
    t = otsu(gray) if thresh == "auto" else int(thresh)
    return (gray < t), t


def label_components(mask: np.ndarray) -> "tuple[np.ndarray, int]":
    """4-邻域连通域标记（不依赖 scipy，BFS 由 numpy 分批扩散实现）。"""
    h, w = mask.shape
    lab = np.zeros((h, w), dtype=np.int32)
    cur = 0
    ys, xs = np.nonzero(mask)
    for y0, x0 in zip(ys.tolist(), xs.tolist()):
        if lab[y0, x0]:
            continue
        cur += 1
        # 波浪式填充：每轮把已有种子的上下左右未标记邻居并入
        front = np.zeros((h, w), dtype=bool)
        front[y0, x0] = True
        lab[front] = cur
        while True:
            nb = np.zeros((h, w), dtype=bool)
            nb[1:, :] |= front[:-1, :]
            nb[:-1, :] |= front[1:, :]
            nb[:, 1:] |= front[:, :-1]
            nb[:, :-1] |= front[:, 1:]
            nb &= mask & (lab == 0)
            if not nb.any():
                break
            lab[nb] = cur
            front = nb
    return lab, cur


def keep_bottom_component(mask: np.ndarray, frac: float = 0.5) -> np.ndarray:
    """
    剔除底座/立柱：与画面底部若干行有大量接触的连通块视为地基，从掩膜里去掉。
    口径与 verify_pose.py 一致（碰到底部 = 地基，不是臂的连杆）。
    """
    h, w = mask.shape
    band = lab_band = None
    lab, n = label_components(mask)
    if n == 0:
        return mask
    band = lab[int(h * (1 - 0.12)):, :]  # 底部 12% 行
    ids, counts = np.unique(band[band > 0], return_counts=True)
    bw = np.zeros(n + 1, dtype=bool)
    for i, c in zip(ids.tolist(), counts.tolist()):
        if c > (h * 0.12) * frac * 0.05:  # 与底部有实质接触
            bw[i] = True
    out = mask & ~bw[lab]
    return out


# ---------------------------------------------------------------------------
# 主逻辑
# ---------------------------------------------------------------------------

def bbox_of(mask: np.ndarray) -> "tuple[int, int, int, int] | None":
    ys, xs = np.nonzero(mask)
    if ys.size == 0:
        return None
    return int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1


def measure(path: Path, thresh, margin: int, drop_base: bool, min_area_frac: float,
            aspect_max: float, vertex: int, current):
    img = Image.open(path).convert("L")
    if vertex and vertex < max(img.size):  # 复现 verify_pose 的下采样口径
        img = img.resize((img.width // vertex, img.height // vertex), Image.BILINEAR)
    gray = np.asarray(img, dtype=np.uint8)
    h, w = gray.shape
    mask, t = make_mask(gray, thresh)

    raw = mask.copy()
    dropped = []
    if drop_base:
        # 逐块判断：细长 / 小面积 / 触底的块 = 线缆·噪点·底座，不进 ROI
        lab, n = label_components(mask)
        keep = np.zeros_like(mask)
        H = h
        for i in range(1, n + 1):
            m = lab == i
            area = int(m.sum())
            if area < min_area_frac * h * w:
                dropped.append(("小面积", i, area))
                continue
            ys, xs = np.nonzero(m)
            bh = int(ys.max() - ys.min()) + 1
            bw = int(xs.max() - xs.min()) + 1
            aspect = max(bh, bw) / max(1, min(bh, bw))
            touch_bottom = bool(m[int(H * 0.985):, :].any())
            if aspect > aspect_max:
                dropped.append((f"细长(aspect={aspect:.1f})", i, area))
                continue
            if touch_bottom and bh > 0.5 * H:
                dropped.append(("底座/立柱(触底)", i, area))
                continue
            keep |= m
        mask = keep

    bb = bbox_of(mask)
    res = {
        "file": str(path),
        "shape": [h, w],
        "thresh_used": t,
        "bbox_arm": bb,
        "foreground_pct": round(100.0 * mask.sum() / (h * w), 2),
        "foreground_raw_pct": round(100.0 * raw.sum() / (h * w), 2),
        "dropped_components": [{"reason": r, "id": i, "area": a} for r, i, a in dropped],
    }
    if bb:
        x0, y0, x1, y1 = bb
        sug = (
            max(0, x0 - margin),
            max(0, y0 - margin),
            min(w, x1 + margin),
            min(h, y1 + margin),
        )
        res["roi_suggested"] = list(sug)
        res["roi_suggested_cover_pct"] = round(100.0 * (sug[2] - sug[0]) * (sug[3] - sug[1]) / (h * w), 2)
    if current:
        cx0, cy0, cx1, cy1 = current
        cov = mask[cy0:cy1, cx0:cx1].sum()
        res["roi_current"] = list(current)
        res["roi_current_contains_pct"] = round(100.0 * cov / max(1, int(mask.sum())), 2)
        # 溢出 = 臂的暗像素落在当前 ROI 之外（反解会被裁掉），是旧 ROI 的直接告警
        res["arm_pixels_outside_current"] = int(mask.sum() - cov)
    return res, mask, gray


def main() -> int:
    ap = argparse.ArgumentParser(description="量出机械臂暗件的紧包围盒，给出 ROI 建议值")
    ap.add_argument("images", nargs="+", type=Path)
    ap.add_argument("--thresh", default="auto", help="'auto'（Otsu）或整数，默认 auto")
    ap.add_argument("--margin", type=int, default=20, help="建议 ROI 的四周留白（px），默认 20")
    ap.add_argument("--current", default=None, help="当前 ROI x0,y0,x1,y1，用于对比")
    ap.add_argument("--no-drop-base", action="store_true", help="不剔除线缆/底座，全暗像素都算")
    ap.add_argument("--min-area", type=float, default=0.0015, help="最小连通块面积占比，默认 0.15%%")
    ap.add_argument("--aspect-max", type=float, default=8.0, help="长宽比超过此值视为线缆，默认 8")
    ap.add_argument("--vertex", type=int, default=2, help="下采样系数（与 verify_pose 对齐），设 1 关闭")
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--view", type=Path, default=None, help="导出可视化（包围盒 + 建议 ROI 叠加）")
    args = ap.parse_args()

    if args.thresh != "auto":
        try:
            args.thresh = int(args.thresh)
        except ValueError:
            print(f"[fatal] --thresh 只能是 'auto' 或整数，收到 {args.thresh!r}", file=sys.stderr)
            return 2

    current = None
    if args.current:
        try:
            current = tuple(int(v) for v in args.current.split(","))
            assert len(current) == 4
        except Exception:
            print(f"[fatal] --current 需要 x0,y0,x1,y1，收到 {args.current!r}", file=sys.stderr)
            return 2

    out = []
    for p in args.images:
        if not p.exists():
            print(f"[fatal] 找不到 {p}", file=sys.stderr)
            return 2
        res, mask, gray = measure(
            p, args.thresh, args.margin, not args.no_drop_base,
            args.min_area, args.aspect_max, args.vertex, current,
        )
        out.append(res)

        print(f"\n=== {p.name} ===")
        print(f"  尺寸 {res['shape'][1]}x{res['shape'][0]}  阈值 {res['thresh_used']}"
              f"  前景 {res['foreground_raw_pct']}% → 剔杂后 {res['foreground_pct']}%")
        if res["dropped_components"]:
            for d in res["dropped_components"]:
                print(f"  剔除连通块 #{d['id']}：{d['reason']}，area={d['area']}")
        if res.get("bbox_arm"):
            x0, y0, x1, y1 = res["bbox_arm"]
            print(f"  臂紧包围盒 ({x0}, {y0}, {x1}, {y1})  宽 {x1-x0} 高 {y1-y0}")
            s = res["roi_suggested"]
            print(f"  ✅ 建议 ROI ({s[0]}, {s[1]}, {s[2]}, {s[3]})  覆盖 {res['roi_suggested_cover_pct']}% 画面")
        if current:
            print(f"  当前 ROI {tuple(res['roi_current'])} 含臂像素 {res['roi_current_contains_pct']}%"
                  f"  ⚠️ 落在 ROI 外的臂像素 {res['arm_pixels_outside_current']}")
            if res["roi_current_contains_pct"] < 99.0:
                print("  ⚠️ 当前 ROI **裁掉了机械臂**，反解必然错 —— 按下表建议值更新")

        if args.view:
            rgb = np.stack([gray] * 3, axis=-1).copy()

            def rect(arr, x0, y0, x1, y1, color):
                """画框，**边界一律钳到画面内** —— ROI 来自别的分辨率时 x1/y1 会越界，
                不钳会直接 IndexError 崩掉（正是本脚本要避免的那类"莫名崩溃"）。"""
                x0 = max(0, min(x0, arr.shape[1] - 1))
                x1 = max(0, min(x1, arr.shape[1]))
                y0 = max(0, min(y0, arr.shape[0] - 1))
                y1 = max(0, min(y1, arr.shape[0]))
                if x1 <= x0 or y1 <= y0:
                    print(f"  ! 框 ({x0},{y0},{x1},{y1}) 与画面 {arr.shape[1]}x{arr.shape[0]}"
                          f" 无交集，已跳过绘制（口径不匹配？）")
                    return
                arr[y0:y1, [x0, x1 - 1]] = color
                arr[[y0, y1 - 1], x0:x1] = color

            if res.get("bbox_arm"):
                x0, y0, x1, y1 = res["bbox_arm"]
                rect(rgb, x0, y0, x1, y1, [0, 255, 0])
                s = res["roi_suggested"]
                rect(rgb, s[0], s[1], s[2], s[3], [255, 0, 0])
            if current:
                cx0, cy0, cx1, cy1 = current
                rect(rgb, cx0, cy0, cx1, cy1, [0, 160, 255])
            v = args.view if len(args.images) == 1 else args.view.with_name(
                f"{args.view.stem}_{p.stem}{args.view.suffix}")
            Image.fromarray(rgb).save(v)
            print(f"  # 可视化 -> {v}")

    if args.json:
        args.json.write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"\n# JSON -> {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
