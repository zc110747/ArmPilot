#!/usr/bin/env python3
"""把 RobotModel 的侧视骨架**拟合到实拍照片**上，反解关节角（客观、可复现）。

为什么不用「爪尖跟踪 / 圆拟合」
----------------------------
单相机二维投影下，爪尖是夹爪的两片薄板 + 阴影，帧间会**换身份**，
圆拟合的残差无法反映真实误差（实测 S7 拟合半径 150px 折合 47mm，
小于大臂 80mm —— 物理上不可能）。

本脚本改用**整条骨架对掩膜的距离**做目标函数：
  1. 白底分割得到机械臂掩膜（黑件）-> 距离变换 DT（每像素到最近掩膜像素的距离）
  2. 把模型的 3 段连杆（大臂 80 / 小臂 80 / 手部 40，mm）在矢状面内投影成折线
  3. 用 Hooke-Jeeves 模式搜索最小化「骨架采样点到最近掩膜像素」的平均距离

待拟合参数（初始值取当前 yaml 的 HOME 位，最终由照片决定）
  s     像素/毫米
  ox,oy 肩枢轴在画面中的位置
  ths   肩关节角（0 = 大臂竖直向上，+ 向前）
  the   肘关节**绝对**倾角（0 = 小臂指向天顶，+ 向前倾）

用法
----
    python tools/fit_pose.py .workbuddy/captures/w2_S8/S8_090.jpg --ref
    python tools/fit_pose.py .workbuddy/captures/w2_S8 --sweep the
    python tools/fit_pose.py .workbuddy/captures/w2_S7 --sweep ths
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_sweep import DEFAULT_ROI, DEFAULT_THRESH, mask_of  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
YAML = ROOT / "config" / "robot.yaml"


def read_yaml_numbers() -> dict:
    """从 robot.yaml 抓取连杆长度与 homePose（避免引入 PyYAML 依赖）。"""
    text = YAML.read_text(encoding="utf-8")
    out: dict = {}
    for name, key in (("column_link", "L0"), ("upper_arm_link", "L1"),
                      ("forearm_link", "L2"), ("tool_link", "L3")):
        m = re.search(rf"- id: {name}\b.*?length:\s*([\d.]+)", text, re.S)
        if m:
            out[key] = float(m.group(1))
    m = re.search(r"homePose:\s*\n((?:\s+\w+:.*\n)+)", text)
    if m:
        out["home"] = {k: float(v) for k, v in re.findall(r"(\w+):\s*(-?[\d.]+)", m.group(1))}
    return out


def distance_transform(mask: np.ndarray) -> np.ndarray:
    """倒角距离变换（两遍扫描，无 scipy 依赖）：每像素到最近 True 像素的距离。"""
    h, w = mask.shape
    inf = 1e9
    d = np.where(mask, 0.0, inf)
    for y in range(h):
        row = d[y]
        if y:
            row[:] = np.minimum(row, d[y - 1] + 1.0)
        for x in range(1, w):
            if row[x] > row[x - 1] + 1.0:
                row[x] = row[x - 1] + 1.0
        for x in range(w - 2, -1, -1):
            if row[x] > row[x + 1] + 1.0:
                row[x] = row[x + 1] + 1.0
    for y in range(h - 1, -1, -1):
        row = d[y]
        if y < h - 1:
            row[:] = np.minimum(row, d[y + 1] + 1.0)
        for x in range(1, w):
            if row[x] > row[x - 1] + 1.0:
                row[x] = row[x - 1] + 1.0
        for x in range(w - 2, -1, -1):
            if row[x] > row[x + 1] + 1.0:
                row[x] = row[x + 1] + 1.0
    return d


# ---------------------------------------------------------------------------
# 骨架模型参数
# ---------------------------------------------------------------------------
# ⚠️ 用**模块级单一来源**而不是层层传参：骨架模型是整条反解链路的**模型假设**，
#    从代价函数到合成真值（verify_pose.synth_mask）必须共用同一份。漏传一处就会
#    变成"用 A 模型去拟合 B 模型生成的图"—— 那种自检必然全绿，却什么也没验证。
#
# rod_gap = 小臂「平行杆对」两根杆轴线的**间距（mm）**。
#   0  == 原始的单折线模型（逐值不差，向后兼容）。
#   >0 == 把 elbow→wrist 画成 ±rod_gap/2 的两条平行杆，以表达 meArm 的平行四连杆
#         （见 docs/decisions.md D38 与 docs/hardware-measurement.md §2）。
_SKELETON: dict[str, float] = {"rod_gap": 0.0}


def set_skeleton_rod_gap_mm(value: float) -> None:
    """设定小臂平行杆对的轴线间距（mm）。0 = 退回单折线模型。"""
    _SKELETON["rod_gap"] = float(value)


def skeleton_rod_gap_mm() -> float:
    return _SKELETON["rod_gap"]


def skeleton_polylines(ths, the, l1, l2, l3, n=110, rod_gap=0.0):
    """矢状面骨架的**折线段列表**（mm，相对肩枢轴；屏幕坐标 x 前 = +x, y 下 = +y）。

    返回 `[(k,2) ndarray, ...]` —— 分开返回是为了让渲染端能把每一段**独立**画出来。
    若拼成一条折线再交给 `ImageDraw.line`，画到断点处会拉出一条本不存在的连线。
    """
    tr, te = np.radians(ths), np.radians(the)
    p_sh = np.zeros(2)
    u1 = np.array([np.sin(tr), -np.cos(tr)])             # 大臂方向（相对肩角的绝对角 = ths）
    u2 = np.array([np.sin(te), -np.cos(te)])             # 小臂/手部方向（**绝对角**）
    n2 = np.array([u2[1], -u2[0]])                       # u2 的单位法向（杆间距方向）
    p_el = p_sh + l1 * u1
    p_wr = p_el + l2 * u2
    p_tcp = p_wr + l3 * u2
    total = l1 + l2 + l3
    out = []

    def emit(a, b, seg):
        k = max(3, int(round(n * seg / total)))
        t = np.linspace(0, 1, k)[:, None]
        out.append(a + (b - a) * t)

    emit(p_sh, p_el, l1)
    if rod_gap > 0.0:
        # 平行杆对：两条与 u2 平行、法向偏移 ±rod_gap/2 的等长杆
        for sign in (-1.0, 1.0):
            off = n2 * (sign * rod_gap / 2.0)
            emit(p_el + off, p_wr + off, l2)
    else:
        emit(p_el, p_wr, l2)
    emit(p_wr, p_tcp, l3)
    return out, np.array([p_sh, p_el, p_wr, p_tcp])


def skeleton(ths, the, l1, l2, l3, n=110, rod_gap=None):
    """矢状面骨架采样点（拼接后的单一阵列；与 `skeleton_polylines` 等价）。

    `rod_gap=None` 时取模块级设定（`skeleton_rod_gap_mm()`）。
    """
    gap = skeleton_rod_gap_mm() if rod_gap is None else float(rod_gap)
    lines, joints = skeleton_polylines(ths, the, l1, l2, l3, n=n, rod_gap=gap)
    return np.vstack(lines), joints


def make_cost(dt, mask_pts, geom, roi_lt):
    """构造对称 Chamfer 目标：

      项 A（骨架 -> 掩膜）：骨架点若在掩膜外就被罚（DT>0）；
      项 B（掩膜 -> 骨架）：掩膜像素到最近骨架点的距离，防止骨架缩成一小段。

    单纯只做 A 会退化（掩膜很厚，骨架只要落在里面就是 0），必须加 B。
    """
    l1, l2, l3 = geom

    def cost(params):
        s, ox, oy, ths, the = params
        if s <= 0.8 or s > 8.0:
            return 1e6
        if not (-90 < ths < 90) or not (30 < the < 175):
            return 1e6
        pts, _ = skeleton(ths, the, l1, l2, l3)
        px = ox + pts[:, 0] * s - roi_lt[0]
        py = oy + pts[:, 1] * s - roi_lt[1]
        h, w = dt.shape
        ix = np.rint(px).astype(int)
        iy = np.rint(py).astype(int)
        # 边界判据必须用**取整后**的整数索引：若用浮点 px/py 判，py = h-0.4 这类值会
        # 通过 px.max() < w 检查、却在 rint() 后变成 h 而越界（把种子放到 ROI 边缘即可复现）。
        if ix.min() < 0 or ix.max() >= w or iy.min() < 0 or iy.max() >= h:
            return 1e6
        a = float(dt[iy, ix].mean())
        d = np.hypot(mask_pts[:, 0][:, None] - px[None, :],
                     mask_pts[:, 1][:, None] - py[None, :])
        b = float(d.min(axis=1).mean())
        return a + b

    return cost


def pattern_search(x0, score, steps, iters=260, shrink=0.55):
    """Hooke-Jeeves 坐标模式搜索（无 scipy 依赖）。"""
    x = list(x0)
    fx = score(x)
    st = list(steps)
    for _ in range(iters):
        improved = False
        for i in range(len(x)):
            for sign in (1, -1):
                cand = list(x)
                cand[i] += sign * st[i]
                fc = score(cand)
                if fc < fx - 1e-12:
                    x, fx, improved = cand, fc, True
                    break
        if not improved:
            st = [v * shrink for v in st]
            if max(st) < 1e-4:
                break
    return x, fx


def main() -> int:
    ap = argparse.ArgumentParser(description="FK 骨架 ↔ 实拍照片拟合")
    ap.add_argument("target", help="单张图 或 扫描目录")
    ap.add_argument("--sweep", choices=["ths", "the", "both"], default=None,
                    help="扫描目录：只放开 ths / 只放开 the / 两个都放开（后者用来判定解耦）")
    ap.add_argument("--ref", action="store_true", help="同时输出参考位姿与 yaml HOME 的对照")
    ap.add_argument("--roi", type=int, nargs=4, default=list(DEFAULT_ROI))
    ap.add_argument("--thresh", type=int, default=DEFAULT_THRESH)
    ap.add_argument("--guess", type=float, nargs=5, default=None,
                    metavar=("S", "OX", "OY", "THS", "THE"))
    ap.add_argument("--overlay", default=None)
    ap.add_argument("--anchor", default=None,
                    help="指定哪张图做 5 参数自由拟合（像素尺度与肩枢轴由它锁定），"
                         "例如 --anchor .workbuddy/captures/w2_S8/S8_090.jpg")
    ap.add_argument("--base-region", default="w2_S7",
                    help="用于剔除底座/立柱的扫描**目录名**（在 .workbuddy/captures 下）")
    a = ap.parse_args()

    nums = read_yaml_numbers()
    l0 = nums.get("L0", 60.0)
    l1 = nums.get("L1", 80.0)
    l2 = nums.get("L2", 80.0)
    l3 = nums.get("L3", 40.0)
    print(f"# robot.yaml 连杆 立柱={l0} 大臂={l1} 小臂={l2} 手部={l3}  homePose={nums.get('home')}")

    roi, thresh = tuple(a.roi), a.thresh
    # 底座+立柱 = 扫 S7 时始终为暗的像素（与关节角无关，剔除后拟合只受臂形影响）
    from segment_arm import stack_masks, dilate
    s7 = stack_masks(Path(a.base_region).name, roi, thresh)
    inter7 = s7[0][1].copy()
    for _ang, mm in s7[1:]:
        inter7 &= mm
    base_region = dilate(inter7, 6)
    print(f"# 底座/立柱参考区来自 {Path(a.base_region).name}：{int(inter7.sum())} px → 膨胀后剔除")

    targets = ([a.target] if Path(a.target).is_file()
               else sorted(str(p) for p in Path(a.target).glob("*.jpg")))

    guess = a.guess or [2.4, 570.0, 470.0, 0.0, 105.0]
    fixed = None
    fps = []

    ths = the = None
    if a.anchor:
        # 先用锚点帧把 (s, ox, oy) 锁死；锚点通常取 RESET 位（全部舵机 90°），
        # 这样标定的绝对角度就与模型的 homePose 定义在同一个姿态上。
        aim = Image.open(a.anchor)
        am, al, at = mask_of(aim, roi, thresh)
        adt = distance_transform(am)
        ays, axs = np.nonzero(am & ~base_region)          # 覆盖项同样剔除底座/立柱
        astep = max(1, axs.size // 1600)
        apts = np.column_stack([axs[::astep], ays[::astep]]).astype(float)
        ax, af = pattern_search(guess, make_cost(adt, apts, (l1, l2, l3), (al, at)),
                                [0.4, 30.0, 30.0, 8.0, 8.0])
        print(f"\n锚点 {Path(a.anchor).name}: s={ax[0]:.3f}px/mm 肩枢轴=({ax[1]:.1f},{ax[2]:.1f}) "
              f"ths={ax[3]:+.2f}° the={ax[4]:+.2f}°  残差={af:.3f}px")
        fixed = (ax[0], ax[1], ax[2])
        ths, the = ax[3], ax[4]
        fps.append((Path(a.anchor).stem, *ax))

    for i, p in enumerate(targets):
        im = Image.open(p)
        m, l, t = mask_of(im, roi, thresh)
        dt = distance_transform(m)
        h, w = m.shape
        cover = m & ~base_region
        ys, xs = np.nonzero(cover)
        step = max(1, xs.size // 1600)                 # 掩膜点下采样，控制单次评估开销
        mask_pts = np.column_stack([xs[::step], ys[::step]]).astype(float)
        geom = (l1, l2, l3)
        if fixed is None:
            x, f = pattern_search(guess, make_cost(dt, mask_pts, geom, (l, t)),
                                  [0.4, 30.0, 30.0, 8.0, 8.0])
            s, ox, oy, ths, the = x
            print(f"\n参考帧 {Path(p).name}: s={s:.3f}px/mm 肩枢轴=({ox:.1f},{oy:.1f}) "
                  f"ths={ths:+.2f}° the={the:+.2f}°  残差={f:.3f}px")
            fixed = (s, ox, oy)
            if a.ref:
                home = nums.get("home", {})
                print(f"        yaml HOME: shoulder={home.get('shoulder')}  elbow={home.get('elbow')}")
        else:
            s, ox, oy = fixed
            if a.anchor and Path(p).stem == Path(a.anchor).stem:
                continue                                   # 锚点已算过，跳过
            base_cost = make_cost(dt, mask_pts, geom, (l, t))
            if a.sweep == "both":
                def sc2(q):
                    return base_cost([s, ox, oy, q[0], q[1]])
                q, f = pattern_search([ths, the], sc2, [6.0, 6.0], iters=260)
                ths, the = q[0], q[1]
            elif a.sweep == "ths":
                def sc(q):
                    return base_cost([s, ox, oy, q[0], the])
                q, f = pattern_search([ths], sc, [6.0], iters=160)
                ths = q[0]
            else:
                def sc(q, the0=the):
                    return base_cost([s, ox, oy, ths, q[0]])
                q, f = pattern_search([the], sc, [6.0], iters=160)
                the = q[0]
            print(f"  {Path(p).name}: ths={ths:+.2f}° the={the:+.2f}°  残差={f:.3f}px")
        fps.append((Path(p).stem, s, ox, oy, ths, the))

    # ---- 舵机角 -> 关节角 的增益
    if len(fps) > 1 and a.sweep:
        if a.sweep == "the":
            col = 4
        elif a.sweep == "ths":
            col = 3
        else:
            col = None   # both：用 the - ths（小臂**相对**肩角）来判解耦
        if a.sweep == "both":
            sv = [(int(re.search(r"_(\d{3})$", n).group(1)), v[3], v[4]) for n, *v in fps]
            sv.sort()
            print("\n  舵机   ths      the(绝对)   相对角=the-ths")
            for s_, t1, t2 in sv:
                print(f"  {s_:>4} {t1:>+8.2f} {t2:>+10.2f} {t2 - t1:>+14.2f}")
            d_abs = sv[-1][2] - sv[0][2]
            d_rel = (sv[-1][2] - sv[-1][1]) - (sv[0][2] - sv[0][1])
            d_sh = sv[-1][1] - sv[0][1]
            print(f"\n  肩关节 Δ={d_sh:+.2f}°   小臂绝对角 Δ={d_abs:+.2f}°   小臂相对角 Δ={d_rel:+.2f}°")
            print("  判据：小臂**绝对角**几乎不变 ⇒ 平行四连杆解耦（coupling gain = -1）")
            print("        小臂**相对角**几乎不变 ⇒ 串联网普通关节（无 coupling）")
            return 0

        sv = [(int(re.search(r"_(\d{3})$", n).group(1)), v[col]) for n, *v in fps]
        sv.sort()
        (s0, j0), (s1, j1) = sv[0], sv[-1]
        print(f"\n舵机 {s0}->{s1} (Δ{s1 - s0}°)  关节 {j0:+.2f}->{j1:+.2f}° (Δ{j1 - j0:+.2f}°)")
        print(f"  ⇒ 增益 {abs(j1 - j0) / abs(s1 - s0):.4f} 关节度/舵机度"
              f"   ⇒ yaml scale = {abs(s1 - s0) / abs(j1 - j0):.4f}")
        print("  逐帧：", "  ".join(f"{s}→{j:+.1f}" for s, j in sv))
        # 线性度
        ss = np.array([s for s, _ in sv], dtype=float)
        jj = np.array([j for _, j in sv], dtype=float)
        if len(ss) > 2:
            A = np.polyfit(ss, jj, 1)
            r = jj - np.polyval(A, ss)
            print(f"  线性拟合 关节 = {A[0]:.4f}·舵机 + {A[1]:.2f}   最大偏差 {np.abs(r).max():.2f}°")

    if a.overlay:
        row = fps[0]
        _, s, ox, oy, ths, the = row
        im = Image.open(targets[0]).crop(roi).convert("RGB")
        dr = ImageDraw.Draw(im)
        pts, joints = skeleton(ths, the, l1, l2, l3, n=40)
        l0 = (ox - roi[0], oy - roi[1])
        xy = [(l0[0] + q[0] * s, l0[1] + q[1] * s) for q in pts]
        dr.line(xy, fill=(80, 255, 140), width=3)
        for j in joints:
            cx, cy = l0[0] + j[0] * s, l0[1] + j[1] * s
            dr.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], outline=(255, 120, 80), width=3)
        w2, h2 = im.size
        im.resize((w2 * 2, h2 * 2), Image.LANCZOS).save(a.overlay)
        print(f"\n叠加图 -> {a.overlay}（绿=模型骨架，橙圈=肩/肘/腕/TCP）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
