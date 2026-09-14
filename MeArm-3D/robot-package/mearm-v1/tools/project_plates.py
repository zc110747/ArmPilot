#!/usr/bin/env python3
"""project_plates.py —— 相机标定 + 把 robot.yaml 的板件几何**投影到实拍帧**，直接产出纹理四角。

为什么不用「自动分割出目标板」
    实测否掉了这条路：机械臂的板、立柱、底座、舵机、控制板**全是同一种黑色**，
    且物理上通过关节块与螺栓**连成一体**。暗色连通域必然是"整条臂 + 底座"，
    不存在"只含一块板"的连通域。所以目标板的位置必须**由模型投影给出**，不是分割出来的。

为什么正交模型就够
    相机在机械臂正前方（−Y），三块目标板的大面法向实测都是 (0,1,0) ⇒ 板面**平行于像平面**
    ⇒ 投影退化为「缩放 + 平移 + 面内旋转」。板厚 5mm 的前后表面差异在正交投影下不可见，
    取"法向最朝相机"的那一面即可。画面约定：右 = 世界 +X，上 = 世界 +Z：
        u = ox + s·X(mm)
        v = oy − s·Z(mm)

目标函数（三项相乘，任一项退化都把总分压下去）
    ① 板内暗色比例：投影多边形内灰度 < THRESH 的比例（板是黑色件）。
    ② 空隙亮色比例：三块板联合 bbox 之内、联合多边形之外的像素应是亮背景。
       ★ 防退化：实测只优化①时，s=2.5 会把三块板挤成一团塞进左下底座暗区，
       拿到 97.4% 的假性满分（而 oy=770 已在画面外）。
    ③ 框边命中「暗↔亮」边界的比例：板上轮廓与亮背景的交界。

⚠️ 目标函数**不足以唯一定位**，必须目视复核
    实测：画面暗区太大（底座/立柱/线缆都暗），框整体平移 ~70px 后三项分数照样不低。
    所以标定结果一律要**看叠加图**确认框贴合板边，必要时用 `--s/--ox/--oy` 人工覆盖。
    同理，产出纹理后要**看纹理**：四角对了就是一块规整的板（能看出螺栓/镂空），
    错了立刻是歪的或夹进背景。数字自洽 ≠ 定位正确。

★ 提高分辨率的唯一办法：物理靠近相机
    实测（D58）：`base` 旋转对分辨率**零和**（净 1.015），肩/肘摆动**只改朝向不改 px/mm**
    —— 因为摆动平面 X–Z **平行于像平面**。所以想提高 px/mm，只能**把臂挪近相机**（或把相机挪近）。

    ⚠️ **但不能一口气挪到 1.6×**：笔记本相机基本是**固定焦点**，靠太近会直接失焦，
    那样"放大"换来的是"更糊"，净亏。正确做法是**扫距离找甜点**：
    `s` 尽量大，同时边缘过渡 `ramp_px` 不超 2.5，且三块板都还在画面内。
    所以本脚本加了 `--capture` + 一句话 CONCLUSION，把"挪一点 → 量一次"压成一条命令。

用法
    # ① 自动标定（在 HOME 位帧上搜索 s/ox/oy）
    python tools/project_plates.py FRAME.jpg --search --overlay out.png --tiles assets/textures/mearm/tiles

    # ② 距离扫描循环：开摄像头抓帧 → 标定 → 给一句话结论（挪近了就重复这条）
    node tools/set_joints.mjs --home                       # 先回 HOME，保证与上次可比
    python tools/project_plates.py --capture --search --snapshot .workbuddy/captures/sweep/d080.jpg \
           --overlay .workbuddy/captures/sweep/ov080.png

    # ③ 用已标定参数直接投影（相机与机械臂相对位置未变时）
    python tools/project_plates.py FRAME.jpg --tiles assets/textures/mearm/tiles

    # ④ 人工覆盖某一项
    python tools/project_plates.py FRAME.jpg --ox 530 --oy 645 --s 3.8 --tiles ...

    # ⑤ 抓帧时不是 HOME 位，就显式给关节角（否则投影会整体错位）
    python tools/project_plates.py FRAME.jpg --joints base=0,shoulder=30,elbow=112.62,gripper=50

退出码：0 正常；2 输入不可用。
"""
from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path


def _find_repo_root(start: Path) -> Path:
    """向上找同时含 `core/` 与 `robot-package/` 的那一层。

    刻意不用 `parents[N]`：本脚本从 `tools/` 搬进 `robot-package/mearm-v1/tools/` 后，
    那个深度常量会静默指向另一个**真实存在**的目录（`robot-package/mearm-v1`），
    报错会推迟到很远的地方才出现。向上找标记则在任意深度都成立。
    """
    for anc in (start, *start.parents):
        if (anc / "core").is_dir() and (anc / "robot-package").is_dir():
            return anc
    raise SystemExit(
        "✗ 无法从 %s 向上找到仓库根"
        "（需同时存在 core/ 与 robot-package/ 目录）" % start)


_REPO_ROOT = _find_repo_root(Path(__file__).resolve().parent)


import numpy as np
from PIL import Image, ImageDraw

ROOT = _REPO_ROOT
sys.path.insert(0, str(ROOT / "simulation" / "mujoco"))
from model import MeArmSim                     # noqa: E402
from robotcfg import load_physics, load_robot  # noqa: E402

THRESH = 90
SC = 4                                          # 搜索用降采样倍数
CAM_DIR = np.array([0.0, -1.0, 0.0])            # 相机视线（从 -Y 看向 +Y）

# 纹理分辨率目标 + 边缘过渡上限。与 capture_texture.py 的默认值同源同值。
# ★ 两者**互相制约**：靠得越近 `s` 越大，但固定焦点相机迟早失焦（`ramp` 上升）。
#   扫距离就是在找这两条曲线的交点 —— 所以结论里必须**同时**报这两个数。
TARGET_PX_PER_MM = 6.0
RAMP_MAX_PX = 2.5

# 搜索 s 的默认范围：以 CALIB 为中心留足余量（0.6×~1.8× 足够覆盖"挪近约 1.6 倍"）。
# 刻意**不无限放宽** —— 范围太宽会放大假性满分的风险（见文件头目标函数 ②）。
S_BAND = (0.60, 1.80)
# 搜索边界（画面 1280x720；世界原点＝肩枢轴，大致在画面中部偏下）。
OX_LO, OX_HI = -200.0, 1200.0
OY_LO, OY_HI = 0.0, 1000.0

# 标定结果：2026-09-13 在 HOME 位帧上实测定标（相机与机械臂相对位置未动时可直接复用）。
# 复核图 saw 红框贴合竖直板段、纹理为"黑底 + 两枚螺栓"。
CALIB = {"s": 3.80, "ox": 530.0, "oy": 645.0}

# 待投影的板：(MJCF geom 名, 输出纹理名, 大面尺寸 mm)
# 输出名必须与 tools/make_texture.py 的 --list 同源，否则贴图阶段会静默找不到文件。
JOBS = [
    ("upper_arm_link_vis", "upper_arm_link", (22.0, 74.0)),
    ("forearm_link_vis", "forearm_link", (18.0, 68.0)),
    ("column_link_vis0", "column_link_side", (30.0, 22.0)),
]


def load_make_texture():
    spec = importlib.util.spec_from_file_location("mt", ROOT / "tools" / "make_texture.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["mt"] = mod
    spec.loader.exec_module(mod)
    return mod


def load_capture_texture():
    """复用 capture_texture 的取帧与轮廓锐度。

    刻意 import 而不是重写：取帧的"取最后一帧"约定与锐度的归一化公式都是踩过坑的，
    复制一份必然漂移（本项目已因"两份清单"栽过一次，见 make_texture `--list` 的双向比对）。
    """
    spec = importlib.util.spec_from_file_location("ct", ROOT / "tools" / "capture_texture.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["ct"] = mod
    spec.loader.exec_module(mod)
    return mod


def facing_face_mm(model, data, gid: int) -> tuple[np.ndarray, float]:
    """取该 box「法向最朝相机」的面，返回世界顶点 4 个（mm，按绕面法向角度排序）。"""
    size = np.asarray(model.geom_size[gid], float)
    xpos = np.asarray(data.geom_xpos[gid], float)
    xmat = np.asarray(data.geom_xmat[gid], float).reshape(3, 3)

    best = None
    for axis in range(3):
        for sgn in (-1.0, 1.0):
            cos = float(np.dot(xmat[:, axis] * sgn, CAM_DIR))
            if best is None or cos > best[0]:
                best = (cos, axis, sgn)
    cos, axis, sgn = best

    tang = [j for j in range(3) if j != axis]
    raw = []
    for a in (-1.0, 1.0):
        for b in (-1.0, 1.0):
            loc = np.zeros(3)
            loc[axis] = sgn * size[axis]
            loc[tang[0]] = a * size[tang[0]]
            loc[tang[1]] = b * size[tang[1]]
            raw.append(xpos + xmat @ loc)
    # 必须按绕面法向的角度排序，否则 polygon() 画出的是自交的蝴蝶结
    u_ax, v_ax = xmat[:, tang[0]], xmat[:, tang[1]]

    def polar(w):
        d = w - xpos
        return np.arctan2(float(np.dot(d, v_ax)), float(np.dot(d, u_ax)))

    raw.sort(key=polar)
    return np.array(raw) * 1000.0, cos


def parse_joints(text: str | None) -> dict[str, float]:
    robot = load_robot()
    pose = dict(robot.home_pose)
    if text:
        for kv in text.split(","):
            k, _, v = kv.partition("=")
            if not _:
                raise SystemExit(f"关节参数要写成 name=value: {kv!r}")
            pose[k.strip()] = float(v)
    return pose


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="相机标定 + 板件几何投影")
    ap.add_argument("frame", nargs="?", type=Path, help="实拍帧（建议 HOME 位）；用 --capture 时可省略")
    ap.add_argument("--joints", help="抓帧时的关节角，如 base=0,shoulder=30,elbow=112.62,gripper=50")
    ap.add_argument("--search", action="store_true", help="重新搜索 s/ox/oy（否则用内置 CALIB）")
    ap.add_argument("--s", type=float, help="像素/毫米（覆盖）")
    ap.add_argument("--ox", type=float, help="世界原点在画面中的 x（覆盖）")
    ap.add_argument("--oy", type=float, help="世界原点在画面中的 y（覆盖）")
    ap.add_argument("--overlay", type=Path, help="输出叠加复核图")
    ap.add_argument("--tiles", type=Path, help="输出纹理目录（按 JOBS 的名字）")
    ap.add_argument("--px-per-mm", type=float, default=10.0, help="纹理归一化密度，默认 10")
    ap.add_argument("--no-tiles", action="store_true", help="只标定/投影，不出纹理")
    ap.add_argument("--capture", action="store_true",
                    help="先开摄像头抓一帧（替代给 frame 路径）—— 距离扫描循环用")
    ap.add_argument("--device", help="DirectShow 设备名（默认与 capture_texture 一致）")
    ap.add_argument("--snapshot", type=Path, help="把抓到的帧存到这里（留作对比证据）")
    ap.add_argument("--target", type=float, default=TARGET_PX_PER_MM,
                    help="目标 px/mm（默认 %(default)s）")
    ap.add_argument("--s-range", dest="s_range",
                    help=f"搜索 s 的范围 lo,hi（默认 CALIB.s × {S_BAND[0]:g}~{S_BAND[1]:g}）")
    args = ap.parse_args(argv)

    if args.capture:
        ct = load_capture_texture()
        device = args.device or ct.DEFAULT_DEVICE
        print(f"抓帧：{device} …")
        img = ct.grab_frame(device)
        print(f"  实拍 {img.size[0]}x{img.size[1]}")
        if args.snapshot:
            args.snapshot.parent.mkdir(parents=True, exist_ok=True)
            img.save(args.snapshot)
            print(f"  存 → {args.snapshot}")
    else:
        if args.frame is None:
            print("✗ 要么给 frame 路径，要么加 --capture", file=sys.stderr)
            return 2
        if not args.frame.exists():
            print(f"✗ 找不到帧: {args.frame}", file=sys.stderr)
            return 2
        img = Image.open(args.frame)

    if args.s_range:
        try:
            s_lo, s_hi = (float(v) for v in args.s_range.split(","))
        except ValueError:
            print(f"✗ --s-range 要写成 lo,hi：{args.s_range!r}", file=sys.stderr)
            return 2
    else:
        s0 = args.s if args.s is not None else CALIB["s"]
        s_lo, s_hi = s0 * S_BAND[0], s0 * S_BAND[1]

    pose = parse_joints(args.joints)
    sim = MeArmSim(robot=load_robot(), physics=load_physics())
    sim.reset(pose)
    model, data = sim.model, sim.data
    gid_of = {model.geom(i).name: i for i in range(model.ngeom)}

    print(f"姿态: {dict(sorted(pose.items()))}")
    faces = {}
    print("\n=== 各板面向相机的面（世界 mm）===")
    for gname, out_name, face_mm in JOBS:
        gid = gid_of.get(gname)
        if gid is None:
            print(f"  ✗ MJCF 里找不到 geom {gname}", file=sys.stderr)
            return 2
        verts, cos = facing_face_mm(model, data, gid)
        faces[gname] = verts
        print(f"  {out_name:18s} {gname:24s} cos={cos:+.3f}  "
              f"Z[{verts[:,2].min():7.1f},{verts[:,2].max():7.1f}]  "
              f"跨度 {np.ptp(verts[:,0]):.1f}x{np.ptp(verts[:,2]):.1f}mm")

    gray = img.convert("L")
    W, H = img.size
    SW, SH = W // SC, H // SC
    mask_small = np.asarray(gray.resize((SW, SH), Image.NEAREST), float) < THRESH
    print(f"\n帧 {W}x{H}（搜索用 {SW}x{SH}）  暗像素 {mask_small.mean()*100:.2f}%")

    m8 = mask_small.astype(np.int8)
    grad = (np.abs(np.diff(m8, axis=0, prepend=0))
            + np.abs(np.diff(m8, axis=1, prepend=0))).astype(float)

    canvas = Image.new("1", (SW, SH), 0)
    dcanvas = ImageDraw.Draw(canvas)
    union = Image.new("1", (SW, SH), 0)
    dunion = ImageDraw.Draw(union)

    def evaluate(s: float, ox: float, oy: float):
        plate_scores = {}
        edge_hits = []
        dunion.rectangle([0, 0, SW, SH], fill=0)
        for gname, _, _ in JOBS:
            v = faces[gname]
            pts = list(zip(((ox + s * v[:, 0]) / SC).tolist(), ((oy - s * v[:, 2]) / SC).tolist()))
            dcanvas.rectangle([0, 0, SW, SH], fill=0)        # ①
            dcanvas.polygon(pts, fill=1)
            sel = np.asarray(canvas, bool)
            n = int(sel.sum())
            plate_scores[gname] = float(mask_small[sel].mean()) if n else 0.0
            dcanvas.rectangle([0, 0, SW, SH], fill=0)        # ③
            dcanvas.polygon(pts, outline=1, width=1)
            esel = np.asarray(canvas, bool)
            if esel.any():
                edge_hits.append(float(grad[esel].mean()))
            dunion.polygon(pts, fill=1)

        usel = np.asarray(union, bool)                       # ②
        ys, xs = np.nonzero(usel)
        if ys.size == 0:
            return 0.0, plate_scores, 0.0, 0
        y0, y1, x0, x1 = int(ys.min()), int(ys.max()), int(xs.min()), int(xs.max())
        gap = ~usel[y0:y1 + 1, x0:x1 + 1]
        n_gap = int(gap.sum())
        if n_gap < 30:                                       # 板挤成一团 ⇒ 判退化
            return 0.0, plate_scores, 0.0, n_gap
        gap_bright = float((~mask_small[y0:y1 + 1, x0:x1 + 1][gap]).mean())
        edge_hit = float(np.mean(edge_hits)) if edge_hits else 0.0
        inside = float(np.mean(list(plate_scores.values())))
        return inside * gap_bright * (edge_hit / 1.5 + 0.05), plate_scores, gap_bright, n_gap

    def search(s_range, ox_range, oy_range, best):
        for s in s_range:
            for ox in ox_range:
                for oy in oy_range:
                    sc, ps, gb, ng = evaluate(s, ox, oy)
                    if best is None or sc > best[0]:
                        best = (sc, float(s), float(ox), float(oy), ps, gb, ng)
        return best

    s_step = (s_hi - s_lo) / 15.0                # 粗搜 16 档，精化再 ±1 档（步长 1/5）
    if args.search:
        print(f"\n搜索 s ∈ [{s_lo:.2f}, {s_hi:.2f}]  步长 {s_step:.3f}")
        best = search(np.arange(s_lo, s_hi + 1e-9, s_step),
                      np.arange(OX_LO, OX_HI + 1e-9, 50.0),
                      np.arange(OY_LO, OY_HI + 1e-9, 50.0), None)
        print(f"粗搜: {best[0]*100:5.1f}  s={best[1]:.2f} ox={best[2]:.0f} oy={best[3]:.0f}")
        _, s0, ox0, oy0, *_ = best
        best = search(np.arange(max(s_lo, s0 - s_step), min(s_hi, s0 + s_step) + 1e-9, s_step / 5.0),
                      np.arange(ox0 - 50, ox0 + 50 + 1e-9, 5.0),
                      np.arange(oy0 - 50, oy0 + 50 + 1e-9, 5.0), best)
        score, s, ox, oy, ps, gb, ng = best
        print(f"精化: {score*100:5.1f}  s={s:.3f} px/mm  ox={ox:.1f} oy={oy:.1f}")
        # ★ 边界命中检测：最优落在边界上 = 范围设窄了，这时的 s 是"被截断的值"，不能当成读数
        if s <= s_lo * 1.02 or s >= s_hi * 0.98:
            print(f"  ⚠️ s 落在搜索边界上 ⇒ 范围设窄了，用 --s-range {s*0.6:.1f},{s*1.8:.1f} 放宽重跑")
        if not (OX_LO + 1 < ox < OX_HI - 1) or not (OY_LO + 1 < oy < OY_HI - 1):
            print(f"  ⚠️ ox/oy 也贴到搜索边界 ⇒ 机械臂可能大半在画面外，先确认画面再信读数")
        print("  ⚠️ 请核对叠加图 —— 目标函数不足以唯一定位（见文件头）")
    else:
        s = args.s if args.s is not None else CALIB["s"]
        ox = args.ox if args.ox is not None else CALIB["ox"]
        oy = args.oy if args.oy is not None else CALIB["oy"]
        print(f"\n标定参数: s={s:.2f} px/mm  ox={ox:.0f} oy={oy:.0f}（内置 CALIB，--search 可重标）")

    # ★ 重跑一次 evaluate：搜索过程中 union/canvas 指的是"最后一个候选"而不是"最优候选"，
    # 直接拿来算锐度会**张冠李戴**（数字看着正常，量的却是错位置）。用最终参数重算一遍。
    score, ps, gb, ng = evaluate(s, ox, oy)

    for gname, out_name, _ in JOBS:
        print(f"  {out_name:18s} 框内暗色 {ps[gname]*100:5.1f}%")
    print(f"  空隙亮色 {gb*100:5.1f}%（{ng} 像素）")
    arm_px = 74.0 * s
    print(f"  折算：大臂板 74mm → {arm_px:.0f}px")

    # ---------------------------------------------------------------------
    # ★ 一句话结论 —— 距离扫描循环的核心输出（挪一点 → 跑一次 → 看这四行）
    # ---------------------------------------------------------------------
    ct = load_capture_texture()
    gray_small = np.asarray(gray.resize((SW, SH), Image.NEAREST), np.float64)
    union_small = np.asarray(union, bool)

    pts_all = np.vstack([np.column_stack([ox + s * faces[g][:, 0], oy - s * faces[g][:, 2]])
                         for g, _, _ in JOBS])
    x0f, x1f = float(pts_all[:, 0].min()), float(pts_all[:, 0].max())
    y0f, y1f = float(pts_all[:, 1].min()), float(pts_all[:, 1].max())
    pad = 8
    cropped = x0f < pad or y0f < pad or x1f > W - pad or y1f > H - pad

    if union_small.any():
        plate_mean = float(gray_small[union_small].mean())
        bg_mean = float(gray_small[~union_small].mean())
        # ★ 锐度量在**投影出来的板轮廓**上：板内部是均匀黑，量内部永远量不出"糊"（见 capture_texture）
        sharp = ct.boundary_sharpness(np.asarray(gray, np.float64), union_small, SC)
        ramp = max(1.0, bg_mean - plate_mean) / (2.0 * max(1e-6, sharp))
    else:
        sharp = 0.0
        ramp = float("inf")

    print("\n" + "=" * 74)
    print(f"CONCLUSION  s={s:.3f}px/mm  arm74={arm_px:.0f}px  ramp={ramp:.2f}px  "
          f"inside={np.mean(list(ps.values()))*100:.1f}%  gap={gb*100:.1f}%  score={score*100:.1f}")
    print(f"  投影范围 x[{x0f:.0f},{x1f:.0f}] y[{y0f:.0f},{y1f:.0f}] / 画面 {W}x{H}"
          + ("   ⚠️ 已出画" if cropped else "   在画面内 ✓"))
    if cropped:
        print("  ✗ 三块板没能全在画面内 ⇒ 退回去一点（或把臂往画面中心挪），否则纹理四角会被裁掉")
    elif ramp > RAMP_MAX_PX:
        print(f"  ✗ 失焦：边缘过渡 {ramp:.2f}px > 上限 {RAMP_MAX_PX:g} ⇒ **别再靠近了**，"
              f"这已是本机相机（固定焦点）的极限")
        print("    ⇒ 选项① 退回一点保住清晰度（s 会降）；选项② 位置不动、人工标四角："
              "python tools/make_texture.py --corners")
    elif s >= args.target:
        print(f"  ✓ 达标：s={s:.2f} ≥ 目标 {args.target:g} px/mm，边缘过渡 {ramp:.2f}px ≤ {RAMP_MAX_PX:g}")
        print("    ⇒ 可以出纹理了：同一条命令加 --tiles assets/textures/mearm/tiles，**然后目视复核纹理**")
    else:
        print(f"  → 还差 {args.target / s:.2f}×：把大臂板在画面里的长度从 {arm_px:.0f}px 调到 "
              f"≥{74 * args.target:.0f}px")
        print("    （画面里大 1.6 倍 = 距离缩到 0.62 倍。挪一点，再跑同一条命令）")
    print("=" * 74)

    if args.overlay:
        over = img.convert("RGB")
        dr = ImageDraw.Draw(over)
        palette = [(255, 60, 60), (60, 220, 60), (70, 140, 255)]
        for (gname, out_name, face_mm), col in zip(JOBS, palette):
            v = faces[gname]
            pts = list(zip((ox + s * v[:, 0]).tolist(), (oy - s * v[:, 2]).tolist()))
            dr.polygon(pts, outline=col)
            dr.line(pts + [pts[0]], fill=col, width=3)
            dr.text((pts[0][0] + 4, pts[0][1] + 4),
                    f"{out_name} {face_mm[0]:.0f}x{face_mm[1]:.0f}mm", fill=col)
        args.overlay.parent.mkdir(parents=True, exist_ok=True)
        over.save(args.overlay)
        print(f"\n叠加复核图 → {args.overlay}   ← **先看这张**：框要贴合板边")

    if args.tiles and not args.no_tiles:
        mt = load_make_texture()
        rgb = img.convert("RGB")
        args.tiles.mkdir(parents=True, exist_ok=True)
        print()
        for gname, out_name, face_mm in JOBS:
            v = faces[gname]
            corners = np.column_stack([ox + s * v[:, 0], oy - s * v[:, 2]])
            tile = mt.build_tile(rgb, mt.order_corners(corners), face_mm, args.px_per_mm)
            dst = args.tiles / f"{out_name}.png"
            tile.save(dst)
            print(f"  纹理 → {dst}  {tile.size[0]}x{tile.size[1]}px "
                  f"（{face_mm[0]:.0f}x{face_mm[1]:.0f}mm @{args.px_per_mm:g}px/mm）")
        print("  ⚠️ 再看一眼纹理：四角对了应是一块规整的板（有螺栓/镂空），错了会歪或夹进背景")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
