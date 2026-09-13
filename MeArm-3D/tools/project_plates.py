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

用法
    # 自动标定（在 HOME 位帧上搜索 s/ox/oy）
    python tools/project_plates.py FRAME.jpg --search --overlay out.png --tiles assets/textures/mearm/tiles

    # 用已标定参数直接投影（相机与机械臂相对位置未变时）
    python tools/project_plates.py FRAME.jpg --tiles assets/textures/mearm/tiles

    # 人工覆盖某一项
    python tools/project_plates.py FRAME.jpg --ox 530 --oy 645 --s 3.8 --tiles ...

    # 抓帧时不是 HOME 位，就显式给关节角（否则投影会整体错位）
    python tools/project_plates.py FRAME.jpg --joints base=0,shoulder=30,elbow=112.62,gripper=50

退出码：0 正常；2 输入不可用。
"""
from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "simulation" / "mujoco"))
from model import MeArmSim                     # noqa: E402
from robotcfg import load_physics, load_robot  # noqa: E402

THRESH = 90
SC = 4                                          # 搜索用降采样倍数
S_LO, S_HI = 3.8, 5.6                           # 74mm 大臂板在 720p 里约 280~410px
CAM_DIR = np.array([0.0, -1.0, 0.0])            # 相机视线（从 -Y 看向 +Y）

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
    ap.add_argument("frame", type=Path, help="实拍帧（建议 HOME 位）")
    ap.add_argument("--joints", help="抓帧时的关节角，如 base=0,shoulder=30,elbow=112.62,gripper=50")
    ap.add_argument("--search", action="store_true", help="重新搜索 s/ox/oy（否则用内置 CALIB）")
    ap.add_argument("--s", type=float, help="像素/毫米（覆盖）")
    ap.add_argument("--ox", type=float, help="世界原点在画面中的 x（覆盖）")
    ap.add_argument("--oy", type=float, help="世界原点在画面中的 y（覆盖）")
    ap.add_argument("--overlay", type=Path, help="输出叠加复核图")
    ap.add_argument("--tiles", type=Path, help="输出纹理目录（按 JOBS 的名字）")
    ap.add_argument("--px-per-mm", type=float, default=10.0, help="纹理归一化密度，默认 10")
    ap.add_argument("--no-tiles", action="store_true", help="只标定/投影，不出纹理")
    args = ap.parse_args(argv)

    if not args.frame.exists():
        print(f"✗ 找不到帧: {args.frame}", file=sys.stderr)
        return 2

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

    img = Image.open(args.frame)
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

    if args.search:
        best = search(np.arange(S_LO, S_HI + 0.001, 0.25),
                      np.arange(-100.0, 1001.0, 25.0), np.arange(100.0, 901.0, 25.0), None)
        print(f"\n粗搜: {best[0]*100:5.1f}  s={best[1]:.2f} ox={best[2]:.0f} oy={best[3]:.0f}")
        _, s0, ox0, oy0, *_ = best
        best = search(np.arange(max(S_LO, s0 - 0.3), min(S_HI, s0 + 0.3) + 0.001, 0.05),
                      np.arange(ox0 - 25, ox0 + 25.001, 5), np.arange(oy0 - 25, oy0 + 25.001, 5), best)
        score, s, ox, oy, ps, gb, ng = best
        print(f"精化: {score*100:5.1f}  s={s:.3f} px/mm  ox={ox:.1f} oy={oy:.1f}")
        print(f"  ⚠️ 请核对叠加图 —— 目标函数不足以唯一定位（见文件头）")
    else:
        s = args.s if args.s is not None else CALIB["s"]
        ox = args.ox if args.ox is not None else CALIB["ox"]
        oy = args.oy if args.oy is not None else CALIB["oy"]
        score, ps, gb, ng = evaluate(s, ox, oy)
        print(f"\n标定参数: s={s:.2f} px/mm  ox={ox:.0f} oy={oy:.0f}（内置 CALIB，--search 可重标）")

    for gname, out_name, _ in JOBS:
        print(f"  {out_name:18s} 框内暗色 {ps[gname]*100:5.1f}%")
    print(f"  空隙亮色 {gb*100:5.1f}%（{ng} 像素）")
    print(f"  折算：大臂板 74mm → {74*s:.0f}px")

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
