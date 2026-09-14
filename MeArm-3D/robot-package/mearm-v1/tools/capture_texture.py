"""采集判定 —— 从本机摄像头抓一帧，当场判定它能不能用来做纹理，并给出**该往哪动**。

## 为什么需要这个工具

`tools/make_texture.py` 能把照片变成纹理，但它**在拍完之后**才告诉你四角估偏了。
而采集是一个**闭环**：摆姿态 → 拍 → 判定 → 调整 → 再拍。
如果判定靠肉眼，每张照片都要人来回比划"够不够正对、够不够清晰、要不要靠近"，很慢且不稳。

本工具把 `docs/texture-capture-guide.md` §3 的四项硬性判据**变成数字**，并且
**从数字反推出动作**（靠近多少 / 绕哪根轴转 / 长边要不要横过来）。

## 与 make_texture.py 的分工（刻意的）

| | 本工具 | make_texture.py |
|---|---|---|
| 时机 | **采集时**，每张都跑 | 采集完成后，跑一次 |
| 分辨率 | **降采样**（默认 1/6）—— 只要够判几何 | **全分辨率** —— 要出纹理 |
| 产物 | 判定报告 + 复核图（红框） | 归一化纹理 tile |
| 失败时 | 告诉你**怎么调** | 报错并让你用 `--corners` |

**刻意不在这里做纹理**：降采样后四角精度只有 ±6px，拿来出纹理会糊。
本工具只负责"这张能不能用"，通过了就把**原图**交给 `make_texture.py`。

## 关键实测约束（决定了判定阈值）

本机内置摄像头是 `Integrated Camera`，**最高只有 1280×720**（见
`ffmpeg -f dshow -list_options true`）。因此：

- 板件要填满画面长边才可能拿到 6 px/mm 以上
- **画面的 1280 比 720 长** ⇒ 把板的长边**横过来**放，能白拿 1.78 倍分辨率。
  这条建议由 `advise()` 自动产出，不是让人记的规矩。

## 用法

```bash
python tools/capture_texture.py --plate upper_arm_link          # 抓一帧并判定
python tools/capture_texture.py --plate upper_arm_link --debug  # 附原始数字
python tools/capture_texture.py --plate base_link --save        # 通过则存入 raw/
python tools/capture_texture.py --from shot.jpg --plate forearm_link
python tools/capture_texture.py --list-devices
```
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from collections import deque
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
from PIL import Image, ImageDraw, ImageFilter

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_texture as mt  # noqa: E402  （同目录，直调脚本时 tools/ 在 sys.path[0]）

ROOT = _REPO_ROOT
CAPTURE_DIR = ROOT / ".workbuddy" / "captures" / "scene"

DEFAULT_DEVICE = "Integrated Camera"
DEFAULT_TARGET_PX_PER_MM = 6.0
# 边缘过渡宽度上限（px）。1.0 = 理想硬边（单像素跳变）。
#
# ⚠️ 刻意**不用梯度的绝对值**当判据。绝对值同时被**光照**缩放：
#    真机那一帧的背景墙只比黑板亮 119 级灰度，斜率天然只有 53；
#    换个亮一点的场景，同一条清晰的边会给出 100+。用绝对值当阈值，
#    等于把"灯够不够亮"当成"焦准不准"。
# 改用"过渡宽度 = ΔI / (2·斜率)"：理想硬边恒为 1.0，与光照、与放大率都无关。
# 实测校准（Integrated Camera 720p，真机场景）：
#    真机抓拍 1.12 px · 高斯模糊 r=1 → 1.8 px · r=2 → 3.1 px · r=3 → 4.6 px
# 取 2.5 ⇒ 放行真机现在的画质，挡住"明显虚焦"。
DEFAULT_RAMP_MAX_PX = 2.5
# 对边长度比落在 [1-tol, 1+tol] 之外判为"有透视"（即没有正对）
PARALLEL_TOL = 0.04


# ---------------------------------------------------------------------------
# 取帧
# ---------------------------------------------------------------------------
def list_devices() -> int:
    """列出 DirectShow 视频设备（ffmpeg 把设备表打到 stderr）。"""
    proc = subprocess.run(
        ["ffmpeg", "-hide_banner", "-list_devices", "true", "-f", "dshow", "-i", "dummy"],
        capture_output=True,
    )
    text = (proc.stderr or b"").decode("utf-8", "replace")
    lines = [ln.strip() for ln in text.splitlines() if '"' in ln]
    print("DirectShow 视频设备：")
    for ln in lines:
        if "(video)" in ln:
            print("  " + ln)
    return 0


def _split_jpegs(data: bytes) -> list[bytes]:
    """从连续的 MJPEG 字节流里切出每张 JPEG。

    ffmpeg 用 `-f image2pipe -vcodec mjpeg` 会把多帧 JPEG 首尾拼接，
    **不是**一个多帧容器，所以按 SOI/EOI 标记切。
    """
    out: list[bytes] = []
    start = data.find(b"\xff\xd8")
    while start != -1:
        end = data.find(b"\xff\xd9", start + 2)
        if end == -1:
            break
        out.append(data[start:end + 2])
        start = data.find(b"\xff\xd8", end + 2)
    return out


def grab_frame(device: str, warmup: int = 8, timeout: float = 20.0) -> Image.Image:
    """抓一帧**已经预热好**的画面。

    ⚠️ **刻意取最后一帧而不是第一帧**：dshow 设备启动瞬间会吐出黑帧/绿帧，
    抓 `-frames:v 1` 常常拿到它就是全黑。一次多抓几帧、只用最后一帧，
    比"抓一帧 → 发现是黑的 → 重试"稳，也少一次设备开关。
    """
    import io

    cmd = [
        "ffmpeg", "-hide_banner", "-loglevel", "error",
        "-f", "dshow", "-rtbufsize", "128M", "-i", f"video={device}",
        "-frames:v", str(max(1, warmup)),
        "-f", "image2pipe", "-vcodec", "mjpeg", "pipe:1",
    ]
    proc = subprocess.run(cmd, capture_output=True, timeout=timeout)
    frames = _split_jpegs(proc.stdout or b"")
    if not frames:
        err = (proc.stderr or b"").decode("utf-8", "replace").strip()
        raise RuntimeError(
            f"没抓到画面（设备 {device!r}）。ffmpeg 说：{err or '（无输出）'}\n"
            "  排查：① 设备名是否与 --list-devices 一致；② 相机是否被别的程序占用；"
            "③ Windows 隐私设置是否允许桌面应用访问相机。")
    return Image.open(io.BytesIO(frames[-1])).convert("RGB")


# ---------------------------------------------------------------------------
# 掩膜 → 目标轮廓
# ---------------------------------------------------------------------------
def largest_component(mask: np.ndarray) -> tuple[np.ndarray, int]:
    """取最大连通域（4 邻域 BFS）。

    ⚠️ 为什么必须取连通域而不是直接用整张掩膜：机械臂有**多块板**同时入画，
    整张掩膜 PCA 出来的是**整条臂**的外接四边形，而不是目标板的那一块。
    那会给出"看起来合理但完全错的"四角 —— 与 D56 同族的错法。

    纯 Python BFS，跑在降采样掩膜上（默认 1/6 ≈ 213×120），耗时可忽略。
    本机没有 cv2 / scipy.ndimage，所以不引第三方。
    """
    h, w = mask.shape
    labels = np.zeros((h, w), dtype=bool)
    best: np.ndarray | None = None
    best_size = 0
    ys, xs = np.nonzero(mask)
    for y0, x0 in zip(ys.tolist(), xs.tolist()):
        if labels[y0, x0]:
            continue
        comp = np.zeros((h, w), dtype=bool)
        comp[y0, x0] = True
        labels[y0, x0] = True
        queue = deque([(y0, x0)])
        size = 0
        while queue:
            y, x = queue.popleft()
            size += 1
            if y > 0 and mask[y - 1, x] and not labels[y - 1, x]:
                labels[y - 1, x] = comp[y - 1, x] = True
                queue.append((y - 1, x))
            if y + 1 < h and mask[y + 1, x] and not labels[y + 1, x]:
                labels[y + 1, x] = comp[y + 1, x] = True
                queue.append((y + 1, x))
            if x > 0 and mask[y, x - 1] and not labels[y, x - 1]:
                labels[y, x - 1] = comp[y, x - 1] = True
                queue.append((y, x - 1))
            if x + 1 < w and mask[y, x + 1] and not labels[y, x + 1]:
                labels[y, x + 1] = comp[y, x + 1] = True
                queue.append((y, x + 1))
        if size > best_size:
            best_size, best = size, comp
    if best is None:
        return np.zeros((h, w), dtype=bool), 0
    return best, best_size


def boundary_sharpness(gray: np.ndarray, comp_small: np.ndarray, scale: int) -> float:
    """轮廓锐度：板边界一圈上，灰度梯度最大值的中位数。

    ⚠️ **刻意不在板内部量**（这一版之前是错的）。板面是**均匀的黑色亚克力**，
    内部本来就没有高频：一块对不上焦但表面干净的板，内部方差同样接近 0 ——
    测不出"糊"。合成帧上它给出 0，真机上它会给出一个随机的小数字。
    **归因错了对象**，与 D56 同族。

    真正会因失焦而垮掉的是**轮廓**：对焦时 240→30 的跳变集中在 1px，
    失焦时摊成 5~10px 的斜坡。所以在边界上量梯度才对得上"糊不糊"。

    做法：在**降采样尺度**上取边界带（腐蚀 XOR 原掩膜），膨胀 1 小像素以兜住
    降采样映射误差，再把每个边界小像素映射到全分辨率的 (2·scale+1)² 窗口取最大梯度。
    中位数而不是均值 —— 边界上少量像素会落在螺丝/导线等杂部上，均值会被拖走。
    """
    h, w = gray.shape
    if h < 3 or w < 3:
        return 0.0
    gy = np.zeros_like(gray)
    gx = np.zeros_like(gray)
    gy[1:-1, :] = gray[2:, :] - gray[:-2, :]
    gx[:, 1:-1] = gray[:, 2:] - gray[:, :-2]
    gmag = np.hypot(gx, gy) / 2.0

    mimg = Image.fromarray((comp_small * 255).astype(np.uint8))
    inner = np.asarray(mimg.filter(ImageFilter.MinFilter(3))) > 127
    band = comp_small & ~inner
    if not band.any():
        return 0.0
    band = np.asarray(
        Image.fromarray((band * 255).astype(np.uint8)).filter(ImageFilter.MaxFilter(3))
    ) > 127

    ys, xs = np.nonzero(band)
    r = max(1, scale)
    vals = np.empty(len(ys), dtype=np.float64)
    for i, (y, x) in enumerate(zip(ys.tolist(), xs.tolist())):
        cy, cx = y * scale + scale // 2, x * scale + scale // 2
        y0, y1 = max(0, cy - r), min(h, cy + r + 1)
        x0, x1 = max(0, cx - r), min(w, cx + r + 1)
        vals[i] = gmag[y0:y1, x0:x1].max()
    return float(np.median(vals))


def _edge_lengths(corners: np.ndarray) -> tuple[float, float, float, float]:
    """四角 -> (上, 右, 下, 左) 四条边长，单位 px。顺序见 order_corners。"""
    tl, tr, br, bl = corners
    return (float(np.linalg.norm(tr - tl)), float(np.linalg.norm(br - tr)),
            float(np.linalg.norm(bl - br)), float(np.linalg.norm(tl - bl)))


# ---------------------------------------------------------------------------
# 判定 + 动作建议
# ---------------------------------------------------------------------------
def analyze(img: Image.Image, *, thresh: float, scale: int) -> dict:
    """降采样 → 分割 → 取最大连通域 → 四角与各项指标。"""
    w, h = img.size
    sw, sh = max(16, w // scale), max(16, h // scale)
    small = img.resize((sw, sh), Image.BILINEAR)
    gray_small = np.asarray(small.convert("L")).astype(np.float64)

    mask = mt.denoise(mt.segment(gray_small, thresh), radius=1)
    comp, size = largest_component(mask)
    if size < 24:
        return {"ok_mask": False, "mask_pixels": size}

    # 四角在降采样坐标里估计，再乘回原图尺度
    corners_small = mt.quad_from_mask(comp)
    corners = corners_small * float(scale)

    ys, xs = np.nonzero(comp)
    bbox_small = (int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1)
    bbox = tuple(v * scale for v in bbox_small)

    top, right, bottom, left = _edge_lengths(corners)
    long_px = max(top, right, bottom, left)
    short_px = min(top, right, bottom, left)
    # "长边横着放" 判据：上边比左边长 ⇒ 长边沿画面横向
    long_is_horizontal = top > left

    bbox_w = bbox[2] - bbox[0]
    bbox_h = bbox[3] - bbox[1]
    rect_fill = float(size) * (scale ** 2) / max(1.0, bbox_w * bbox_h)

    gray = np.asarray(img.convert("L")).astype(np.float64)
    inside = comp  # 在降采样坐标上取灰度统计，避免再算一遍多边形
    plate_mean = float(gray_small[inside].mean())
    bg_mean = float(gray_small[~inside].mean())

    return {
        "ok_mask": True,
        "frame_wh": (w, h),
        "corners": corners,
        "bbox": bbox,
        "mask_pixels": size,
        "edges": (top, right, bottom, left),
        "long_px": long_px,
        "short_px": short_px,
        "long_is_horizontal": long_is_horizontal,
        "rect_fill": rect_fill,
        "plate_mean": plate_mean,
        "bg_mean": bg_mean,
        "sharpness": boundary_sharpness(gray, comp, scale),
        "thresh": float(thresh),
    }


def evaluate(m: dict, *, face_mm: tuple[float, float], target_px_per_mm: float,
             ramp_max_px: float) -> tuple[str, list[str], list[str]]:
    """指标 → (状态, 阻塞项, 提示项)。状态取 "ok" / "reject" / "undecidable"。

    ⚠️ **"不可判定"与"判定不通过"必须分开**（本项目 D40/D44 的纪律：没有正面证据不下断言）。

    机械臂的板件是**螺栓连成一体的**，所以"画面里最大的暗色连通域"经常是
    **整条臂**而不是目标板。此时若照常算对边长度，会得出"左边 885 / 右边 313 px，
    差 182%"这种数字 —— 看着像"板歪得厉害"，实际是**归因错了对象**：
    人照着去调倾斜，调一天也调不好，因为问题根本不在倾斜。
    **归因错误比不判定更贵**，所以这里先判"能不能判"，再判"板好不好"。

    **阻塞项**必须改掉才能用；**提示项**只是提醒，不影响这张能不能用。
    建议一律给出**方向 + 量**：倾斜方向由"哪一对对边更短"推出 ——
    **短的那一对离镜头更远**。
    """
    short_mm, long_mm = face_mm
    frame_w, frame_h = m["frame_wh"]
    blockers: list[str] = []
    notes: list[str] = []

    # --- 0) 先判「这一帧能不能判定」 ---------------------------------------
    bx0, by0, bx1, by1 = (float(v) for v in m["bbox"])
    margin = 6
    touches_border = (bx0 <= margin or by0 <= margin
                      or bx1 >= frame_w - margin or by1 >= frame_h - margin)
    ratio = m["long_px"] / max(1e-6, m["short_px"])
    expect = long_mm / short_mm
    ratio_off = abs(ratio / expect - 1.0)

    if touches_border or ratio_off > 0.3:
        why = []
        if touches_border:
            cover = (bx1 - bx0) * (by1 - by0) / (frame_w * frame_h) * 100
            why.append(f"最大连通域贴边（外接框 {bx1 - bx0:.0f}×{by1 - by0:.0f} px，"
                       f"占画面 {cover:.0f}%）")
        if ratio_off > 0.3:
            why.append(f"长宽比 {ratio:.2f} 与目标板 {expect:.2f} 差 {ratio_off * 100:.0f}%")
        blockers.append("**不可判定** —— " + "；".join(why))
        blockers.append("→ 画面里最大的暗色连通域**不是目标板**，多半是整条臂连成一片"
                        "（板件由螺栓连成一体，暗色部分天然连通）")
        blockers.append("→ 两个办法任选：① 把相机推到只剩目标板在画面里；"
                        "② 在目标板后面插一张**白色卡片**，把相邻的暗色臂件挡住"
                        "（项目既有做法，见 docs/hardware-measurement.md §0）")
        return "undecidable", blockers, notes

    # --- 1) 分辨率 -----------------------------------------------------------
    px_long = m["long_px"] / long_mm
    px_short = m["short_px"] / short_mm
    px_used = min(px_long, px_short)
    if px_used < target_px_per_mm:
        need = m["long_px"] * (target_px_per_mm / px_used)
        fill_now = m["long_px"] / (frame_w if m["long_is_horizontal"] else frame_h)
        fill_need = need / (frame_w if m["long_is_horizontal"] else frame_h)
        blockers.append(
            f"分辨率不足：{px_used:.1f} < {target_px_per_mm:.1f} px/mm"
            f"（长边 {px_long:.1f} / 短边 {px_short:.1f}）")
        if (not m["long_is_horizontal"]) and frame_w > frame_h:
            gain = frame_w / frame_h
            blockers.append(
                f"→ 长边现在是**竖向**（可用的只有画面高 {frame_h}）。把它转成**横向**后，"
                f"在顶到画面边之前还能再靠近 {gain:.2f} 倍 —— px/mm 上限 ×{gain:.2f}")
        else:
            blockers.append(
                f"→ 让长边占到画面长边的 {fill_need * 100:.0f}%（现在 {fill_now * 100:.0f}%），"
                f"约等于再靠近 {(1 - fill_now / fill_need) * 100:.0f}%")

    # --- 2) 四角是否完整入画 -------------------------------------------------
    edge_hit = []
    for name, (x, y) in zip(("左上", "右上", "右下", "左下"), m["corners"]):
        if x < margin or y < margin or x > frame_w - margin or y > frame_h - margin:
            edge_hit.append(name)
    if edge_hit:
        blockers.append(f"板出画了（{('、'.join(edge_hit))}贴边）—— 退远一点，四角要完整可见")

    # --- 3) 正对（透视） ----------------------------------------------------
    top, right, bottom, left = m["edges"]
    v_ratio = top / bottom if bottom else 1.0        # 上下这一对对边
    h_ratio = left / right if right else 1.0         # 左右这一对对边
    tilt = max(v_ratio, 1 / v_ratio) - 1.0
    skew = max(h_ratio, 1 / h_ratio) - 1.0
    if tilt > PARALLEL_TOL:
        farther = "上边" if top < bottom else "下边"
        blockers.append(
            f"上下有透视：上边 {top:.0f} / 下边 {bottom:.0f} px（差 {tilt * 100:.1f}%）"
            f"—— {farther}离镜头更远")
        blockers.append("→ 把板的上沿/下沿朝镜头转，或把相机降到板面中心高度")
    if skew > PARALLEL_TOL:
        farther = "左边" if left < right else "右边"
        blockers.append(
            f"左右有透视：左边 {left:.0f} / 右边 {right:.0f} px（差 {skew * 100:.1f}%）"
            f"—— {farther}离镜头更远")
        blockers.append("→ 绕竖直轴把板转向镜头，或让板面正对再拍")

    # --- 4) 清晰度（归一化后的边缘过渡宽度） --------------------------------
    # ramp_px = ΔI / (2·斜率)：理想硬边 = 1.0。见 DEFAULT_RAMP_MAX_PX 的注释 ——
    # 用绝对值当判据会把"场景暗"误判成"焦不准"。
    delta_i = max(1.0, m["bg_mean"] - m["plate_mean"])
    ramp_px = delta_i / (2.0 * max(1e-6, m["sharpness"]))
    if ramp_px > ramp_max_px:
        blockers.append(
            f"偏糊：边缘过渡约 {ramp_px:.1f} px（上限 {ramp_max_px:.1f}）"
            "—— 网络摄像头对焦偏软，靠近一点通常就实了")

    # --- 5) 提示项（不阻塞） ------------------------------------------------
    # ⚠️ 判据是"**离分割阈值还有多少余量**"，不是"板和背景差多少"。
    #    因为 segment() 就是 `gray < thresh`：真正危险的是灰度**贴近阈值** ——
    #    背景贴近则阈值稍一波动整面墙就被当成板，板贴近则板的一部分被当成背景。
    #    （早先这里写的是 contrast < 40，那是**恒不成立**的死代码：背景必须亮过
    #      thresh 才能分割得开，所以"板与背景的差"永远大于 60。死判据比没有更坏 ——
    #      它在报告里显示为"已检查"。）
    thresh = float(m.get("thresh", mt.DEFAULT_THRESH))
    plate_margin = thresh - m["plate_mean"]
    bg_margin = m["bg_mean"] - thresh
    if min(plate_margin, bg_margin) < 25:
        notes.append(
            f"分割余量偏小：板离阈值 {plate_margin:.0f}、背景离阈值 {bg_margin:.0f}"
            f"（阈值 {thresh:.0f}）—— 灰度稍一波动就会错分，建议垫白纸拉开对比")
    if m["rect_fill"] < 0.55:
        notes.append(
            f"目标轮廓填充率只有 {m['rect_fill'] * 100:.0f}% —— 真机臂板是**镂空桁架**"
            "（外接盒 22×74 但中间是空的），加上可能还有其他臂件入画。"
            "这意味着直接烘纹理会把背景色带进孔洞；贴图阶段需要做孔洞掩膜，见 D57")

    return ("reject" if blockers else "ok"), blockers, notes


def draw_overlay(img: Image.Image, m: dict, plate: str) -> Image.Image:
    """在帧上画出估计的四角（红）与外接框（黄），用于**目视复核**。

    ⚠️ 复核图不是可选的：`quad_from_mask` 的前提是"掩膜里主要就是目标板"，
    前提不成立时它给出的四角**看起来依然合理**。唯一能发现这件事的办法是看图。
    """
    out = img.copy()
    d = ImageDraw.Draw(out)
    corners = [(float(x), float(y)) for x, y in m["corners"]]
    d.polygon(corners, outline=(255, 60, 60), width=3)
    x0, y0, x1, y1 = (int(v) for v in m["bbox"])
    d.rectangle([x0, y0, x1, y1], outline=(255, 210, 60), width=1)
    for label, (x, y) in zip(("TL", "TR", "BR", "BL"), corners):
        d.ellipse([x - 5, y - 5, x + 5, y + 5], outline=(255, 60, 60), width=2)
        d.text((x + 8, y - 18), label, fill=(255, 60, 60))
    d.text((10, 10), f"{plate}  long={m['long_px']:.0f}px  "
                     f"short={m['short_px']:.0f}px  sharp={m['sharpness']:.0f}",
           fill=(255, 210, 60))
    return out


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    mt.ensure_utf8_stdout()
    ap = argparse.ArgumentParser(
        description="摄像头抓帧 → 判定能否用作纹理 → 给出该往哪动",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="判定通过后跑： python tools/make_texture.py <plate>")
    ap.add_argument("--plate", help="目标板件名（见 make_texture.py --list）")
    ap.add_argument("--device", default=DEFAULT_DEVICE, help=f"DirectShow 设备名（默认 {DEFAULT_DEVICE}）")
    ap.add_argument("--from", dest="src", help="改用已有的图片文件，不开摄像头")
    ap.add_argument("--list-devices", action="store_true", help="列出视频设备")
    ap.add_argument("--target-px-per-mm", type=float, default=DEFAULT_TARGET_PX_PER_MM)
    ap.add_argument("--ramp-max-px", type=float, default=DEFAULT_RAMP_MAX_PX,
                    help="边缘过渡宽度上限，px（1.0 = 理想硬边；默认 2.5）")
    ap.add_argument("--thresh", type=float, default=mt.DEFAULT_THRESH)
    ap.add_argument("--analysis-scale", type=int, default=6, help="判定用降采样倍数（默认 6）")
    ap.add_argument("--warmup", type=int, default=8, help="丢弃前 N 帧（默认 8）")
    ap.add_argument("--save", action="store_true", help="判定通过则存入 assets/textures/mearm/raw/")
    ap.add_argument("--debug", action="store_true", help="打印原始指标")
    args = ap.parse_args(argv)

    if args.list_devices:
        return list_devices()

    if not args.plate:
        ap.error("需要 --plate（或 --list-devices）")
    sizes = mt.load_plate_sizes()
    if args.plate not in sizes:
        print(f"✗ 未知板件名 {args.plate!r}；可用：{', '.join(sorted(sizes))}")
        return 2
    face = sizes[args.plate]

    if args.src:
        img = Image.open(args.src).convert("RGB")
        src_desc = args.src
    else:
        print(f"抓帧中（{args.device}，丢弃前 {args.warmup} 帧）…")
        img = grab_frame(args.device, warmup=args.warmup)
        src_desc = args.device

    m = analyze(img, thresh=args.thresh, scale=max(1, args.analysis_scale))
    print()
    print("=" * 62)
    print(f"采集判定 · {args.plate}（大面 {face[0]:.0f} × {face[1]:.0f} mm）")
    print("=" * 62)
    print(f"  来源          {src_desc}")
    print(f"  帧尺寸        {img.size[0]}×{img.size[1]}")

    if not m["ok_mask"]:
        print(f"  ✗ 没找到目标轮廓（掩膜只有 {m['mask_pixels']} 像素）")
        print("    → 板太亮 / 背景太暗 ⇒ 分割失效；或板没在画面里。")
        print(f"    → 试着调 --thresh（现在 {args.thresh:.0f}），或垫白纸做背景。")
        CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
        img.save(CAPTURE_DIR / f"{args.plate}_nofind.png")
        return 1

    status, blockers, notes = evaluate(m, face_mm=face,
                                      target_px_per_mm=args.target_px_per_mm,
                                      ramp_max_px=args.ramp_max_px)
    ok = status == "ok"
    # 归一化后的边缘过渡宽度（见 DEFAULT_RAMP_MAX_PX）：理想硬边 = 1.0
    ramp_px = max(1.0, m["bg_mean"] - m["plate_mean"]) / (2.0 * max(1e-6, m["sharpness"]))
    if status == "undecidable":
        # 不可判定时，"四角/对边/px/mm"都是拿**整条臂**算出来的虚数。
        # 打出来会被当成真实指标（12 px/mm 看着还挺好），所以只打真正被用到的原始事实。
        bx0, by0, bx1, by1 = (int(v) for v in m["bbox"])
        cover = (bx1 - bx0) * (by1 - by0) / (img.size[0] * img.size[1]) * 100
        print(f"  最大连通域    外接框 {bx1 - bx0}×{by1 - by0} px · 占画面 {cover:.0f}%"
              f" · 长宽比 {m['long_px'] / max(1e-6, m['short_px']):.2f}")
        print(f"  目标板        大面 {face[0]:.0f}×{face[1]:.0f} mm"
              f" · 长宽比 {face[1] / face[0]:.2f}")
        print(f"  边缘过渡      {ramp_px:.2f} px（1.00 = 理想硬边，上限 {args.ramp_max_px:.1f}）")
        print(f"  灰度          板 {m['plate_mean']:.0f} / 背景 {m['bg_mean']:.0f}")
    else:
        top, right, bottom, left = m["edges"]
        px_long = m["long_px"] / face[1]
        px_short = m["short_px"] / face[0]
        print(f"  边缘长度      上 {top:.0f} · 右 {right:.0f} · 下 {bottom:.0f} · 左 {left:.0f} px")
        print(f"  px/mm         长边 {px_long:.1f} · 短边 {px_short:.1f}"
              f"  → 取 {min(px_long, px_short):.1f}（目标 {args.target_px_per_mm:.1f}）")
        print(f"  长边朝向      {'横向' if m['long_is_horizontal'] else '竖向'}")
        print(f"  轮廓填充率    {m['rect_fill'] * 100:.0f}%")
        print(f"  边缘过渡      {ramp_px:.2f} px（1.00 = 理想硬边，上限 {args.ramp_max_px:.1f}）")
        print(f"  灰度          板 {m['plate_mean']:.0f} / 背景 {m['bg_mean']:.0f}")
    if args.debug:
        print(f"  [debug] 四角 px  {[(round(float(x), 1), round(float(y), 1)) for x, y in m['corners']]}")
        print(f"  [debug] bbox     {m['bbox']}")
        print(f"  [debug] 掩膜像素 {m['mask_pixels']}（降采样 1/{args.analysis_scale}）")

    print()
    for line in blockers:
        if line.startswith("→"):
            print(f"     {line}")
        elif line.startswith("**不可判定**"):
            print(f"  ? {line}")
        else:
            print(f"  ✗ {line}")
    for line in notes:
        print(f"  ⚠ {line}")
    if ok:
        print("  ✓ 判定通过 —— 这张可以用来做纹理")
        print(f"    → 下一步： python tools/make_texture.py {args.plate}")
    elif status == "undecidable":
        print("  ⚠ 注意：这是「这一帧判不了」，不是「板不合格」——"
              "先让目标板单独成块，再拍一次")

    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
    overlay = draw_overlay(img, m, args.plate)
    shot = CAPTURE_DIR / f"{args.plate}.png"
    over = CAPTURE_DIR / f"{args.plate}_corners.png"
    overlay.save(over)
    print()
    if status == "undecidable":
        print(f"  复核图  {over.relative_to(ROOT)}   ← 看红框：它框住的会是**整条臂**，"
              "这正是不合格的原因")
    else:
        print(f"  复核图  {over.relative_to(ROOT)}   ← **先看这张，确认红框贴合板边**")

    if ok and args.save:
        mt.RAW_DIR.mkdir(parents=True, exist_ok=True)
        raw = mt.RAW_DIR / f"{args.plate}.jpg"
        img.save(raw, quality=95)
        print(f"  原图    {raw.relative_to(ROOT)}")
    elif ok:
        print(f"  （加 --save 可把原图存入 {mt.RAW_DIR.relative_to(ROOT)}/）")
    else:
        img.save(shot)
        print(f"  原图    {shot.relative_to(ROOT)}（未通过，未存入 raw/）")

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
