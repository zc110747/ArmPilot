#!/usr/bin/env python3
"""从实拍照片生成板件纹理 —— 透视校正 + 归一化 + 目视复核图。

背景（为什么需要它）
--------------------
采集指南（`docs/texture-capture-guide.md`）要求"板的大面正对镜头"，但**人手拍照必然
带透视**（远边短、近边长的梯形）。直接把带透视的照片贴到板面上，纹理会被拉歪 ——
而且这种歪**看起来不像 bug，像"模型有点怪"**，很难归因。

本脚本把照片里板的**任意四边形**映射成**按真实尺寸归一化的矩形纹理**：
    raw/<name>.jpg ──分割──▶ 掩膜 ──PCA 估四角──▶ 透视校正 ──▶ 填孔 ──▶ tiles/<name>.png

归一化的目标是**真实尺寸**（来自 `config/robot.yaml` 的板件 `size`，单位 mm），
所以不同板件的纹理密度天然一致，不需要在照片里放标尺。

为什么还要"填孔"（ADR D61 / D62）
---------------------------------
实板是**镂空桁架**，而模型的 `*_vis` 是**实心 box**。照片里"镂空透出的背景 /
被四角框进来的相邻板 / 悬在板前的线缆"烘到实心盒上，会把背景色一起带进去 ——
**比不贴更假**。所以默认把非板面区域填成**板面材质**，让纹理与实心几何语义一致。

判据见 `fill_face_background()`，三层，缺一不可：
    ① **连通性**（不是纯亮度 —— 纯亮度会把板面上的螺栓一起填掉）；
    ② **形状**（区分"螺栓"与"被暗区包围的线缆小段" —— 只判面积会把后者留下）；
    ③ **色差**（揪出亮度低于阈值、但偏暖的**暗线缆** —— 只判亮度永远去不掉它）。

铁律
----
1. **四角必须目视复核**。PCA 估计在背景不干净（阴影 / 别的零件 / 线缆）时会给出
   看似合理但**错**的四角 —— 这与 D39/D47 里"量包围盒必须目视复核"是同一类教训。
   每次运行都会落一张 `<name>_corners.png`（含"未填孔 / 填充后"两栏），**先看图再信结果**。
2. 估计不可靠时用 `--corners` 手动给四个角（顺序见 `--help`），不要硬调阈值凑。
3. 本脚本只处理**外观**，不碰运动学/物理（冻结守卫 D55 对外观层放行）。
4. **填孔默认开启**（`--no-fill-holes` 可关）。填完是"近黑板 + 螺栓亮点" ——
   这不是"丢了信息"，实板本来就是这样，只是模型还没有镂空几何。
   每次运行会打印**四边亮比例**：框精确贴合板时四条边应都接近 0；
   实测当前四角四边亮 18~62% ⇒ **框比板大**，纹理内容会被压缩（见 D62）。

用法
----
    <python> tools/make_texture.py --list                  # 列出可处理的板件
    <python> tools/make_texture.py --all                   # 处理 raw/ 下所有照片
    <python> tools/make_texture.py upper_arm_link          # 处理一张
    <python> tools/make_texture.py upper_arm_link --corners "x,y;x,y;x,y;x,y"
    <python> tools/make_texture.py upper_arm_link --no-fill-holes   # 看未填孔的校正结果
    <python> tools/make_texture.py --selftest              # 合成数据自测（不需要照片）
"""

from __future__ import annotations

import argparse
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
import yaml
from PIL import Image, ImageDraw, ImageFilter

ROOT = _REPO_ROOT
sys.path.insert(0, str(ROOT / "core" / "python"))

from robopkg import PackageError, declared_path  # noqa: E402

#: 本工具**只**服务 MeArm-V1 的板件（显式写 id，不读选择器 `default`）。
MEARM_V1 = "mearm-v1"

#: 真值路径来自该包 manifest 的**声明**。Phase 2 之前这里写的是
#: `ROOT / "config" / "robot.yaml"`（那时真值还在仓库根的 `config/` 下）。
try:
    ROBOT_YAML = declared_path(MEARM_V1, "model.config")
except PackageError as exc:  # pragma: no cover - 只在包被搬坏时触发
    raise SystemExit(f"✗ 无法从 manifest 解析 {MEARM_V1} 的 model.config：{exc}") from exc

RAW_DIR = ROOT / "assets" / "textures" / "mearm" / "raw"
TILE_DIR = ROOT / "assets" / "textures" / "mearm" / "tiles"
PREVIEW_DIR = ROOT / ".workbuddy" / "analysis" / "texture"

# 默认按 10 px/mm 归一化：22mm 宽 → 220px（足够近看，单张纹理 ~200KB 级）
DEFAULT_PX_PER_MM = 10.0
# 白底暗件分割阈值 —— 与项目既有做法一致（docs/hardware-measurement.md §0）
DEFAULT_THRESH = 90

# 板面 / 非板面（镂空透出的背景、被框进来的相邻件、线缆）的分界阈值。
# 实测纹理亮度呈**双峰**：板面峰 0~20、非板面峰 110~125，中间 30~90 是谷底，
# 60 落在谷底平台上 ⇒ 对阈值不敏感。数据见 `.workbuddy/captures/hole_probe.py`。
DEFAULT_FACE_THRESH = 60.0
# 被板面**完全包围**的亮区：面积占比 < 此值当"螺栓"保留，≥ 当"封闭镂空"填掉。
DEFAULT_KEEP_MAX_FRAC = 0.03
# ★ 但**只靠面积不够**：被暗区包围的**线缆小段**也会满足面积条件（实测踩到 ——
#   大臂纹理左下多出一块橙色残留）。加形状判据：
#   螺栓接近圆/方（bbox 填充率 ≈ π/4 = 0.785），线缆段是不规则细长条。
DEFAULT_BLOB_FILL_MIN = 0.60    # 亮斑的 bbox 填充率下限（面积 / 外接矩形面积）
DEFAULT_BLOB_ASPECT = 2.0       # 亮斑的外接矩形长宽比上限（螺栓 ≈ 1）


def ensure_utf8_stdout() -> None:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")  # type: ignore[union-attr]
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 板件尺寸（从 robot.yaml 读，不硬编码）
# ---------------------------------------------------------------------------
def big_face_mm(size: list[float]) -> tuple[float, float]:
    """plate 的 `size` = [宽X, 厚Y, 长Z]（mm）→ 大面的两个边长 (短边, 长边)。

    大面 = **去掉最小那一维**后剩下两维张成的面（法向沿最小那一维）。
    例：[22, 5, 74] → (22, 74)（法向 ±Y）；[94, 82, 7] → (82, 94)（法向 ±Z）。
    """
    s = sorted(float(v) for v in size)
    assert len(s) == 3, f"plate size 应为 3 个数，得到 {size}"
    return s[1], s[2]


# details[] 里值得贴图的板件给可读名（**与采集指南的表格逐字一致**）。
# 不在这里的会退化成 "<link>_detail<i>"，但那种件通常不需要贴图。
DETAIL_ALIASES: dict[tuple[str, int], str] = {
    ("column_link", 0): "column_link_side",     # 立柱侧板（两片，同尺寸同纹理）
    ("column_link", 1): "column_link_side",
    ("upper_arm_link", 0): "upper_arm_link",    # 与大臂主板同尺寸 ⇒ 共用一张纹理
    ("upper_arm_link", 3): "upper_arm_brace",   # 端部横撑
    ("forearm_link", 0): "forearm_link",        # 与小臂主板同尺寸
    ("forearm_link", 3): "forearm_brace",       # 腕部横撑
}


def texture_name(link_id: str, detail_index: int | None) -> str:
    """纹理文件名（不含扩展名）—— 与采集指南第 2 节的表格对齐。"""
    if detail_index is None:
        return link_id
    return DETAIL_ALIASES.get((link_id, detail_index), f"{link_id}_detail{detail_index}")


def load_plate_sizes() -> dict[str, tuple[float, float]]:
    """robot.yaml → {纹理名: (短边mm, 长边mm)}。

    ⚠️ 同名条目必须**尺寸一致**：`upper_arm_link` 的主件与 detail0 尺寸相同 ——
    它们是**同一块板的两个实例**，本来就该共用一张纹理。
    若哪天它们尺寸不同，说明别名映射错了：此时**报错**而不是静默覆盖
    （静默覆盖会让后一个尺寸悄悄生效，纹理被悄悄拉伸 —— 又一个难查的错）。
    """
    with ROBOT_YAML.open("r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh)

    out: dict[str, tuple[float, float]] = {}
    for link in doc["links"]:
        geo = link.get("geometry") or {}
        lid = link["id"]
        candidates: list[tuple[str, tuple[float, float]]] = []
        if geo.get("type") == "plate":
            candidates.append((texture_name(lid, None), big_face_mm(geo["size"])))
        for i, det in enumerate(link.get("details") or []):
            if det.get("type") == "plate":
                candidates.append((texture_name(lid, i), big_face_mm(det["size"])))

        for name, face in candidates:
            if name in out and out[name] != face:
                raise ValueError(
                    f"纹理名冲突：{name} 同时对应 {out[name]} 与 {face}。"
                    "同名意味着同一个纹理，尺寸必须一致 —— 请检查 DETAIL_ALIASES")
            out[name] = face
    return out


# ---------------------------------------------------------------------------
# 几何：四角估计与排序
# ---------------------------------------------------------------------------
def segment(gray: np.ndarray, thresh: float) -> np.ndarray:
    """白底暗件分割：灰度 < 阈值 视为板件。返回 bool 掩膜。"""
    return gray < thresh


def denoise(mask: np.ndarray, radius: int = 2) -> np.ndarray:
    """先腐蚀再去膨胀（开运算）—— 去掉孤立噪点，但**不**改变整体外轮廓。

    用 PIL 自带的 Min/MaxFilter（本机没有 cv2，也没有 scipy.ndimage）。
    """
    if radius <= 0:
        return mask
    img = Image.fromarray((mask * 255).astype(np.uint8))
    size = radius * 2 + 1
    img = img.filter(ImageFilter.MinFilter(size))
    img = img.filter(ImageFilter.MaxFilter(size))
    return np.asarray(img) > 127


def quad_from_mask(mask: np.ndarray) -> np.ndarray:
    """最小面积外接四边形的四点估计（PCA + 对角极值），顺序为 [TL, TR, BR, BL]。

    等价于 cv2.minAreaRect 的常见做法，但只用 numpy：
      1. PCA 求主轴 → 把点云旋到与轴对齐
      2. 对齐后取 (x+y)、(x−y) 的最大/最小四个极值点 = 四角
      3. 旋回原坐标

    ⚠️ 依赖"掩膜里主要就是目标板"这一前提。背景不干净时会给出**看似合理的错四角**，
       所以必须看复核图。
    """
    ys, xs = np.nonzero(mask)
    if len(xs) < 16:
        raise ValueError(f"掩膜像素太少（{len(xs)}）—— 阈值不合适或照片里没有板件")
    pts = np.column_stack([xs, ys]).astype(np.float64)
    mean = pts.mean(axis=0)
    q = pts - mean

    cov = np.cov(q.T)
    _, vecs = np.linalg.eigh(cov)          # 特征值升序
    axes = vecs[:, ::-1]                   # 主方向在前
    rot = q @ axes                         # 旋到主轴坐标

    s = rot[:, 0] + rot[:, 1]
    d = rot[:, 0] - rot[:, 1]
    corners_rot = rot[[np.argmax(s), np.argmax(d), np.argmin(s), np.argmin(d)]]
    corners = corners_rot @ axes.T + mean
    return order_corners(corners)


def order_corners(corners: np.ndarray) -> np.ndarray:
    """把 4 个点排成 [左上, 右上, 右下, 左下]（图像坐标，y 向下）。"""
    c = np.asarray(corners, dtype=np.float64)
    assert c.shape == (4, 2), f"需要 4 个二维点，得到 {c.shape}"
    order = np.argsort(c[:, 1])
    top = c[order[:2]]
    bottom = c[order[2:]]
    top = top[np.argsort(top[:, 0])]
    bottom = bottom[np.argsort(bottom[:, 0])]
    return np.array([top[0], top[1], bottom[1], bottom[0]])


# ---------------------------------------------------------------------------
# 校正
# ---------------------------------------------------------------------------
def homography(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """标准 DLT：4 点对应的 3×3 单应矩阵（src → dst）。"""
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    A = []
    for (x, y), (u, v) in zip(src, dst):
        A.append([x, y, 1, 0, 0, 0, -u * x, -u * y])
        A.append([0, 0, 0, x, y, 1, -v * x, -v * y])
    h, *_ = np.linalg.lstsq(np.asarray(A), dst.reshape(-1), rcond=None)
    return np.append(h, 1.0).reshape(3, 3)


# ---------------------------------------------------------------------------
# 重采样内核
#
# ⚠️ **坐标约定必须写死在这里**（否则就是下一个"看起来能用、结果微偏"的坑）：
#     输出像素**索引** (i, j) 对应源坐标 H·(i, j, 1)；
#     且两张图的四角按 **左上 / 右上 / 右下 / 左下** 一一对应，
#     即输出索引 (0,0)↔源(0,0)、（out_w-1,0)↔源 A、（out_w-1,out_h-1)↔源 B、
#     (0,out_h-1)↔源 C。
#
# ★ 实测教训（ADR D56）：**PIL 的 `Image.transform(QUAD)` 不能用于这条链路**。
#     同一组四角、同一个 round-trip，PIL QUAD 给出**系统性 +12~17px 平移**
#     （刻度线 185/370/554 → 197/387/567），而本内核偏差 <1px。
#     这是"PIL 内部约定与自己的 homography 不一致"造成的静默几何错位 ——
#     它不会报错、看起来完全正常，只是纹理被悄悄挪了一点点。
# ---------------------------------------------------------------------------
def _sample_bilinear(src: np.ndarray, u: np.ndarray, v: np.ndarray
                     ) -> tuple[np.ndarray, np.ndarray]:
    """双线性采样。`src` 可为 (h,w) 或 (h,w,C)；u/v 是输出网格的源坐标。"""
    h, w = src.shape[:2]
    # 半像素容差：双线性采样在边缘 ±0.5px 内可以 clamp 到边界像素。
    # 用严格的 [0, w-1] 会让"四角估计差半像素"的边界行被判成区域外、填上背景色 ——
    # 表现成**纹理边缘一圈白边**（而真实照片的四角估计永远差那么一点点）。
    inside = (u >= -0.5) & (u <= w - 0.5) & (v >= -0.5) & (v <= h - 0.5)

    uu = np.clip(u, 0, w - 1)
    vv = np.clip(v, 0, h - 1)
    u0 = np.floor(uu).astype(int)
    v0 = np.floor(vv).astype(int)
    u1 = np.clip(u0 + 1, 0, w - 1)
    v1 = np.clip(v0 + 1, 0, h - 1)
    du = uu - u0
    dv = vv - v0
    if src.ndim == 3:                      # 让权重能广播到通道维
        du = du[..., None]
        dv = dv[..., None]

    sampled = (src[v0, u0] * (1 - du) * (1 - dv) + src[v0, u1] * du * (1 - dv)
               + src[v1, u0] * (1 - du) * dv + src[v1, u1] * du * dv)
    return sampled, inside


def _warp(src: np.ndarray, h_mat: np.ndarray, out_w: int, out_h: int,
          fill: float = 255.0) -> np.ndarray:
    """按单应 `h_mat`（输出坐标 → 源坐标）重采样。四角约定见上方注释块。"""
    ys, xs = np.mgrid[0:out_h, 0:out_w]
    p = np.stack([xs.ravel().astype(float), ys.ravel().astype(float),
                  np.ones(out_w * out_h)])
    q = h_mat @ p
    u = (q[0] / q[2]).reshape(out_h, out_w)
    v = (q[1] / q[2]).reshape(out_h, out_w)

    sampled, inside = _sample_bilinear(src, u, v)
    if src.ndim == 3:
        out = np.full((out_h, out_w, src.shape[2]), fill)
    else:
        out = np.full((out_h, out_w), fill)
    out[inside] = sampled[inside]
    return out


def rectify(img: Image.Image, corners: np.ndarray, out_w: int, out_h: int) -> Image.Image:
    """把 `img` 里由 `corners`([TL,TR,BR,BL]) 围出的四边形拉成 out_w×out_h 矩形。

    ★ 刻意**不用** PIL 的 `Image.transform(QUAD)` —— 理由与实测数字见上方注释块。
    """
    arr = np.asarray(img.convert("RGB") if img.mode not in ("L", "RGB") else img,
                     dtype=np.float64)
    dst_corners = np.array(
        [[0, 0], [out_w - 1, 0], [out_w - 1, out_h - 1], [0, out_h - 1]], dtype=np.float64)
    h_mat = homography(dst_corners, np.asarray(corners, dtype=np.float64))  # 输出 → 源
    return Image.fromarray(np.clip(_warp(arr, h_mat, out_w, out_h), 0, 255).astype(np.uint8))


def project(texture: Image.Image, canvas_wh: tuple[int, int],
            dst_quad: np.ndarray) -> Image.Image:
    """把 `texture` 按单应投影到白底画布上的 `dst_quad`（模拟"拍照产生的透视"）。

    仅用于 `--selftest` —— 有了它，校正管线就有**已知真值**可对。
    与 `rectify` 共用 `_warp` 与同一套坐标约定，所以 round-trip 可验证。
    """
    t = np.asarray(texture.convert("L"), dtype=np.float64)
    th, tw = t.shape
    src_corners = np.array([[0, 0], [tw - 1, 0], [tw - 1, th - 1], [0, th - 1]],
                           dtype=np.float64)
    h_mat = homography(src_corners, np.asarray(dst_quad, dtype=np.float64))  # 纹理 → 画布
    canvas = _warp(t, np.linalg.inv(h_mat), canvas_wh[0], canvas_wh[1], fill=255.0)
    return Image.fromarray(np.clip(canvas, 0, 255).astype(np.uint8))


# ---------------------------------------------------------------------------
# 孔洞掩膜：把"非板面"区域填成板面材质
#
# 为什么需要（ADR D61）：实板是**镂空桁架**，而模型的 `*_vis` 是**实心 box**。
# 照片里"镂空透出的背景 / 被四角框进来的相邻板 / 悬在板前的线缆"烘到实心盒上，
# 会把背景色一起带进去 —— **比不贴更假**。填成板面色后，纹理与实心几何语义一致。
#
# 判据是**连通性**，不是纯亮度。纯亮度会把板面上的螺栓一起填掉（螺栓也是亮的），
# 而连通性天然区分两者：
#     接触纹理边界的亮区  = 板外的东西 ⇒ 填
#     被板面完全包围的亮区 = 板上的东西 ⇒ 按面积判断（小=螺栓保留，大=镂空填掉）
# ---------------------------------------------------------------------------
def _grow_mask(mask: np.ndarray, radius: int = 1) -> np.ndarray:
    """二值掩膜膨胀（本机无 scipy/cv2，用 PIL 的 MaxFilter）。"""
    if radius <= 0:
        return mask
    img = Image.fromarray((mask * 255).astype(np.uint8))
    img = img.filter(ImageFilter.MaxFilter(radius * 2 + 1))
    return np.asarray(img) > 127


def _flood_bright_from_border(bright: np.ndarray) -> np.ndarray:
    """从四条边的亮像素泛洪，返回**与纹理边界连通**的亮区掩膜。

    这是"孔洞 vs 表面细节"的判据核心。
    """
    h, w = bright.shape
    seen = np.zeros_like(bright)
    stack: list[tuple[int, int]] = []

    def push(y: int, x: int) -> None:
        if bright[y, x] and not seen[y, x]:
            seen[y, x] = True
            stack.append((y, x))

    for x in range(w):
        push(0, x)
        push(h - 1, x)
    for y in range(h):
        push(y, 0)
        push(y, w - 1)

    while stack:
        y, x = stack.pop()
        if y > 0:
            push(y - 1, x)
        if y + 1 < h:
            push(y + 1, x)
        if x > 0:
            push(y, x - 1)
        if x + 1 < w:
            push(y, x + 1)
    return seen


def _label_mask(mask: np.ndarray) -> tuple[np.ndarray, int]:
    """连通域标注（四邻域）。返回 (labels, 区域数)。本机没有 scipy/cv2 —— 手写。"""
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    cur = 0
    for sy, sx in zip(*np.nonzero(mask)):
        if lab[sy, sx]:
            continue
        cur += 1
        y0, x0 = int(sy), int(sx)
        lab[y0, x0] = cur
        stack = [(y0, x0)]
        while stack:
            y, x = stack.pop()
            for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not lab[ny, nx]:
                    lab[ny, nx] = cur
                    stack.append((ny, nx))
    return lab, cur


def fill_face_background(tile: Image.Image, *, thresh: float = DEFAULT_FACE_THRESH,
                         keep_max_frac: float = DEFAULT_KEEP_MAX_FRAC,
                         grow: int = 1) -> tuple[Image.Image, dict]:
    """把纹理里的**非板面区域**填成板面材质。返回 (填充后纹理, 统计信息)。

    判据（连通性）：
      - 亮区**接触纹理边界** ⇒ 板外（背景 / 相邻件 / 线缆）⇒ 填；
      - 亮区**被板面完全包围**且面积占比 < `keep_max_frac` ⇒ 螺栓等表面细节 ⇒ 保留；
      - 被包围但面积占比 ≥ `keep_max_frac` ⇒ 封闭镂空 ⇒ 填。

    填充色 = **板面像素的中位色**（实测 RGB(1,1,20)：近黑带蓝，**不是纯黑**；
    填 (0,0,0) 会在板面上留下肉眼可见的色差补丁）。

    `grow` 把填充掩膜外扩若干像素，吃掉透视校正插值留下的半亮过渡带。
    """
    rgb = np.asarray(tile.convert("RGB"), dtype=np.float64)
    gray = rgb.mean(axis=2)
    total = gray.size

    face = gray < thresh
    bright = ~face
    face_frac = float(face.mean())
    if face_frac < 0.20:
        raise ValueError(
            f"板面像素只占 {face_frac*100:.1f}% —— 阈值 {thresh:.0f} 不合适，"
            "或四角没框住板（先用 preview 复核图确认，不要硬调阈值凑）")

    # 填充色：只在板面像素上取中位，避免被亮区带偏
    fill_color = np.array([float(np.median(rgb[face, c])) for c in range(3)])

    # ★★ 最干净的"四角贴合吗"判据：**纹理四条边上的亮像素比例**。
    #    框精确贴合板 ⇒ 四条边全落在板上 ⇒ 应为 0；哪条边亮得多，就是哪条边没贴。
    #    它比"框外面积"更直接，且**可归因**（上/下/左/右各给一个数）。
    #    注意：板上镂空若开口到板端，也会顶到纹理的上下边 ⇒ 上下边亮不一定是框没贴，
    #    要配合下面的 border/enclosed 拆解一起读。
    band = 3
    edge_bright = (float(bright[0:band, :].mean()), float(bright[-band:, :].mean()),
                   float(bright[:, 0:band].mean()), float(bright[:, -band:].mean()))

    # ★ 另一组诊断：把"接触边界的亮区"与"被板面包围的亮区"分开 ——
    #   border_only   = 接触纹理边界 ⇒ 框比板大（框进了板外的东西）或镂空开口到板端
    #   enclosed_fill = 被板面完全包围 ⇒ 板上的封闭镂空
    border_only = _flood_bright_from_border(bright)
    to_fill = border_only.copy()

    # ★ 暗部的线缆：亮度**低于**阈值（所以被判成"板面"），但它是**暖色**。
    #   板面是近黑带蓝（实测 R−B 中位 −19、p90 −8、p99 +33），橙色线缆 R−B > 15
    #   ⇒ 色差是把暗线缆与板面分开的可靠判据。实测大臂纹理左下残留 2.08% 的暗红线缆，
    #   **只靠亮度永远去不掉**（它在亮度谷里）。
    #   带占比守卫：板面若整体偏暖（暖色 > 20%）说明判据前提不成立，此时不启用。
    warm = (rgb[:, :, 0] - rgb[:, :, 2]) > 15.0
    warm_frac = float(warm.mean())
    if warm_frac <= 0.20:
        to_fill |= warm
    else:
        warm_frac = -1.0            # 负值 = 判据被守卫弃用（打印时可分辨）
        warm = np.zeros_like(warm)

    enclosed = bright & ~border_only
    labels, n_lab = _label_mask(enclosed)
    enclosed_fill = np.zeros_like(to_fill)
    kept: list[dict] = []
    for lid in range(1, n_lab + 1):
        sel = labels == lid
        area = int(sel.sum())
        ys, xs = np.nonzero(sel)
        bw = int(xs.max() - xs.min() + 1)
        bh = int(ys.max() - ys.min() + 1)
        fill_ratio = area / float(bw * bh)
        aspect = bw / float(bh)
        info = {"area": area, "frac": area / total,
                "center": (float(xs.mean()), float(ys.mean())),
                "fill_ratio": fill_ratio, "aspect": aspect}
        # 螺栓 = **小** 且 **接近圆/方**。两个条件缺一不可：
        #   只判面积 ⇒ 线缆小段也被当螺栓留下（实测）；只判形状 ⇒ 大片圆背景被留下。
        is_bolt = (area / total < keep_max_frac
                   and fill_ratio >= DEFAULT_BLOB_FILL_MIN
                   and 1.0 / DEFAULT_BLOB_ASPECT <= aspect <= DEFAULT_BLOB_ASPECT)
        if is_bolt:
            kept.append(info)                                # 螺栓等表面细节 ⇒ 保留
        else:
            enclosed_fill |= sel                             # 镂空 / 线缆段 ⇒ 填
            to_fill |= sel
            info["filled_as_hole"] = True
            kept.append(info)

    border_frac = float(border_only.mean())
    enclosed_frac = float(enclosed_fill.mean())
    to_fill = _grow_mask(to_fill, grow)
    out = rgb.copy()
    out[to_fill] = fill_color
    out = Image.fromarray(np.clip(out, 0, 255).astype(np.uint8))

    stats = {
        "face_frac": face_frac,
        "filled_frac": float(to_fill.mean()),
        "border_frac": border_frac,        # 接触边界 ⇒ 框外区域（或开口到板端的镂空）
        "enclosed_frac": enclosed_frac,    # 被包围   ⇒ 封闭镂空
        "edge_bright": edge_bright,        # 四条边的亮比例 ⇒ 四角贴合度的直接判据
        "warm_frac": warm_frac,            # 暖色（暗线缆）像素占比；<0 = 判据被守卫弃用
        "fill_color": tuple(int(round(v)) for v in fill_color),
        "kept_blobs": [k for k in kept if not k.get("filled_as_hole")],
        "enclosed_blobs": len(kept),
        "thresh": thresh,
        "mask": to_fill,          # 仅供复核图可视化，不参与任何判据
    }
    return out, stats


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def build_tile(img: Image.Image, corners: np.ndarray, face_mm: tuple[float, float],
               px_per_mm: float) -> Image.Image:
    """四边形 → 按真实尺寸归一化的纹理（**长边统一放在竖直方向**）。"""
    short_mm, long_mm = face_mm
    quad = corners

    # 先按"照片里的朝向"校正，再看长边落在哪个方向
    w_photo = float(np.linalg.norm(quad[1] - quad[0]))
    h_photo = float(np.linalg.norm(quad[3] - quad[0]))
    long_is_vertical = h_photo >= w_photo
    if long_is_vertical:
        out_w = max(1, round(short_mm * px_per_mm))
        out_h = max(1, round(long_mm * px_per_mm))
    else:
        out_w = max(1, round(long_mm * px_per_mm))
        out_h = max(1, round(short_mm * px_per_mm))

    tile = rectify(img, quad, out_w, out_h)
    if not long_is_vertical:
        tile = tile.rotate(-90, expand=True)   # 统一成长边竖直
    return tile


def _fit_height(im: Image.Image, h: int) -> Image.Image:
    rs = h / max(1, im.height)
    return im.resize((max(1, int(im.width * rs)), h), Image.LANCZOS)


def write_preview(name: str, img: Image.Image, corners: np.ndarray,
                  tile: Image.Image, *, tile_raw: Image.Image | None = None,
                  fill_mask: np.ndarray | None = None) -> Path:
    """复核图：原图叠四角 | 校正后 | （填过孔则）填充后 + 黄色标出被填区域。

    **先看图再信结果。** 最右一栏是判断"该填的填了、不该填的（螺栓）没被填掉"
    的唯一依据 —— 与"量包围盒必须目视复核"是同一类教训（D39/D47）。
    """
    PREVIEW_DIR.mkdir(parents=True, exist_ok=True)
    base = img.convert("RGB").copy()
    draw = ImageDraw.Draw(base)
    poly = [tuple(map(float, p)) for p in corners]
    draw.polygon(poly, outline=(255, 0, 0), width=4)
    for (x, y), lab in zip(corners, ["TL", "TR", "BR", "BL"]):
        draw.ellipse([x - 6, y - 6, x + 6, y + 6], fill=(255, 220, 0))
        draw.text((x + 8, y - 20), lab, fill=(255, 60, 60))

    panels: list[tuple[Image.Image, str]] = [
        (_fit_height(base, 360), "原图 + 估计四角（红框）")]
    if tile_raw is not None:
        panels.append((_fit_height(tile_raw.convert("RGB"), 360), "校正后（未填孔）"))

    shown = tile.convert("RGB")
    if fill_mask is not None and tile_raw is not None:
        arr = np.asarray(shown).astype(np.float64).copy()
        arr[fill_mask] = arr[fill_mask] * 0.45 + np.array([255, 205, 0]) * 0.55
        shown = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    panels.append((_fit_height(shown, 360),
                   "填充后（黄 = 填入的板面色）" if fill_mask is not None
                   else "校正后纹理（应为正矩形）"))

    gap = 16
    width = sum(p.width for p, _ in panels) + gap * (len(panels) - 1)
    canvas = Image.new("RGB", (width, 384), (250, 250, 250))
    d2 = ImageDraw.Draw(canvas)
    x = 0
    for im, lab in panels:
        canvas.paste(im, (x, 0))
        d2.text((x + 6, 364), lab, fill=(0, 0, 0))
        x += im.width + gap

    path = PREVIEW_DIR / f"{name}_corners.png"
    canvas.save(path)
    return path


def parse_corners(spec: str) -> np.ndarray:
    """解析 `--corners "x,y;x,y;x,y;x,y"`（顺序 TL,TR,BR,BL）。"""
    pts = []
    for chunk in spec.replace(" ", "").split(";"):
        if not chunk:
            continue
        x, y = chunk.split(",")
        pts.append((float(x), float(y)))
    if len(pts) != 4:
        raise SystemExit(f"--corners 需要 4 个点，得到 {len(pts)} 个")
    return order_corners(np.array(pts, dtype=np.float64))


def process_one(name: str, face_mm: tuple[float, float], *, thresh: float,
                px_per_mm: float, corners: np.ndarray | None,
                denoise_radius: int, fill_holes: bool = True,
                face_thresh: float = DEFAULT_FACE_THRESH,
                keep_max_frac: float = DEFAULT_KEEP_MAX_FRAC) -> dict:
    matches = sorted(RAW_DIR.glob(f"{name}*.jpg")) + sorted(RAW_DIR.glob(f"{name}*.png"))
    if not matches:
        raise FileNotFoundError(f"raw/ 下找不到 {name}*.jpg —— 见 docs/texture-capture-guide.md")
    src = matches[-1]                      # 同名的多张，取最后一张

    img = Image.open(src)
    gray = np.asarray(img.convert("L"), dtype=np.float64)

    if corners is None:
        mask = denoise(segment(gray, thresh), denoise_radius)
        corners = quad_from_mask(mask)
        how = f"自动估计（阈值 {thresh:.0f}，去噪半径 {denoise_radius}）"
    else:
        how = "手动指定"

    tile_raw = build_tile(img, corners, face_mm, px_per_mm)
    tile, fill_stats = tile_raw, None
    if fill_holes:
        tile, fill_stats = fill_face_background(
            tile_raw, thresh=face_thresh, keep_max_frac=keep_max_frac)

    TILE_DIR.mkdir(parents=True, exist_ok=True)
    tile_path = TILE_DIR / f"{name}.png"
    tile.save(tile_path)
    preview = write_preview(name, img, corners, tile,
                            tile_raw=tile_raw if fill_holes else None,
                            fill_mask=(fill_stats or {}).get("mask"))

    return {"name": name, "source": src.name, "corners_how": how,
            "tile": tile_path, "preview": preview, "fill": fill_stats,
            "size": tile.size, "short_mm": face_mm[0], "long_mm": face_mm[1]}


# ---------------------------------------------------------------------------
# 合成自测：证明「透视校正 + 归一化」几何正确
# ---------------------------------------------------------------------------
def selftest() -> int:
    """合成一张已知几何的"模拟照片"，校正后断言已知标记回到预期位置。

    判据不是"跑通了"，而是**几何正确**：在理想纹理的长边 25% / 50% / 75% 处画横线，
    经"投影成梯形 → 校正回矩形"之后，这些线必须回到原来的相对位置。
    """
    print("==== 合成自测：透视校正 + 归一化 ====")
    ppm = 10.0
    short_mm, long_mm = 22.0, 74.0
    tw, th = int(short_mm * ppm), int(long_mm * ppm)      # 220 × 740

    # 1) 理想纹理：整块都是"板"（**不留白边**）
    #    ⚠️ 曾经这里在四边留了 6px 白边来"模拟板外背景"，结果自测失败 ——
    #    投影的是整块纹理、而对齐用的是 quad，于是分割只拿到**内缩一圈**的深色区域，
    #    校正时把它放大回原尺寸 ⇒ 刻度线位置系统性偏大。
    #    真实场景里"板区域"就是纹理本身（分割出的轮廓即板的轮廓），所以这里必须铺满。
    tile_ref = np.full((th, tw), 60.0)
    marks = {}
    for frac in (0.25, 0.50, 0.75):
        y = int(round(frac * (th - 1)))
        marks[frac] = y
        tile_ref[y - 3:y + 4, int(tw * 0.15):int(tw * 0.85)] = 10.0
    tile_img = Image.fromarray(tile_ref.astype(np.uint8))
    print(f"  理想纹理 {tw}×{th}px（{short_mm}×{long_mm}mm @ {ppm}px/mm）")
    print(f"  刻度线 y = {list(marks.values())}（对应 25% / 50% / 75%）")

    # 2) 制造透视：把矩形投影成一个**明显梯形**（远边短）
    canvas_w, canvas_h = 900, 760
    quad = np.array([[300, 90], [660, 150], [640, 700], [310, 660]], dtype=np.float64)
    photo = project(tile_img, (canvas_w, canvas_h), quad)
    photo_path = PREVIEW_DIR / "selftest_photo.png"
    PREVIEW_DIR.mkdir(parents=True, exist_ok=True)
    photo.save(photo_path)
    print(f"  模拟照片 {canvas_w}×{canvas_h}（四边形 {quad.astype(int).tolist()}）-> {photo_path.name}")

    # 3) 跑校正管线（与真实照片走完全相同的路径）
    gray = np.asarray(photo, dtype=np.float64)
    mask = denoise(segment(gray, DEFAULT_THRESH), 2)
    est = quad_from_mask(mask)
    err_corner = float(np.abs(est - quad).max())
    print(f"  估计四角 vs 真值四角：max|Δ| = {err_corner:.2f} px")

    restored = build_tile(photo, est, (short_mm, long_mm), ppm)
    rest = np.asarray(restored.convert("L"), dtype=np.float64)
    print(f"  校正结果 {restored.size[0]}×{restored.size[1]}（期望 {tw}×{th}）")

    ok = True
    if restored.size != (tw, th):
        print(f"  ✗ 尺寸不符")
        ok = False

    # 4) 断言刻度线回到原位置
    print("  刻度线位置核对（校正后应有暗行）：")
    for frac, y_expect in marks.items():
        window = rest[max(0, y_expect - 25):y_expect + 26, int(tw * 0.3):int(tw * 0.7)]
        if window.size == 0:
            print(f"    {int(frac*100)}%  y={y_expect:3d}  ✗ 窗口越界")
            ok = False
            continue
        row_mean = window.mean(axis=1)
        y_found = int(np.argmin(row_mean)) + max(0, y_expect - 25)
        delta = abs(y_found - y_expect)
        flag = "✓" if delta <= 6 else "✗"
        if delta > 6:
            ok = False
        print(f"    {int(frac*100)}%  y 期望 {y_expect:3d} · 实测 {y_found:3d} · Δ={delta} px  {flag}")

    # 5) 校正后板区域应仍是深色（没有被白背景吃掉）
    inner = rest[th // 4:th * 3 // 4, tw // 4:tw * 3 // 4]
    print(f"  校正后中心区灰度均值 = {inner.mean():.1f}（应接近 60，即板色）")
    if inner.mean() > 120:
        print("  ✗ 校正结果大部分是白背景 —— 四角或重采样顺序有问题")
        ok = False

    # 6) 填孔自测：判据必须是**连通性**，不是纯亮度。
    #    纯亮度会把板面上的螺栓（也是亮的）一起填掉 —— 这里用一个"被板面完全
    #    包围的小亮斑"代表螺栓，断言它必须**活下来**。
    print()
    print("  -- 填孔：连通性判据（不是纯亮度） --")
    synth = np.zeros((th, tw, 3), dtype=np.float64)
    synth[:, :, 0] = 12.0
    synth[:, :, 1] = 12.0
    synth[:, :, 2] = 22.0                      # 近黑带蓝，模拟实测板面色 RGB(1,1,20)
    synth[:, 0:18] = 130.0                     # ① 接触边界 ⇒ 板外 ⇒ 应填
    synth[100:220, 60:170] = 130.0             # ② 被包围但面积大（8%）⇒ 镂空 ⇒ 应填
    yy, xx = np.mgrid[0:th, 0:tw]
    synth[(yy - 400) ** 2 + (xx - 110) ** 2 <= 11 ** 2] = 150.0   # ③ 螺栓 ⇒ 应保留
    filled, fstats = fill_face_background(Image.fromarray(synth.astype(np.uint8)))
    fa = np.asarray(filled, dtype=np.float64)
    face_color = np.array([12.0, 12.0, 22.0])
    for lab, good in (
            ("① 接触边界的亮带 ⇒ 填掉", float(np.abs(fa[400, 8] - face_color).max()) <= 2),
            ("② 被包围的大亮区 ⇒ 填掉", float(np.abs(fa[160, 110] - face_color).max()) <= 2),
            ("③ 被包围的小亮斑（螺栓）⇒ 保留", float(np.abs(fa[400, 110] - 150.0).max()) <= 2)):
        print(f"    {lab}  {'✓' if good else '✗'}")
        if not good:
            ok = False
    print(f"    统计：板面 {fstats['face_frac']*100:.1f}% · 填入 {fstats['filled_frac']*100:.1f}% · "
          f"填充色 RGB{fstats['fill_color']} · 保留亮斑 {len(fstats['kept_blobs'])} 个")

    print()
    print("=" * 60)
    print("自测 " + ("通过 ✓ —— 校正管线几何正确" if ok else "**失败 ✗**"))
    return 0 if ok else 1


def main(argv=None) -> int:
    ensure_utf8_stdout()
    ap = argparse.ArgumentParser(
        description="实拍照片 → 板件纹理（透视校正 + 归一化）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='--corners 顺序：左上;右上;右下;左下，如 "100,80;600,120;580,700;120,660"')
    ap.add_argument("name", nargs="?", help="板件名（见 --list）")
    ap.add_argument("--all", action="store_true", help="处理 raw/ 下所有照片")
    ap.add_argument("--list", action="store_true", help="列出可处理的板件与尺寸")
    ap.add_argument("--corners", help="手动指定四角，跳过自动估计")
    ap.add_argument("--thresh", type=float, default=DEFAULT_THRESH,
                    help=f"白底暗件分割阈值（默认 {DEFAULT_THRESH}）")
    ap.add_argument("--px-per-mm", type=float, default=DEFAULT_PX_PER_MM,
                    help=f"纹理分辨率（默认 {DEFAULT_PX_PER_MM}）")
    ap.add_argument("--denoise-radius", type=int, default=2, help="开运算半径（默认 2）")
    ap.add_argument("--no-fill-holes", dest="fill_holes", action="store_false",
                    help="不把非板面亮区填成板面色（默认会填：实板镂空，模型是实心 box）")
    ap.add_argument("--face-thresh", type=float, default=DEFAULT_FACE_THRESH,
                    help=f"板面/非板面分界阈值（默认 {DEFAULT_FACE_THRESH:.0f}）")
    ap.add_argument("--keep-max-frac", type=float, default=DEFAULT_KEEP_MAX_FRAC,
                    help=f"被板面包围的亮区保留为表面细节的面积上限（默认 {DEFAULT_KEEP_MAX_FRAC}）")
    ap.add_argument("--selftest", action="store_true", help="合成数据自测")
    args = ap.parse_args(argv)

    if args.selftest:
        return selftest()

    sizes = load_plate_sizes()
    if args.list:
        print(f"可处理的板件（尺寸来自 {ROBOT_YAML.relative_to(ROOT)}）：")
        print(f"{'纹理名':<34}{'大面 (短×长 mm)':<20}{'raw/ 里有照片？'}")
        for name, face in sizes.items():
            hit = sorted(RAW_DIR.glob(f"{name}*.jpg")) + sorted(RAW_DIR.glob(f"{name}*.png"))
            print(f"{name:<34}{face[0]:.1f} × {face[1]:.1f}".ljust(54)
                  + (f"✓ {len(hit)} 张" if hit else "— 待拍"))
        return 0

    targets = list(sizes) if args.all else ([args.name] if args.name else [])
    if not targets:
        ap.print_help()
        return 1

    corners = parse_corners(args.corners) if args.corners else None
    results, failed = [], []
    for name in targets:
        if name not in sizes:
            failed.append((name, f"未知板件名（--list 查看可选）"))
            continue
        try:
            results.append(process_one(name, sizes[name], thresh=args.thresh,
                                       px_per_mm=args.px_per_mm, corners=corners,
                                       denoise_radius=args.denoise_radius,
                                       fill_holes=args.fill_holes,
                                       face_thresh=args.face_thresh,
                                       keep_max_frac=args.keep_max_frac))
        except Exception as exc:                       # noqa: BLE001
            failed.append((name, f"{type(exc).__name__}: {exc}"))

    print("==== 纹理生成 ====")
    for r in results:
        print(f"  ✓ {r['name']:<30} {r['short_mm']:.0f}×{r['long_mm']:.0f}mm -> "
              f"{r['size'][0]}×{r['size'][1]}px  ({r['corners_how']}）")
        print(f"      贴图 {r['tile'].relative_to(ROOT)}")
        f = r.get("fill")
        if f:
            kept = f["kept_blobs"]
            extra = f"，最大 {max(k['area'] for k in kept)}px" if kept else ""
            print(f"      填孔 板面 {f['face_frac']*100:4.1f}% · 填入 {f['filled_frac']*100:4.1f}%"
                  f"（框外 {f['border_frac']*100:4.1f}% + 镂空 {f['enclosed_frac']*100:4.1f}%）"
                  f" · 填充色 RGB{f['fill_color']} · 保留 {len(kept)} 个亮斑{extra}")
            t, b, l, r = f["edge_bright"]
            print(f"      四边亮比例 上 {t*100:4.1f}%  下 {b*100:4.1f}%  左 {l*100:4.1f}%  右 {r*100:4.1f}%"
                  "   ← 框贴合板时应都接近 0")
            if min(f["edge_bright"]) > 0.30:
                print("      ⚠️ 四条边亮比例都高 —— 框很可能比板大（去复查四角，别硬调阈值凑）")
        print(f"      复核图 {r['preview'].relative_to(ROOT)}   ← **先看这张图**")
    for name, why in failed:
        print(f"  ✗ {name}: {why}")

    if results:
        print()
        print("⚠️ 四角是自动估计的 —— 请逐个打开复核图确认红框贴合板的四条边。")
        print("   不贴合时用 --corners 手动指定，不要硬调阈值凑。")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
