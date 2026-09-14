"""采集判定（`tools/capture_texture.py`）的回归测试。

## 为什么必须有

`capture_texture.py` 的产物是**给操作者的行动指令**（"靠近一点" / "绕竖直轴转" /
"长边横过来"）。指令错了**不会报错** —— 人会照着一条错的指令去调设备，调很久也调不好。
所以这里的判据不是"函数能跑"，而是：

  1. **归因正确**：整条臂连成一个连通域时，必须说"判不了"，而不是说"板歪了"。
     （D40/D44 的纪律：没有正面证据不下断言。）
     —— 这条对应的 bug 真实发生过：修好之前输出的是
     "左边 885 / 右边 313 px，差 182%"，看着像严重倾斜，实际是拿**整条臂**在算。
  2. **方向正确**：倾斜建议必须指向**更远**的那条边（对边更短 ⇒ 更远）。
     方向说反的后果是人往反方向转，越调越歪且无任何报错。
  3. **量正确**：分辨率不足时的"×1.78 横向增益"来自**实测的 1280×720 上限**，
     不是拍脑袋常数。换相机会让这条测试失败，提醒改文案。
  4. **不误阻塞**：真机臂板是**镂空桁架**（外接盒 22×74，中间是空的），
     轮廓填充率天然偏低 —— 这是**提示**不是**阻塞**，否则每张真机照片都会被判死，
     工具变成"永远说不"的噪音源，人就开始忽略它。

## 两条测试线，刻意分开

- **纯逻辑线**：直接构造指标字典调 `evaluate()`。倾斜、分辨率、方向这些判据都是
  对指标的**算术**，用构造字典测才精确。合成多边形会引入 `quad_from_mask` 的最小外接
  四边形拟合误差 —— 那样测出来的是拟合，不是判定逻辑。
  （实测教训：想合成一个"只有左右倾斜、没有上下倾斜"的多边形是**做不到的** ——
   等腰梯形左右边必然等长，左右不等则上下必不等。硬凑只会得到脆弱且失真的样本。）
- **图像线**：`analyze()` 只吃一张 PIL 图，合成一张跑通 `analyze → evaluate`，
  守住两者的**数据契约**（少一个键就是运行时 KeyError，只在真机采集时才暴露）。
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest
from PIL import Image, ImageDraw

from robopkg import package_dir_of

ROOT = Path(__file__).resolve().parents[2]
#: 被测脚本住在**包内**（Phase 2：MeArm 的相机/纹理工具随包走）——
#: 路径由包的**位置**推导，不写死字符串（包一改位置，这里跟着走）。
MEARM_TOOLS = package_dir_of("mearm-v1") / "tools"
SCRIPT = MEARM_TOOLS / "capture_texture.py"


def _load():
    # capture_texture 在 import 时会把 tools/ 插进 sys.path 并 `import make_texture`，
    # 所以要先注册进 sys.modules，避免 spec 装载时报"部分初始化"。
    spec = importlib.util.spec_from_file_location("capture_texture", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["capture_texture"] = mod
    spec.loader.exec_module(mod)
    return mod


ct = _load()

FACE = (22.0, 74.0)          # upper_arm_link 大面 mm
PLATE_FG, PLATE_BG = 30, 240  # 黑亚克力板 / 白底

# 合成帧的判定缩放。用 2 而不是默认 6：本测试验的是判定逻辑，
# scale=6 时四角精度 ±6px 对 180px 短边就是 3.3%，会顶到 4% 的平行容差，
# 把逻辑测试变成噪声测试。
TEST_SCALE = 2


def _metrics(**over) -> dict:
    """一份"合格帧"的指标基线 —— 各测试只改动它关心的那几个字段。

    基线自检：600px/74mm = 8.1 px/mm ≥ 6；长宽比 3.33 vs 3.36；
    四角离边框都远；轮廓填充 100% ⇒ `evaluate` 必须给 ok。
    """
    base = {
        "ok_mask": True,
        "frame_wh": (1280, 720),
        "corners": np.array([[300.0, 250.0], [900.0, 250.0],
                             [900.0, 430.0], [300.0, 430.0]]),
        "bbox": (300, 250, 900, 430),
        "mask_pixels": 27000,
        "edges": (600.0, 180.0, 600.0, 180.0),
        "long_px": 600.0,
        "short_px": 180.0,
        "long_is_horizontal": True,
        "rect_fill": 1.0,
        "plate_mean": 30.0,
        "bg_mean": 240.0,
        "sharpness": 120.0,
        "thresh": 90.0,
    }
    base.update(over)
    return base


def _ev(m: dict, **kw):
    kw.setdefault("face_mm", FACE)
    kw.setdefault("target_px_per_mm", 6.0)
    kw.setdefault("ramp_max_px", 2.5)
    return ct.evaluate(m, **kw)


# ---------------------------------------------------------------------------
# 0. 基线：一份合格帧必须通过（否则下面所有"拒绝"类测试都可能是假阳性）
# ---------------------------------------------------------------------------
def test_baseline_metrics_pass():
    status, blockers, notes = _ev(_metrics())
    assert status == "ok", f"基线帧被拒：{blockers}\n{notes}"


# ---------------------------------------------------------------------------
# 1. 归因正确：判不了就说判不了，别赖倾斜
# ---------------------------------------------------------------------------
def test_undecidable_when_blob_touches_border_and_never_blames_tilt():
    """整条臂铺满画面并贴边 ⇒ undecidable，且**不得**出现倾斜归因。

    防倒退：修复前这里输出的是"左右有透视：左边 885 / 右边 313 px —— 右边离镜头更远"，
    人会照着去转臂、转不出结果 —— 因为那个四边形框的是整条臂，不是任何一块板。
    """
    m = _metrics(bbox=(0, 0, 744, 690),
                 corners=np.array([[0.0, 0.0], [744.0, 408.0],
                                   [576.0, 672.0], [6.0, 672.0]]),
                 edges=(849.0, 313.0, 576.0, 672.0),
                 long_px=849.0, short_px=313.0)
    status, blockers, _ = _ev(m)
    joined = "\n".join(blockers)

    assert status == "undecidable"
    assert "不可判定" in joined
    assert "离镜头更远" not in joined, \
        "把『框错了对象』说成了『板倾斜』 —— 这正是要防的归因错误"
    assert "白色卡片" in joined or "推到只剩目标板" in joined, \
        "说了判不了，却没给出可执行的办法"
    assert "不对称" not in joined


def test_undecidable_when_aspect_ratio_is_far_off():
    """轮廓长宽比与目标板差太远 ⇒ 框到的不是目标板。"""
    status, blockers, _ = _ev(_metrics(edges=(600.0, 180.0, 600.0, 180.0),
                                       long_px=600.0, short_px=460.0))
    assert status == "undecidable"
    assert any("长宽比" in line for line in blockers), blockers


# ---------------------------------------------------------------------------
# 2. 方向正确：倾斜建议指向更远的那条边
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("edges, expect_far", [
    ((540.0, 180.0, 600.0, 180.0), "上边"),   # 上边短 ⇒ 上边远
    ((600.0, 180.0, 540.0, 180.0), "下边"),   # 下边短 ⇒ 下边远
])
def test_vertical_tilt_points_at_the_shorter_farther_edge(edges, expect_far):
    _, blockers, _ = _ev(_metrics(edges=edges))
    joined = "\n".join(blockers)
    assert "上下有透视" in joined, joined
    assert f"{expect_far}离镜头更远" in joined, f"倾斜方向说反了：\n{joined}"


@pytest.mark.parametrize("edges, expect_far", [
    ((600.0, 216.0, 600.0, 184.0), "左边"),   # 左边短 ⇒ 左边远
    ((600.0, 184.0, 600.0, 216.0), "右边"),   # 右边短 ⇒ 右边远
])
def test_horizontal_tilt_points_at_the_shorter_farther_edge(edges, expect_far):
    _, blockers, _ = _ev(_metrics(edges=edges, short_px=min(edges[1], edges[3])))
    joined = "\n".join(blockers)
    assert "左右有透视" in joined, joined
    assert f"{expect_far}离镜头更远" in joined, f"倾斜方向说反了：\n{joined}"


# ---------------------------------------------------------------------------
# 3. 量正确：横向增益来自实测画面尺寸
# ---------------------------------------------------------------------------
def test_low_resolution_advises_landscape_rotation_with_measured_gain():
    """长边竖放 + 画面横向 ⇒ 建议转横向，并给出由画面尺寸推出的增益。

    这条把"相机上限 1280×720"这个**硬件事实**钉进测试：换相机后增益会变，
    测试会失败并提醒改文案，而不是让文档留着旧数字。
    """
    size = (1280, 720)
    # 竖放的一块板：画面里 90 × 300 px ⇒ 300/74 = 4.05 px/mm < 6
    m = _metrics(frame_wh=size, long_is_horizontal=False,
                 long_px=300.0, short_px=90.0,
                 corners=np.array([[600.0, 200.0], [690.0, 200.0],
                                   [690.0, 500.0], [600.0, 500.0]]),
                 bbox=(600, 200, 690, 500))
    status, blockers, _ = _ev(m)
    joined = "\n".join(blockers)

    assert status == "reject"
    assert "分辨率不足" in joined
    assert "横向" in joined, joined
    assert f"×{size[0] / size[1]:.2f}" in joined, joined


def test_low_resolution_with_landscape_plate_only_advises_moving_closer():
    """长边已横放时不能再靠转向取巧，只能靠近。"""
    m = _metrics(long_px=300.0, short_px=90.0,
                 corners=np.array([[500.0, 300.0], [800.0, 300.0],
                                   [800.0, 390.0], [500.0, 390.0]]),
                 bbox=(500, 300, 800, 390))
    status, blockers, _ = _ev(m)
    joined = "\n".join(blockers)
    assert status == "reject"
    assert "靠近" in joined, joined
    assert "横向" not in joined, "长边已经是横的，不该再叫它转横向"


def test_dark_scene_is_not_mistaken_for_blur():
    """场景暗 ⇒ 斜率的绝对值低，但**边缘照样是硬的**，不能判成糊。

    这是把"用梯度绝对值当阈值"这个错误钉住。数字全部是**真机实测值**：
    真机那一帧背景墙只比黑板亮 119 级、斜率 53；换成亮场景同一条边会给出 100+。
    用绝对值当判据，等于把"灯够不够亮"当成"焦准不准"。
    """
    m = _metrics(plate_mean=24.0, bg_mean=143.0, sharpness=53.0)   # 真机实测
    status, blockers, _ = _ev(m)
    assert status == "ok", f"暗场景被误判成糊：{blockers}"


def test_soft_edge_is_rejected_as_blurry():
    """同一条边糊到约 3 px 过渡 ⇒ 必须拒（真机模糊 r=2 的实测值）。"""
    m = _metrics(plate_mean=24.0, bg_mean=143.0, sharpness=119.0 / (2 * 3.1))
    status, blockers, _ = _ev(m)
    assert status == "reject"
    assert any("偏糊" in b for b in blockers), blockers


# ---------------------------------------------------------------------------
# 4. 不误阻塞：镂空是提示，不是阻塞
# ---------------------------------------------------------------------------
def test_truss_openings_are_a_note_not_a_blocker():
    """真机臂板镂空 ⇒ 填充率天然低。这只能是**提示**。

    升级成阻塞项的话，**每一张真机照片都会被判不合格** ——
    工具变成"永远说不"的噪音源，人就开始忽略它。那等于没有工具。
    """
    status, blockers, notes = _ev(_metrics(rect_fill=0.40))
    assert status == "ok", f"镂空被误判成阻塞项 —— 真机每张都会被判死：{blockers}"
    assert any("镂空" in n or "填充率" in n for n in notes), notes


def test_small_segmentation_margin_is_a_note():
    """灰度贴近分割阈值 ⇒ 提示（阈值稍一波动就会错分）。

    判据是"离阈值的余量"，不是"板和背景差多少"：
    背景必须亮过阈值才分得开，所以"板与背景之差"永远大于 (thresh - plate)，
    用那个当判据是**恒不成立**的死代码 —— 而它在报告里显示为"已检查"。
    """
    _, _, notes = _ev(_metrics(bg_mean=105.0))     # 距阈值 90 只有 15
    assert any("分割余量" in n for n in notes), notes


# ---------------------------------------------------------------------------
# 5. 图像线：analyze → evaluate 的数据契约
# ---------------------------------------------------------------------------
def _frame(polygons, *, size=(1280, 720), bg=PLATE_BG) -> Image.Image:
    """白底 + 填充多边形（先画 fg，再画 bg 即成为"挖洞"）。"""
    img = Image.new("L", size, bg)
    d = ImageDraw.Draw(img)
    for poly, level in polygons:
        d.polygon([tuple(p) for p in poly], fill=level)
    return img.convert("RGB")


def test_analyze_to_evaluate_contract_on_a_good_frame():
    """`analyze` 返回的键必须够 `evaluate` 用 —— 少了就是运行时 KeyError。"""
    poly = [([[300, 250], [900, 250], [900, 430], [300, 430]], PLATE_FG)]
    m = ct.analyze(_frame(poly), thresh=90, scale=TEST_SCALE)
    assert m["ok_mask"]
    status, blockers, _ = _ev(m)
    assert status == "ok", f"好帧被拒：\n" + "\n".join(blockers)


def test_analyze_measures_truss_fill_ratio_below_one():
    """镂空会真的把填充率压下去 —— 这个指标必须能测出真机的镂空。"""
    # 4 个 110×130 的洞，板 600×180 ⇒ 填充率 (108000−57200)/108000 ≈ 0.47
    holes = [([[x, 275], [x + 110, 275], [x + 110, 405], [x, 405]], PLATE_BG)
             for x in (340, 460, 580, 700)]
    poly = [([[300, 250], [900, 250], [900, 430], [300, 430]], PLATE_FG), *holes]
    m = ct.analyze(_frame(poly), thresh=90, scale=TEST_SCALE)
    assert m["rect_fill"] < 0.55, f"镂空没测出来：{m['rect_fill']:.2f}"


def test_analyze_whole_arm_sample_goes_undecidable():
    """用真机上量到的那个"整条臂"四边形做回归样本（见 ADR D57）。

    它同时覆盖 `analyze` 的连通域与四角估计：若哪天 `largest_component`
    退化成"整张掩膜"，这条会立刻发现。
    """
    arm = [([[0, 0], [744, 408], [576, 672], [6, 672]], PLATE_FG)]
    m = ct.analyze(_frame(arm), thresh=90, scale=TEST_SCALE)
    status, blockers, _ = _ev(m)
    assert status == "undecidable", blockers
    # 贴着左/上边框，所以连通域一定触边
    assert "贴边" in "\n".join(blockers)


def test_boundary_sharpness_is_insensitive_to_a_uniform_plate_interior():
    """板内部是均匀黑色时，轮廓锐度仍要能量出"边缘陡不陡"。

    这是把早先的错误指标钉住：原来在**板内部**算拉普拉斯方差，
    对一块表面干净的黑板，无论对焦与否它都接近 0 —— 量的是"表面有没有花纹"，
    不是"糊不糊"。
    """
    gray = np.full((120, 120), 30.0)
    gray[30:90, 30:90] = 240.0            # 亮底上挖一块暗板，边界是硬跳变
    comp = gray < 90                       # 暗板
    v = ct.boundary_sharpness(gray, comp, scale=1)
    assert v > 50, f"硬边界上的轮廓锐度只有 {v:.1f} —— 指标没量在边界上"


def test_boundary_sharpness_drops_when_the_edge_is_blurred():
    """把边界糊掉，轮廓锐度必须显著下降 —— 否则这个指标对"糊"不敏感。"""
    from PIL import ImageFilter

    sharp = np.full((120, 120), 240.0)
    sharp[30:90, 30:90] = 30.0
    comp = sharp < 90
    blurred = np.asarray(
        Image.fromarray(sharp.astype(np.uint8)).filter(ImageFilter.GaussianBlur(3))
    ).astype(np.float64)

    v_sharp = ct.boundary_sharpness(sharp, comp, scale=1)
    v_blur = ct.boundary_sharpness(blurred, comp, scale=1)
    assert v_blur < v_sharp * 0.6, (
        f"糊了以后锐度只从 {v_sharp:.1f} 掉到 {v_blur:.1f} —— 这个指标判不出糊")
