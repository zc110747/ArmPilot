"""纹理管线（`tools/make_texture.py`）的回归测试。

为什么放在 `tests/sim/`：本目录是 `pytest tests/sim` 的收集根，放这里才能跟其余
测试**同一条命令跑到**。落在别处会被遗忘 —— 那和没写一样。

三条判据各自的意图：
  1. **几何正确**（不是"跑通了"）—— 合成一张已知几何的"模拟照片"，校正后断言
     已知标记回到预期像素位置。这是 `selftest()` 做的事，这里把它固化成回归。
  2. **命名与文档一致** —— 采集指南让操作者按文件名放照片，脚本按文件名找照片。
     两者漂移的后果是"用户照做了、脚本却找不到"，且**不会报错**（只是安静地不处理）。
  3. **不退回 PIL QUAD** —— 实测 `Image.transform(QUAD)` 会引入 +12~17px 的系统性
     平移（ADR D56）。这条测试用源码扫描挡住"顺手改回去"。
"""
from __future__ import annotations

import importlib.util
import inspect
import re
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from robopkg import package_dir_of

ROOT = Path(__file__).resolve().parents[2]
#: 被测脚本住在**包内**（Phase 2：MeArm 的相机/纹理工具随包走）——
#: 路径由包的**位置**推导，不写死字符串（包一改位置，这里跟着走）。
MEARM_TOOLS = package_dir_of("mearm-v1") / "tools"
SCRIPT = MEARM_TOOLS / "make_texture.py"
GUIDE = ROOT / "docs" / "texture-capture-guide.md"


def _load():
    spec = importlib.util.spec_from_file_location("make_texture", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


mt = _load()


# ---------------------------------------------------------------------------
# 1. 几何正确性
# ---------------------------------------------------------------------------
def test_selftest_passes():
    """合成数据自测：透视校正 + 归一化必须几何正确。

    这是**唯一**能证明"照片里的梯形被正确拉回矩形"的判据 ——
    只断言"尺寸对了"或"没抛异常"都挡不住系统性的几何错位。
    """
    assert mt.selftest() == 0, "纹理管线自测失败（详见 stdout 的刻度线核对）"


def test_roundtrip_is_identity_for_rectangular_quad():
    """用**矩形** quad 做 project → rectify 的往返，应当近似恒等。

    矩形情形没有透视，所以任何偏差都只可能来自重采样的坐标约定 ——
    这比梯形情形更容易定位问题。
    """
    tw, th = 120, 400
    rowval = np.arange(th) / (th - 1) * 255.0
    tex = np.repeat(rowval[:, None], tw, axis=1)
    tile = Image.fromarray(tex.astype(np.uint8))

    rect = np.array([[200, 100], [200 + tw - 1, 100],
                     [200 + tw - 1, 100 + th - 1], [200, 100 + th - 1]], dtype=np.float64)
    photo = mt.project(tile, (600, 600), rect)
    restored = mt.rectify(photo, mt.order_corners(rect), tw, th)

    back = np.asarray(restored.convert("L"), dtype=np.float64)
    assert back.shape == (th, tw)
    err = np.abs(back - tex).max()
    assert err < 8, f"矩形往返最大误差 {err:.1f} 灰度级（应接近抗锯齿噪声底）"


def test_rectify_rejects_pil_quad_based_mapping():
    """守卫：`rectify` 不得退回 `Image.transform(QUAD)`。

    PIL 的 QUAD 与本模块的 homography 约定不一致，实测带来 +12~17px 的系统性平移
    （刻度线 185/370/554 → 197/387/567，见 ADR D56）。它会静默地把纹理挪一点点 ——
    不报错、不崩溃，只是"看起来有点怪"，属于最难归因的一类错。
    """
    src = inspect.getsource(mt.rectify)
    assert "Transform.QUAD" not in src and "Image.QUAD" not in src, (
        "rectify 又用回了 PIL 的 QUAD —— 它会引入系统性几何平移（ADR D56）。"
        "请用模块内的 homography + _warp。")
    assert "_warp" in src, "rectify 应经由 _warp 重采样"


# ---------------------------------------------------------------------------
# 2. 尺寸推导（大面 = 去掉最小那一维）
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("size, expect", [
    ([22, 5, 74], (22.0, 74.0)),      # 大臂长板：法向 ±Y
    ([18, 5, 68], (18.0, 68.0)),      # 小臂长板
    ([94, 82, 7], (82.0, 94.0)),      # 底座板：法向 ±Z
    ([66, 60, 7], (60.0, 66.0)),      # 立柱底板
    ([30, 6, 22], (22.0, 30.0)),      # 立柱侧板
    ([22, 28, 5], (22.0, 28.0)),      # 端部横撑
])
def test_big_face_mm(size, expect):
    assert mt.big_face_mm(size) == expect


def test_big_face_matches_mujoco_geom_span(robot):
    """大面尺寸必须与 MuJoCo 里同一块板的 geom 跨度一致。

    两个来源（`robot.yaml` 的 size vs 生成的 MJCF geom）如果对不上，
    说明纹理会贴到一块"和物理几何不同尺寸"的面上。
    """
    from model import MeArmSim
    from robotcfg import load_physics

    sim = MeArmSim(robot=robot, physics=load_physics())
    import mujoco

    checked = 0
    for link in robot.links:
        geo = link.geometry
        if not geo or geo.get("type") != "plate":
            continue
        face = set(mt.big_face_mm(geo["size"]))
        body_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, link.id)
        assert body_id >= 0, f"MJCF 里找不到 body {link.id}"
        spans = set()
        for gi in range(sim.model.ngeom):
            if sim.model.geom_bodyid[gi] != body_id:
                continue
            # ⚠️ MuJoCo 的 box geom_size 是**半边长**（且单位为 m）
            #    ⇒ 全尺寸 mm = size × 2 × 1000
            spans.add(tuple(round(float(v) * 2000.0, 3)
                            for v in sim.model.geom_size[gi][:3]))
        assert spans, f"{link.id} 在 MJCF 里没有任何 geom"
        # 至少有一个 geom 的两个较大维度与纹理大面尺寸吻合
        matched = False
        for s in spans:
            top2 = sorted(s)[1:]
            if all(abs(a - b) < 0.05 for a, b in zip(top2, sorted(face))):
                matched = True
        assert matched, (
            f"{link.id}: 纹理大面 {sorted(face)} 与 MJCF geom 跨度 {sorted(spans)} 对不上")
        checked += 1
    assert checked >= 4, f"只核对了 {checked} 块板 —— 预期至少 4 块"


# ---------------------------------------------------------------------------
# 3. 命名与采集指南一致（防文档漂移）
# ---------------------------------------------------------------------------
def test_guide_filenames_are_known_to_the_script():
    """指南里出现的每个 `<name>.jpg` 都必须是脚本认识的名字。

    漂移的后果：操作者按指南命名照片放进 raw/，而脚本根本不去处理它 ——
    而且**不报错**，只是安静地跳过。所以必须由测试来挡。
    """
    guide = GUIDE.read_text(encoding="utf-8")
    # 允许 `foo_02.jpg` 这类"同一块板多拍了几张"的序号后缀 —— 剥掉后再比对基础名
    mentioned = set(re.findall(r"`([a-z][a-z0-9_]*?)(?:_\d+)?\.jpg`", guide))
    assert mentioned, "采集指南里没解析出任何文件名 —— 文档格式变了？"

    known = set(mt.load_plate_sizes())
    unknown = sorted(mentioned - known)
    assert not unknown, (
        f"采集指南提到脚本不认识的文件名：{unknown}\n"
        f"脚本已知：{sorted(known)}\n"
        "两边必须对齐，否则操作者照做也没有效果。")

    # 反向：脚本能处理的名字，指南里至少要交代过（否则操作者不知道要拍它）
    undocumented = sorted(known - mentioned)
    assert not undocumented, (
        f"脚本能处理但指南里没提到的板件：{undocumented} —— 操作者不会去拍它们")


def test_texture_names_are_unique_with_consistent_size():
    """同名纹理必须尺寸一致（同尺寸的不同实例共用一张纹理）。"""
    sizes = mt.load_plate_sizes()
    assert "upper_arm_link" in sizes and "forearm_link" in sizes
    # 别名的效果：同一块板的两个实例合并成一个纹理名
    assert not any(n.endswith("_detail0") for n in sizes), (
        "detail0 未被别名合并 —— 说明 DETAIL_ALIASES 与 robot.yaml 的 details 顺序脱节了")
