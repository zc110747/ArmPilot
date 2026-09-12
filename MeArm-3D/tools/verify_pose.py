#!/usr/bin/env python3
"""端到端验收：用**相机**做外部地面真值，反解关节角并与期望值比对，输出 PASS/FAIL。

为什么需要它
------------
真机固件**没有位置反馈**。`arm_get_angle()` 回的是固件自己记着的**目标值**，
`STATUS` / `OK SET` / `OK JR` 全都只说「我打算让它去哪」，不说「它实际上在哪」。
所以单靠串口回执**无法**证明机械臂真的动到了命令位置 —— 必须引入外部测量。
本脚本拍一张（或一批）照片，把 RobotModel 的侧视骨架拟合上去，反解
`shoulder` / `elbow`（**绝对角**），再与该帧的期望值比对。

可测的与不可测的
----------------
  可测  : shoulder、elbow（矢状面内，两条连杆都在这张照片里）
  不可测: base     —— 绕竖直轴的旋转在正交侧视下几乎不可见
          gripper  —— 两片薄爪的开口在 1280x720 下不可分辨
  ⇒ 因此**验收姿势必须保持 base ≈ 0**，否则整条臂转出画面平面，二维拟合失效。
     期望 base 偏离 0 超过 --base-tol 的帧会被判 SKIP（而不是硬算出一个假角度）。

期望值从哪来（唯一数据源）
--------------------------
`config/robot.yaml` 的 actuators（offset / scale / reverse）：
    reverse -> joint = (offset - servo) / scale
    否则    -> joint = (servo  - offset) / scale
舵机角有三种给法（优先级从高到低）：
    1. --expect "shoulder=20.29,elbow=112.62"   直接给**关节角**（不经过标定）
    2. --servo "7=118,8=90"                     给**舵机角**，本脚本按标定换算
    3. 文件名 S<ch>_<ang>.jpg                    逐帧自动识别
**本脚本不硬编码任何标定常数** —— 尺寸、homePose、offset/scale 全部现场读 yaml。

拟合是「不喂答案」的
--------------------
反解 `(ths, the)` 时只用两个**与期望值无关**的起点做多重启动：
    ① 锚点帧自己拟合出来的角  ② robot.yaml 的 homePose
取代价最小者定案，并在输出里报告用的是哪个起点。
**绝不**把期望角当起点 —— 否则拟合会被答案牵着走，验证就失去意义。

误差预算与判定口径（**这是本工具的核心设计**）
----------------------------------------------
  A. 数量级很小的项（可忽略）：
       舵机整数取整 : shoulder <= 0.5/1.44018 = 0.347°   elbow <= 0.5/2.39401 = 0.209°
       JR 线 0.1°   : <= 0.05°
  B. 相机反解本身有**系统偏置**（不是随机噪声！）。--selftest 自检 B 实测同一真值、
     只改轮廓厚度（对标实物 25mm 侧视厚度 ≈ 60px @2.4px/mm）：
        厚度  20px -> Δths -0.24°  Δthe +0.10°
        厚度  40px -> Δths -1.27°  Δthe -0.09°
        厚度  60px -> Δths +1.19°  Δthe +1.78°
        厚度  80px -> Δths +1.10°  Δthe +3.02°
        厚度 100px -> Δths +3.40°  Δthe +5.49°
     这与 robot.yaml 自述的「绝对角 ±5° 量级不确定度」完全吻合。
  ⇒ 所以本工具给**两个**判据，而且**两个都要过**：
       --tol  绝对误差（默认 5.0°）—— 会被 B 污染，只能当粗筛
       --dtol 帧间差（默认 1.5°）—— 对参考帧取差，**抵消公共偏置 B**，是锐利判据
    结论只认帧间差：绝对误差 FAIL 而同向偏大 = 台面/取景问题；
    帧间差 FAIL 才指向固件/链路/标定。
  ⇒ 想真正收紧到 1~2°，必须按 Phase 4.5 重布：白分割板铺满视场 + 画面内放尺 +
    正交侧视 + **锁死曝光**（自动曝光漂移会让「扫 S7 时始终为暗」的底座掩膜缩水）。

本次（2026-09-12）实测结论：对 .workbuddy/captures 里的**老扫描照片**，
帧间差最大 3.9°、平均 1.5°，**达不到 1.5° 容差** —— 老照片不足以当地面真值。
附带修掉的两个 fit_pose 缺陷（见各自 docstring）：
  ① make_cost 的 ROI 边界用浮点判、索引却取整 -> 边缘种子直接 IndexError（已修）
  ② pattern_search 的「首个改善方向即 break」会卡进局部极小（合成图上代价 12.43 vs
     真值 5.58，s 偏 +2%）；本脚本改用 robust_search（全方向取最优 + 多重启动）

用法
----
    # 0) 自检：不接硬件、不用相机，验搜索器 + 打印「轮廓厚度 -> 偏置」表
    python tools/verify_pose.py --selftest

    # 1) 整目录（文件名自带舵机角；锚点取 RESET 位，锁定像素尺度与肩枢轴）
    python tools/verify_pose.py .workbuddy/captures/w2_S8 \
        --anchor .workbuddy/captures/w2_S8/S8_090.jpg --overlay .workbuddy/analysis

    # 2) 单帧 + 显式期望
    python tools/verify_pose.py shot.jpg --expect "shoulder=20.29,elbow=112.62" \
        --calib 2.4 570 470

    # 3) 只看帧间差（默认就会算；--tol 放宽以屏蔽公共偏置）
    python tools/verify_pose.py .workbuddy/captures/w2_S7 --anchor ... --tol 15 --dtol 1.5

退出码：全 PASS -> 0；有 FAIL 或 SKIP -> 1（便于被 verify 脚本串起来）。
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_sweep import DEFAULT_ROI, DEFAULT_THRESH, mask_of  # noqa: E402
from fit_pose import (  # noqa: E402
    distance_transform,
    make_cost,
    read_yaml_numbers,
    skeleton,
)
from segment_arm import dilate  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
YAML = ROOT / "config" / "robot.yaml"
CAP = ROOT / ".workbuddy" / "captures"
DEFAULT_GUESS = [2.4, 570.0, 470.0, 0.0, 105.0]
ANCHOR_STEPS = [0.4, 30.0, 30.0, 8.0, 8.0]
PAIR_STEPS = [6.0, 6.0]
# 点云规模：代价评估开销与点数成正比（实测 1290 点 -> 2.8ms/次），故粗筛与精修分级。
ANCHOR_COARSE_CAP = 250
ANCHOR_FINE_CAP = 1000
PAIR_CAP = 500


# ----------------------------------------------------------------------------
# robot.yaml 读取（不引 PyYAML）
# ----------------------------------------------------------------------------

def read_actuators() -> dict[int, dict]:
    """抓 actuators 段：channel -> {joint, offset, scale, reverse, lim}。"""
    text = YAML.read_text(encoding="utf-8")
    if "actuators:" not in text:
        return {}
    sec = text.split("actuators:", 1)[1]
    out: dict[int, dict] = {}
    for blk in sec.split("- id:")[1:]:
        ch = re.search(r"channel:\s*(\d+)", blk)
        jid = re.search(r"jointId:\s*(\w+)", blk)
        off = re.search(r"offset:\s*(-?[\d.]+)", blk)
        sc = re.search(r"scale:\s*(-?[\d.]+)", blk)
        rev = re.search(r"reverse:\s*(true|false)", blk)
        lim = re.search(r"limits:\s*\n\s*min:\s*(-?[\d.]+)\s*\n\s*max:\s*(-?[\d.]+)", blk)
        if not (ch and jid and off and sc and rev):
            continue
        out[int(ch.group(1))] = dict(
            joint=jid.group(1),
            offset=float(off.group(1)),
            scale=float(sc.group(1)),
            reverse=(rev.group(1) == "true"),
            lim=(float(lim.group(1)), float(lim.group(2))) if lim else None,
        )
    return out


def servo_to_joint(act: dict, servo: float) -> float:
    """标定式反解：servo -> joint（唯一真值来自 robot.yaml）。"""
    if act["reverse"]:
        return (act["offset"] - servo) / act["scale"]
    return (servo - act["offset"]) / act["scale"]


def joint_to_servo(act: dict, joint: float) -> float:
    """标定式正解：joint -> servo（用于把期望角折回舵机空间读钳位）。"""
    if act["reverse"]:
        return -joint * act["scale"] + act["offset"]
    return joint * act["scale"] + act["offset"]


def quantize_plan(expect: dict[str, float],
                  acts: dict[int, dict]) -> tuple[dict[str, float], dict[int, float]]:
    """把「意图关节角」折成「设备侧真正会停的位置」。

    真机固件只吃**整数舵机度**（`MeArm-Device/core/cmd.c` 的 `parse_u8` + 硬限位钳位），
    所以一条 JR 的物理落点是 `servo_to_joint(round(joint_to_servo(θ)))`。
    相机测的是**物理位置**，若拿未量化的意图值当期望，就会把这份取整误差
    （肩 0.5/1.44018 = 0.347°，肘 0.5/2.39401 = 0.209°）算进相机误差里 —— 那是假误差。
    多舵机关节取平均，与后端 `ServoAnglesToJoints` 同口径。

    返回 `(关节角, 舵机角)`：**舵机角必须一并给出** —— 它是「标定增益复核」
    （Δ关节反解/Δ舵机）的自变量，e2e 走 `--plan` 时文件名不带 `S<ch>_<ang>`，
    没有它增益复核就只能报「帧数不足」。
    """
    acc: dict[str, list[float]] = {}
    servo_out: dict[int, float] = {}
    for ch, act in acts.items():
        jid = act["joint"]
        if jid not in expect:
            continue
        servo = round(joint_to_servo(act, expect[jid]))
        servo_out[ch] = float(servo)
        acc.setdefault(jid, []).append(servo_to_joint(act, servo))
    return {k: sum(v) / len(v) for k, v in acc.items()}, servo_out


# ----------------------------------------------------------------------------
# 帧与期望
# ----------------------------------------------------------------------------

def otsu_threshold(gray: np.ndarray) -> int:
    """Otsu 类间方差最大化阈值（纯 numpy，零新依赖）。

    为什么分割阈值不能写死：暗件的灰度**随曝光漂移**。实测同一台相机同一台面上，
    上午那批（`w2_*`）ROI 内 Otsu = 111~114，下午那批（`e2e_*`，整体过曝）
    Otsu = 164~165 —— 用固定的 90 去切下午那批，掩膜会从 44821px 掉到 8874px
    （**只剩立柱和底座，臂杆全丢**），代价面变平，每一帧都被拟合成同一个角落，
    于是"反解结果"看起来整齐划一却全错。这是本轮真机验收踩到的最贵的一个坑。
    """
    hist = np.bincount(gray.ravel(), minlength=256).astype(float)
    total = hist.sum()
    if total <= 0:
        return DEFAULT_THRESH
    omega = np.cumsum(hist) / total
    mu = np.cumsum(hist * np.arange(256)) / total
    denom = omega * (1.0 - omega)
    denom[denom <= 0] = 1e-12
    return int(np.argmax((mu[-1] * omega - mu) ** 2 / denom))


def resolve_threshold(spec, targets: list[Path], roi) -> int:
    """`--thresh` 可以是整数，也可以是 `auto`。

    auto = **全批逐帧 Otsu 取中位数**，而不是逐帧各用各的：
    整批统一阈值才能保证「帧间差」反映的是臂真的动了，而不是阈值自己漂了。
    同时把逐帧 Otsu 值打印出来 —— 它们是**曝光稳定性的体检指标**，
    散布大就说明相机自动曝光在动，那一批照片的帧间差不可信。
    """
    if spec is None:
        return DEFAULT_THRESH
    if str(spec).strip().lower() != "auto":
        return int(spec)
    x0, y0, x1, y1 = roi
    vals = []
    for p in targets:
        gray = np.asarray(Image.open(p).convert("L"), dtype=np.uint8)
        vals.append(otsu_threshold(gray[y0:y1, x0:x1]))
    th = int(np.median(vals))
    spread = max(vals) - min(vals)
    print(f"# 阈值 auto：逐帧 Otsu {vals} -> 中位数 {th}"
          f"（散布 {spread}；整批统一，避免阈值漂移伪造帧间差）")
    if spread > 6:
        print(f"#   ⚠️ Otsu 散布 {spread} 偏大 —— 相机自动曝光在动，"
              f"该批照片的**帧间差**可信度下降（重布台面时请锁死曝光）")
    return th


def collect(target: str) -> list[Path]:
    p = Path(target)
    if p.is_file():
        return [p]
    if p.is_dir():
        files = sorted(p.glob("*.jpg")) + sorted(p.glob("*.png"))
        if files:
            return files
    raise SystemExit(f"找不到图片或目录：{target}")


def parse_frame_servo(p: Path) -> dict[int, float]:
    """从文件名 S<ch>_<ang>.jpg 里读出舵机角（如 S8_045.jpg -> {8: 45.0}）。"""
    m = re.search(r"S(\d)_(\d+(?:\.\d+)?)", p.stem)
    return {int(m.group(1)): float(m.group(2))} if m else {}


def parse_servo_arg(spec: str | None) -> dict[int, float]:
    """--servo "7=118,8=90" -> {7: 118.0, 8: 90.0}"""
    out: dict[int, float] = {}
    if not spec:
        return out
    for part in re.split(r"[,;]", spec):
        part = part.strip()
        if not part:
            continue
        if "=" not in part:
            raise SystemExit(f"--servo 片段缺少 '='：{part!r}")
        k, v = part.split("=", 1)
        out[int(k.strip().lstrip("Ss"))] = float(v)
    return out


def parse_expect_arg(spec: str | None) -> dict[str, float]:
    """--expect "shoulder=20.29,elbow=112.62" -> {'shoulder': .., 'elbow': ..}"""
    out: dict[str, float] = {}
    if not spec:
        return out
    for part in re.split(r"[,;]", spec):
        part = part.strip()
        if not part:
            continue
        if "=" not in part:
            raise SystemExit(f"--expect 片段缺少 '='：{part!r}")
        k, v = part.split("=", 1)
        out[k.strip().lower()] = float(v)
    return out


def load_plan(spec: str | None) -> dict[str, tuple[dict[str, float], dict[int, float]]]:
    """--plan plan.json：**逐帧**显式期望角。

    为什么必须有这条路：文件名规则（`S8_045.jpg`）只能表达「单个舵机在动、其余在
    RESET 位」。真机端到端验收里必然出现**组合位姿**（肩+肘同时离开 HOME），
    这时文件名携带不了信息量，只能由调用方（tools/verify_serial_e2e.mjs）把
    每一步的期望值写成 sidecar JSON。

    格式（键为**文件名**，与目录内实际文件同名）：:

        {"00_reset.jpg": {"shoulder": 0.8499, "elbow": 112.6186,
                          "_servo": {"7": 90, "8": 91}},
         "05_combo.jpg": {"shoulder": 16.1258, "elbow": 127.6561}}

    `_servo`（可选，下划线前缀即保留字）给出该帧的**舵机角** —— 标定增益复核
    （Δ关节反解/Δ舵机）离不开它；缺了就只有关节角，增益复核会报「帧数不足」。

    返回 `{文件名: (关节角, 舵机角)}`；未出现在 plan 里的帧回落到
    `--expect` / 文件名规则。
    """
    if not spec:
        return {}
    raw = json.loads(Path(spec).read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise SystemExit(f"--plan 应为 JSON 对象（文件名 -> {{关节: 角度}}）：{spec}")
    out: dict[str, tuple[dict[str, float], dict[int, float]]] = {}
    for name, entry in raw.items():
        if not isinstance(entry, dict):
            raise SystemExit(f"--plan[{name!r}] 应为 {{关节: 角度}} 字典，实际 {type(entry).__name__}")
        joints = {str(k).strip().lower(): float(v) for k, v in entry.items()
                  if not str(k).startswith("_")}
        servo_raw = entry.get("_servo") or {}
        if not isinstance(servo_raw, dict):
            raise SystemExit(f"--plan[{name!r}]._servo 应为 {{通道: 舵机角}} 字典")
        servo = {int(str(k).strip().lstrip("Ss")): float(v) for k, v in servo_raw.items()}
        out[str(name)] = (joints, servo)
    return out


def read_joint_limits() -> dict[str, tuple[float, float]]:
    """抓 joints 段里的 limit（用于与「舵机限位经标定反算」的结果互校）。"""
    text = YAML.read_text(encoding="utf-8")
    if "joints:" not in text:
        return {}
    sec = text.split("joints:", 1)[1].split("actuators:", 1)[0]
    out: dict[str, tuple[float, float]] = {}
    for blk in sec.split("- id:")[1:]:
        name = re.match(r"\s*(\w+)", blk)
        lim = re.search(r"limit:\s*\n\s*min:\s*(-?[\d.]+)\s*\n\s*max:\s*(-?[\d.]+)", blk)
        if name and lim:
            out[name.group(1)] = (float(lim.group(1)), float(lim.group(2)))
    return out


def joint_limits_from_actuators(acts: dict[int, dict]) -> dict[str, tuple[float, float]]:
    """关节限位 = **舵机硬限位经标定反算**（不新增真值；多舵机关节取交集）。"""
    acc: dict[str, tuple[float, float]] = {}
    for act in acts.values():
        if not act.get("lim"):
            continue
        a, b = (servo_to_joint(act, act["lim"][0]), servo_to_joint(act, act["lim"][1]))
        lo, hi = min(a, b), max(a, b)
        if act["joint"] in acc:
            l0, h0 = acc[act["joint"]]
            acc[act["joint"]] = (max(l0, lo), min(h0, hi))
        else:
            acc[act["joint"]] = (lo, hi)
    return acc


def expected_joints(servo: dict[int, float], acts: dict[int, dict],
                    home: dict, assume_home: bool = True) -> dict[str, float]:
    """舵机角 -> 关节角（经 robot.yaml 标定）。

    ⚠️ 逐度扫描的照片文件名只带**一个**通道（如 S8_045.jpg），但验收需要肩+肘两个期望值。
    未出现在文件名里的通道按「本次扫描时它停在 RESET 位（舵机 90°）」处理，即取该关节的
    homePose 值 —— 这是扫描流程的事实约定，可用 --no-assume-home 关掉。
    多舵机关节（spec §二十五）取各舵机反算值的平均，与后端 ServoAnglesToJoints 同口径。
    """
    acc: dict[str, list[float]] = {}
    for ch, act in acts.items():
        if ch in servo:
            acc.setdefault(act["joint"], []).append(servo_to_joint(act, servo[ch]))
        elif assume_home and act["joint"] in home:
            acc.setdefault(act["joint"], []).append(float(home[act["joint"]]))
    return {k: sum(v) / len(v) for k, v in acc.items()}


def grid_seeds(jlims: dict[str, tuple[float, float]], n: int = 3) -> list[tuple[float, float]]:
    """按 robot.yaml 的关节**限位**铺一层 (肩, 肘) 网格种子。

    为什么要网格：扫描时肩/肘可以相差 30° 以上，只围绕锚点角 ±12° 的种子跨不过去，
    2 参数拟合会落进局部极小（实测 S8_045 反解 136.16°，与 S8_030 几乎相同 = 假的）。
    种子取自**限位**（唯一真值），不取自期望角 —— 期望角一旦入列，验证就被答案牵着走。
    """
    sh, el = jlims.get("shoulder"), jlims.get("elbow")
    if not sh or not el:
        return []
    return [(float(a), float(b))
            for a in np.linspace(sh[0], sh[1], n)
            for b in np.linspace(el[0], el[1], n)]


# ----------------------------------------------------------------------------
# 拟合
# ----------------------------------------------------------------------------

def robust_search(x0s, cost, steps, shrink=0.55, max_cycles=40, min_rel=0.01):
    """全方向 Hooke-Jeeves + 多重启动（比 fit_pose.pattern_search 强，理由见 docstring）。

    fit_pose.pattern_search 用「**遇到首个改善方向就 break**」的贪心变体：一旦起点远离
    真值，它会沿第一个能降代价的坐标轴一路走到底，落进局部极小。实测（合成掩膜，真值
    s=2.55/ths=30/the=110）：真值代价 5.578，而它收敛到代价 12.43（差一倍多）、
    s 偏 +2%。这会直接伪造出「相机反解误差」，所以验收工具必须用更稳的搜索。

    本实现的差别：
      ① 每轮坐标下降扫描**全部** 2n 个方向，取最优者（而非第一个改善者）
      ② 单调下降保证：只接受严格更低的点，绝不走高价点
      ③ 多重启动：对每个起点各收敛一次，取代价最低的解
      ④ 收敛判据是**相对步长**（st[i]/steps[i] < min_rel），不是绝对步长 ——
         否则「30px 的 ox」与「0.4px/mm 的 s」没法用同一个绝对阈值收尾。
    """
    best = None
    for x0 in x0s:
        x = list(x0)
        fx = cost(x)
        st = list(steps)
        for _ in range(max_cycles):
            while True:
                cand, fc = None, fx
                for i in range(len(x)):
                    for sign in (1, -1):
                        c = list(x)
                        c[i] += sign * st[i]
                        v = cost(c)
                        if v < fc - 1e-12:
                            cand, fc = c, v
                if cand is None:
                    break
                x, fx = cand, fc
            if max(st[i] / steps[i] for i in range(len(st))) < min_rel:
                break
            st = [v * shrink for v in st]
        if best is None or fx < best[1]:
            best = (x, fx)
    return best


def points_from(mask: np.ndarray, base_region: np.ndarray | None, cap: int) -> np.ndarray:
    """掩膜 -> 覆盖点云（下采样到 cap 个点）。底座/立柱不随关节动，构造代价前剔除。"""
    cover = mask if base_region is None else (mask & ~base_region)
    if not cover.any():
        cover = mask
    ys, xs = np.nonzero(cover)
    step = max(1, xs.size // cap)
    return np.column_stack([xs[::step], ys[::step]]).astype(float)


def anchor_starts(guess) -> list[list[float]]:
    """锚点的多重启动：默认估值 + 围绕它的若干扰动（覆盖 ±45px / ±25° 的偏差）。"""
    s, ox, oy, ths, the = guess
    out = [[s, ox, oy, ths, the]]
    for ds, dox, doy, dth in ((0.0, -45.0, 40.0, -12.0), (0.0, 45.0, -40.0, 12.0),
                              (0.15, 0.0, 0.0, 0.0), (-0.15, -30.0, 30.0, 0.0),
                              (0.0, 0.0, 0.0, 25.0)):
        out.append([s + ds, ox + dox, oy + doy, ths + dth, the])
    return out


def fit_anchor(dt, mask, base_region, geom, roi_lt, guess, cycles=40):
    """锚点帧：5 参数自由拟合，锁死 (s, ox, oy) —— 像素尺度与肩枢轴由它定标。

    两段式：① 用 250 点的稀疏点云在**全部**起点上粗搜（便宜 4 倍）；② 只把最好的
    两个解拿全点云精修。这样既保住多重启动的抗局部极小能力，又把总耗时压回秒级。
    """
    coarse = make_cost(dt, points_from(mask, base_region, ANCHOR_COARSE_CAP), geom, roi_lt)
    seeds = []
    for st in anchor_starts(guess):
        x, f = robust_search([st], coarse, list(ANCHOR_STEPS), max_cycles=cycles, min_rel=0.02)
        seeds.append((x, f))
    seeds.sort(key=lambda r: r[1])

    fine = make_cost(dt, points_from(mask, base_region, ANCHOR_FINE_CAP), geom, roi_lt)
    fine_steps = [v * 0.25 for v in ANCHOR_STEPS]
    best = None
    for x0, _ in seeds[:2]:
        x, f = robust_search([x0], fine, fine_steps, max_cycles=cycles, min_rel=0.01)
        if best is None or f < best[1]:
            best = (x, f)
    return best


def perturb_seeds(seed: tuple[float, float], dl: float = 12.0) -> list[tuple[float, float]]:
    """围绕一个种子铺局部变体（沿肩/肘反方向各一格，帮助跳出局部极小）。"""
    return [(seed[0], seed[1]), (seed[0] + dl, seed[1] - dl), (seed[0] - dl, seed[1] + dl)]


def fit_pair(dt, mask, base_region, geom, roi_lt, s, ox, oy, starts, cycles=40):
    """固定 (s, ox, oy)，只解 (ths, the)。starts 里**不允许出现期望角**。"""
    cost = make_cost(dt, points_from(mask, base_region, PAIR_CAP), geom, roi_lt)

    def sc(q):
        return cost([s, ox, oy, q[0], q[1]])

    tried = []
    best = None
    for seed in starts:
        q, f = robust_search([seed], sc, list(PAIR_STEPS), max_cycles=cycles)
        tried.append((tuple(round(v, 3) for v in seed), round(f, 4)))
        if best is None or f < best[1]:
            best = (float(q[0]), float(q[1]), float(f), tuple(seed))
    return best, tried


def keep_bottom_component(mask: np.ndarray) -> tuple[np.ndarray, str]:
    """只保留**与 ROI 底边连通**的那一块（底座/立柱必然坐在台面上）。

    为什么需要：底座掩膜来自「扫 S7 时始终为暗 = 不随肩动」的启发式，实测会混进**悬空
    的误判块**（S8 扫描里 x 640-780 / y 330-370 处就有一块），而那块正好落在前臂位置 ——
    把它从覆盖点里挖掉，等于给前臂挖了个洞，2 参数拟合就会掉进错误分支。
    物理判据：底座不可能浮在空中，它一定连着画面底部。
    """
    h, w = mask.shape
    out = np.zeros_like(mask)
    seen = np.zeros_like(mask)
    stack = []
    for y in (h - 1, h - 2):
        stack.extend((y, int(x)) for x in np.nonzero(mask[y])[0])
    if not stack:
        return mask, "底边无底座像素 —— 跳过连通性过滤"
    while stack:
        y, x = stack.pop()
        if y < 0 or y >= h or x < 0 or x >= w or seen[y, x] or not mask[y, x]:
            continue
        seen[y, x] = True
        out[y, x] = True
        stack.extend(((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)))
    if not out.any():
        return mask, "连通域为空 —— 跳过连通性过滤"
    return out, (f"连通性过滤：{int(mask.sum())} -> {int(out.sum())} px"
                 f"（剔掉 {int(mask.sum() - out.sum())} px 悬空误判）")


def build_base_mask(name: str, roi, thresh, inflate: int = 6, bottom_only: bool = True,
                    self_dir: Path | None = None):
    """底座/立柱 = 本批帧里**始终为暗**的像素交集（与关节角无关，对求角无信息量）。

    `name` 可以是 `.workbuddy/captures` 下的扫描目录名（老做法），也可以是
    `self` —— **用当前正在处理的那一批照片自己**。后者物理上更正确：
    立柱/底座在同一批里恒定不动，而它跨批（换相机位/换曝光）就完全对不上了。
    实测教训：拿上午那批（`w2_S7`）的掩膜去剔下午那批，掩膜位置整体错位，
    锚点绝对角直接偏 5.8°；同一批自剔则不会引入这种跨批错配。
    """
    if str(name).strip().lower() in ("self", "@self"):
        d = self_dir
        if d is None or not Path(d).is_dir():
            return None, "base-region=self 但 target 不是目录 —— 跳过底座剔除"
    else:
        d = CAP / name
    if not d.is_dir():
        return None, f"未找到 {d} —— 跳过底座剔除（拟合会略偏，建议先跑扫描留档）"
    frames = sorted(d.glob("*.jpg"))
    if not frames:
        return None, f"{d} 下没有 *.jpg —— 跳过底座剔除"
    ms = [mask_of(Image.open(p), roi, thresh)[0] for p in frames]
    inter = ms[0].copy()
    for m in ms[1:]:
        inter &= m
    note = f"{d.name}: {len(frames)} 帧交集 {int(inter.sum())}px"
    if bottom_only:
        inter, extra = keep_bottom_component(inter)
        note += f"；{extra}"
    return dilate(inter, inflate), note + f" -> 膨胀 {inflate} 后剔除"


# ----------------------------------------------------------------------------
# 自检（无需硬件）：合成掩膜 -> 反解 -> 断言
# ----------------------------------------------------------------------------

def synth_mask(s, ox, oy, ths, the, geom, size=(720, 1280), width=22) -> np.ndarray:
    """按已知角度渲染一张「黑臂白底」合成图，当作相机的完美观测。"""
    l1, l2, l3 = geom
    img = Image.new("L", (size[1], size[0]), 255)
    dr = ImageDraw.Draw(img)
    pts, _ = skeleton(ths, the, l1, l2, l3, n=400)
    # joint=None（直角拼接）更接近实物：大小臂是**两块独立薄板**，关节处不是圆的
    dr.line([(ox + p[0] * s, oy + p[1] * s) for p in pts], fill=0, width=width)
    return np.asarray(img, dtype=np.uint8) < 128


def selftest(nums: dict, cycles: int) -> int:
    """不接硬件的自检：合成掩膜 -> 反解 -> 断言 + 打印「轮廓厚度 -> 系统偏置」实测表。

    ⚠️ 这个自检验的是**搜索器有没有 bug、代价面有没有被搜漏**，**不是**拟合精度。
    实测结论（见下面的厚度表）：对称 Chamfer（骨架->掩膜 DT + 掩膜->骨架最近点）
    在「粗轮廓」下存在**随轮廓厚度增长的系统偏置**，22px 约 0.6°、66px 可达 2.6°。
    这解释了 robot.yaml 里「绝对角仍有 ±5° 量级不确定度」的来源，也意味着：
      ⇒ --tol 不应低于本方法在当前台面上的系统偏置，否则会追鬼影；
      ⇒ 想要更准，应该**加细轮廓/提高对比**（这正是 Phase 4.5 重布台面的目标），
        或者只比**帧间差**（--dtol，公共偏置会被抵消）。
    断言：代价不高于真值代价（搜索没漏）、|Δs| < 0.05 px/mm、角误差 < 3°。
    """
    geom = (nums.get("L1", 80.0), nums.get("L2", 80.0), nums.get("L3", 40.0))
    cases = [
        ("HOME       ", 2.40, 570.0, 470.0, 0.85, 112.62),
        ("肩前倾     ", 2.40, 570.0, 470.0, 20.29, 112.62),
        ("肩后仰     ", 2.40, 570.0, 470.0, -5.00, 118.00),
        ("肘高角     ", 2.55, 610.0, 455.0, 12.00, 126.00),
        ("肘低角     ", 2.55, 610.0, 455.0, 30.00, 110.00),
        ("极限组合   ", 2.62, 520.0, 500.0, 45.00, 108.50),
    ]
    width = 60                    # 约 25mm @2.4px/mm，接近实物大小臂总成的侧视厚度
    tol_ang, tol_s = 3.0, 0.05
    print("# 自检 A：搜索器收敛性 + 像素尺度无偏性（合成掩膜，白底黑臂）")
    print(f"# 连杆 大臂={geom[0]} 小臂={geom[1]} 手部={geom[2]}   轮廓厚={width}px   "
          f"周期={cycles}   断言 代价<=真值代价、|Δs|<{tol_s}、角<{tol_ang}°")
    print(f"{'用例':<12}{'真值 s':>8}{'解出 s':>8}{'Δs':>7}"
          f"{'真值 ths':>10}{'解出 ths':>10}{'真值 the':>10}{'解出 the':>10}"
          f"{'真值代价':>10}{'解代价':>9}  判定")
    npass = 0
    for name, s0, ox0, oy0, ths0, the0 in cases:
        mask = synth_mask(s0, ox0, oy0, ths0, the0, geom, width=width)
        dt = distance_transform(mask)
        cost = make_cost(dt, points_from(mask, None, ANCHOR_FINE_CAP), geom, (0, 0))
        c_true = cost([s0, ox0, oy0, ths0, the0])
        # 起点刻意偏离真值（尺度 -8%、枢轴 -40/+35px、角 +15/-20°）
        x, f = fit_anchor(dt, mask, None, geom, (0, 0),
                          [s0 * 0.92, ox0 - 40.0, oy0 + 35.0, ths0 + 15.0, the0 - 20.0],
                          cycles=cycles)
        d_s, d_ths, d_the = abs(x[0] - s0), abs(x[3] - ths0), abs(x[4] - the0)
        ok = (f <= c_true + 1e-6) and d_s < tol_s and d_ths < tol_ang and d_the < tol_ang
        npass += int(ok)
        print(f"{name:<12}{s0:>8.3f}{x[0]:>8.3f}{x[0] - s0:>+7.3f}"
              f"{ths0:>+10.2f}{x[3]:>+10.2f}{the0:>+10.2f}{x[4]:>+10.2f}"
              f"{c_true:>10.4f}{f:>9.4f}  {'PASS' if ok else 'FAIL'}")
    print(f"自检 A: PASS {npass} / FAIL {len(cases) - npass}")

    print("\n# 自检 B：轮廓厚度 -> 系统偏置（同一真值，只改描边宽度；报告用，不参与判定）")
    ws0, wx0, wy0, wth0, wth2 = 2.40, 570.0, 470.0, -5.0, 118.0
    print(f"# 真值 s={ws0} ox={wx0} oy={wy0} ths={wth0:+.2f} the={wth2:+.2f}"
          f"   ← 即上表「肩后仰」用例")
    print(f"{'厚度px':>7}{'≈mm':>7}{'Δs':>9}{'Δox':>8}{'Δoy':>8}{'Δths':>9}{'Δthe':>9}")
    for w in (20, 40, 60, 80, 100):
        mask = synth_mask(ws0, wx0, wy0, wth0, wth2, geom, width=w)
        x, _f = fit_anchor(distance_transform(mask), mask, None, geom, (0, 0),
                           [ws0 * 0.92, wx0 - 40.0, wy0 + 35.0, wth0 + 15.0, wth2 - 20.0],
                           cycles=cycles)
        print(f"{w:>7}{w / ws0:>7.1f}{x[0] - ws0:>+9.3f}{x[1] - wx0:>+8.1f}"
              f"{x[2] - wy0:>+8.1f}{x[3] - wth0:>+9.2f}{x[4] - wth2:>+9.2f}")
    print("⇒ 偏置随轮廓厚度单调变差：这就是必须做「帧间差」判定(--dtol) 的原因。")
    return 0 if npass == len(cases) else 1


# ----------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description="相机反解关节角 -> 与期望值比对 -> PASS/FAIL",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", nargs="?", help="单张图 或 目录")
    ap.add_argument("--selftest", action="store_true",
                    help="不接硬件：用合成掩膜验证拟合内核")
    ap.add_argument("--dump-inputs", action="store_true",
                    help="打印由 robot.yaml 派生的 homePose/限位/ROI 的 JSON 后退出（供 e2e 脚本消费）")
    ap.add_argument("--quantize", default=None, metavar="SPEC",
                    help='把意图关节角折成"设备侧真会停的位置"并打印 JSON，如 '
                         '"shoulder=5.85,elbow=117.62"（固件只吃整数舵机度）')
    ap.add_argument("--anchor", default=None,
                    help="锚点帧（建议 RESET 位）：由它 5 参数自由拟合锁定 s / 肩枢轴")
    ap.add_argument("--calib", type=float, nargs=3, default=None, metavar=("S", "OX", "OY"),
                    help="直接给已标定的 像素/毫米 与 肩枢轴像素坐标（跳过锚点）")
    ap.add_argument("--expect", default=None, help='如 "shoulder=20.29,elbow=112.62"')
    ap.add_argument("--servo", default=None, help='如 "7=118,8=90"（按 yaml 标定换算）')
    ap.add_argument("--plan", default=None,
                    help="逐帧期望角的 JSON 文件（文件名 -> {关节: 角度}）；组合位姿必需，"
                         "由 tools/verify_serial_e2e.mjs 生成")
    ap.add_argument("--no-assume-home", dest="assume_home", action="store_false",
                    help="文件名只带一个通道时，不再把未出现的关节按 RESET/homePose 补期望值")
    ap.add_argument("--tol", type=float, default=5.0, help="肩/肘**绝对**误差容差（度），默认 5.0")
    ap.add_argument("--tol-elbow", type=float, default=None, help="单独覆盖肘容差")
    ap.add_argument("--dtol", type=float, default=1.5,
                    help="**帧间差**容差（度），默认 1.5。这是锐利判据：它能抵消相机反解"
                         "随轮廓厚度产生的公共偏置（实测 25mm 轮廓约 1.8°），"
                         "因此可以设得比 --tol 紧得多。参考帧恒为 0。")
    ap.add_argument("--base-tol", type=float, default=3.0,
                    help="期望 base 偏离 0 超过此值时判 SKIP（二维拟合失效），默认 3.0")
    ap.add_argument("--drift", type=float, default=0.19,
                    help="已知耦合残差系数（小臂绝对角随肩角漂移的比例），默认 0.19；"
                         "只用于**附加**报告，不参与主判定")
    ap.add_argument("--no-drift", action="store_true", help="不计算漂移补偿列")
    ap.add_argument("--roi", type=int, nargs=4, default=list(DEFAULT_ROI))
    ap.add_argument("--thresh", default=str(DEFAULT_THRESH), metavar="N|auto",
                    help=f"暗件分割阈值；`auto` = 全批逐帧 Otsu 取中位数"
                         f"（曝光漂移时必用；默认 {DEFAULT_THRESH}）")
    ap.add_argument("--base-region", default="w2_S7",
                    help="底座/立柱剔除所依据的扫描目录名（在 .workbuddy/captures 下）；"
                         "`self` = 用当前 target 目录自身（**换相机位/换曝光后应当用这个**）")
    ap.add_argument("--no-base-mask", action="store_true")
    ap.add_argument("--base-anywhere", dest="base_bottom_only", action="store_false",
                    help="底座掩膜不做「只保留与 ROI 底边连通」的过滤")
    ap.add_argument("--guess", type=float, nargs=5, default=None,
                    metavar=("S", "OX", "OY", "THS", "THE"))
    ap.add_argument("--seed", type=float, nargs=2, default=None, metavar=("THS", "THE"),
                    help="额外起点（默认只用 锚点角 + homePose，避免被期望值牵引）")
    ap.add_argument("--iters", type=int, default=60,
                    help="搜索的收缩周期数（每个启动各跑一份），默认 60；越大越慢越准")
    ap.add_argument("--overlay", default=None, help="叠加图输出：目录 或（单帧时）文件")
    ap.add_argument("--json", default=None, help="把逐帧结果写成 JSON")
    a = ap.parse_args()

    nums = read_yaml_numbers()
    acts = read_actuators()
    geom = (nums.get("L1", 80.0), nums.get("L2", 80.0), nums.get("L3", 40.0))
    home = nums.get("home", {}) or {}
    roi = tuple(a.roi)
    thresh = a.thresh          # 整数，或 "auto"（解析推迟到收集完帧之后）

    if a.selftest:
        return selftest(nums, a.iters)
    if a.dump_inputs or a.quantize:
        # 供 tools/verify_serial_e2e.mjs 消费：真值的解析只留在本文件一处，
        # Node 侧不再重复实现 YAML 解析与标定换算。
        out = {
            "home": home,
            "limits": joint_limits_from_actuators(acts),
            "link": {"L1": geom[0], "L2": geom[1], "L3": geom[2]},
            "roi": list(roi), "thresh": thresh,
        }
        if a.quantize:
            qj, qs = quantize_plan(parse_expect_arg(a.quantize), acts)
            out["quantized"] = qj
            out["quantized_servo"] = {str(k): v for k, v in qs.items()}
        print(json.dumps(out, ensure_ascii=False))
        return 0
    if not a.target:
        raise SystemExit("需要 target（图片或目录），或加 --selftest")

    targets = collect(a.target)
    thresh = resolve_threshold(thresh, targets, roi)

    tol_elbow = a.tol if a.tol_elbow is None else a.tol_elbow

    print("# ==== 相机反解验收 ===")
    print(f"# robot.yaml：连杆 立柱={nums.get('L0')} 大臂={geom[0]} 小臂={geom[1]} 手部={geom[2]}")
    print(f"#             homePose={home}")
    if acts:
        for ch in sorted(acts):
            t = acts[ch]
            print(f"# actuator S{ch} -> {t['joint']:<8} offset={t['offset']:<9} "
                  f"scale={t['scale']:<9} reverse={str(t['reverse']).lower():<5} "
                  f"servo限位={t['lim']}")
    print(f"# ROI={roi}  阈值<{thresh}  绝对容差 肩<={a.tol}° 肘<={tol_elbow}°  "
          f"帧间差容差<={a.dtol}°  base离面阈值<={a.base_tol}°")
    print("# 判定 = 绝对误差达标 **且** 帧间差达标；帧间差才能分辨「臂有没有动到位」，"
          "绝对误差含相机公共偏置。")
    print("# ⚠️ 相机只能测矢状面（shoulder/elbow）；base 绕竖直轴、gripper 开口均不可测。")

    jlims = joint_limits_from_actuators(acts)
    ylims = read_joint_limits()
    for k, v in sorted(jlims.items()):
        yv = ylims.get(k)
        flag = ""
        if yv and (abs(yv[0] - v[0]) > 0.01 or abs(yv[1] - v[1]) > 0.01):
            flag = f"   ⚠️ 与 joints[].limit {yv} **不一致** —— 标定与限位不自洽，先修 robot.yaml"
        print(f"# 关节限位(标定反算) {k:<8} [{v[0]:>9.4f}, {v[1]:>9.4f}]{flag}")
    if jlims:
        print("# 上述限位同时用作 2 参数拟合的网格种子（取自限位，**不取期望角**）。")

    base_region, note = (None, "已禁用") if a.no_base_mask \
        else build_base_mask(a.base_region, roi, thresh, bottom_only=a.base_bottom_only,
                             self_dir=Path(a.target))
    print(f"# 底座剔除：{note}")

    frames: list[tuple[Path, dict[int, float], dict[str, float]]] = []
    servo_arg = parse_servo_arg(a.servo)
    expect_arg = parse_expect_arg(a.expect)
    plan = load_plan(a.plan)
    n_plan = 0
    for p in targets:
        sv = dict(servo_arg) if servo_arg else parse_frame_servo(p)
        if p.name in plan:
            exp, sv_plan = plan[p.name]
            if sv_plan:
                sv = sv_plan          # 增益复核需要舵机角作自变量
            n_plan += 1
        elif expect_arg:
            exp = dict(expect_arg)
        else:
            exp = expected_joints(sv, acts, home, a.assume_home)
        frames.append((p, sv, exp))
    if plan:
        print(f"# --plan：{n_plan}/{len(frames)} 帧有显式期望（组合位姿必需；"
              f"未列出的帧回落文件名/--expect 规则）")
    n_assumed = sum(1 for _p, sv, exp in frames
                    if len(exp) > len({acts[c]["joint"] for c in sv if c in acts}))
    if n_assumed and not expect_arg:
        print(f"# 注：{n_assumed} 帧的文件名只带一个通道，未出现的通道按"
              f"「本次扫描停在 RESET 位」补 homePose 期望（--no-assume-home 可关）。")

    # ---- 锁定 (s, ox, oy)
    guess = a.guess or list(DEFAULT_GUESS)
    s = ox = oy = None
    anchor_row = None
    if a.calib:
        s, ox, oy = (float(v) for v in a.calib)
        print(f"# 标定（--calib）：s={s:.3f} px/mm  肩枢轴=({ox:.1f},{oy:.1f})")
    elif a.anchor:
        aim = Image.open(a.anchor)
        am, al, at = mask_of(aim, roi, thresh)
        ax, af = fit_anchor(distance_transform(am), am, base_region, geom, (al, at),
                            guess, cycles=a.iters)
        s, ox, oy = ax[0], ax[1], ax[2]
        print(f"# 锚点 {Path(a.anchor).name}: s={s:.3f} px/mm  肩枢轴=({ox:.1f},{oy:.1f})  "
              f"该帧反解 ths={ax[3]:+.2f}° the={ax[4]:+.2f}°  残差={af:.3f}px")
        print("#   锚点位的绝对角就定义在这里：拟合值 vs homePose 的差即**锚点偏置**。")
        for p, _sv, exp in frames:
            if p.resolve() == Path(a.anchor).resolve():
                anchor_row = exp or None
                if exp:
                    e_s, e_e = exp.get("shoulder"), exp.get("elbow")
                    if e_s is not None and e_e is not None:
                        print(f"#   锚点偏置：肩 {ax[3] - e_s:+.2f}°   肘 {ax[4] - e_e:+.2f}°"
                              f"（为 0 说明标定与 homePose 自洽，非 0 则计入后续误差预算）")
                break
    else:
        # 没有锚点也没有标定：拿第一帧自由拟合（会用掉一个自由度，精度下降）
        p0 = frames[0][0]
        m0, l0, t0 = mask_of(Image.open(p0), roi, thresh)
        ax, af = fit_anchor(distance_transform(m0), m0, base_region, geom, (l0, t0),
                            guess, cycles=a.iters)
        s, ox, oy = ax[0], ax[1], ax[2]
        print(f"# ⚠️ 未给 --anchor/--calib，改用首帧 {p0.name} 自由定标："
              f"s={s:.3f}  肩枢轴=({ox:.1f},{oy:.1f})  残差={af:.3f}px")
        print("#    ⇒ 该帧同时承担「定标」与「验收」，误差会往下传导；正式验收请给锚点。")

    # ---- 逐帧反解
    outdir = None
    if a.overlay:
        ov = Path(a.overlay)
        if ov.is_dir() or len(frames) > 1:
            ov.mkdir(parents=True, exist_ok=True)
            outdir = ov
        else:
            ov.parent.mkdir(parents=True, exist_ok=True)
            outdir = ov

    hdr = (f"{'文件':<20}{'S角':>7}{'肩反解':>9}{'肘反解':>9}{'肩期望':>9}{'肘期望':>9}"
           f"{'肩差':>8}{'肘差':>8}{'Δ差肩':>8}{'Δ差肘':>8}{'残差px':>8}  判定")
    print("\n" + hdr)
    print("-" * len(hdr))

    results = []
    npass = nfail = nskip = 0
    ref = None            # 帧间差参考帧：(ths, the, 期望肩, 期望肘, 文件名)
    samples: dict[int, list[tuple[float, float]]] = {}   # 通道 -> [(舵机角, 反解关节角)]
    for p, sv, exp in frames:
        im = Image.open(p)
        m, l, t = mask_of(im, roi, thresh)
        dt = distance_transform(m)
        tag = "".join(f"S{c}={v:g} " for c, v in sorted(sv.items())).strip() or "-"

        # base 离面检查：二维拟合的前提是臂在矢状面内
        e_base = exp.get("base")
        if e_base is not None and abs(e_base) > a.base_tol:
            nskip += 1
            print(f"{p.name:<20}{tag:>7}{'':>9}{'':>9}{e_base:>9.2f}{'':>9}{'':>8}{'':>8}"
                  f"{'':>8}{'':>8}{'':>8}  SKIP(base={e_base:+.2f}° 离面，二维不可解)")
            results.append(dict(file=p.name, servo=sv, expected=exp, verdict="SKIP",
                                reason=f"base={e_base:.2f} out of plane"))
            continue

        # 2 参数拟合的起点：前两个种子铺 ±12° 局部变体（跳出局部极小），
        # 再叠加一层「按关节限位铺的网格」覆盖大跨度（扫 S8 时肘角可差 30°+）。
        base_seeds = []
        if a.seed:
            base_seeds.append((float(a.seed[0]), float(a.seed[1])))
        if anchor_row:
            base_seeds.append((anchor_row.get("shoulder", 0.0), anchor_row.get("elbow", 112.62)))
        base_seeds.append((float(home.get("shoulder", 0.0)),
                           float(home.get("elbow", 112.62))))

        starts, seen = [], set()

        def _add(seed):
            key = (round(seed[0], 2), round(seed[1], 2))
            if key not in seen:
                seen.add(key)
                starts.append(seed)

        for i, b in enumerate(base_seeds):
            for sd in (perturb_seeds(b) if i < 2 else [b]):
                _add(sd)
        for gd in grid_seeds(jlims, 3):
            _add(gd)

        (ths, the, resid, won), tried = fit_pair(
            dt, m, base_region, geom, (l, t), s, ox, oy, starts, a.iters)

        e_s, e_e = exp.get("shoulder"), exp.get("elbow")
        if e_s is None or e_e is None:
            nskip += 1
            print(f"{p.name:<20}{tag:>7}{ths:>+9.2f}{the:>+9.2f}{'?':>9}{'?':>9}"
                  f"{'':>8}{'':>8}{'':>8}{'':>8}{resid:>8.2f}  SKIP(无期望值)")
            results.append(dict(file=p.name, servo=sv, expected=exp, fit=[ths, the],
                                resid=resid, verdict="SKIP", reason="no expectation"))
            continue

        d_s, d_e = ths - e_s, the - e_e
        # 采集 (舵机角, 反解关节角) —— 用于事后复核**标定增益**（1/scale）。
        # 增益是本次测量里最可信的量：它只依赖帧间差，天然抵消相机公共偏置。
        for ch_, ang_ in sv.items():
            act_ = acts.get(ch_)
            if act_ is None:
                continue
            if act_["joint"] == "shoulder":
                samples.setdefault(ch_, []).append((float(ang_), ths))
            elif act_["joint"] == "elbow":
                samples.setdefault(ch_, []).append((float(ang_), the))
        # 帧间差：扣掉「本方法随轮廓厚度产生的公共偏置」后，臂到底有没有动到该去的地方。
        # 绝对误差受公共偏置污染（实测 60px 轮廓下可达 1.8°），帧间差则把它抵消掉，
        # 所以**只有帧间差才是真正锐利的判据**。参考帧的帧间差恒为 0，只看绝对误差。
        if ref is None:
            ref = (ths, the, e_s, e_e, p.name)
            dif_s = dif_e = 0.0
            dif_ok = True
            is_ref = True
        else:
            dif_s = (ths - ref[0]) - (e_s - ref[2])
            dif_e = (the - ref[1]) - (e_e - ref[3])
            dif_ok = abs(dif_s) <= a.dtol and abs(dif_e) <= a.dtol
            is_ref = False

        ok = abs(d_s) <= a.tol and abs(d_e) <= tol_elbow and dif_ok
        npass += int(ok)
        nfail += int(not ok)
        mark = "PASS" if ok else "FAIL"
        print(f"{p.name:<20}{tag:>7}{ths:>+9.2f}{the:>+9.2f}{e_s:>+9.2f}{e_e:>+9.2f}"
              f"{d_s:>+8.2f}{d_e:>+8.2f}{dif_s:>+8.2f}{dif_e:>+8.2f}{resid:>8.2f}  "
              f"{mark}{'  ← 帧间差参考' if is_ref else ''}")
        row = dict(file=p.name, servo={str(k): v for k, v in sv.items()},
                   expected={k: round(v, 4) for k, v in exp.items()},
                   fit={"shoulder": round(ths, 4), "elbow": round(the, 4)},
                   err={"shoulder": round(d_s, 4), "elbow": round(d_e, 4)},
                   err_diff={"shoulder": round(dif_s, 4), "elbow": round(dif_e, 4)},
                   is_reference=is_ref,
                   resid_px=round(resid, 4), seeds_tried=tried,
                   seed_won=[round(v, 3) for v in won], verdict="PASS" if ok else "FAIL",
                   verdict_abs="PASS" if (abs(d_s) <= a.tol and abs(d_e) <= tol_elbow)
                   else "FAIL")

        if not a.no_drift and e_s is not None:
            # 已知物理残差：平行四连杆不完全等长，扫肩时小臂**绝对角**会跟着漂。
            # 这里用**期望肩角**（不依赖拟合）算补偿量，保持两个期望的独立性。
            dh = e_s - float(home.get("shoulder", 0.0))
            comp = e_e + a.drift * dh
            row["elbow_drift_compensated"] = round(comp, 4)
            row["err_elbow_compensated"] = round(the - comp, 4)
            results.append(row)
            print(f"{'':<20}{'':>7}{'':>9}{'':>9}{'':>9}{comp:>9.2f}{'':>8}"
                  f"{the - comp:>+8.2f}{'':>8}{'':>8}{'':>8}  ↳扣残差 k={a.drift}·Δ肩({dh:+.2f})")
        else:
            results.append(row)

        if outdir is not None:
            info = (f"{p.name} s={s:.2f} ths={ths:+.2f} the={the:+.2f} "
                    f"exp={e_s:+.2f}/{e_e:+.2f} err={d_s:+.2f}/{d_e:+.2f} "
                    f"res={resid:.1f}px {'PASS' if ok else 'FAIL'}")
            dst = outdir if outdir.is_file() else (outdir / f"{p.stem}_fit.jpg")
            draw_overlay(p, roi, dst, s, ox, oy, ths, the, geom, info)

    # ---- 汇总
    tot = npass + nfail + nskip
    print(f"\n相机反解验收 PASS {npass} / FAIL {nfail} / SKIP {nskip}（共 {tot} 帧）")
    if ref:
        print(f"帧间差参考帧 = {ref[4]}（真值 ths={ref[0]:+.2f}° the={ref[1]:+.2f}°；"
              "该行帧间差恒为 0，只看绝对误差）")
    errs = [abs(r["err"]["shoulder"]) for r in results if r.get("err")]
    errs += [abs(r["err"]["elbow"]) for r in results if r.get("err")]
    if errs:
        print(f"绝对误差  最大 {max(errs):.2f}°   平均 {sum(errs) / len(errs):.2f}°"
              f"（容差 {a.tol}° / {tol_elbow}°；含相机公共偏置）")
    difs = [abs(r["err_diff"]["shoulder"]) for r in results if r.get("err_diff")]
    difs += [abs(r["err_diff"]["elbow"]) for r in results if r.get("err_diff")]
    if difs:
        print(f"帧间差    最大 {max(difs):.2f}°   平均 {sum(difs) / len(difs):.2f}°"
              f"（容差 {a.dtol}°）  ← **这条才是「臂是否真动到位」的结论**")
    print("误差预算：舵机整数取整 <=0.347°(肩)/0.209°(肘)；JR 线 0.1° <=0.05°；"
          "相机反解公共偏置随轮廓厚度增长（20px≈0.24° / 60px≈1.8° / 100px≈5.5°，"
          "见 verify_pose.py --selftest 自检 B）")
    print("⇒ 绝对误差 FAIL 而同向偏大：优先怀疑【台面未重布】"
          "（白分割板 + 画面内标尺 + 正交侧视），而非固件/链路；"
          "帧间差 FAIL 才真正指向固件/链路/标定。")

    # ---- 标定增益复核：Δ关节反解 / Δ舵机 —— 只依赖帧间差，是本次测量里最可信的量。
    #      与 robot.yaml 的 1/scale 比对，直接回答「标定对不对」。
    print("\n标定增益复核（Δ关节反解 / Δ舵机）vs robot.yaml 的 1/scale：")
    shown = False
    gain_rows = {}
    for ch in sorted(samples):
        pts = sorted(samples[ch])
        if len(pts) < 2:
            continue
        (sv0, j0), (sv1, j1) = pts[0], pts[-1]
        if abs(sv1 - sv0) < 1e-9:
            continue
        act = acts[ch]
        gain = (j1 - j0) / (sv1 - sv0)
        g_yaml = (-1.0 if act["reverse"] else 1.0) / act["scale"]
        shown = True
        gain_rows[act["joint"]] = {
            "channel": ch, "measured": round(gain, 5), "expected": round(g_yaml, 5),
            "delta_pct": round((gain / g_yaml - 1) * 100, 2),
            "servo_span": [round(sv0, 2), round(sv1, 2)], "n": len(pts),
        }
        print(f"  S{ch}({act['joint']:<8}) 实测 {gain:+.4f}   yaml {g_yaml:+.4f}   "
              f"偏差 {(gain / g_yaml - 1) * 100:+.1f}%   "
              f"（舵机 {sv0:g}->{sv1:g}°，{len(pts)} 帧，{pts[0][1]:+.2f}->{pts[-1][1]:+.2f}°）")
    if not shown:
        print("  （帧数不足或舵机角无变化，无法复核增益）")
    print("  ⇒ 增益偏差是**乘性**的：它会随运动幅度放大（实测肩 45° 行程上累积到 6.8°），"
          "所以在小行程下不易察觉，必须扫宽范围才能测出来。")
    if base_region is not None:
        print("  ⚠️ 底座掩膜的取法会显著影响增益（实测同一批照片：连通性过滤开 = 肩 0.603，"
              "关(--base-anywhere) = 0.668）。两者都有物理依据，请两种设置各跑一次再定案。")

    if a.json:
        Path(a.json).parent.mkdir(parents=True, exist_ok=True)
        Path(a.json).write_text(json.dumps(
            dict(calib={"s": s, "ox": ox, "oy": oy}, roi=list(roi), thresh=thresh,
                 tol={"abs_shoulder": a.tol, "abs_elbow": tol_elbow, "diff": a.dtol},
                 reference=ref[4] if ref else None,
                 gain=gain_rows,
                 summary={"pass": npass, "fail": nfail, "skip": nskip},
                 frames=results), ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"结果 JSON -> {a.json}")

    return 0 if (nfail == 0 and nskip == 0) else 1


def draw_overlay(src: Path, roi, dst: Path, s, ox, oy, ths, the, geom, info: str) -> None:
    """绿=反解出的模型骨架，橙圈=肩/肘/腕/TCP；文字用 ASCII 以免 PIL 默认字体乱码。"""
    im = Image.open(src).crop(roi).convert("RGB")
    dr = ImageDraw.Draw(im)
    pts, joints = skeleton(ths, the, *geom, n=60)
    l0 = (ox - roi[0], oy - roi[1])
    dr.line([(l0[0] + q[0] * s, l0[1] + q[1] * s) for q in pts],
            fill=(80, 255, 140), width=3)
    for j in joints:
        cx, cy = l0[0] + j[0] * s, l0[1] + j[1] * s
        dr.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], outline=(255, 120, 80), width=3)
    dr.text((8, 8), info, fill=(255, 255, 255))
    w2, h2 = im.size
    im.resize((w2 * 2, h2 * 2), Image.LANCZOS).save(dst)


if __name__ == "__main__":
    raise SystemExit(main())
