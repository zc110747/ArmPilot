#!/usr/bin/env python3
"""由官方 SO-ARM101 模型生成 ArmPilot 的 `config/robots/so-arm101/robot.yaml`。

## 为什么用生成器而不是手写这份 yaml

因为它是**纯派生数据**，而"手抄 60 多个浮点数"这件事的错误率不是零。
生成器让每个数字都可追溯到官方文件的某一行；`--check` 模式还能把
"模型改了但 yaml 没跟着改"变成一条**会失败的测试**（与 `mearm.xml` 的
`test_generated_mjcf_is_in_sync_with_config` 同一套做法）。

## 输入（**两份官方文件，各用其权威之处**）

```text
assets/models/so-arm101/official/so101_new_calib.urdf   ← 结构 / joint / axis / limit / visual
assets/models/so-arm101/official/so101_new_calib.xml    ← TCP 帧（gripperframe site）+ 材质颜色
```

spec §十 明确：URDF 管结构、MJCF 管仿真，**不要求二者自动互转**。
这里让 MJCF 只出两样东西 —— TCP 帧与颜色 —— 都属于"URDF 里没有或与 MJCF 不一致"的量：

- **TCP 帧**：两份官方文件在同一点上用了**不同朝向**（URDF `gripper_frame_joint` 是
  `Ry(π)`，MJCF `gripperframe` site 是 `Ry(π/2)`，差 90°）。本项目**取 MJCF** ——
  因为 ArmPilot 的 MuJoCo 侧读的就是这个 site，取它才能让 FK 与 MuJoCo 一致到 0.0007°。
  依据与实测见 `assets/models/so-arm101/official/SOURCE.md` §4.2。
- **颜色**：URDF 的 `<visual>` 不带材质，MJCF 的 `<material rgba=...>` 才是官方配色。

## 派生规则（**只换单位，不改数值**）

| 量 | 官方 | ArmPilot | 换算 |
|---|---|---|---|
| 长度 | m | mm | ×1000 |
| 角度 | rad | deg | ×180/π |
| 欧拉角约定 | fixed-axis XYZ（`rpy`） | 可声明 | 写 `rotationConvention: rpy`，**数值原样** |

输出保留 9 位小数。这不是"精度不够"：官方 URDF 的数值本身只有 6 位有效数字
（`1.5708` 就是被截断的 π/2 ⇒ 换算成 `90.000210459°`，这是**文件的**截断，不是本脚本的误差），
保留 9 位小数已经比源数据更精细一个数量级（≤5e-10 deg / mm）。

## 用法

```bash
python tools/gen_so_arm101_robot_yaml.py            # 写入 yaml
python tools/gen_so_arm101_robot_yaml.py --check    # 只校验是否同步（CI / pytest 用）
```
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
OFFICIAL_DIR = REPO_ROOT / "assets" / "models" / "so-arm101" / "official"
URDF_PATH = OFFICIAL_DIR / "so101_new_calib.urdf"
MJCF_PATH = OFFICIAL_DIR / "so101_new_calib.xml"
OUT_PATH = REPO_ROOT / "config" / "robots" / "so-arm101" / "robot.yaml"

#: 网格在该 yaml 的 `file` 字段里的前缀 —— 相对 `assets/models/`，
#: 与前端 `meshRegistry` 的 key 空间一致（它按 `assets/models/` 之后的子串建索引）
MESH_PREFIX = "so-arm101/official/assets/"

#: 官方 URDF 的 `<crosslink>`… 无此概念，故无需处理

#: 关节输出顺序：**根 → 末端**，且 `gripper_frame` 必须排在 `gripper` 之前。
#:
#: 后者不是风格问题：`RobotModel.kinematicChain()` 对同一父连杆的多个子关节
#: **只取数组里第一个**，而 `gripper_link` 同时挂着 `gripper_frame`（TCP 帧）
#: 与 `gripper`（活动爪）。把 TCP 帧排前面，主链才会走到 `gripper_frame_link`。
JOINT_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper_frame",
    "gripper",
]

#: 连杆输出顺序：同样根 → 末端（渲染与阅读顺序）
LINK_ORDER = [
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "lower_arm_link",
    "wrist_link",
    "gripper_link",
    "gripper_frame_link",
    "moving_jaw_so101_v1_link",
]

#: TCP 固定关节 id 与目标连杆。名字刻意沿用官方 `gripper_frame_joint` 的语汇，
#: 但**朝向取自 MJCF 的 `gripperframe` site**（见模块 docstring）。
TCP_JOINT_ID = "gripper_frame"
TCP_CHILD_LINK = "gripper_frame_link"


def num(value: float) -> str:
    """规范化数字字面量：9 位小数、去尾零、消除 `-0.0`、避免指数记法。"""
    rounded = round(float(value), 9)
    if rounded == 0:
        rounded = 0.0
    text = f"{rounded:.9f}".rstrip("0").rstrip(".")
    return text if text not in ("", "-") else "0.0"


def vec(values) -> str:
    return "[" + ", ".join(num(v) for v in values) + "]"


def rad_to_deg(v: float) -> float:
    return float(v) * 180.0 / math.pi


def linear_to_hex(rgba: str) -> str:
    """MJCF 的 `<material rgba>` → `#rrggbb`。

    ⚠️ 这里做了一次**色彩空间编码**，不是"随便挑个颜色"：
    MuJoCo 的 rgba 是**线性**光强，而 three.js（`ColorManagement.enabled = true`）
    把 `#rrggbb` 当 **sRGB** 解释后再转到线性工作空间。直接把线性值写成 hex
    会再被 sRGB 解码一次，颜色会明显偏暗。故此处正向 sRGB 编码，使两端**线性值相同**。
    """
    parts = [float(x) for x in rgba.split()]
    r, g, b = parts[0], parts[1], parts[2]

    def encode(c: float) -> int:
        c = min(max(c, 0.0), 1.0)
        s = 12.92 * c if c <= 0.0031308 else 1.055 * (c ** (1 / 2.4)) - 0.055
        return int(round(s * 255))

    return "#{:02x}{:02x}{:02x}".format(encode(r), encode(g), encode(b))


class Urdf:
    def __init__(self, path: Path):
        root = ET.parse(path).getroot()
        self.robot_name = root.get("name", "")
        self.joints: dict[str, dict] = {}
        self.order: list[str] = []  # URDF 里的声明顺序（仅作记录）
        for j in root.findall("joint"):
            origin = j.find("origin")
            axis = j.find("axis")
            limit = j.find("limit")
            name = j.get("name", "")
            self.joints[name] = {
                "name": name,
                "type": j.get("type", ""),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "xyz": [float(x) for x in (origin.get("xyz") if origin is not None else "0 0 0").split()],
                "rpy": [float(x) for x in (origin.get("rpy") if origin is not None else "0 0 0").split()],
                "axis": [float(x) for x in (axis.get("xyz") if axis is not None else "0 0 0").split()],
                "lower": float(limit.get("lower", 0.0)) if limit is not None else 0.0,
                "upper": float(limit.get("upper", 0.0)) if limit is not None else 0.0,
            }
            self.order.append(name)

        self.links: dict[str, dict] = {}
        for link in root.findall("link"):
            visuals = []
            for v in link.findall("visual"):
                origin = v.find("origin")
                geom = v.find("geometry")
                mesh = geom.find("mesh") if geom is not None else None
                if mesh is None:
                    continue
                filename = mesh.get("filename", "")
                scale_attr = mesh.get("scale")
                visuals.append(
                    {
                        "basename": Path(filename).name,
                        "stem": Path(filename).stem,
                        "scale": [float(x) for x in scale_attr.split()] if scale_attr else None,
                        "xyz": [float(x) for x in (origin.get("xyz") if origin is not None else "0 0 0").split()],
                        "rpy": [float(x) for x in (origin.get("rpy") if origin is not None else "0 0 0").split()],
                    }
                )
            self.links[link.get("name", "")] = {"name": link.get("name", ""), "visuals": visuals}


class Mjcf:
    def __init__(self, path: Path):
        root = ET.parse(path).getroot()
        self.robot_name = root.get("model", "")
        self.materials = {
            m.get("name"): m.get("rgba")
            for m in root.iter("material")
            if m.get("name") and m.get("rgba")
        }
        #: 铰链关节的 range（rad，**满精度**）—— 这是 MuJoCo 实际执行的区间
        self.joint_range: dict[str, tuple[float, float]] = {}
        for joint in root.iter("joint"):
            if joint.get("type", "hinge") != "hinge":
                continue
            name = joint.get("name")
            rng = joint.get("range")
            if name and rng:
                lo, hi = (float(x) for x in rng.split())
                self.joint_range[name] = (lo, hi)
        self.site_local: dict[str, dict] = {}
        for body in root.iter("body"):
            for site in body.findall("site"):
                name = site.get("name")
                if not name:
                    continue
                pos = site.get("pos") or "0 0 0"
                quat = site.get("quat") or "1 0 0 0"
                self.site_local[name] = {
                    "parent_body": body.get("name"),
                    "pos": [float(x) for x in pos.split()],
                    "quat": [float(x) for x in quat.split()],
                }

    def limits_deg(self, joint_name: str) -> tuple[float, float]:
        """关节限位（degree），取自 MJCF 的 `range`。

        **为什么限位不用 URDF**：两份官方文件在限位上**精度不同** ——
        URDF 把 rad 截断到 6 位小数（`1.91986`），MJCF 保留满精度（`1.9198621771937616`
        恰好 = 110°）。差异 ≥1e-6 rad 量级，而 MuJoCo 执行的是 **MJCF 的** range。
        取 MJCF 才能保证「ArmPilot 声明的可达区间」与「仿真的可达区间」**同一个**，
        否则关节扫描扫到声明的端点会在 MuJoCo 里被 `autolimits` 悄悄夹住。
        """
        lo, hi = self.joint_range[joint_name]
        return rad_to_deg(lo), rad_to_deg(hi)

    def color_for_mesh(self, stem: str) -> str | None:
        rgba = self.materials.get(f"{stem}_material")
        return linear_to_hex(rgba) if rgba else None

    def quat_to_urdf_rpy_deg(self, quat_wxyz) -> list[float]:
        """MJCF 的 `quat`（w x y z）→ URDF `rpy`（fixed-axis XYZ，degree）。

        MuJoCo 的 site `quat` 给出的是**相对父 body** 的旋转，与 URDF 的 joint
        `origin rpy` 语义相同 —— 所以可以直接放在 `origin.rotation` + `'rpy'` 上。
        """
        w, x, y, z = quat_wxyz
        # 四元数 → 旋转矩阵（行主序）
        r = [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
        # 提取 fixed-axis XYZ（= Rz·Ry·Rx）：
        #   R20 = -sin(pitch) ⇒ pitch = asin(-R20)
        #   R00/R10 定 yaw，R21/R22 定 roll
        sin_pitch = min(max(-r[2][0], -1.0), 1.0)
        pitch = math.asin(sin_pitch)
        if abs(sin_pitch) < 0.9999999:
            yaw = math.atan2(r[1][0], r[0][0])
            roll = math.atan2(r[2][1], r[2][2])
        else:  # 万向锁：把 roll 归零
            yaw = 0.0
            roll = math.atan2(-r[0][1], r[1][1])
        return [rad_to_deg(roll), rad_to_deg(pitch), rad_to_deg(yaw)]


def parent_of(urdf: Urdf, link_id: str) -> str | None:
    for joint in urdf.joints.values():
        if joint["child"] == link_id:
            return joint["parent"]
    return None


def build_yaml() -> str:
    urdf = Urdf(URDF_PATH)
    mjcf = Mjcf(MJCF_PATH)

    # ---- 一致性自检：官方两份文件必须描述同一条链 -------------------------
    site = mjcf.site_local.get("gripperframe")
    if site is None:
        raise SystemExit("MJCF 里找不到 gripperframe site —— 官方模型变了？")
    if site["parent_body"] != "gripper":
        raise SystemExit(
            f"gripperframe 的父 body 是 {site['parent_body']!r}，预期 'gripper'"
            "（= URDF 的 gripper_link）—— 官方模型改了 body 命名？"
        )

    # 限位：两份文件精度不同（URDF 截断到 6 位小数），但必须一致到「截断误差」以内。
    # 容忍 1e-4 rad（≈0.0057°）—— 比实测最大差 3.7e-6 rad 宽一个数量级以上，
    # 既不会被精度噪声触发，又能抓住「某个关节的限位被谁改过」这类真事故。
    LIMIT_TOL_RAD = 1e-4
    for name in JOINT_ORDER:
        if name == TCP_JOINT_ID:
            continue
        u = urdf.joints[name]
        if u["type"] != "revolute":
            continue
        m_lo, m_hi = mjcf.joint_range[name]
        for label, uv, mv in (("lower", u["lower"], m_lo), ("upper", u["upper"], m_hi)):
            if abs(uv - mv) > LIMIT_TOL_RAD:
                raise SystemExit(
                    f"关节 {name} 的 {label} 在 URDF({uv}) 与 MJCF({mv}) 相差 "
                    f"{abs(uv - mv):.3e} rad，超出容忍 {LIMIT_TOL_RAD:.0e} —— "
                    "两份官方文件已不一致，需人工确认取哪个。"
                )
    urdf_names = {n for n, j in urdf.joints.items() if j["type"] == "revolute"}
    if urdf_names != set(mjcf.joint_range):
        raise SystemExit(
            f"URDF 与 MJCF 的铰链关节集合不同："
            f"只URDF={sorted(urdf_names - set(mjcf.joint_range))} "
            f"只MJCF={sorted(set(mjcf.joint_range) - urdf_names)}"
        )

    lines: list[str] = []
    add = lines.append

    # ---- 文件头 -----------------------------------------------------------
    add("# ⚠️ 本文件由 `tools/gen_so_arm101_robot_yaml.py` 生成 —— 请勿手改。")
    add("#    改上游模型后请重跑生成器；`pytest tests/sim -q` 会核对本文件是否与官方模型同步。")
    add("#")
    add("# 来源：TheRobotStudio/SO-ARM100 @ eecbe3e0a9ebb23e25ad7b2759b03884c6660903")
    add("#       assets/models/so-arm101/official/so101_new_calib.urdf  （结构 / origin / axis / 网格）")
    add("#       assets/models/so-arm101/official/so101_new_calib.xml   （限位 / TCP 帧 / 材质颜色）")
    add("# 派生：位置 m→mm（×1000）、角度 rad→deg（×180/π）—— 只换单位，数值不改；")
    add("#       限位取自 MJCF（满精度 = MuJoCo 实际执行的区间；URDF 只截断到 6 位小数）；")
    add("#       欧拉角以 `rotationConvention: rpy` 原样承载 URDF 的 fixed-axis XYZ。")
    add("# 详见 assets/models/so-arm101/official/SOURCE.md")
    add("version: 1")
    add("")

    # ---- robot 元数据 -----------------------------------------------------
    add("robot:")
    add("  id: so-arm101")
    add("  name: SO-ARM101")
    add("  # 只读元数据：让「基线属于哪个模型」可被机器检查（不参与任何运动学计算）")
    add("  model: SO-ARM101")
    add("  version: 1.0.0")
    add("  units: mm")
    add("  tcp:")
    add(f"    joint: {TCP_JOINT_ID}")
    add("    offset: [0.0, 0.0, 0.0]")
    add("  # 官方 README：new_calib 各关节的虚拟零点。qpos=0 时 MuJoCo 的 gripperframe")
    add("  # 世界位姿 = [391.36190, -0.01126, 226.46875] mm —— 该值由 Sim2Sim 测试")
    add("  # （tests/sim2sim，FK ↔ MuJoCo）逐位核对，不靠人眼。")
    add("  homePose:")
    for name in JOINT_ORDER:
        if name == TCP_JOINT_ID:
            continue  # 固定关节无自由度：不进 homePose、无执行器
        joint = urdf.joints[name]
        if joint["type"] != "revolute":
            continue  # 同上
        add(f"    {name}: 0.0")
    add("")

    # ---- links -----------------------------------------------------------
    add("# 连杆：`length` 恒为 0 —— URDF 把「父坐标系 → 本关节坐标系」的完整位移都写在")
    add("# `joints[].origin.position` 里；拆一个 Tz(length) 出来只会凭空多出一个真值。")
    add("# 全部显示件都来自官方 `<visual>`，**不重做几何**。")
    add("links:")
    for link_id in LINK_ORDER:
        link = urdf.links.get(link_id)
        if link is None:
            raise SystemExit(f"URDF 里没有连杆 {link_id}")
        add(f"  - id: {link_id}")
        parent = parent_of(urdf, link_id)
        add(f"    parent: {parent if parent else 'null'}")
        add("    length: 0.0")
        visuals = link["visuals"]
        if not visuals:
            # 纯坐标系连杆（TCP 帧）：没有可视件，但必须在图里，否则固定关节没有 childLink
            add("    geometry: {type: none}")
            add("")
            continue

        def emit_visual(visual: dict, indent: str, mapping: bool) -> None:
            """输出一个 mesh 显示件。

            ⚠️ schema 里 `geometry` 是**单个对象**、`details` 才是**列表** ——
            两者缩进规则不同，故必须显式区分（少一个 `- ` 会让 geometry 变成
            一个列表，而 loadRobotModel 的 parseGeometry 按对象解析 ⇒ 直接抛错）。
            """
            head = indent if mapping else f"{indent}- "
            body = indent if mapping else f"{indent}  "
            add(f"{head}type: mesh")
            add(f"{body}file: {MESH_PREFIX}{visual['basename']}")
            add(f"{body}position: {vec([v * 1000.0 for v in visual['xyz']])}")
            add(f"{body}rotation: {vec([rad_to_deg(v) for v in visual['rpy']])}")
            add(f"{body}rotationConvention: rpy")
            color = mjcf.color_for_mesh(visual["stem"])
            if color:
                add(f'{body}color: "{color}"')
            if visual["scale"]:
                add(f"{body}scale: {vec(visual['scale'])}")

        # 首个 visual 作 `geometry`（主件，**映射**），其余进 `details`（附加件，**列表**）——
        # 沿用既有 schema 的主/附之分（渲染层据此建 link 对象）。
        add("    geometry:")
        emit_visual(visuals[0], "      ", mapping=True)
        if len(visuals) > 1:
            add("    details:")
            for visual in visuals[1:]:
                emit_visual(visual, "      ", mapping=False)
        add("")
    # 去掉最后多出的空行（保持单个文件末尾换行由写盘统一处理）
    while lines and lines[-1] == "":
        lines.pop()
    add("")

    # ---- joints ----------------------------------------------------------
    add("# 关节：6 个 revolute（5 DOF 臂 + 夹爪）+ 1 个 fixed（TCP 帧）。")
    add("# `role` 一律不填 —— 本机的 shoulder_lift/elbow_flex 恰好能对上 MeArm 的角色名，")
    add("# 但 wrist_flex / wrist_roll 在现有枚举里无对应项，只填一半会让「有 role」变成")
    add("# 一个没有判据的半真命题；UI 缺省回退到 `joint.name`（如 `shoulder_pan`），")
    add("# 对 6 关节臂而言比 J1/J2/J3 更可读。")
    add("joints:")
    for name in JOINT_ORDER:
        if name == TCP_JOINT_ID:
            add(f"  - id: {TCP_JOINT_ID}")
            add("    # TCP 帧：**朝向取自 MJCF 的 gripperframe site**（与 URDF 的 gripper_frame_joint")
            add("    # 相差 90°；取 MJCF 才能让 FK 与 MuJoCo 一致。依据见 SOURCE.md §4.2）。")
            add("    type: fixed")
            add(f"    parentLink: gripper_link")
            add(f"    childLink: {TCP_CHILD_LINK}")
            add("    axis: [0.0, 0.0, 1.0]")
            add("    origin:")
            add(f"      position: {vec([v * 1000.0 for v in site['pos']])}")
            add(f"      rotation: {vec(mjcf.quat_to_urdf_rpy_deg(site['quat']))}")
            add("      rotationConvention: rpy")
            add("")
            continue
        joint = urdf.joints[name]
        add(f"  - id: {name}")
        add(f"    type: {joint['type']}")
        add(f"    parentLink: {joint['parent']}")
        add(f"    childLink: {joint['child']}")
        add(f"    axis: {vec(joint['axis'])}")
        add("    origin:")
        add(f"      position: {vec([v * 1000.0 for v in joint['xyz']])}")
        add(f"      rotation: {vec([rad_to_deg(v) for v in joint['rpy']])}")
        add("      rotationConvention: rpy")
        # 限位取自 MJCF（满精度，= MuJoCo 实际执行的区间），不是 URDF 的 6 位截断值
        lo_deg, hi_deg = mjcf.limits_deg(name)
        add(f"    limit: {{min: {num(lo_deg)}, max: {num(hi_deg)}}}")
        add("")
    while lines and lines[-1] == "":
        lines.pop()
    add("")

    # ---- actuators -------------------------------------------------------
    add("# 执行器：官方 MJCF 是 6 个 <position>，`ctrlrange` **等于**关节范围、`gear=1`")
    add("# ⇒ 关节空间位置伺服，**不存在 offset/scale/reverse 标定**。")
    add("# 故 `unit: joint`：`limits` 就是关节可达区间本身，而不是 0..180 的舵机行程。")
    add("# 绝不为了塞进 0..180 而编一段并不存在的 scale —— 那会让标定从事实退化成凑数。")
    add("actuators:")
    channel = 0
    for name in JOINT_ORDER:
        if name == TCP_JOINT_ID:
            continue
        joint = urdf.joints[name]
        if joint["type"] != "revolute":
            continue
        channel += 1
        lo, hi = mjcf.limits_deg(name)
        add(f"  - id: so101_s{channel}_{name}")
        add(f"    jointId: {name}")
        add(f"    channel: {channel}")
        add("    offset: 0.0")
        add("    scale: 1.0")
        add("    reverse: false")
        add("    unit: joint")
        add(f"    limits: {{min: {num(lo)}, max: {num(hi)}}}")
    add("")

    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="生成 SO-ARM101 的 robot.yaml")
    parser.add_argument(
        "--check",
        action="store_true",
        help="只校验磁盘上的 yaml 是否与官方模型同步（不写盘）；不同步返回 1",
    )
    args = parser.parse_args(argv)

    text = build_yaml()
    if args.check:
        if not OUT_PATH.is_file():
            print(f"[gen_so_arm101] 缺少 {OUT_PATH.relative_to(REPO_ROOT)}", file=sys.stderr)
            return 1
        current = OUT_PATH.read_text(encoding="utf-8")
        if current != text:
            print(
                f"[gen_so_arm101] {OUT_PATH.relative_to(REPO_ROOT)} 与官方模型不同步：\n"
                "  请运行 `python tools/gen_so_arm101_robot_yaml.py` 重新生成。",
                file=sys.stderr,
            )
            return 1
        print(f"[gen_so_arm101] OK —— {OUT_PATH.relative_to(REPO_ROOT)} 与官方模型同步")
        return 0

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(text, encoding="utf-8", newline="\n")
    print(f"[gen_so_arm101] 已写入 {OUT_PATH.relative_to(REPO_ROOT)}（{len(text)} 字符）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
