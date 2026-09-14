"""从 `model/robot.yaml` + `physics/physics.yaml` 生成标准 URDF（`urdf/mearm-v1.urdf`）。

用法::

    python robot-package/mearm-v1/tools/gen_urdf.py            # 写入 manifest 声明的 urdf 路径
    python robot-package/mearm-v1/tools/gen_urdf.py --stdout   # 打印到标准输出
    python robot-package/mearm-v1/tools/gen_urdf.py -o /tmp/m.urdf

设计要点
--------
**为什么是"生成"而不是"手写 URDF"**
    与 `gen_model.py` 同一纪律：`robot.yaml`（运动学/标定/限位）+ `physics.yaml`（惯量/接触）
    是唯一真值源；URDF 是**可重复生成的整合产物**，禁止手工编辑（重跑即覆盖）。
    这样 robot.yaml/physics.yaml 的"单一真值"约束不被破坏，URDF 又是一份**标准格式**
    的整合模型文件（几何 + 运动学 + 惯量 + 接触 + 标定 + HOME），供 Three.js / ROS /
    MoveIt / rviz 等标准工具直接消费。

**URDF 表达不了的量怎么办**
    舵机标定（offset/scale/reverse/channel）、homePose、外观纹理参数 —— 这些 URDF 标准
    格式没有对应元素，收进文件末尾的 `<armpilot>` 扩展块。标准 URDF 解析器会忽略未知
    顶层元素，故整份文件仍是合法标准 URDF；项目的加载器从 `<armpilot>` 读这些量。

**关节角语义（与 gen_model.py 一致）**
    URDF joint value = **局部角**。平行四连杆耦合用 `<mimic>` 表达：
        elbow : 局部 = 112.6185771989 − shoulder   （⇒ 小臂世界绝对倾角恒 ≈112.62°）
        tool  : 局部 = 90 − elbow                 （⇒ 爪恒水平，被动腕）
    `<limit>` 写**局部角**区间（由耦合 + 父关节局部区间精确推出），不是 robot.yaml 里
    的"世界绝对角"限位 —— 后者只用于上层校验，记录在 `<armpilot>` 块里不丢。

**几何单位**
    robot.yaml 长度 mm（box/cylinder/sphere 的 size 是**全尺寸**）；URDF 用 m、全尺寸。
    MuJoCo 用半尺寸、three.js 用 mm 全尺寸 —— 本项目 URDF 直接取 mm→m，与 three.js 一致。
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml


def _find_repo_root(start: Path) -> Path:
    for anc in (start, *start.parents):
        if (anc / "core").is_dir() and (anc / "robot-package").is_dir():
            return anc
    raise SystemExit(
        f"✗ 无法从 {start} 向上找到仓库根（需同时存在 core/ 与 robot-package/ 目录）")


_REPO_ROOT = _find_repo_root(Path(__file__).resolve().parent)
for _p in (_REPO_ROOT / "simulation" / "mujoco", _REPO_ROOT / "core" / "python"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from robopkg import declared_path  # noqa: E402
from robotcfg import (  # noqa: E402
    PROJECT_ROOT,
    ConfigError,
    JointCfg,
    LinkCfg,
    PhysicsCfg,
    RobotCfg,
    load_physics,
    load_robot,
)
from units import mm2m, deg2rad, ensure_utf8_stdout  # noqa: E402

MEARM_V1 = "mearm-v1"
DEFAULT_SERVO_SIZE_MM = (22.8, 12.2, 22.5)


def default_out() -> Path:
    """缺省输出 = 本包 manifest 里 `model.urdf` 声明的文件。"""
    return declared_path(MEARM_V1, "model.urdf", must_exist=False)


def _rel(p: Path | str) -> str:
    p = Path(p).resolve()
    try:
        return p.relative_to(_REPO_ROOT).as_posix()
    except ValueError:
        return str(p)


# ---------------------------------------------------------------------------
# 格式化
# ---------------------------------------------------------------------------

def _f(v: float, nd: int = 9) -> str:
    if v == 0:
        return "0"
    return f"{v:.{nd}g}"


def _vec(v: Sequence[float], nd: int = 9) -> str:
    return " ".join(_f(float(x), nd) for x in v)


def _hex_to_rgba(color: str | None):
    if not color or not color.startswith("#") or len(color) != 7:
        return (0.6, 0.6, 0.6, 1.0)
    r = int(color[1:3], 16) / 255.0
    g = int(color[3:5], 16) / 255.0
    b = int(color[5:7], 16) / 255.0
    return (r, g, b, 1.0)


def _euler_rpy_deg(rot: Sequence[float]) -> str:
    """URDF `<origin rpy>` = 固定轴 X/Y/Z（rad）。本模型所有旋转均单轴，顺序无关。"""
    return _vec([deg2rad(float(x)) for x in rot])


# ---------------------------------------------------------------------------
# 惯量（来自 physics.yaml，全 SI；不得再做 mm→m）
# ---------------------------------------------------------------------------

def _inertial_xml(body_id: str, physics: PhysicsCfg, indent: str) -> str:
    spec = physics.body_inertia_of(body_id)
    mass = float(spec["mass"])
    com = [float(x) for x in (spec.get("com") or (0, 0, 0))]
    shape = str(spec.get("shape", "point"))
    if shape == "box":
        a, b, c = (float(x) for x in spec["size"])
        ix = mass / 12.0 * (b * b + c * c)
        iy = mass / 12.0 * (a * a + c * c)
        iz = mass / 12.0 * (a * a + b * b)
    elif shape == "rod":
        length = float(spec["length"])
        ix = iy = mass * length * length / 12.0
        iz = max(mass * 0.02 * 0.02 / 6.0, 1e-9)
    elif shape == "point":
        r = 0.02
        ix = iy = iz = mass * r * r / 6.0
    else:
        raise ConfigError(f"{body_id}: 未知 inertia shape {shape!r}")
    return (f'{indent}<inertial>'
            f'<origin xyz="{_vec(com)}"/>'
            f'<mass value="{_f(mass)}"/>'
            f'<inertia ixx="{_f(ix)}" ixy="0" ixz="0" '
            f'iyy="{_f(iy)}" iyz="0" izz="{_f(iz)}"/>'
            f'</inertial>')


# ---------------------------------------------------------------------------
# 视觉几何（robot.yaml geometry + details）
# ---------------------------------------------------------------------------

def _material_registry() -> dict[str, str]:
    """颜色 → material 名。"""
    return {
        "#232830": "mearm_dark",
        "#2f3644": "mearm_servo",
        "#2b3446": "mearm_steel",
        "#2b323c": "mearm_jaw",
        "#161a21": "mearm_bolt",
    }


def _visual_elem(spec: Mapping[str, Any], default_pos_mm: Sequence[float],
                 indent: str, name_hint: str) -> list[str]:
    gtype = str(spec.get("type", "none"))
    if gtype in ("none", "jaw"):
        # 爪由渲染层特例化（与 MJCF 同因），URDF 不生成视觉体。
        return []
    pos = spec.get("position") or default_pos_mm
    rot = spec.get("rotation") or (0, 0, 0)
    color = spec.get("color") or "#232830"
    mat = _material_registry().get(color, "mearm_dark")
    xyz = _vec(mm2m(x) for x in pos)
    rpy = _euler_rpy_deg(rot)
    origin = f'xyz="{xyz}" rpy="{rpy}"'

    if gtype in ("plate", "box"):
        a, b, c = (mm2m(float(x)) for x in spec["size"])   # URDF box = 全尺寸
        geom = f'<box size="{_vec([a, b, c])}"/>'
    elif gtype == "cylinder":
        r = mm2m(float(spec["radius"]))
        h = mm2m(float(spec["height"]))                     # URDF cylinder length = 全高
        geom = f'<cylinder radius="{_f(r)}" length="{_f(h)}"/>'
    elif gtype == "sphere":
        r = mm2m(float(spec["radius"]))
        geom = f'<sphere radius="{_f(r)}"/>'
    elif gtype == "servo":
        s = spec.get("size") or DEFAULT_SERVO_SIZE_MM
        a, b, c = (mm2m(float(x)) for x in s)
        geom = f'<box size="{_vec([a, b, c])}"/>'
    else:
        raise ConfigError(f"未知显示几何类型 {gtype!r}")

    return [f'{indent}<visual name="{name_hint}">'
            f'<origin {origin}/>'
            f'<geometry>{geom}</geometry>'
            f'<material name="{mat}"/>'
            f'</visual>']


def _visuals_xml(link: LinkCfg, indent: str) -> list[str]:
    out: list[str] = []
    geom = link.geometry or {}
    if geom.get("type") not in (None, "none"):
        out += _visual_elem(geom, (0.0, 0.0, link.length / 2.0), indent, f"{link.id}_vis")
    for i, d in enumerate(link.details or ()):
        out += _visual_elem(d, (0.0, 0.0, 0.0), indent, f"{link.id}_vis{i}")
    return out


# ---------------------------------------------------------------------------
# 碰撞几何（physics.yaml contact.shapes，全 SI）
# ---------------------------------------------------------------------------

def _collision_xml(body_id: str, physics: PhysicsCfg, indent: str) -> list[str]:
    shapes = physics.contact.get("shapes") or {}
    if body_id not in shapes:
        return []
    s = shapes[body_id]
    gtype = str(s["type"])
    pos = _vec([float(x) for x in (s.get("pos") or (0, 0, 0))])
    size = [float(x) for x in s["size"]]
    if gtype == "box":
        geom = f'<box size="{_vec(size)}"/>'
    elif gtype == "sphere":
        geom = f'<sphere radius="{_f(size[0])}"/>'
    elif gtype == "capsule":
        # URDF capsule size = [radius, length]，length = 2×半长（MuJoCo 半长）
        geom = f'<capsule radius="{_f(size[0])}" length="{_f(2.0 * size[1])}"/>'
    else:
        return []  # 非标准几何（如自定义 quat 的 capsule）跳过，碰撞由 MJCF 承担
    return [f'{indent}<collision name="{body_id}_col">'
            f'<origin xyz="{pos}"/>'
            f'<geometry>{geom}</geometry>'
            f'</collision>']


# ---------------------------------------------------------------------------
# 关节局部角区间（耦合 → 精确局部区间）
# ---------------------------------------------------------------------------

def _local_range_deg(robot: RobotCfg, joint: JointCfg) -> tuple[float, float]:
    """返回该关节 **局部角** 的 [min, max]（deg）。

    无耦合：robot.yaml 的限位即局部角（base / shoulder / gripper 无耦合，局部=绝对）。
    有耦合（平行四连杆）：局部角 = `offset + gain × 父关节局部角`，由耦合 offset
    （世界绝对锁定量）+ 父关节局部区间**精确**推出。

    ⚠️ 不能用 robot.yaml 里 elbow/tool 的"世界绝对角限位"去加耦合偏移——那是 S8/S6
       舵机映射值，平行四连杆下世界绝对角其实是常数（elbow≈112.62°、tool=90°），
       搬它只会得到一段**更窄**的错误区间，反而把合法局部角裁掉。
    """
    if not joint.coupling:
        return joint.limit_min, joint.limit_max
    other_id, gain = joint.coupling
    plo, phi = _local_range_deg(robot, robot.joint(other_id))
    off = _mimic_offset_deg(robot, joint)
    a = off + gain * plo
    b = off + gain * phi
    return (a, b) if a <= b else (b, a)


def _mimic_offset_deg(robot: RobotCfg, joint: JointCfg) -> float:
    """耦合锁定的「世界绝对角」常量：elbow=HOME.elbow，tool=其限位(=90)。"""
    if joint.id in robot.home_pose:
        return float(robot.home_pose[joint.id])
    return float(joint.limit_max)


# ---------------------------------------------------------------------------
# 主生成
# ---------------------------------------------------------------------------

def build_urdf(robot: RobotCfg, physics: PhysicsCfg) -> str:
    L: list[str] = []
    add = L.append
    mats = _material_registry()

    # 舵机能力（写进 <armpilot>）
    srv = physics.servo

    # ---- 头部 ----
    add('<?xml version="1.0" encoding="utf-8"?>')
    add("<!--")
    add(f"  ⚠️ 本文件由 {_rel(Path(__file__))} 生成 —— 请勿手工编辑。")
    add(f"     真值来源: {_rel(robot.source_path)} ({robot.id} / {robot.name}, "
        f"{len(robot.links)} 连杆 / {len(robot.movable_joints())} 自由度 + "
        f"{len(robot.passive_joints())} 被动关节)")
    add(f"     物理参数: {_rel(physics.source_path)}")
    add(f"     重新生成: python {_rel(Path(__file__))}")
    add("-->")
    add(f'<robot name="{robot.id}" xmlns:xacro="http://www.ros.org/wiki/xacro">')

    # ---- 材质 ----
    for color, name in mats.items():
        r, g, b, _ = _hex_to_rgba(color)
        add(f'  <material name="{name}">'
            f'<color rgba="{_f(r, 4)} {_f(g, 4)} {_f(b, 4)} 1"/>'
            f'</material>')

    # ---- 刚体树 ----
    chain = robot.chain_from_root()
    depth_of: dict[str, int] = {robot.root_link().id: 1}
    for link, joint in chain:
        if joint is None:
            continue
        depth_of[link.id] = depth_of[joint.parent_link] + 1

    emitted: set[str] = set()

    def emit(link: LinkCfg, joint: JointCfg | None) -> None:
        d = "  " * depth_of[link.id]
        if joint is None:
            add(f'{d}<link name="{link.id}">')
        else:
            add(f'{d}<link name="{link.id}">')

        inner = d + "  "
        add(_inertial_xml(link.id, physics, inner))
        for line in _collision_xml(link.id, physics, inner):
            add(line)
        for line in _visuals_xml(link, inner):
            add(line)
        add(f"{d}</link>")

        # 关节
        if joint is not None and not joint.is_fixed:
            pd = depth_of[link.id]
            jd = "  " * pd
            parent_link = robot.link(joint.parent_link)
            # URDF joint origin = 子帧相对父帧 = 父 length 沿 +Z + origin 偏移
            op = [joint.origin_position[0],
                  joint.origin_position[1],
                  joint.origin_position[2] + parent_link.length]
            xyz = _vec(mm2m(x) for x in op)
            rpy = _euler_rpy_deg(joint.origin_rotation)
            axis = _vec(joint.axis)
            lo, hi = _local_range_deg(robot, joint)
            effort = _f(float(srv["max_torque_nm"]))
            vel = _f(float(srv["max_velocity"]["rad_per_s"]))
            if joint.is_passive:
                add(f'{jd}<joint name="{joint.id}" type="revolute">')
                add(f'{jd}  <parent link="{joint.parent_link}"/>')
                add(f'{jd}  <child link="{link.id}"/>')
                add(f'{jd}  <origin xyz="{xyz}" rpy="{rpy}"/>')
                add(f'{jd}  <axis xyz="{axis}"/>')
                # 被动腕：mimic elbow 锁定世界绝对角=90°
                off = deg2rad(_mimic_offset_deg(robot, joint))
                add(f'{jd}  <mimic joint="{joint.coupling[0]}" multiplier="-1" '
                    f'offset="{_f(off)}"/>')
                add(f'{jd}  <limit lower="{_f(deg2rad(lo))}" upper="{_f(deg2rad(hi))}" '
                    f'effort="0" velocity="0"/>')
                add(f'{jd}</joint>')
            elif joint.coupling:
                add(f'{jd}<joint name="{joint.id}" type="revolute">')
                add(f'{jd}  <parent link="{joint.parent_link}"/>')
                add(f'{jd}  <child link="{link.id}"/>')
                add(f'{jd}  <origin xyz="{xyz}" rpy="{rpy}"/>')
                add(f'{jd}  <axis xyz="{axis}"/>')
                # 平行四连杆耦合：elbow 局部 = 112.62 − shoulder
                off = deg2rad(_mimic_offset_deg(robot, joint))
                add(f'{jd}  <mimic joint="{joint.coupling[0]}" multiplier="-1" '
                    f'offset="{_f(off)}"/>')
                add(f'{jd}  <limit lower="{_f(deg2rad(lo))}" upper="{_f(deg2rad(hi))}" '
                    f'effort="{effort}" velocity="{vel}"/>')
                add(f'{jd}</joint>')
            else:
                add(f'{jd}<joint name="{joint.id}" type="revolute">')
                add(f'{jd}  <parent link="{joint.parent_link}"/>')
                add(f'{jd}  <child link="{link.id}"/>')
                add(f'{jd}  <origin xyz="{xyz}" rpy="{rpy}"/>')
                add(f'{jd}  <axis xyz="{axis}"/>')
                add(f'{jd}  <limit lower="{_f(deg2rad(lo))}" upper="{_f(deg2rad(hi))}" '
                    f'effort="{effort}" velocity="{vel}"/>')
                add(f'{jd}</joint>')

        # TCP 帧（固定连杆，挂在 tool 帧 +40mm）
        if robot.tcp_joint and joint is not None and joint.id == robot.tcp_joint:
            tdepth = "  " * depth_of[link.id]
            tinner = tdepth + "  "
            off = [mm2m(float(x)) for x in robot.tcp_offset]
            add(f'{tdepth}<link name="tcp"/>')
            add(f'{tinner}<joint name="tcp_fixed" type="fixed">')
            add(f'{tinner}  <parent link="{link.id}"/>')
            add(f'{tinner}  <child link="tcp"/>')
            add(f'{tinner}  <origin xyz="{_vec(off)}"/>')
            add(f'{tinner}</joint>')

        for child_joint in robot.children_of(link.id):
            child_link = robot.link(child_joint.child_link)
            if child_link.id in emitted:
                raise ConfigError(f"连杆 {child_link.id} 被重复生成（图不是树？）")
            emitted.add(child_link.id)
            emit(child_link, child_joint)

    emitted.add(robot.root_link().id)
    emit(robot.root_link(), None)

    # ---- <armpilot> 扩展块（标准解析器忽略；项目加载器读这里）----
    add("")
    add("  <!-- ============================================================")
    add("       armpilot 扩展块：URDF 标准格式表达不了的量集中于此。")
    add("       标准 URDF 解析器会忽略本块；ArmPilot 加载器从这里读")
    add("       舵机标定 / HOME / 外观。真值仍来自 robot.yaml + physics.yaml。")
    add("       ============================================================ -->")
    add('  <armpilot>')
    # 舵机标定
    add('    <actuators>')
    for a in robot.actuators:
        add(f'      <actuator name="servo_{a.channel}" joint="{a.joint_id}" '
            f'channel="{a.channel}" offset="{_f(float(a.offset))}" '
            f'scale="{_f(float(a.scale))}" reverse="{"true" if a.reverse else "false"}"/>')
    add('    </actuators>')
    # HOME
    add('    <home_pose>')
    for k, v in robot.home_pose.items():
        add(f'      <joint name="{k}" deg="{_f(float(v))}"/>')
    add('    </home_pose>')
    # 外观（仅渲染，不参与 FK/IK/物理）
    ap = getattr(robot, "appearance", None) or {}
    tp = (ap.get("texturedPlate") or {}) if isinstance(ap, Mapping) else {}
    if tp:
        add('    <appearance>')
        for k, v in tp.items():
            add(f'      <param name="{k}" value="{v!r}"/>')
        add('    </appearance>')
    add('  </armpilot>')

    add('</robot>')
    return "\n".join(L) + "\n"


def main(argv: Sequence[str] | None = None) -> int:
    ensure_utf8_stdout()
    ap = argparse.ArgumentParser(description="从 robot.yaml + physics.yaml 生成标准 URDF")
    ap.add_argument("-o", "--out", default=str(default_out()),
                    help="输出路径（缺省 = 本包 manifest 声明的 model.urdf）")
    ap.add_argument("--stdout", action="store_true", help="只打印，不写文件")
    ap.add_argument("--robot", default=None, help="robot.yaml 路径")
    ap.add_argument("--physics", default=None, help="physics.yaml 路径")
    args = ap.parse_args(argv)

    robot = load_robot(args.robot)
    physics = load_physics(args.physics)
    urdf = build_urdf(robot, physics)

    if args.stdout:
        sys.stdout.write(urdf)
        return 0

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(urdf, encoding="utf-8", newline="\n")
    rel = out.relative_to(PROJECT_ROOT) if PROJECT_ROOT in out.resolve().parents else out
    print(f"[gen_urdf] 已生成 {rel}  ({len(urdf)} bytes)")
    print(f"[gen_urdf] 连杆 {len(robot.links)} · 自由度 {len(robot.movable_joints())} · "
          f"被动关节 {len(robot.passive_joints())} · 执行器 {len(robot.actuators)}")
    for j in robot.qpos_joints():
        lo, hi = _local_range_deg(robot, j)
        tag = "被动" if j.is_passive else "    "
        print(f"[gen_urdf] {tag} {j.id:<9} 局部区间 {lo:+9.4f}..{hi:+9.4f}°"
              f"  世界绝对限位 {j.limit_min:+9.4f}..{j.limit_max:+9.4f}°")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
