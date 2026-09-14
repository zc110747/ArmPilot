"""从 `config/robot.yaml` + `config/physics.yaml` 生成 MuJoCo MJCF（`mearm.xml`）。

用法::

    python simulation/mujoco/gen_model.py              # 写入 mearm.xml
    python simulation/mujoco/gen_model.py --stdout     # 打印到标准输出
    python simulation/mujoco/gen_model.py -o /tmp/m.xml

设计要点
--------
**为什么是"生成"而不是"手写 XML"**
    spec §4/§11 要求"机械臂参数只有一个来源"。本项目的那个来源是
    `config/robot.yaml`（前端、Go 后端、固件三方共用，且已有在线互检机制）。
    若手写一份 XML，参数真值立刻变成两份 —— 改一处忘一处，且**不会报错**。
    因此：yaml 是唯一真值，`mearm.xml` 是**可重复生成的产物**；
    **禁止手工编辑 mearm.xml**（重跑本脚本即覆盖）。

**body.pos 的推导（与 FK 逐项对应）**
    前端 `fk.ts`：
        T_J = T_parent · Tz(parentLink.length) · T(origin.position)
                       · R_eulerXYZ(origin.rotation) · R_axis(θ)
    MuJoCo：
        body_frame = parent_frame · T(body.pos) · R(body.quat) · R_axis(θ)
    逐项对应 ⇒
        body.pos  = origin.position + [0, 0, parentLink.length]
        body.quat = eulerXYZ(origin.rotation)   （本项目当前全为 0，但一般化实现）

**关节角语义**
    MuJoCo 的 hinge `qpos` 是**局部角**，而 robot.yaml 的 `elbow` 是**绝对倾角**。
    转换在 `units.JointAngleMap` 一处完成；本脚本只负责把 **range** 也换算到局部空间
    （否则限位会被解释错，见 ARCHITECTURE_ANALYSIS §6.3）。
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


def _find_repo_root(start: Path) -> Path:
    """向上找同时含 `core/` 与 `robot-package/` 的那一层。

    刻意不用 `parents[N]`：本脚本从 `simulation/mujoco/` 搬进
    `robot-package/mearm-v1/tools/` 后，深度常量会静默指向另一个**真实存在**的目录，
    报错会推迟到很远的地方才出现。向上找标记则在任意深度都成立。
    """
    for anc in (start, *start.parents):
        if (anc / "core").is_dir() and (anc / "robot-package").is_dir():
            return anc
    raise SystemExit(
        f"✗ 无法从 {start} 向上找到仓库根（需同时存在 core/ 与 robot-package/ 目录）")


_REPO_ROOT = _find_repo_root(Path(__file__).resolve().parent)

#: 依赖**只加指向目录**，不加本脚本自己的目录 —— 本脚本已经不住在 Core 里了：
#: `robotcfg` / `units` 是 Core 的仿真层（spec §2：包依赖 Core，Core 不认识包）。
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

#: 本生成器**只**服务 MeArm-V1（SO-101 的 MJCF 是官方原样文件，不经生成）。
MEARM_V1 = "mearm-v1"


def default_out() -> Path:
    """缺省输出 = 本包 manifest 里 `simulation.mjcf` **声明**的那个文件。

    ★ 这一处是本轮搬迁的实质 bug：原先它写的是 `Path(__file__).parent / "mearm.xml"`，
    即"生成到我自己旁边"。脚本从 `simulation/mujoco/` 搬到包内 tools/ 后，
    这句话会**静默**变成"生成到 robot-package/mearm-v1/tools/mearm.xml" ——
    而 `simulation/mujoco/mearm.xml` 仍然存在（它才是仿真真正加载的那份）
    ⇒ 改了代码没生效，且没有任何报错。
    """
    return declared_path(MEARM_V1, "simulation.mjcf", must_exist=False)


def _rel(p: Path | str) -> str:
    """仓库相对路径（用于写进产物注释 / 人读的日志）；不在仓库内则原样返回。"""
    p = Path(p).resolve()
    try:
        return p.relative_to(_REPO_ROOT).as_posix()
    except ValueError:
        return str(p)


DEFAULT_SERVO_SIZE_MM = (22.8, 12.2, 22.5)


# ---------------------------------------------------------------------------
# 四元数小工具（与 three.js / fk.ts 的 Euler 'XYZ' 约定一致）
# ---------------------------------------------------------------------------


def _quat_mul(a: tuple[float, float, float, float], b: tuple[float, float, float, float]):
    """Hamilton 积，顺序 (w, x, y, z) —— 与 MuJoCo 的 quat 布局相同。"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_axis(axis: str, deg: float):
    h = deg2rad(deg) / 2.0
    c, s = math.cos(h), math.sin(h)
    if axis == "x":
        return (c, s, 0.0, 0.0)
    if axis == "y":
        return (c, 0.0, s, 0.0)
    return (c, 0.0, 0.0, s)


def _quat_normalize(q):
    n = math.sqrt(sum(v * v for v in q))
    if n == 0:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(v / n for v in q)  # type: ignore[return-value]


def _quat_euler_xyz_deg(rot: Sequence[float]):
    """`R = Rx(rx) · Ry(ry) · Rz(rz)`。

    与 `frontend/src/robot/kinematics/transform.ts::mat4EulerXYZ` **逐项等价**
    （已按列主序展开核对过，见该函数 m[0..10] 的表达式）。
    """
    rx, ry, rz = (float(v) for v in rot)
    if rx == 0 and ry == 0 and rz == 0:
        return (1.0, 0.0, 0.0, 0.0)
    return _quat_normalize(
        _quat_mul(_quat_mul(_quat_axis("x", rx), _quat_axis("y", ry)), _quat_axis("z", rz))
    )


def _fmt(v: float, nd: int = 9) -> str:
    """格式化浮点。

    ⚠️ 用 `%g` 而不是 `%.6f`：惯量这类量常在 1e-5~1e-6 量级，
    定点格式会把它截成 **"0"**，而 MuJoCo 对零惯量会直接报错
    （或更糟：静默产生一个不受力矩的关节）。`%g` 会自动切科学计数法，
    MuJoCo 的 XML 解析器完全支持。
    """
    if v == 0:
        return "0"
    return f"{v:.{nd}g}"


def _vec(v: Sequence[float], nd: int = 9) -> str:
    return " ".join(_fmt(float(x), nd) for x in v)


def _quat_str(q) -> str:
    return _vec(q, 8)


def _hex_to_rgba(color: str | None, alpha: float = 1.0) -> tuple[float, float, float, float]:
    if not color or not color.startswith("#") or len(color) != 7:
        return (0.6, 0.6, 0.6, alpha)
    r = int(color[1:3], 16) / 255.0
    g = int(color[3:5], 16) / 255.0
    b = int(color[5:7], 16) / 255.0
    return (r, g, b, alpha)


# ---------------------------------------------------------------------------
# 关节局部角范围（保守外接）
# ---------------------------------------------------------------------------


def local_range_deg(robot: RobotCfg, joint: JointCfg, padding: float) -> tuple[float, float]:
    """把关节的**绝对角**限位换算成 MuJoCo hinge 需要的**局部角**区间。

    `局部角 = 关节角 + gain × 被耦合关节角`
    ⇒ 区间端点要把被耦合关节拉到对侧极限（gain 为负时下限取对方 max）。
    """
    lo, hi = joint.limit_min, joint.limit_max
    if joint.coupling:
        other_id, gain = joint.coupling
        other = robot.joint(other_id)
        if gain >= 0:
            lo += gain * other.limit_min
            hi += gain * other.limit_max
        else:
            lo += gain * other.limit_max
            hi += gain * other.limit_min
    return lo - padding, hi + padding


def _axes_parallel(a: Sequence[float], b: Sequence[float], tol: float = 1e-9) -> tuple[bool, float]:
    """两根轴是否共线；共线时给出同向系数（+1 同向 / −1 反向）。"""
    na = math.sqrt(sum(float(x) * float(x) for x in a))
    nb = math.sqrt(sum(float(x) * float(x) for x in b))
    if na == 0.0 or nb == 0.0:
        raise ConfigError("关节 axis 为零向量")
    ua = [float(x) / na for x in a]
    ub = [float(x) / nb for x in b]
    cross = (
        ua[1] * ub[2] - ua[2] * ub[1],
        ua[2] * ub[0] - ua[0] * ub[2],
        ua[0] * ub[1] - ua[1] * ub[0],
    )
    if max(abs(c) for c in cross) > tol:
        return False, 0.0
    dot = sum(x * y for x, y in zip(ua, ub))
    return True, (1.0 if dot >= 0.0 else -1.0)


def absolute_lock_terms(robot: RobotCfg, joint: JointCfg) -> list[tuple[str, float]]:
    """求「该被动关节的**绝对角** = 锁定值」对应的 qpos 线性项。

    在串联网里，某关节坐标系的**绝对转角** = 从根到它、且转轴与它**共线**的全部关节的
    局部角之代数和；而 MuJoCo 的 `qpos` 恰好就是这些局部角。因此约束写成这些 qpos 的
    加权和恒等于锁定角。详见 `build_xml` 里 `<tendon>` / `<equality>` 那两段的说明。

    返回 `[(关节 id, 系数)]`，系数 = ±1（轴同向 / 反向）。走链到该关节为止。
    """
    if not joint.is_passive:
        raise ConfigError(f"{joint.id} 不是被动关节，无需锁定项")
    terms: list[tuple[str, float]] = []
    for _link, j in robot.chain_from_root():
        if j is None:
            continue
        if j.origin_rotation != (0.0, 0.0, 0.0):
            raise ConfigError(
                f"被动关节 {joint.id} 的锁定路径上遇到带 origin.rotation 的关节 {j.id} —— "
                f"此时绝对角不再是 qpos 的线性组合，请改用显式四杆建模"
            )
        parallel, sign = _axes_parallel(j.axis, joint.axis)
        if parallel:
            terms.append((j.id, sign))
        if j.id == joint.id:
            break
    if not terms or terms[-1][0] != joint.id:
        raise ConfigError(f"被动关节 {joint.id} 不在根可达的链上，无法生成锁定约束")
    if terms[-1][1] != 1.0:
        raise ConfigError(
            f"被动关节 {joint.id} 与其自身轴的方向判据异常（系数 {terms[-1][1]}）—— 请人工确认"
        )
    return terms


# ---------------------------------------------------------------------------
# 惯量
# ---------------------------------------------------------------------------


def inertial_xml(body_id: str, physics: PhysicsCfg, indent: str) -> str:
    """生成 `<inertial>`。

    ⚠️ physics.yaml 已声明**全 SI（米）**，故本函数**不得**再做 mm→m。
    早期版本在这里对 com / length 又乘了一次 1e-3，把 40mm 写成 40µm，
    惯量随之掉到 1e-11 —— 格式化后直接显示成 "0"，是个**完全静默**的错误。
    """
    spec = physics.body_inertia_of(body_id)
    mass = float(spec["mass"])
    com = [float(x) for x in (spec.get("com") or (0, 0, 0))]        # 米
    shape = str(spec.get("shape", "point"))

    if shape == "box":
        a, b, c = (float(x) for x in spec["size"])                  # 米
        ix = mass / 12.0 * (b * b + c * c)
        iy = mass / 12.0 * (a * a + c * c)
        iz = mass / 12.0 * (a * a + b * b)
    elif shape == "rod":
        length = float(spec["length"])                              # 米
        ix = iy = mass * length * length / 12.0
        iz = max(mass * 0.02 * 0.02 / 6.0, 1e-9)                    # 细杆绕自身轴，取极小值
    elif shape == "point":
        r = 0.02
        ix = iy = iz = mass * r * r / 6.0
    else:
        raise ConfigError(f"{body_id}: 未知 inertia shape {shape!r}")

    return (
        f'{indent}<inertial pos="{_vec(com)}" mass="{_fmt(mass)}" '
        f'diaginertia="{_vec([ix, iy, iz])}"/>'
    )


# ---------------------------------------------------------------------------
# 几何
# ---------------------------------------------------------------------------


def collision_geom_xml(body_id: str, physics: PhysicsCfg, indent: str) -> list[str]:
    """碰撞几何。尺寸与位置**已是米**（physics.yaml 全 SI），不再换算。"""
    shapes = physics.contact.get("shapes") or {}
    if body_id not in shapes:
        raise ConfigError(f"physics.yaml 的 contact.shapes 缺少 {body_id}")
    s = shapes[body_id]
    gtype = str(s["type"])
    pos = _vec([float(x) for x in (s.get("pos") or (0, 0, 0))])
    size = [float(x) for x in s["size"]]
    if gtype in ("capsule", "cylinder"):
        size_attr = _vec([size[0], size[1]])
    elif gtype == "box":
        size_attr = _vec(size)
    elif gtype == "sphere":
        size_attr = _fmt(size[0])
    else:
        raise ConfigError(f"{body_id}: 未知 collision 类型 {gtype!r}")

    # 可选 quat：允许把 primitive 的**默认朝向**转过去。
    # 用途（不是"视觉修模型"，是实打实的几何语义）：MuJoCo 的 capsule/cylinder
    # 默认沿 **+Z**，而夹爪的开合轴是局部的 +X。把爪的碰撞胶囊沿 X 摆放后，
    # 它绕自身轴旋转不改变占位 ⇒ gripper 开合不会让碰撞包络摆动。
    quat = s.get("quat")
    quat_attr = f' quat="{_quat_str([float(x) for x in quat])}"' if quat is not None else ""
    return [
        f'{indent}<geom name="{body_id}_coll" class="collision" type="{gtype}" '
        f'size="{size_attr}" pos="{pos}"{quat_attr} rgba="0.15 0.85 0.45 0.35"/>'
    ]


def _geom_spec_to_xml(
    spec: Mapping[str, Any],
    default_pos_mm: Sequence[float],
    indent: str,
    name_hint: str,
) -> list[str]:
    """把 robot.yaml 的显示几何规格翻译成 MuJoCo visual geom。

    轴约定差异（必须处理）：
      three.js 的 `CylinderGeometry` 沿 **+Y**，MuJoCo 的 `cylinder` 沿 **+Z**
      ⇒ 需在本地先补一个 `Rx(90°)`。
    """
    gtype = str(spec.get("type", "none"))
    if gtype == "none":
        return []
    if gtype == "jaw":
        # 夹爪在**渲染层是特例**：两片爪各绕自己的齿轮轴啮合开合，本体由轮廓挤出生成；
        # 而 MuJoCo 刚体树里只有一个 `jaw_link` body（`gripper` 是单铰链）—— 结构上
        # **无法一对一映射**，所以这里不生成 visual geom。两条理由：
        #   ① 它的物理占位由 `physics.yaml` 的 `contact.shapes[jaw_link]`（胶囊）承担，
        #      MuJoCo 会照常渲染该碰撞体 ⇒ viewer 里不会"凭空少一个件"，物理也不变；
        #   ② 若在这条通用通道里"猜一个盒子"，等于让生成器**发明几何** ——
        #      而生成器的职责只是翻译 yaml，几何真值必须来自配置文件。
        # 前端对应实现：`buildRobotObject3D.createGeometryObject` 的
        # `case 'none': case 'jaw': return null;`（爪另有 createGripperJaws）。
        return []

    pos = spec.get("position") or default_pos_mm
    rot = spec.get("rotation") or (0, 0, 0)
    rgba = _hex_to_rgba(spec.get("color"))
    attrs = f'rgba="{_vec(rgba, 3)}"'

    if gtype in ("plate", "box"):
        a, b, c = (mm2m(float(x)) for x in spec["size"])
        q = _quat_euler_xyz_deg(rot)
        return [
            f'{indent}<geom name="{name_hint}" class="visual" type="box" '
            f'size="{_vec([a / 2, b / 2, c / 2])}" pos="{_vec(mm2m(x) for x in pos)}" '
            f'quat="{_quat_str(q)}" {attrs}/>'
        ]
    if gtype == "cylinder":
        r = mm2m(float(spec["radius"]))
        h = mm2m(float(spec["height"]))
        # 先做 Y→Z 轴修正，再叠加声明的 rotation
        q = _quat_mul(_quat_euler_xyz_deg(rot), _quat_axis("x", 90.0))
        return [
            f'{indent}<geom name="{name_hint}" class="visual" type="cylinder" '
            f'size="{_vec([r, h / 2])}" pos="{_vec(mm2m(x) for x in pos)}" '
            f'quat="{_quat_str(q)}" {attrs}/>'
        ]
    if gtype == "sphere":
        r = mm2m(float(spec["radius"]))
        return [
            f'{indent}<geom name="{name_hint}" class="visual" type="sphere" '
            f'size="{_fmt(r)}" pos="{_vec(mm2m(x) for x in pos)}" {attrs}/>'
        ]
    if gtype == "servo":
        s = spec.get("size") or DEFAULT_SERVO_SIZE_MM
        a, b, c = (mm2m(float(x)) for x in s)
        q = _quat_euler_xyz_deg(rot)
        return [
            f'{indent}<geom name="{name_hint}" class="visual" type="box" '
            f'size="{_vec([a / 2, b / 2, c / 2])}" pos="{_vec(mm2m(x) for x in pos)}" '
            f'quat="{_quat_str(q)}" {attrs}/>'
        ]
    raise ConfigError(f"未知显示几何类型 {gtype!r}")


def visual_geoms_xml(link: LinkCfg, indent: str) -> list[str]:
    out: list[str] = []
    geom = link.geometry or {}
    if geom.get("type") not in (None, "none"):
        out += _geom_spec_to_xml(geom, (0.0, 0.0, link.length / 2.0), indent, f"{link.id}_vis")
    for i, d in enumerate(link.details or ()):
        out += _geom_spec_to_xml(d, (0.0, 0.0, 0.0), indent, f"{link.id}_vis{i}")
    return out


# ---------------------------------------------------------------------------
# 主生成
# ---------------------------------------------------------------------------


def build_xml(robot: RobotCfg, physics: PhysicsCfg) -> str:
    L: list[str] = []
    add = L.append
    servo = physics.servo
    srv_damping = float(servo["damping"])
    srv_friction = float(servo["frictionloss"])
    srv_armature = float(servo["armature"])
    max_torque = float(servo["max_torque_nm"])
    kp = float(servo["kp"])
    kv = float(servo["kv"])
    padding = float(physics.limits.get("range_padding_deg", 0.0))

    contact = physics.contact
    fr = [float(x) for x in contact["friction"]]
    while len(fr) < 3:                       # MuJoCo 的 friction 需要 3 个分量
        fr.append(0.0)
    friction = _vec(fr[:3], 6)
    solref = _vec([float(x) for x in contact["solref"]])
    solimp = _vec([float(x) for x in contact["solimp"]])
    margin = _fmt(float(contact.get("margin", 0.0)))
    condim = int(contact.get("condim", 3))
    groups = contact.get("groups") or {}
    g_world = int(groups.get("world", 1))
    g_arm = int(groups.get("arm", 2))
    aff_world = g_world | g_arm
    aff_arm = g_world | g_arm

    gx, gy, gz = physics.gravity

    # ---- 头部 -----------------------------------------------------------
    # 路径一律**从实际加载到的文件反推**，不写死字符串：写死的话，真值随包搬走后
    # 产物里会留下一条指向不存在文件的"重新生成"指令（本轮搬迁实测到了这一条）。
    add('<?xml version="1.0" encoding="utf-8"?>')
    add("<!--")
    add(f"  ⚠️ 本文件由 {_rel(Path(__file__))} 生成 —— 请勿手工编辑。")
    add(f"     真值来源: {_rel(robot.source_path)} ({robot.id} / {robot.name}, {len(robot.links)} 连杆 / "
        f"{len(robot.movable_joints())} 自由度 + {len(robot.passive_joints())} 被动关节 / "
        f"{len(robot.qpos_joints())} 个 qpos 坐标)")
    add(f"     物理参数: {_rel(physics.source_path)} ({physics.source_path.name})")
    add(f"     重新生成: python {_rel(Path(__file__))}")
    add("-->")
    add(f'<mujoco model="{robot.id}">')
    add(f'  <compiler angle="radian" autolimits="true"/>')
    add(
        f'  <option timestep="{_fmt(physics.ts_physics)}" '
        f'integrator="{physics.solver.get("integrator", "implicitfast")}" '
        f'solver="{physics.solver.get("solver", "Newton")}" '
        f'iterations="{int(physics.solver.get("iterations", 80))}" '
        f'tolerance="{_fmt(float(physics.solver.get("tolerance", 1e-10)), 12)}" '
        f'gravity="{_vec([gx, gy, gz])}"/>'
    )

    # ---- 默认类 ----------------------------------------------------------
    add("  <default>")
    add(f'    <geom friction="{friction}" solref="{solref}" solimp="{solimp}" '
        f'margin="{margin}" condim="{condim}" density="0"/>')
    add(f'    <default class="world">')
    add(f'      <geom contype="{g_world}" conaffinity="{aff_world}" group="1" density="0"/>')
    add(f'    </default>')
    add(f'    <default class="visual">')
    add(f'      <geom contype="0" conaffinity="0" group="2" density="0" condim="1" '
        f'margin="0" solref="0.02 1"/>')
    add(f'    </default>')
    add(f'    <default class="collision">')
    add(f'      <geom contype="{g_arm}" conaffinity="{aff_arm}" group="3" density="0"/>')
    add(f'    </default>')
    add(f'    <default class="arm">')
    add(f'      <joint type="hinge" damping="{_fmt(srv_damping)}" '
        f'frictionloss="{_fmt(srv_friction)}" armature="{_fmt(srv_armature)}" limited="true"/>')
    add(f'      <position kp="{_fmt(kp)}" kv="{_fmt(kv)}" '
        f'forcerange="{-max_torque} {max_torque}"/>')
    add(f'    </default>')
    add("  </default>")

    # ---- 世界 -----------------------------------------------------------
    add('  <worldbody>')
    floor = contact.get("floor") or {}
    if floor.get("enabled", True):
        fsize = _vec([float(x) for x in floor.get("size", [0.6, 0.6, 0.01])])
        fz = float(floor.get("z", 0.0))
        add(f'    <geom name="floor" class="world" type="plane" size="{fsize}" '
            f'pos="0 0 {_fmt(fz)}" rgba="0.45 0.47 0.5 1"/>')
    table = contact.get("table") or {}
    if table.get("enabled", False):
        tsize = [float(x) for x in table.get("size", [0.12, 0.12, 0.005])]
        tpos = [float(x) for x in table.get("pos", [0.12, 0.0, 0.005])]
        add(f'    <body name="table" pos="{_vec(tpos)}">')
        add(f'      <geom name="table_top" class="world" type="box" size="{_vec(tsize)}" '
            f'rgba="0.62 0.55 0.42 1"/>')
        add(f'    </body>')

    # ---- 刚体树 ----------------------------------------------------------
    chain = robot.chain_from_root()
    # 记录每个 body 的深度（用于缩进）与父子关系
    depth_of: dict[str, int] = {robot.root_link().id: 2}
    for link, joint in chain:
        if joint is None:
            continue
        depth_of[link.id] = depth_of[joint.parent_link] + 1

    emitted: set[str] = set()

    def emit(link: LinkCfg, joint: JointCfg | None) -> None:
        d = "  " * depth_of[link.id]
        if joint is None:
            add(f'{d}<body name="{link.id}" pos="0 0 0">')
        else:
            parent_link = robot.link(joint.parent_link)
            pos_mm = (
                joint.origin_position[0],
                joint.origin_position[1],
                joint.origin_position[2] + parent_link.length,
            )
            quat = _quat_euler_xyz_deg(joint.origin_rotation)
            if joint.origin_rotation != (0.0, 0.0, 0.0):
                add(f'{d}<body name="{link.id}" pos="{_vec(mm2m(x) for x in pos_mm)}" '
                    f'quat="{_quat_str(quat)}">')
            else:
                add(f'{d}<body name="{link.id}" pos="{_vec(mm2m(x) for x in pos_mm)}">')

        inner = d + "  "
        add(inertial_xml(link.id, physics, inner))

        if joint is not None and not joint.is_fixed:
            lo, hi = local_range_deg(robot, joint, padding)
            add(
                f'{inner}<joint name="{joint.id}" class="arm" axis="{_vec(joint.axis)}" '
                f'range="{_fmt(deg2rad(lo))} {_fmt(deg2rad(hi))}"/>'
            )
        for line in collision_geom_xml(link.id, physics, inner):
            add(line)
        for line in visual_geoms_xml(link, inner):
            add(line)

        # TCP site：挂在该 link 的原点上（tcp.joint 指定的关节所在的 link）
        if robot.tcp_joint and joint is not None and joint.id == robot.tcp_joint:
            off = [mm2m(float(x)) for x in robot.tcp_offset]
            add(f'{inner}<site name="tcp" pos="{_vec(off)}" size="0.004" '
                f'rgba="1 0.85 0.2 1" group="4"/>')

        for child_joint in robot.children_of(link.id):
            child_link = robot.link(child_joint.child_link)
            if child_link.id in emitted:
                raise ConfigError(f"连杆 {child_link.id} 被重复生成（图不是树？）")
            emitted.add(child_link.id)
            emit(child_link, child_joint)

        add(f"{d}</body>")

    emitted.add(robot.root_link().id)
    emit(robot.root_link(), None)
    add("  </worldbody>")

    # ---- 接触排除 --------------------------------------------------------
    pairs = contact.get("exclude_pairs") or []
    if pairs:
        add("  <contact>")
        for a, b in pairs:
            add(f'    <exclude body1="{a}" body2="{b}"/>')
        add("  </contact>")

    # ---- 执行器 ----------------------------------------------------------
    add("  <actuator>")
    for joint in robot.movable_joints():
        lo, hi = local_range_deg(robot, joint, padding)
        for a in robot.actuators_for(joint.id):
            add(
                f'    <position name="servo_{a.channel}" class="arm" joint="{joint.id}" '
                f'ctrlrange="{_fmt(deg2rad(lo))} {_fmt(deg2rad(hi))}"/>'
            )
    add("  </actuator>")

    # ---- 被动关节的锁定约束 ---------------------------------------------------
    # 被动关节（本机的腕 `tool`）在 MJCF 里**必须**建模成 hinge —— 它是个真实存在的
    # 转动副，爪就是被它带着保持水平的。但它**没有输入**：它的**绝对角**被连杆锁死。
    #
    # ⚠️ 约束的写法是本题最容易错的一处，踩过坑（见 docs/decisions.md D70）：
    #   在串联网里，某关节坐标系的**绝对转角** = 从根到它、且转轴与它**平行同向**的
    #   全部关节的**局部角**之和。而 MuJoCo 的 `qpos` **恰好就是这些局部角**
    #   （见 `units.JointAngleMap.to_qpos`：`局部角 = 关节角 + gain × 被耦合关节角`）。
    #   所以约束是「这一串 qpos 的**加权和** = 锁定角」，而不是「它自己 = 锁定角 − 某一个关节角」。
    #
    #   第一版按 `<equality><joint joint1="tool" joint2="elbow">` 写，表达成
    #   `q_tool = 90° − q_elbow`。但 `q_elbow` 是**局部角**（= θe − θs），少了一项，
    #   于是稳态恰好偏在「离锁定值一个 shoulder 角（0.8499°）」的地方 ——
    #   偏差数值与当时的肩角一模一样，是"漏项"的典型指纹，但看上去完全像"约束太软"。
    #   实测把 solref 收紧 10 倍，残差**一点没变**（+0.8499°），只多出 20° 的过冲。
    #
    #   正解：`<fixed>` 腱把这一串关节的 qpos 线性组合成一个标量，再用
    #   `<equality><tendon>` 把它钉在锁定角上。
    passive_joints = robot.passive_joints()
    if passive_joints:
        add("  <tendon>")
        for j in passive_joints:
            add(f'    <fixed name="{j.id}_lock_tendon">')
            for jid, coef in absolute_lock_terms(robot, j):
                add(f'      <joint joint="{jid}" coef="{_fmt(coef)}"/>')
            add("    </fixed>")
        add("  </tendon>")
        add("  <equality>")
        for j in passive_joints:
            add(
                f'    <tendon name="{j.id}_lock" tendon1="{j.id}_lock_tendon" '
                f'polycoef="{_fmt(deg2rad(j.limit_min), 12)} 0 0 0 0"/>'
            )
        add("  </equality>")

    add("</mujoco>")
    return "\n".join(L) + "\n"


def main(argv: Sequence[str] | None = None) -> int:
    ensure_utf8_stdout()
    ap = argparse.ArgumentParser(description="从 robot.yaml + physics.yaml 生成 MuJoCo MJCF")
    ap.add_argument("-o", "--out", default=str(default_out()),
                    help="输出路径（缺省 = 本包 manifest 声明的 simulation.mjcf）")
    ap.add_argument("--stdout", action="store_true", help="只打印，不写文件")
    ap.add_argument("--robot", default=None,
                    help="robot.yaml 路径（缺省 = 该包 manifest 的 model.config）")
    ap.add_argument("--physics", default=None,
                    help="physics.yaml 路径（缺省 = 该包 manifest 的 model.physics）")
    args = ap.parse_args(argv)

    robot = load_robot(args.robot)
    physics = load_physics(args.physics)
    padding = float(physics.limits.get("range_padding_deg", 0.0))
    xml = build_xml(robot, physics)

    if args.stdout:
        sys.stdout.write(xml)
        return 0

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(xml, encoding="utf-8", newline="\n")
    rel = out.relative_to(PROJECT_ROOT) if PROJECT_ROOT in out.resolve().parents else out
    print(f"[gen_model] 已生成 {rel}  ({len(xml)} bytes)")
    print(f"[gen_model] 连杆 {len(robot.links)} · 自由度 {len(robot.movable_joints())} · "
          f"被动关节 {len(robot.passive_joints())} · qpos 坐标 {len(robot.qpos_joints())} · "
          f"执行器 {len(robot.actuators)}")
    for j in robot.qpos_joints():
        lo, hi = local_range_deg(robot, j, padding)
        tag = "被动" if j.is_passive else "    "
        print(f"[gen_model] {tag} {j.id:<9} 绝对限位 {j.limit_min:+9.4f}..{j.limit_max:+9.4f}°"
              f"  →  局部 range {lo:+9.4f}..{hi:+9.4f}°")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
