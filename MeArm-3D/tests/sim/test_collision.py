# -*- coding: utf-8 -*-
"""Phase 5 验收：碰撞 / 摩擦 / 自碰撞 / 接触稳定性。

对应 spec：
  §15 碰撞场景（臂↔地面 / 臂↔自身 / 臂↔工作台）
  §16 自碰撞用 exclude + contype/conaffinity 逐步开启
  §17 合理摩擦（**禁止**设成无限大）
  §36 Acceptance 7（碰撞验证）

判据纪律（这两条是血泪换来的，别简化）：

  * **"接触表里有记录"不是证据。** MuJoCo 对 `dist > 0` 的软接触同样会生成
    contact 记录 —— 实测下压位形 `jaw_link↔table_top` 的 `dist = +1.44mm`
    却实实在在顶着 0.233 N·m 的反力矩。要证明"接触在施力"，必须看
    `data.qfrc_constraint`（关节上的约束反力矩）或执行器是否被顶到力矩上限。
    只断言 `data.ncon > 0` 的测试，把台面的碰撞体删掉也照样能过。

  * **"接触没了"同样不是证据。** 必须配一个**对照**：同一个命令下禁用台面后
    平衡位置显著不同，否则无法排除"台面本来就没起作用"。
"""
from __future__ import annotations

import numpy as np
import pytest

from conftest import SIM_DIR
from model import MeArmSim

# 相邻连杆的排除对：这些 body 在几何上**必然重叠**（枢轴处共用一段），
# 必须靠 <exclude> 抑制，否则会产生"自穿模"伪接触。
ADJACENT_EXCLUDE_MIN_OVERLAPS = 3


# ---------------------------------------------------------------------------
# 位形与辅助
# ---------------------------------------------------------------------------


def press_down_pose(robot) -> dict[str, float]:
    """最深的下压位形：肩 / 肘同时取真机上界（**从 robot.yaml 取，不写死数字**）。"""
    return {
        "base": 0.0,
        "shoulder": robot.joint("shoulder").limit_max,
        "elbow": robot.joint("elbow").limit_max,
        "gripper": 0.0,
    }


def geom_id(sim, name: str) -> int:
    import mujoco

    i = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, name)
    assert i >= 0, f"模型里找不到 geom {name}"
    return i


def table_geom(sim) -> int:
    return geom_id(sim, "table_top")


def set_table_enabled(sim, enabled: bool) -> None:
    """运行时开关工作台。

    这条**可以**运行时改：`geom_contype/conaffinity` 是在碰撞过滤时逐对查询的，
    不经过 broadphase 的 AABB 缓存，所以改动立即生效（实测禁用后 TCP 由
    44.05mm 掉到 15.79mm）。

    ⚠️ 但 `geom_pos` / `geom_size` 就**不能**这么改 —— 见 `variant_sim()`。
    """
    gid = table_geom(sim)
    sim.model.geom_contype[gid] = 1 if enabled else 0
    sim.model.geom_conaffinity[gid] = 3 if enabled else 0


def variant_sim(**geom_edits) -> MeArmSim:
    """用 MjSpec 在**加载期**改几何，产出一个变体仿真实例。

    ⚠️ 这是本项目踩过的坑，务必记住：
    运行时写 `model.geom_pos[...] = ...` / `model.geom_size[...] = ...` 之后，
    `mj_forward` **不会**重算静态 geom 的 `data.geom_xpos` 和 broadphase AABB，
    碰撞检测于是**完全无视**这次改动 —— 实测把工作台从 z=15mm 抬到 z=100mm，
    臂的稳态位置与接触对一字不变（仍是原台面在挡）。这些是**编译期**属性，
    只能在 `spec.compile()` 之前改。

    用法：`variant_sim(floor={"pos": [0, 0, 0.03]})`
    """
    import mujoco

    spec = mujoco.MjSpec.from_file(str(SIM_DIR / "mearm.xml"))
    for name, fields in geom_edits.items():
        g = spec.geom(name)
        for k, v in fields.items():
            setattr(g, k, v)
    return MeArmSim(xml_text=spec.to_xml())


def contact_pairs(sim) -> set[frozenset[str]]:
    """当前接触对，按 **body 名** 归一（geom 名到 body 名的映射由 MuJoCo 给出）。"""
    import mujoco

    out: set[frozenset[str]] = set()
    for k in range(sim.data.ncon):
        c = sim.data.contact[k]
        names = []
        for g in (c.geom1, c.geom2):
            b = int(sim.model.geom_bodyid[g])
            names.append(mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_BODY, b) or "?")
        out.add(frozenset(names))
    return out


def contact_dists(sim, body: str) -> list[float]:
    """某个 body 参与的所有接触的 `dist`（米）。"""
    import mujoco

    out = []
    for k in range(sim.data.ncon):
        c = sim.data.contact[k]
        bodies = [mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_BODY,
                                    int(sim.model.geom_bodyid[g])) or "?"
                  for g in (c.geom1, c.geom2)]
        if body in bodies:
            out.append(float(c.dist))
    return out


def geom_distance(sim, name1: str, name2: str, distmax: float = 1.0) -> float:
    """两个 geom 的**几何**最近距离（米）。负值 = 重叠。

    这是判断"排除对是否必要"的唯一可靠手段：接触表里看不到重叠，
    因为 <exclude> 恰好把记录抹掉了 —— 只看接触表会得出"根本不重叠"的错觉。
    """
    import mujoco

    fromto = np.zeros(6, dtype=np.float64)
    return float(mujoco.mj_geomDistance(sim.model, sim.data,
                                        geom_id(sim, name1), geom_id(sim, name2),
                                        distmax, fromto))


def min_geom_distance(sim, names_a, names_b, distmax: float = 1.0) -> float:
    return min(geom_distance(sim, a, b, distmax)
               for a in names_a for b in names_b)


def dof_index(sim, joint_id: str) -> int:
    """关节 id → 它在 `qvel` / `qfrc_*` 里的自由度下标（不是关节序号！）。"""
    import mujoco

    j = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
    return int(sim.model.jnt_dofadr[j])


# ---------------------------------------------------------------------------
# 1. 基线：不该有接触的地方必须一个接触都没有
# ---------------------------------------------------------------------------


def test_home_pose_has_no_spurious_contact(sim, robot):
    """HOME 位是**干净的悬空基线**：ncon == 0。

    这条守着一个实测过的建模缺陷：立柱的碰撞 capsule 曾经一路延伸到 z=0，
    与地面**相切**，于是 HOME 位永远挂着 1 个"立柱戳在地上"的伪接触
    （dist=0.000），污染 `contact_count` 并让所有接触类断言失去确定性。
    """
    sim.reset(dict(robot.home_pose))
    sim.settle(3.0)
    assert sim.data.ncon == 0, f"HOME 位出现了伪接触：{contact_pairs(sim)}"


def test_base_and_column_do_not_touch_ground(sim, robot):
    """底盘 / 立柱的碰撞体不得与地面接触（它们是固定体或位于底盘之上）。"""
    sim.reset(dict(robot.home_pose))
    sim.settle(3.0)
    pairs = contact_pairs(sim)
    assert frozenset({"world", "column_link"}) not in pairs
    assert frozenset({"world", "base_link"}) not in pairs


# ---------------------------------------------------------------------------
# 2. 自碰撞：exclude 的**必要性**与**有效性**（两侧都要证）
# ---------------------------------------------------------------------------


def test_adjacent_links_really_overlap(sim, robot):
    """exclude 的**必要性**：相邻连杆在几何上确实重叠。

    只断言"接触表里没有相邻对"，把 <exclude> 删掉、甚至把碰撞体全删掉，
    测试都会通过 —— 那是自证。必须先证明"不加 exclude 就一定会出伪接触"。
    """
    sim.reset(dict(robot.home_pose))
    pairs = [("base_link", "column_link"), ("column_link", "upper_arm_link"),
             ("upper_arm_link", "forearm_link"), ("forearm_link", "tool_link"),
             ("tool_link", "jaw_link")]
    overlaps = [geom_distance(sim, f"{a}_coll", f"{b}_coll") for a, b in pairs]
    n_overlap = sum(1 for d in overlaps if d < 0.0)
    assert n_overlap >= ADJACENT_EXCLUDE_MIN_OVERLAPS, (
        f"只有 {n_overlap} 对相邻连杆重叠（实测 {overlaps}）—— "
        f"若确实不重叠，就说明模型几何变了，exclude 的存在理由需要重新论证")


def test_excluded_pairs_never_appear_in_contacts(sim, robot, physics):
    """exclude 的**有效性**：扫描多个姿态，接触表里绝不能出现被排除的相邻对。"""
    excluded = {frozenset(p) for p in physics.contact["exclude_pairs"]}
    shoulder_max = robot.joint("shoulder").limit_max
    poses = [
        dict(robot.home_pose),
        press_down_pose(robot),
        {"base": 0.0, "shoulder": 0.0, "elbow": robot.joint("elbow").limit_min, "gripper": 0.0},
        {"base": 0.0, "shoulder": shoulder_max, "elbow": robot.joint("elbow").limit_min, "gripper": 0.0},
        {"base": robot.joint("base").limit_min, "shoulder": 20.0,
         "elbow": robot.joint("elbow").limit_min, "gripper": 0.0},
        {"base": robot.joint("base").limit_max, "shoulder": 20.0,
         "elbow": robot.joint("elbow").limit_min, "gripper": 0.0},
        {"base": 0.0, "shoulder": 30.0, "elbow": 130.0, "gripper": 0.0},
    ]
    for p in poses:
        sim.reset(p)
        sim.settle(1.5)
        bad = contact_pairs(sim) & excluded
        assert not bad, f"位形 {p} 出现被排除的相邻连杆伪接触：{bad}"


def test_collision_groups_match_config(sim, physics):
    """spec §16：contype/conaffinity 必须来自 physics.yaml 的 groups，不能是凑出来的。"""
    groups = physics.contact["groups"]
    world_mask, arm_mask = int(groups["world"]), int(groups["arm"])

    assert sim.model.geom_contype[geom_id(sim, "floor")] == world_mask
    assert sim.model.geom_contype[table_geom(sim)] == world_mask
    for body in ("base_link", "column_link", "upper_arm_link", "forearm_link",
                 "tool_link", "jaw_link"):
        gid = geom_id(sim, f"{body}_coll")
        assert sim.model.geom_contype[gid] == arm_mask, f"{body} 的 contype 与配置不符"

    # 世界 与 臂 必须在掩码意义上互相可碰：(A.contype & B.conaffinity) != 0
    floor_c, floor_a = int(sim.model.geom_contype[geom_id(sim, "floor")]), \
        int(sim.model.geom_conaffinity[geom_id(sim, "floor")])
    arm_c, arm_a = int(sim.model.geom_contype[geom_id(sim, "tool_link_coll")]), \
        int(sim.model.geom_conaffinity[geom_id(sim, "tool_link_coll")])
    assert (arm_c & floor_a) != 0 and (floor_c & arm_a) != 0


def test_visual_geoms_never_collide(sim):
    """Visual / Collision 分离（spec §6）：视觉几何必须完全退出碰撞检测。

    识别方式用 **geom group**（配置约定：1=世界 / 2=视觉 / 3=碰撞）。
    ⚠️ 不能用"名字是否以 `_coll` 结尾"来认 —— 那样会把 floor / table_top
    这些**世界几何**误判成视觉件（它们本来就该带碰撞掩码），断言立刻变红。
    """
    import mujoco

    n_vis = 0
    for i in range(sim.model.ngeom):
        if int(sim.model.geom_group[i]) != 2:
            continue
        n_vis += 1
        name = mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, i) or "?"
        assert int(sim.model.geom_contype[i]) == 0, f"视觉 geom {name} 的 contype 非 0"
        assert int(sim.model.geom_conaffinity[i]) == 0, f"视觉 geom {name} 的 conaffinity 非 0"
    assert n_vis > 0, "一个视觉几何都没找到，说明这个断言没测到东西"


# ---------------------------------------------------------------------------
# 3. 臂 ↔ 工作台（spec §15）
# ---------------------------------------------------------------------------


def test_arm_contacts_table_when_pressed_down(sim, robot, physics):
    """深前倾下压 ⇒ 臂端与工作台建立接触，且接触点落在台面顶面上。"""
    sim.reset(press_down_pose(robot))
    sim.settle(4.0)
    pairs = contact_pairs(sim)
    hit = [p for p in pairs if "table" in p and ({"tool_link", "jaw_link"} & p)]
    assert hit, f"下压后未与工作台接触：{pairs}"

    table = physics.contact["table"]
    top_z = float(table["pos"][2]) + float(table["size"][2])
    # 接触点必须贴着台面顶面（几何一致性）
    zs = [float(sim.data.contact[k].pos[2]) for k in range(sim.data.ncon)]
    assert min(abs(z - top_z) for z in zs) < 0.003, (
        f"接触点离台面顶面太远：z={[round(z * 1000, 2) for z in zs]}mm, 顶面={top_z * 1000}mm")


def test_table_exerts_real_constraint_force(sim, robot, physics):
    """★ 台面**真的在施力** —— 用约束反力矩与执行器满力矩证明。

    这是"不许伪造真实物理"（spec §37）最直接的落点：
    若台面的碰撞体被删掉，`qfrc_constraint` 会归零、执行器也不再顶满，
    这条断言立刻失败。
    """
    sim.reset(press_down_pose(robot))
    sim.settle(4.0)

    tau_c = np.array(sim.data.qfrc_constraint, dtype=float)
    sh, el = dof_index(sim, "shoulder"), dof_index(sim, "elbow")
    assert abs(tau_c[sh]) > 0.05, f"肩关节上没有约束反力矩（{tau_c[sh]}）—— 接触没在施力"
    assert abs(tau_c[el]) > 0.02, f"肘关节上没有约束反力矩（{tau_c[el]}）"

    # 舵机被顶到力矩上限：说明它"全力往下压但压不动"
    tau_a = abs(float(sim.data.qfrc_actuator[sh]))
    assert tau_a == pytest.approx(float(physics.servo["max_torque_nm"]), rel=0.02), (
        f"肩执行器力矩 {tau_a} 未达到上限 {physics.servo['max_torque_nm']}")


def test_table_changes_equilibrium(sim, robot):
    """★ 对照：同一命令下，禁用台面后平衡位置必须显著更低。

    "有接触"是必要条件不是充分条件 —— 必须证明**台面改变了结果**。
    """
    pose = press_down_pose(robot)
    sim.reset(pose)
    sim.settle(4.0)
    z_with = float(sim.end_effector_mm()[2])

    sim.reset(pose)
    set_table_enabled(sim, False)
    sim.settle(4.0)
    z_without = float(sim.end_effector_mm()[2])

    assert z_with - z_without > 20.0, (
        f"台面几乎没有改变平衡位置（有 {z_with:.2f}mm / 无 {z_without:.2f}mm）—— "
        f"这个对照失效了")


def test_no_deep_penetration_under_press(sim, robot):
    """软接触下不得出现深层穿透（spec §15 的物理合理性）。"""
    sim.reset(press_down_pose(robot))
    sim.settle(4.0)
    ds = [float(sim.data.contact[k].dist) for k in range(sim.data.ncon)]
    assert ds, "下压位形没有接触"
    assert min(ds) > -0.002, f"出现 {min(ds) * 1000:.2f}mm 的深层穿透"


def test_table_never_touches_base_or_column(sim, robot):
    """台面从 x=60mm 起，必须避开底盘包络（否则初始就顶住仿真）。"""
    for base in (robot.joint("base").limit_min, 0.0, robot.joint("base").limit_max):
        sim.reset({"base": base, "shoulder": 0.0, "elbow": robot.joint("elbow").limit_min,
                   "gripper": 0.0})
        sim.settle(2.0)
        pairs = contact_pairs(sim)
        assert not any("table" in p and ({"base_link", "column_link"} & p) for p in pairs), \
            f"base={base}° 时工作台碰到了底盘/立柱：{pairs}"


# ---------------------------------------------------------------------------
# 4. 臂 ↔ 地面（spec §15）：通路验证 + 能力边界固化
# ---------------------------------------------------------------------------


def test_ground_collision_channel_works(robot):
    """"臂↔地面"的通路验证：把地面抬进可达范围，碰撞必须立刻生效。

    地面与工作台走**完全相同**的 geom / contact / solver 代码路径，
    区别只是位置 —— 所以用"抬高地面"来验证通路，而不是去改臂的几何。

    ⚠️ 抬高地面必须**重新加载模型**（`variant_sim`）：运行时改 `geom_pos`
    不会被 `mj_forward` 采纳，实测改完地面纹丝不动（见 `variant_sim` 的说明）。
    """
    sim = variant_sim(floor={"pos": [0.0, 0.0, 0.030]})
    set_table_enabled(sim, False)
    sim.reset(press_down_pose(robot))
    sim.settle(4.0)
    pairs = contact_pairs(sim)
    assert any("world" in p and ({"tool_link", "jaw_link"} & p) for p in pairs), \
        f"地面抬高后仍未发生接触：{pairs}"


def test_arm_cannot_reach_ground_within_real_limits(sim, robot):
    """★ 能力边界固化：真机限位内，臂的碰撞包络**够不到地面**。

    这不是缺陷，是机构自身的几何事实 —— 最深的姿态（肩/肘同时取上界）
    TCP 也只到 ~15.8mm，而 tool 的碰撞 capsule 半径 12mm ⇒ 最低点约 3.8mm。
    把它写成断言，是为了防止有人"顺手"放宽限位或加大碰撞体来让测试变绿。
    """
    set_table_enabled(sim, False)
    sim.reset(press_down_pose(robot))
    sim.settle(6.0, tolerance_rad=1e-4, hold_s=0.5)

    d = min_geom_distance(sim, ("tool_link_coll", "jaw_link_coll"), ("floor",))
    assert d > 0.0, f"臂竟然碰到了地面（{d * 1000:.2f}mm）—— 能力边界已变，请复核"
    assert d < 0.010, f"离地面 {d * 1000:.2f}mm，比预期远太多（预期几毫米）"
    assert sim.data.ncon == 0, f"无台面时应无接触，实测 {contact_pairs(sim)}"


# ---------------------------------------------------------------------------
# 5. 摩擦（spec §17）与接触稳定性
# ---------------------------------------------------------------------------


def test_friction_is_finite_and_comes_from_config(sim, physics):
    """摩擦必须**有限**且**逐位等于配置值**（spec §17：禁止无限大）。"""
    cfg = [float(x) for x in physics.contact["friction"]]
    assert cfg[0] > 0.0, "滑动摩擦必须为正"
    assert cfg[0] < 10.0, "滑动摩擦大得不像话（接近「无限大」）—— 违反 spec §17"

    fr = np.array(sim.model.geom_friction, dtype=float)
    uniq = {tuple(np.round(r, 12)) for r in fr}
    assert len(uniq) == 1, f"模型里出现了 {len(uniq)} 种不同的摩擦组合：{sorted(uniq)}"
    assert tuple(cfg) == pytest.approx(tuple(uniq.pop()), rel=1e-12)


def test_contact_response_is_stable(sim, robot):
    """压在工作台上持续 5s：位置漂移要小、接触要稳定、不出 NaN。"""
    sim.reset(press_down_pose(robot))
    sim.settle(4.0)
    q0 = np.array(sim.data.qpos, dtype=float)
    n0 = sim.data.ncon

    sim.step_seconds(5.0)

    q1 = np.array(sim.data.qpos, dtype=float)
    assert np.isfinite(q1).all(), "积分发散了（qpos 含 NaN/Inf）"
    drift_deg = float(np.max(np.abs(q1 - q0))) * 180.0 / np.pi
    assert drift_deg < 0.1, f"接触下 5s 内漂移了 {drift_deg:.4f}° —— 接触不稳定"
    assert sim.data.ncon == n0, "接触数在静止保持期间发生了变化"


def test_settle_is_reachable_under_contact(sim, robot):
    """接触位形的 `settle()` 必须能真正收敛（否则后继测试都在读未稳态）。"""
    sim.reset(press_down_pose(robot))
    sim.settle(6.0, tolerance_rad=1e-4, hold_s=0.5)
    assert float(np.max(np.abs(sim.data.qvel))) < 1e-4
