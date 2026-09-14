"""冻结守卫：「运动学 + 物理」真值不得在无意识的情况下被改动。

用户要求「保存目前的运动学物理数据不变」。本文件把那句话变成**机器可检查**的，
并且 —— 更重要的 —— 检查**守卫本身是否真的有辨识力**。

为什么必须测"守卫有辨识力"
--------------------------
一条永远返回 `assert True` 的守卫，和没有守卫在报告里长得一模一样（都是绿色）。
所以除了「真值没变」之外，还必须证明：
  · 改**外观几何** ⇒ 放行（否则做视觉建模时守卫会被绕过或删掉）
  · 改**运动学/物理** ⇒ 报错

这些变异全部在**内存里**做（`copy.deepcopy` 后改 dict），**不碰** 真值 yaml 文件 ——
测试不该有"跑挂了就把真值改坏"的尾部风险。

判据分级（见 core/tools/freeze_baseline.py 顶部说明）：
  L1 整文件 sha256       记录用
  L2 语义核心 sha256     真判据（只覆盖运动学/物理字段，剔除 geometry/details）

Phase 2：真值文件路径**不再硬编码**，一律问 `fb.paths()`（= 该包 manifest 的声明）。
"""
from __future__ import annotations

import copy
import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
#: 冻结守卫是 **Core 机制**（它盯的是"真值没被偷改"，不属任何一台机器人）⇒ 住 core/tools/。
FREEZE_SCRIPT = ROOT / "core" / "tools" / "freeze_baseline.py"


def _load_freeze():
    """从文件加载（core/tools/ 不是 Python 包，不能 import）。"""
    spec = importlib.util.spec_from_file_location("freeze_baseline", FREEZE_SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


fb = _load_freeze()

#: MeArm-V1 —— 冻结守卫服务的对象（显式写 id，与工具侧、conftest 同一条纪律）
MEARM_V1 = "mearm-v1"
ROBOT_YAML, PHYSICS_YAML, BASELINE_JSON = fb.paths(MEARM_V1)
#: 冻结基线 JSON 里 `files` 的键 = 真值文件的仓库相对路径（不写死）
ROBOT_REL, PHYSICS_REL = fb._rel_names(MEARM_V1)


def _robot_doc() -> dict:
    with ROBOT_YAML.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _physics_doc() -> dict:
    with PHYSICS_YAML.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _link_index(doc: dict) -> int:
    """取一个有 length 的连杆（base_link 的 length=0，拿它测"改长度"测不出东西）。"""
    for i, link in enumerate(doc["links"]):
        if float(link["length"]) > 0:
            return i
    raise AssertionError("robot.yaml 里没有任何 length>0 的连杆 —— 模型已不是这个机构了")


# ---------------------------------------------------------------------------
# 1. 核心判据：真值确实等于冻结基线
# ---------------------------------------------------------------------------
def test_baseline_file_exists():
    assert BASELINE_JSON.exists(), (
        f"冻结基线缺失：{BASELINE_JSON.relative_to(ROOT)}\n"
        "首次使用执行：python tools/freeze_baseline.py --update")


def test_kinematics_and_physics_match_frozen_baseline():
    problems = fb.check(MEARM_V1)
    assert not problems, (
        "运动学/物理真值已偏离冻结基线：\n" + "\n".join(problems))


# ---------------------------------------------------------------------------
# 2. ★ 守卫有辨识力：对外观不敏感、对真值敏感
# ---------------------------------------------------------------------------
def test_kinematic_core_ignores_presentation_geometry():
    """改外观（color / size / position / details）**不得**触发冻结违规。

    这是"图像采集 → 更真实的视觉建模"能推进的前提：视觉层必须可以自由改。
    """
    doc = _robot_doc()
    base = fb.digest(fb.robot_kinematic_core(doc))

    mutated = copy.deepcopy(doc)
    mutated["links"][0]["geometry"]["color"] = "#ff0000"           # 换配色
    mutated["links"][0]["geometry"]["size"] = [999, 999, 999]      # 换外形尺寸
    mutated["links"][0]["geometry"]["position"] = [1, 2, 3]        # 挪位置
    mutated["links"][0]["details"] = []                            # 删掉细节件

    assert fb.digest(fb.robot_kinematic_core(mutated)) == base, (
        "外观几何被算进了运动学核心摘要 —— 那么任何视觉建模都会被判违规，"
        "守卫会被绕过。请检查 KINEMATIC_LINK_KEYS 是否误含 geometry/details。")


@pytest.mark.parametrize("field, mutate, label", [
    ("length", lambda l: l.__setitem__("length", float(l["length"]) + 1.0), "连杆长度"),
    ("parent", lambda l: l.__setitem__("parent", "__changed__"), "父连杆"),
])
def test_kinematic_core_is_sensitive_to_link_topology(field, mutate, label):
    doc = _robot_doc()
    base = fb.digest(fb.robot_kinematic_core(doc))
    mutated = copy.deepcopy(doc)
    mutate(mutated["links"][_link_index(mutated)])
    assert fb.digest(fb.robot_kinematic_core(mutated)) != base, (
        f"改了{label}（links[].{field}）但运动学核心摘要没变 —— 守不住这一项")


def test_kinematic_core_is_sensitive_to_joint_limit_and_coupling():
    doc = _robot_doc()
    base = fb.digest(fb.robot_kinematic_core(doc))

    by_limit = copy.deepcopy(doc)
    by_limit["joints"][1]["limit"]["min"] = by_limit["joints"][1]["limit"]["min"] - 1.0
    assert fb.digest(fb.robot_kinematic_core(by_limit)) != base, "改了关节限位但摘要没变"

    by_coupling = copy.deepcopy(doc)
    elbow = next(j for j in by_coupling["joints"] if j["id"] == "elbow")
    assert "coupling" in elbow, "elbow 的耦合关系消失了 —— 绝对角语义被破坏"
    elbow["coupling"]["gain"] = -0.81
    assert fb.digest(fb.robot_kinematic_core(by_coupling)) != base, (
        "改了 coupling.gain（小臂绝对角语义的核心）但摘要没变")


def test_kinematic_core_is_sensitive_to_actuator_calibration():
    """标定值必须冻结 —— 它们由实拍反解得到，改了等于换了一台机器。"""
    doc = _robot_doc()
    base = fb.digest(fb.robot_kinematic_core(doc))
    mutated = copy.deepcopy(doc)
    for act in mutated["actuators"]:
        if act["id"] == "servo_7":
            act["offset"] = float(act["offset"]) + 1.0
            break
    else:
        raise AssertionError("找不到 servo_7 —— 执行器表已不是这个机构了")
    assert fb.digest(fb.robot_kinematic_core(mutated)) != base, (
        "改了舵机标定 offset 但运动学核心摘要没变")


def test_physics_core_is_sensitive_to_gravity_and_friction():
    doc = _physics_doc()
    base = fb.digest(fb.physics_core(doc))

    by_gravity = copy.deepcopy(doc)
    by_gravity["gravity"] = [0.0, 0.0, -9.80]
    assert fb.digest(fb.physics_core(by_gravity)) != base, "改了重力但物理核心摘要没变"

    by_friction = copy.deepcopy(doc)
    by_friction["contact"]["friction"] = [99.0, 99.0, 99.0]
    assert fb.digest(fb.physics_core(by_friction)) != base, "改了摩擦但物理核心摘要没变"

    by_timestep = copy.deepcopy(doc)
    by_timestep["timestep"]["physics"] = 0.002
    assert fb.digest(fb.physics_core(by_timestep)) != base, "改了物理步长但摘要没变"


def test_physics_core_ignores_non_physics_sections():
    """`recording` / `deterministic` 是 I/O 与运行配置，不是物理量 —— 改动应放行。"""
    doc = _physics_doc()
    base = fb.digest(fb.physics_core(doc))
    mutated = copy.deepcopy(doc)
    mutated["recording"]["enabled"] = not bool(mutated["recording"].get("enabled", False))
    mutated["deterministic"]["seed"] = 12345
    assert fb.digest(fb.physics_core(mutated)) == base, (
        "recording / deterministic 被算进了物理核心 —— 改个记录开关就报违规，"
        "守卫会变得烦人到被绕过")


# ---------------------------------------------------------------------------
# 3. 报错必须可读：只说"某个哈希变了"帮不上任何忙
# ---------------------------------------------------------------------------
def test_diff_summary_names_the_changed_field():
    base = fb.snapshot(MEARM_V1)
    cur = copy.deepcopy(base)
    cur["kinematic_summary"]["links"][_link_index(cur["kinematic_summary"])]["length"] += 1.0
    cur["physics_summary"]["gravity"] = [0.0, 0.0, -9.80]     # 标量列表分支

    lines = fb._diff_summary(base, cur)
    text = "\n".join(lines)
    assert "length" in text, f"差异摘要没指出改的是 length：\n{text}"
    assert "gravity" in text, f"差异摘要没指出改的是 gravity：\n{text}"


# ---------------------------------------------------------------------------
# 4. 基线确实来自**当前**真值（防"基线是某份陈旧副本造出来的"）
# ---------------------------------------------------------------------------
def test_baseline_summary_is_consistent_with_live_config(robot, physics):
    base = fb._read_baseline(MEARM_V1)
    assert base is not None

    for jid in robot.joint_order():
        joint = robot.joint(jid)
        entry = next((j for j in base["kinematic_summary"]["joints"] if j["id"] == jid), None)
        assert entry is not None, f"基线里缺关节 {jid}"
        assert entry["limit"]["min"] == pytest.approx(float(joint.limit_min), abs=1e-9)
        assert entry["limit"]["max"] == pytest.approx(float(joint.limit_max), abs=1e-9)

    live_gravity = [float(x) for x in fb._load_yaml(PHYSICS_YAML)["gravity"]]
    assert base["physics_summary"]["gravity"] == live_gravity
    # 键名 = 真值文件的仓库相对路径（Phase 2 起随包走），**不写死**：
    # 写死会在搬迁后变成一个"找不到键"的假故障，或者更糟 —— 读到一份陈旧的同名键。
    assert base["robot_id"] == MEARM_V1
    assert base["files"][PHYSICS_REL]["physics_core_sha256"] == \
        fb.snapshot(MEARM_V1)["files"][PHYSICS_REL]["physics_core_sha256"]
    assert base["files"][ROBOT_REL]["kinematic_core_sha256"] == \
        fb.snapshot(MEARM_V1)["files"][ROBOT_REL]["kinematic_core_sha256"]
