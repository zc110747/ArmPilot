# -*- coding: utf-8 -*-
"""统一 Sim2Sim 回归矩阵（`runSim2Sim(robot)`）· pytest 侧。

## 这个文件回答的问题

> 「**同一套**判据，能不能同时覆盖两台完全不同的机器人？」

而不是"每台机器人各自有一套验收、各自都能绿" —— 后者正是 spec 要避免的：
两套标准各自自洽时，"抽象成立"这句话就没有判据了。

## 覆盖层次

| 层 | 判据 | 失败意味着 |
|---|---|---|
| 矩阵 | 选择器里的**每一台**都跑得出报告 | 新增机器人没人管 |
| 容差登记 | `fk_tolerance_mm(id)` 不抛错 | 有人悄悄给容差留了个"够大"的缺省 |
| FK 三侧面 | `前端↔参考`、`参考↔MuJoCo` 都在**该机器人登记的**容差内 | 几何/描述漂移 |
| 冻结结构 | 用例 id 列表、关节顺序、能力声明逐位一致 | 判据本身被改过 |
| 冻结数值 | tcp 三侧面与快照一致（前端/参考 1e-9、MuJoCo 1e-6） | 实现行为变了 |
| **能力诚实性** | `solverKind == 'none'` ⇒ 报告里**没有任何关节解、没有工作空间** | 伪造 IK |
| 交叉一致性 | MeArm 的 `sim2sim.json` 与既有 4 份黄金基线的**同名用例** FK 逐位相同 | 统一框架偷偷换了 MeArm 的判据 |

最后一条是本轮"抽出统一框架"这件事**唯一的**风险点：把判据收敛到一处时
很容易顺手改掉 MeArm 的期望值（"反正新框架自己算一遍也能对上"）。
它用两批**独立生成**的产物互相对照，把这条风险钉住。
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from robopkg import declared_path  # noqa: E402
from sim2sim import (
    FK_TOL_MM,
    fk_tolerance_mm,
    joint_cases,
    run_sim2sim,
    sim2sim_matrix,
)

ROOT = Path(__file__).resolve().parents[2]

#: 冻结快照用的随机用例数 —— 必须与 `tools/run_sim2sim.py --freeze` 时一致
N_RANDOM = 24

#: 数值比对容差（**比物理容差严得多**，用于"和快照是否同一份数据"这类判据）
SNAPSHOT_TOL_FRONTEND_REF_MM = 1e-9
SNAPSHOT_TOL_MUJOCO_MM = 1e-6


def _snapshot_path(robot_id: str) -> Path:
    """快照路径 = **该包 manifest 声明的** `tests.cases` 目录（不在这里拼路径）。

    Phase 2 之前这里硬编码 `tests/baseline/<id>/`；那种写法在搬迁时会漏改，
    而漏改的表现是"文件找不到"（还算好）或"读到了仍存在的另一个同名文件"（很糟）。
    """
    return declared_path(robot_id, "tests.cases") / "sim2sim.json"


def _load_snapshot(robot_id: str) -> dict:
    path = _snapshot_path(robot_id)
    if not path.is_file():
        raise FileNotFoundError(
            f"缺少 Sim2Sim 快照 {path}\n"
            f"  生成：python core/tools/run_sim2sim.py --robot {robot_id} "
            f"--n-random {N_RANDOM} --freeze"
        )
    return json.loads(path.read_text(encoding="utf-8"))


# ---------------------------------------------------------------------------
# 矩阵层
# ---------------------------------------------------------------------------

def test_tolerance_registered_for_every_selector_robot(selector_robot_ids):
    """选择器里每台机器人都必须**登记过** FK 容差（且不靠缺省值）。"""
    missing = [rid for rid in selector_robot_ids if rid not in FK_TOL_MM]
    assert missing == [], (
        f"这些机器人没有登记 FK 容差：{missing}\n"
        f"  请在 simulation/mujoco/sim2sim.py 的 FK_TOL_MM 里写清「容差 + 理由」"
    )
    for rid in selector_robot_ids:
        tol, reason = fk_tolerance_mm(rid)
        assert tol > 0 and reason.strip(), f"{rid} 的容差必须 > 0 且带理由"


def test_unknown_robot_tolerance_raises():
    """未登记的 id **必须报错**，而不是回落到某个"够大"的缺省容差。"""
    with pytest.raises(KeyError):
        fk_tolerance_mm("no-such-robot")


def test_joint_cases_are_deterministic_and_robot_agnostic(robot):
    """同一份 robot.yaml 两次枚举必须**逐位相同**（冻结数值比对的前提）。"""
    a = joint_cases(robot, n_random=4)
    b = joint_cases(robot, n_random=4)
    assert a == b

    ids = [c["id"] for c in a]
    # 顺序约定（随机用例**排在最后**，便于看出"哪几条是显式枚举的"）
    assert ids[:2] == ["zero", "home"]
    assert "all_min" in ids and "all_max" in ids
    # 计数公式：2（zero/home）+ 3×可动关节（min/mid/max）+ 2（角点）+ n_random
    expected = 2 + 3 * len(robot.movable_joints()) + 2 + 4
    assert len(ids) == expected, f"用例数 {len(ids)} != {expected}"
    assert len(set(ids)) == len(ids), "用例 id 有重复"

    # 每条用例的关节集合都必须**恰好**是这台机器人的可动关节
    want = sorted(j.id for j in robot.movable_joints())
    for c in a:
        assert sorted(c["joints"]) == want


@pytest.mark.parametrize("robot_id", sorted(FK_TOL_MM))
def test_sim2sim_runs_for_every_registered_robot(robot_id, bridge_factory):
    """统一入口对每台登记过的机器人都能产出报告，且 FK 落在**自己的**容差内。"""
    report = run_sim2sim(robot_id, bridge=bridge_factory(robot_id), n_random=N_RANDOM)

    assert report.fk_cases, "至少要有一组 FK 用例"
    assert report.max_fk_ref_mujoco_mm <= report.tolerance_mm, (
        f"{robot_id} 的「参考 ↔ MuJoCo」残差 {report.max_fk_ref_mujoco_mm:.6e} mm "
        f"超出登记容差 {report.tolerance_mm:g} mm\n  容差依据：{report.tolerance_reason}"
    )
    # 前端与参考都消费同一份 robot.yaml ⇒ 它们之间必须是"数值级一致"，
    # 与"官方两份文件精度差"无关，所以这里用 MM_TOL 量级而不是机器人容差。
    assert report.max_fk_frontend_ref_mm < 1e-6, (
        f"{robot_id} 的「前端 ↔ 参考」残差 {report.max_fk_frontend_ref_mm:.6e} mm 过大 —— "
        f"两边读的是同一份 robot.yaml，不该有几何级差异"
    )


def test_matrix_covers_all_selector_robots(selector_robot_ids, bridge_factory):
    """回归矩阵 == 选择器的全部机器人（不多不少），且每台都能给出摘要行。

    刻意**复用**会话级桥（`bridge_factory`）而不是调 `sim2sim_matrix()` 让它
    自建 —— 后者每台机器人各起一次 Vite，在矩阵里就是纯粹的开销。
    """
    reports = [
        run_sim2sim(rid, bridge=bridge_factory(rid), n_random=0)
        for rid in selector_robot_ids
    ]
    assert [r.robot_id for r in reports] == selector_robot_ids
    for r in reports:
        assert r.line(), f"{r.robot_id} 渲染不出摘要行"
        assert 0 < r.timestep_s <= 0.01, f"{r.robot_id} 的 timestep 不合理：{r.timestep_s}"
        assert r.max_velocity_deg_s > 0
        # 报告的自洽性：统计值必须真的来自用例集合，而不是另算的一份
        assert r.summary()["fkCases"] == len(r.fk_cases)


def test_sim2sim_matrix_public_entry_self_contained():
    """`sim2sim_matrix(id)` 自己起桥也能跑通（公开入口不依赖任何夹具）。"""
    reports = sim2sim_matrix(["so-arm101"], n_random=0)
    assert len(reports) == 1
    assert reports[0].robot_id == "so-arm101"


# ---------------------------------------------------------------------------
# 能力诚实性（**禁止伪造 IK**）
# ---------------------------------------------------------------------------

def test_no_robot_fabricates_ik(selector_robot_ids, bridge_factory):
    """`solverKind == 'none'` 的机器人：报告里不得出现任何关节解或工作空间数据。"""
    offenders: list[str] = []
    seen_none = False
    seen_solver = False

    for rid in selector_robot_ids:
        report = run_sim2sim(rid, bridge=bridge_factory(rid), n_random=0)
        if report.capability.has_ik:
            seen_solver = True
            assert report.ik_cases, f"{rid} 声明有逆解器，却一条 IK 用例都没有"
            assert any(c.solved for c in report.ik_cases), (
                f"{rid} 声明有逆解器，但没有任何一条用例解出来 —— 声明与实现不符"
            )
            continue

        seen_none = True
        assert report.ik_supported is False
        assert report.ik_note, f"{rid} 没有逆解器，必须在 ik_note 里说明原因"
        for case in report.ik_cases:
            assert case.outcome == "unavailable", (
                f"{rid} 没有逆解器，用例 {case.id} 却返回 {case.outcome!r}"
            )
            assert case.reason == "NOT_IMPLEMENTED", (
                f"{rid} 的拒绝理由应为 NOT_IMPLEMENTED，实际 {case.reason!r}"
            )
            assert case.joints is None, (
                f"{rid} 没有逆解器，却给出了关节解 {case.joints} —— 这就是伪造"
            )
            assert case.closed_loop_mm is None
        # 报告里不得出现工作空间字段（没测过的东西不写）
        payload = report.as_dict()
        assert "workspace" not in payload
        assert "geometry" not in payload

    assert seen_none and seen_solver, (
        "本用例的前提是『选择器里同时存在有逆解器与没有逆解器的机器人』；"
        "只覆盖一种情形时它是空转的"
    )


def test_so_arm101_reports_no_solver_and_no_fabricated_numbers(bridge_factory):
    """SO-ARM101 的具体事实（写成显式断言，避免上面的通用用例被"优化"掉）。"""
    report = run_sim2sim("so-arm101", bridge=bridge_factory("so-arm101"), n_random=0)
    assert report.capability.solver_kind == "none"
    assert report.capability.positioning_dof == 5
    assert report.capability.supports_orientation is False
    assert report.joint_order == [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    ]
    # 官方 MJCF 的 timestep 是 0.002（MeArm 是 0.001）—— 取自 MJCF 本身，不是配置抄写
    assert report.timestep_s == pytest.approx(0.002)
    # STS3215 空载 0.222 s/60° @12V ⇒ 270.27 °/s
    assert report.max_velocity_deg_s == pytest.approx(270.27, rel=1e-6)


# ---------------------------------------------------------------------------
# 冻结快照
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("robot_id", sorted(FK_TOL_MM))
def test_snapshot_structure_matches(robot_id, bridge_factory):
    """用例 id 列表 / 关节顺序 / 能力声明必须与快照**逐位一致**。"""
    snap = _load_snapshot(robot_id)
    report = run_sim2sim(robot_id, bridge=bridge_factory(robot_id), n_random=N_RANDOM)

    assert report.joint_order == snap["jointOrder"]
    assert report.capability.solver_kind == snap["capability"]["solverKind"]
    assert report.capability.positioning_dof == snap["capability"]["positioningDof"]
    assert report.ik_supported == snap["ikSupported"]
    assert snap["nRandomCases"] == N_RANDOM, (
        "快照的随机用例数与测试不一致 —— 那是两套判据，不是同一批数据"
    )

    assert [c.id for c in report.fk_cases] == [c["id"] for c in snap["fkCases"]]
    assert [c.id for c in report.ik_cases] == [c["id"] for c in snap["ikCases"]]
    for case, ref in zip(report.fk_cases, snap["fkCases"]):
        assert case.joints == pytest.approx(ref["joints"], abs=1e-12)


@pytest.mark.parametrize("robot_id", sorted(FK_TOL_MM))
def test_snapshot_values_match(robot_id, bridge_factory):
    """数值层面：三侧面 TCP 与快照一致。"""
    snap = _load_snapshot(robot_id)
    report = run_sim2sim(robot_id, bridge=bridge_factory(robot_id), n_random=N_RANDOM)

    for case, ref in zip(report.fk_cases, snap["fkCases"]):
        np.testing.assert_allclose(
            case.tcp_frontend, ref["tcpFrontend"],
            atol=SNAPSHOT_TOL_FRONTEND_REF_MM, rtol=0,
            err_msg=f"{robot_id}/{case.id} 前端 FK 与快照不一致",
        )
        np.testing.assert_allclose(
            case.tcp_ref, ref["tcpRef"],
            atol=SNAPSHOT_TOL_FRONTEND_REF_MM, rtol=0,
            err_msg=f"{robot_id}/{case.id} 参考 FK 与快照不一致",
        )
        np.testing.assert_allclose(
            case.tcp_mujoco, ref["tcpMujoco"],
            atol=SNAPSHOT_TOL_MUJOCO_MM, rtol=0,
            err_msg=f"{robot_id}/{case.id} MuJoCo TCP 与快照不一致",
        )

    for case, ref in zip(report.ik_cases, snap["ikCases"]):
        assert case.outcome == ref["outcome"]
        assert case.reason == ref["reason"]
        if ref["closedLoopMm"] is not None:
            assert case.closed_loop_mm == pytest.approx(ref["closedLoopMm"], abs=1e-9)


def test_mearm_sim2sim_agrees_with_the_four_file_golden_baseline(bridge_factory):
    """**交叉一致性**：统一框架没有偷偷改变 MeArm 的判据。

    `robot-package/mearm-v1/tests/cases/*.json`（4 份）由 `tools/gen_mearm_v1_baseline.py`
    在冻结时采集；`sim2sim.json` 由**另一个**工具（`tools/run_sim2sim.py`）采集。
    两批产物的**同名用例**（zero / home / *_min|mid|max / all_min|max）的 FK
    TCP 必须逐位相同。

    这是"抽出统一框架"这件事唯一的风险点：把判据收敛到一处时很容易顺手
    改掉 MeArm 的期望值（新框架自己算一遍当然也能对上自己）。两批独立采集
    的产物互相印证，才把这条风险钉住。
    """
    golden = json.loads(
        (declared_path("mearm-v1", "tests.cases") / "fk_cases.json").read_text(encoding="utf-8")
    )
    golden_by_id = {c["id"]: c for c in golden["cases"]}

    report = run_sim2sim("mearm-v1", bridge=bridge_factory("mearm-v1"), n_random=0)
    compared = 0
    for case in report.fk_cases:
        ref = golden_by_id.get(case.id)
        if ref is None:
            continue          # 随机用例两边数量不同，不参与交叉比对
        np.testing.assert_allclose(
            case.tcp_frontend, ref["tcpFrontend"],
            atol=1e-9, rtol=0,
            err_msg=f"统一框架的 {case.id} 与黄金基线的同一用例不一致",
        )
        compared += 1
    assert compared >= 8, f"交叉比对只覆盖了 {compared} 条用例，太少（判据形同虚设）"
