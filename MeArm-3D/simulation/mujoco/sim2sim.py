# -*- coding: utf-8 -*-
"""统一 Sim2Sim 框架 —— `runSim2Sim(robot)`。

## 它统一了什么

本项目的 Sim2Sim（"同一组关节角 / 同一个目标点，在前端与 MuJoCo 两侧是否落到同一处"）
原本是**为 MeArm-V1 写的**：判据写死在 `tests/sim2sim/` 里，采集入口写死在
`tools/gen_mearm_v1_baseline.py` 里。引入第二台机器人后，如果照抄一份，
就会出现"两台机器人各有一套验收标准"——而两套标准各自都能自己绿，
"一致"这件事就失去了意义。

⇒ 于是把**判据与采集**收敛到本模块，只有一个入口：

```python
from sim2sim import run_sim2sim, sim2sim_matrix

report = run_sim2sim("mearm-v1")            # 单台
for r in sim2sim_matrix():                  # 回归矩阵：选择器声明的全部机器人
    print(r.line())
```

## 三条独立侧面（缺一条就变成自证）

| 侧面 | 来源 | 它证明什么 |
|---|---|---|
| `frontend` | `frontend/tests/tools/kinematics-bridge.mjs`（Vite SSR 加载**真实** `fk.ts` / `ik.ts`） | 浏览器里跑的就是这套代码 |
| `ref` | `simulation/mujoco/fkref.py`（Python 独立实现，**不引用前端**） | 两套独立实现互相对得上 |
| `mujoco` | 编译后的 `MjModel`（`mj_forward`，纯运动学求值） | 几何定义层面对得上 |

⚠️ **绝不允许**把 `ref` 换成"再调一次 bridge"：那会把三侧面退化成一条自证链。

## 能力（`capability`）决定 IK 段——不是 `if robot == …`

`capability.solverKind` 来自前端引擎的**声明**（`SoArm101Kinematics.capability`）。
`'none'` 的机器人：

* IK 段**不做任何求解**（一次都不调用 `solveIk`）；
* 但仍然**发几个目标点过去**，把"前端确实拒绝了"这件事**记录下来** ——
  记录一次拒绝是证据，编一组"看起来合理"的关节角才是伪造；
* `report.ik_supported == False`，`note` 里写清"暂不提供"以及**为什么**。

## 容差为什么按机器人分表且**必须写明理由**

`mearm-v1` —— `robot.yaml` 与 `fkref.py` 都是我们写的、同源 ⇒ `~1e-13 mm`
  残差极小，故容差取 1e-6（留 7 个数量级余量）。

（历史上有第二台机器人时，容差按机器人分表登记；当前仓库只有 mearm-v1，
  故表中仅一项。新增机器人**必须**在此登记「容差 + 理由」，且不能留缺省。）

所以容差**不设缺省值**：未在表中登记的 id 一律**报错**。
悄悄给个"够大"的缺省，等于把"新增机器人时没人想过它的精度来源"这件事藏起来。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np

from fkref import fk_tcp_mm
from robotcfg import RobotCfg, SimDriverCfg, load_robot_by_id, load_robot_selector, resolve_robot_entry
from units import ensure_utf8_stdout

ensure_utf8_stdout()

# ---------------------------------------------------------------------------
# 容差表：id → (容差 mm, 理由)。**没有缺省值**。
# ---------------------------------------------------------------------------

FK_TOL_MM: dict[str, tuple[float, str]] = {
    "mearm-v1": (
        1e-6,
        "robot.yaml 与 fkref.py 同源（都是本项目写的），实测残差 ~1e-13 mm；"
        "1e-6 留 7 个数量级余量",
    ),
}

#: 固定 seed —— 与 `tools/gen_mearm_v1_baseline.py` 同一套随机源约定
SEED = 20260914


def fk_tolerance_mm(robot_id: str) -> tuple[float, str]:
    """取该机器人的 FK 容差与理由；未登记一律报错（**不允许**静默给缺省）。"""
    try:
        return FK_TOL_MM[robot_id]
    except KeyError:
        raise KeyError(
            f"机器人 {robot_id!r} 没有登记 FK 容差。\n"
            f"  已知：{sorted(FK_TOL_MM)}\n"
            f"  请在 simulation/mujoco/sim2sim.py 的 FK_TOL_MM 里登记**容差 + 理由**："
            f"容差必须能说出它对应哪一条真实的精度来源，而不是"
            f'"先看跑出来是多少再填"。'
        ) from None


# ---------------------------------------------------------------------------
# 能力
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class RobotCapability:
    """前端引擎声明的能力（`KinematicsEngine.capability` 的只读投影）。"""

    solver_kind: str
    positioning_dof: int
    supports_orientation: bool

    @property
    def has_ik(self) -> bool:
        """有没有逆解器。**判据是声明，不是robot id**。"""
        return self.solver_kind != "none"

    @classmethod
    def from_bridge_info(cls, info: Mapping[str, Any]) -> "RobotCapability":
        cap = info.get("capability")
        if not isinstance(cap, Mapping):
            raise ValueError(
                "桥返回的模型元信息里没有 capability —— "
                "无法判断这台机器人有没有逆解器（本框架拒绝猜）"
            )
        return cls(
            solver_kind=str(cap.get("solverKind", "unknown")),
            positioning_dof=int(cap.get("positioningDof", 0)),
            supports_orientation=bool(cap.get("supportsOrientation", False)),
        )


# ---------------------------------------------------------------------------
# 用例枚举（机器人无关）
# ---------------------------------------------------------------------------

def joint_cases(robot: RobotCfg, *, n_random: int = 24, seed: int = SEED) -> list[dict[str, Any]]:
    """**显式枚举**的关节位形用例集（零位 / HOME / 各关节限位与中点 / 角点 / 合法随机）。

    与 `tools/gen_mearm_v1_baseline.py:joint_cases()` 同一套构造规则 ——
    刻意保持一致：MeArm 的黄金数据就是按这些规则采集的，
    换规则会让"两台机器人跑的是同一套判据"这句话不成立。
    """
    movable = robot.movable_joints()
    home = {j.id: float(robot.home_pose[j.id]) for j in movable}
    cases: list[dict[str, Any]] = []

    def add(cid: str, label: str, joints: Mapping[str, float]) -> None:
        cases.append({"id": cid, "label": label, "joints": dict(joints)})

    add("zero", "关节空间原点（全 0°）", {j.id: 0.0 for j in movable})
    add("home", "HOME 位", home)

    for j in movable:
        for tag, val in (
            ("min", j.limit_min),
            ("mid", (j.limit_min + j.limit_max) / 2),
            ("max", j.limit_max),
        ):
            add(f"{j.id}_{tag}", f"{j.id} = {tag}", {**home, j.id: float(val)})

    add("all_min", "全部关节取限位下限", {j.id: float(j.limit_min) for j in movable})
    add("all_max", "全部关节取限位上限", {j.id: float(j.limit_max) for j in movable})

    if n_random > 0:
        rng = np.random.default_rng(seed)
        for i in range(n_random):
            add(
                f"rand_{i:03d}",
                f"合法随机位形 #{i}（seed={seed}）",
                {j.id: float(rng.uniform(j.limit_min, j.limit_max)) for j in movable},
            )

    return cases


# ---------------------------------------------------------------------------
# 结果容器
# ---------------------------------------------------------------------------

@dataclass
class FkCase:
    id: str
    label: str
    joints: dict[str, float]
    tcp_frontend: tuple[float, float, float]
    tcp_ref: tuple[float, float, float]
    tcp_mujoco: tuple[float, float, float]

    @property
    def delta_frontend_ref(self) -> float:
        return _dist(self.tcp_frontend, self.tcp_ref)

    @property
    def delta_ref_mujoco(self) -> float:
        return _dist(self.tcp_ref, self.tcp_mujoco)


@dataclass
class IkCase:
    """一条 IK 用例的结果。

    `outcome` 只有三种取值，**刻意区分"没算"与"算了不行"**：

    | outcome | 含义 | 什么时候出现 |
    |---|---|---|
    | `solved` | 解出来了 | 有逆解器且目标可达 |
    | `refused` | 算了，但没有解 | 有逆解器但目标越界 / 撞限位 |
    | `unavailable` | **根本没算** | 该机器人没有逆解器（`solverKind == 'none'`） |
    """

    id: str
    target: tuple[float, float, float]
    outcome: str
    reason: str | None = None
    joints: dict[str, float] | None = None
    closed_loop_mm: float | None = None

    @property
    def solved(self) -> bool:
        return self.outcome == "solved"


@dataclass
class Sim2SimReport:
    robot_id: str
    model: str
    model_version: str
    joint_order: list[str]
    capability: RobotCapability
    tolerance_mm: float
    tolerance_reason: str
    timestep_s: float
    max_velocity_deg_s: float
    fk_cases: list[FkCase] = field(default_factory=list)
    ik_cases: list[IkCase] = field(default_factory=list)
    ik_supported: bool = False
    ik_note: str = ""

    # -- 统计 ---------------------------------------------------------------

    @property
    def max_fk_frontend_ref_mm(self) -> float:
        return max((c.delta_frontend_ref for c in self.fk_cases), default=0.0)

    @property
    def max_fk_ref_mujoco_mm(self) -> float:
        return max((c.delta_ref_mujoco for c in self.fk_cases), default=0.0)

    @property
    def ik_solved(self) -> int:
        return sum(1 for c in self.ik_cases if c.solved)

    @property
    def max_closed_loop_mm(self) -> float:
        got = [c.closed_loop_mm for c in self.ik_cases if c.closed_loop_mm is not None]
        return max(got, default=0.0)

    @property
    def fk_ok(self) -> bool:
        return self.max_fk_ref_mujoco_mm <= self.tolerance_mm

    def summary(self) -> dict[str, Any]:
        return {
            "robotId": self.robot_id,
            "model": self.model,
            "solverKind": self.capability.solver_kind,
            "positioningDof": self.capability.positioning_dof,
            "fkCases": len(self.fk_cases),
            "fkMaxFrontendVsRefMm": self.max_fk_frontend_ref_mm,
            "fkMaxRefVsMujocoMm": self.max_fk_ref_mujoco_mm,
            "fkToleranceMm": self.tolerance_mm,
            "fkOk": self.fk_ok,
            "ikSupported": self.ik_supported,
            "ikCases": len(self.ik_cases),
            "ikSolved": self.ik_solved,
            "ikMaxClosedLoopMm": self.max_closed_loop_mm,
            "timestepS": self.timestep_s,
            "maxVelocityDegS": self.max_velocity_deg_s,
        }

    def line(self) -> str:
        """回归矩阵的一行（人类可读）。"""
        ik = (
            f"IK {self.ik_solved}/{len(self.ik_cases)} · 闭环 ≤{self.max_closed_loop_mm:.3e} mm"
            if self.ik_supported
            else f"IK 未提供（solverKind={self.capability.solver_kind}）"
        )
        return (
            f"{self.robot_id:<11} {self.model:<11} "
            f"FK {len(self.fk_cases):>3} 例 · 前端↔参考 ≤{self.max_fk_frontend_ref_mm:.3e} · "
            f"参考↔MuJoCo ≤{self.max_fk_ref_mujoco_mm:.3e} mm "
            f"(容差 {self.tolerance_mm:g}) · {ik}"
        )

    def as_dict(self) -> dict[str, Any]:
        return {
            "robotId": self.robot_id,
            "model": self.model,
            "modelVersion": self.model_version,
            "jointOrder": list(self.joint_order),
            "capability": {
                "solverKind": self.capability.solver_kind,
                "positioningDof": self.capability.positioning_dof,
                "supportsOrientation": self.capability.supports_orientation,
            },
            "toleranceMm": self.tolerance_mm,
            "toleranceReason": self.tolerance_reason,
            "timestepS": self.timestep_s,
            "maxVelocityDegS": self.max_velocity_deg_s,
            "ikSupported": self.ik_supported,
            "ikNote": self.ik_note,
            "summary": self.summary(),
            "fkCases": [
                {
                    "id": c.id,
                    "joints": c.joints,
                    "tcpFrontend": list(c.tcp_frontend),
                    "tcpRef": list(c.tcp_ref),
                    "tcpMujoco": list(c.tcp_mujoco),
                }
                for c in self.fk_cases
            ],
            "ikCases": [
                {
                    "id": c.id,
                    "target": list(c.target),
                    "outcome": c.outcome,
                    "reason": c.reason,
                    "joints": c.joints,
                    "closedLoopMm": c.closed_loop_mm,
                }
                for c in self.ik_cases
            ],
        }


def _dist(a: Iterable[float], b: Iterable[float]) -> float:
    pa, pb = np.asarray(list(a), dtype=float), np.asarray(list(b), dtype=float)
    return float(np.linalg.norm(pa - pb))


# ---------------------------------------------------------------------------
# 主入口
# ---------------------------------------------------------------------------

def run_sim2sim(
    robot_id: str,
    *,
    bridge: Any | None = None,
    n_random: int = 24,
    n_refusal_probes: int = 3,
    sim: Any | None = None,
) -> Sim2SimReport:
    """对**任意机器人**跑同一套 Sim2Sim 回归。

    `bridge` 省略时本函数自建并负责关闭；显式传入则由调用方负责生命周期
    （测试里传 session 级夹具，避免每台机器人各起一次 Vite）。
    `sim` 同理（省略时按 id 建一个 `RobotSim`）。
    """
    from ikbridge import KinematicsBridge      # 延迟导入：避免 tools/ 侧的循环
    from model import RobotSim

    entry = resolve_robot_entry(robot_id)
    robot = load_robot_by_id(robot_id)
    own_bridge = bridge is None
    bridge = bridge or KinematicsBridge(robot_id=robot_id)
    try:
        info = bridge.model_info()
        capability = RobotCapability.from_bridge_info(info)
        tol, tol_reason = fk_tolerance_mm(robot_id)

        driver: SimDriverCfg = _driver_of(entry)
        if sim is None:
            sim = RobotSim(robot=robot, robot_id=robot_id)

        cases = joint_cases(robot, n_random=n_random)

        # ---- FK 三侧面 ----------------------------------------------------
        fk_res = bridge.forward(
            [{"id": i, "joints": c["joints"]} for i, c in enumerate(cases)]
        )
        by_idx = {int(r["id"]): r for r in fk_res}

        fk_cases: list[FkCase] = []
        for i, c in enumerate(cases):
            got = by_idx.get(i)
            if not got or not got.get("success"):
                raise RuntimeError(f"前端 FK 对用例 {c['id']} 求值失败：{got}")
            # ⚠️ `reset()` 而不是 `settle()`：reset 内部只做 `mj_forward`，
            # 是**纯运动学求值**，不推进物理、不引入重力/接触/迭代器噪声。
            sim.reset(dict(c["joints"]))
            fk_cases.append(
                FkCase(
                    id=c["id"],
                    label=c["label"],
                    joints=c["joints"],
                    tcp_frontend=tuple(float(v) for v in got["tcp"]),
                    tcp_ref=tuple(float(v) for v in fk_tcp_mm(robot, c["joints"])),
                    tcp_mujoco=tuple(float(v) for v in sim.end_effector_mm()),
                )
            )

        report = Sim2SimReport(
            robot_id=robot_id,
            model=robot.model or robot.name,
            model_version=robot.model_version or "",
            joint_order=robot.joint_order(),
            capability=capability,
            tolerance_mm=tol,
            tolerance_reason=tol_reason,
            timestep_s=float(sim.model.opt.timestep),
            max_velocity_deg_s=float(driver.max_velocity_deg_s),
            fk_cases=fk_cases,
            ik_supported=capability.has_ik,
        )

        # ---- IK 段 --------------------------------------------------------
        if capability.has_ik:
            _collect_ik_solved(report, robot, bridge, sim)
        else:
            _collect_ik_unavailable(report, robot, bridge, sim, n_refusal_probes)

        return report
    finally:
        if own_bridge:
            bridge.close()


def _driver_of(entry) -> SimDriverCfg:
    from robotcfg import load_sim_driver

    return load_sim_driver(entry)


def _collect_ik_solved(
    report: Sim2SimReport, robot: RobotCfg, bridge: Any, sim: Any
) -> None:
    """有逆解器：`target → IK → q → MuJoCo → TCP'` 闭环。

    目标点**由实现自己产生**（取 FK 结果），不手挑坐标 —— 免得把
    "我猜它能到"写进判据。另外补几个显式越界点，用来钉住错误码。
    """
    movable = {j.id for j in robot.movable_joints()}
    reachable = [c for c in report.fk_cases if c.id in ("zero", "home") or c.id.startswith("rand_")]
    targets: list[tuple[str, tuple[float, float, float]]] = [
        (f"reachable_{c.id}", c.tcp_frontend) for c in reachable
    ]
    # 显式越界：远离工作空间的点（必须被判为不可达，而不是"随便给个解"）
    far = np.asarray(report.fk_cases[0].tcp_frontend, dtype=float)
    span = float(np.linalg.norm(far)) or 1.0
    direction = far / span
    for k, mult in (("far_x10", 10.0), ("behind", -2.0)):
        pt = tuple(float(v) for v in direction * span * mult)
        targets.append((f"out_of_workspace_{k}", pt))

    req = [{"id": i, "target": list(t)} for i, (_, t) in enumerate(targets)]
    results = bridge.solve(req)

    for i, (label, target) in enumerate(targets):
        got = results[i]
        if got.get("success"):
            q = {k: float(v) for k, v in got["joints"].items()}
            # 未参与解算的关节（如夹爪）不进闭环判据
            q_core = {k: v for k, v in q.items() if k in movable}
            sim.reset(q_core)
            tcp_after = sim.end_effector_mm()
            report.ik_cases.append(
                IkCase(
                    id=label,
                    target=tuple(float(v) for v in target),
                    outcome="solved",
                    joints=q,
                    closed_loop_mm=_dist(tcp_after, target),
                )
            )
        else:
            report.ik_cases.append(
                IkCase(
                    id=label,
                    target=tuple(float(v) for v in target),
                    outcome="refused",
                    reason=str(got.get("reason") or "UNKNOWN"),
                )
            )
    report.ik_note = "逆解器可用（solverKind=analytic）⇒ 已做 XYZ→IK→q→MuJoCo 闭环"


def _collect_ik_unavailable(
    report: Sim2SimReport,
    robot: RobotCfg,
    bridge: Any,
    sim: Any,
    n_probes: int,
) -> None:
    """**没有**逆解器：不求解，只把"前端确实拒绝了"记录下来。

    为什么还要发请求：一条"我发过去了、它明确拒绝了、理由是这个"的记录，
    与"我们没测"是两件完全不同的事。前者是**证据**，后者是空白。

    ⚠️ 这里**不产生任何关节角**，也**不构造工作空间**（没测过的东西不写）。
    """
    probes: list[tuple[str, tuple[float, float, float]]] = []
    home = report.fk_cases[0] if report.fk_cases else None
    if home is not None:
        base = np.asarray(home.tcp_frontend, dtype=float)
        for k, factor in (("home", 1.0), ("near", 0.5), ("far", 2.0)):
            probes.append((f"probe_{k}", tuple(float(v) for v in base * factor)))
    probes = probes[: max(1, n_probes)]

    results = bridge.solve([{"id": i, "target": list(t)} for i, (_, t) in enumerate(probes)])
    for i, (label, target) in enumerate(probes):
        got = results[i]
        report.ik_cases.append(
            IkCase(
                id=label,
                target=tuple(float(v) for v in target),
                outcome="unavailable",
                reason=str(got.get("reason") or "UNKNOWN"),
            )
        )
    report.ik_note = (
        f"{robot.name} 暂不提供逆运动学（solverKind={report.capability.solver_kind}）—— "
        f"前端对 {len(probes)} 个探测目标全部如实返回 NOT_IMPLEMENTED，"
        f"**本报告不含任何伪造的关节解，也不含工作空间数据**（没有测量依据）"
    )


def sim2sim_matrix(
    robot_ids: Iterable[str] | None = None,
    **kwargs: Any,
) -> list[Sim2SimReport]:
    """**回归矩阵**：选择器声明的**全部**机器人各跑一遍。

    刻意默认取"选择器里的全部"而不是写死两台 —— 新增机器人自动进入矩阵，
    未登记 FK 容差会在 `fk_tolerance_mm()` 处**报错**（而不是静默跳过）。
    """
    ids = list(robot_ids) if robot_ids is not None else list(load_robot_selector().robots.keys())
    return [run_sim2sim(rid, **kwargs) for rid in ids]


def project_root() -> Path:
    return Path(__file__).resolve().parents[2]
