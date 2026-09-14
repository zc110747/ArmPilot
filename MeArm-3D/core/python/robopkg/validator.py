"""`PackageValidator` —— 包契约校验（spec §18）。

## 它守的是什么

三份声明**必须互相说得通**，而这在当前工程里是**三处独立维护的**：

```text
config/robots.yaml            说：有哪些机器人、各自的文件在哪     ← 三端共用
robot-package/<id>/manifest   说：我是谁、我有什么能力、我的指针   ← 本次新增
robot.yaml                    说：几何/关节/限位/标定的真值        ← 权威真值
frontend RobotRegistry        说：我有没有这台机器人的引擎实现      ← 代码
```

任何一处改了、另一处没跟上，**都不会自己报错** —— 只会表现为
"某台机器人在某个功能上静默走了旧路径"或"加载了另一台"。
⇒ 所以校验器的价值不在于"检查文件存在"，而在于**把四份声明对起来**。

## 两条纪律

1. **宁可报错，不要兜底**。未知 id 不回退到 default（回退 = 静默换成另一台机器人）。
2. **`warn` 只用于"确实值得人看一眼、但不影响正确性"的事**。
   本项目对"无害告警"的态度是：把错的告警留着比没有告警更糟（见 Go 侧 `physics_kind` 分支）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from .errors import PackageError
from .loader import ResolvedRobot, list_package_ids, load_robot
from .manifest import Manifest, find_manifest_for_config, load_manifest
from .root import PACKAGES_DIR, PROJECT_ROOT, load_robotcfg, repo_relative, to_abs


@dataclass(frozen=True)
class Issue:
    level: str  # 'error' | 'warn'
    code: str
    where: str
    message: str

    def line(self) -> str:
        icon = "✘" if self.level == "error" else "!"
        return f"  {icon} [{self.code}] {self.where}：{self.message}"


@dataclass
class ValidationReport:
    robot_id: str
    issues: list[Issue] = field(default_factory=list)
    resolved: ResolvedRobot | None = None

    def error(self, code: str, where: str, message: str) -> None:
        self.issues.append(Issue("error", code, where, message))

    def warn(self, code: str, where: str, message: str) -> None:
        self.issues.append(Issue("warn", code, where, message))

    @property
    def errors(self) -> list[Issue]:
        return [i for i in self.issues if i.level == "error"]

    @property
    def warnings(self) -> list[Issue]:
        return [i for i in self.issues if i.level == "warn"]

    @property
    def ok(self) -> bool:
        return not self.errors

    def summary(self) -> str:
        if self.ok and not self.warnings:
            return f"✓ {self.robot_id}：包契约全部通过"
        head = f"{'✓' if self.ok else '✘'} {self.robot_id}：{len(self.errors)} error / {len(self.warnings)} warn"
        return "\n".join([head, *(i.line() for i in self.issues)])


# ---------------------------------------------------------------------------


def _exists_rel(rel: str | None) -> bool:
    if rel is None:
        return True
    return to_abs(rel).exists()


def _check_pointers(rep: ValidationReport, m: Manifest) -> None:
    """所有指针必须存在 —— 逐个点名，不合并成一句"有文件缺失"。"""
    for label, rel in (
        ("model.config", m.model.config),
        ("model.physics", m.model.physics),
        ("simulation.mjcf", m.simulation.mjcf),
        ("model.generated_by", m.model.generated_by),
        ("simulation.generated_by", m.simulation.generated_by),
        ("kinematics.fk.entry", m.kinematics.fk_entry),
        ("kinematics.ik.entry", m.kinematics.ik_entry),
        ("tests.cases", m.tests.cases),
        ("tests.frozen", m.tests.frozen),
        *[(f"tests.tools[{i}]", t) for i, t in enumerate(m.tests.tools)],
        *[(f"package.hash_sources[{i}]", s) for i, s in enumerate(m.hash_sources)],
    ):
        if not _exists_rel(rel):
            rep.error(
                "POINTER_MISSING",
                f"{m.id}.{label}",
                f"指向的路径不存在：{repo_relative(to_abs(rel)) if rel else rel!r}",
            )


def _check_kinematics_decl(rep: ValidationReport, m: Manifest) -> None:
    """运动学声明内部自洽。"""
    k = m.kinematics
    c = m.capabilities

    if k.fk_type == "custom" and k.fk_entry is None:
        rep.error(
            "FK_CUSTOM_WITHOUT_ENTRY",
            f"{m.id}.kinematics.fk",
            "type: custom 必须给出 entry（否则 Core 不知道该调用谁）",
        )
    if k.fk_type == "engine" and k.fk_entry is not None:
        rep.error(
            "FK_ENGINE_WITH_ENTRY",
            f"{m.id}.kinematics.fk",
            "type: engine 表示「用 Core 的通用链式 FK」，再给 entry 是矛盾声明"
            "（两份 FK 会分家，且渲染与判据各用一份、不报错）",
        )

    if k.ik_type == "custom" and k.ik_entry is None:
        rep.error(
            "IK_CUSTOM_WITHOUT_ENTRY",
            f"{m.id}.kinematics.ik",
            "type: custom 必须给出 entry",
        )
    if k.ik_type == "none" and k.ik_entry is not None:
        rep.error(
            "IK_NONE_WITH_ENTRY",
            f"{m.id}.kinematics.ik",
            "type: none（没有逆解）却给了 entry —— 二者必须一致",
        )

    # 能力 ↔ 实现
    if c.ik and k.ik_type == "none":
        rep.error(
            "CAPABILITY_IK_WITHOUT_SOLVER",
            f"{m.id}.capabilities.ik",
            "声明 ik: true 但 kinematics.ik.type = none ⇒ 上层会去调一个不存在的求解器",
        )
    if (not c.ik) and k.ik_type != "none":
        rep.error(
            "SOLVER_WITHOUT_CAPABILITY_IK",
            f"{m.id}.capabilities.ik",
            f"声明 ik: false 却提供了 ik.type = {k.ik_type!r} 的求解器 ⇒ 能力声明撒谎",
        )
    if c.ik and not c.position:
        rep.error(
            "IK_WITHOUT_POSITION",
            f"{m.id}.capabilities.position",
            "有逆解却声明 position: false ⇒ 两份声明互相矛盾",
        )
    if c.position and not c.ik:
        rep.error(
            "POSITION_WITHOUT_IK",
            f"{m.id}.capabilities.position",
            "声明支持位置目标（position: true）却没有逆解（ik: false）"
            " —— 位置目标必须经逆解才能变成关节角（本例请改为 position: false，"
            "并在文档 / UI 里明确「只能关节空间定位」）",
        )
    if c.orientation and not c.ik:
        rep.error(
            "ORIENTATION_WITHOUT_IK",
            f"{m.id}.capabilities.orientation",
            "没有逆解却声明支持姿态目标（姿态逆解是逆解的子集，spec §12）",
        )
    if c.simulation and m.simulation.mjcf is None:
        rep.error(
            "SIMULATION_WITHOUT_MJCF",
            f"{m.id}.capabilities.simulation",
            "声明有仿真能力却没给 simulation.mjcf 指针",
        )


def _check_against_config(rep: ValidationReport, m: Manifest) -> None:
    """能力声明必须与 `robot.yaml` 的**事实**对得上（这是防"声明撒谎"的硬判据）。"""
    robotcfg = load_robotcfg()
    try:
        robot = robotcfg.load_robot(to_abs(m.model.config))
    except Exception as exc:  # robotcfg.ConfigError 及其它
        rep.error("ROBOT_CONFIG_INVALID", f"{m.id}.model.config", f"无法解析：{exc}")
        return

    roles = tuple(j.role for j in robot.joints)
    derived_gripper = "gripper" in roles
    if m.capabilities.gripper != derived_gripper:
        rep.error(
            "CAPABILITY_GRIPPER_MISMATCH",
            f"{m.id}.capabilities.gripper",
            f"manifest 声明 {m.capabilities.gripper}，而 {repo_relative(to_abs(m.model.config))} "
            f"的关节角色集是 {roles} ⇒ 推导为 {derived_gripper}",
        )

    dof = sum(1 for j in robot.joints if j.is_dof)
    if dof <= 0:
        rep.error(
            "NO_DOF",
            f"{m.id}.model.config",
            f"{repo_relative(to_abs(m.model.config))} 里一个 revolute 关节都没有（dof=0）",
        )

    # 一个 config 只能属于一个包：两个包指向同一份真值 = 两份"包身份"共享一份数据，
    # 于是"改这台"会静默改变"那台"。
    try:
        owner = find_manifest_for_config(m.model.config)
        if owner.id != m.id:
            rep.error(
                "CONFIG_SHARED_BY_TWO_PACKAGES",
                f"{m.id}.model.config",
                f"该 robot.yaml 同时也被包 {owner.id!r} 声明",
            )
    except PackageError as exc:
        rep.error("CONFIG_OWNER_UNRESOLVED", f"{m.id}.model.config", str(exc))


def _check_against_selector(rep: ValidationReport, m: Manifest) -> None:
    """manifest 与选择器必须逐字段一致（两处声明同一件事，不允许各说各话）。"""
    robotcfg = load_robotcfg()
    try:
        sel = robotcfg.load_robot_selector()
    except Exception as exc:
        rep.error("SELECTOR_INVALID", "config/robots.yaml", f"无法解析：{exc}")
        return

    entry = sel.robots.get(m.id)
    if entry is None:
        rep.error(
            "NOT_IN_SELECTOR",
            f"{m.id}",
            f"包存在但 config/robots.yaml 未声明它（三端都读选择器 ⇒ 这台机器人对它们不存在）；"
            f"已声明: {' / '.join(sel.robots)}",
        )
        return

    def cmp(label: str, sel_val: str | None, man_val: str | None) -> None:
        a = None if sel_val is None else repo_relative(to_abs(sel_val))
        b = None if man_val is None else repo_relative(to_abs(man_val))
        if a != b:
            rep.error(
                "SELECTOR_MANIFEST_MISMATCH",
                f"{m.id}.{label}",
                f"选择器说 {a!r}，manifest 说 {b!r} —— 必须一致（否则加载路径二义）",
            )

    if entry.name != m.name:
        rep.error(
            "SELECTOR_NAME_MISMATCH",
            f"{m.id}.name",
            f"选择器说 {entry.name!r}，manifest 说 {m.name!r}",
        )
    cmp("model.config", entry.config_path, m.model.config)
    cmp("model.physics", entry.physics_path, m.model.physics)
    cmp("simulation.mjcf", entry.mjcf_path, m.simulation.mjcf)
    if entry.tcp_site != m.simulation.tcp_site:
        rep.error(
            "SELECTOR_TCPSITE_MISMATCH",
            f"{m.id}.simulation.tcp_site",
            f"选择器说 {entry.tcp_site!r}，manifest 说 {m.simulation.tcp_site!r}"
            "（site 名写错 ⇒ FK 与 MuJoCo 各算一个点，且都宣称「成功」）",
        )

    # model.config 指向的文件里 `robot.id` 与包 id **有意不同**（mearm-v1 vs mearm）。
    # 这里不比对，只把两者都摊出来 —— 它是最容易搞混的一处（见 loader 模块 docstring）。
    try:
        robot = robotcfg.load_robot(to_abs(m.model.config))
        if robot.id == m.id and robot.id != entry.id:
            rep.warn(
                "MODEL_ID_EQUALS_PACKAGE_ID",
                f"{m.id}.model.config",
                f"模型 id 与包 id 恰好相同（{robot.id!r}）—— 历史里两者是不同的，"
                "确认一下不是把包 id 误填成了模型 id",
            )
    except Exception:
        pass  # 已在 _check_against_config 里报过


def validate_package(robot_id: str) -> ValidationReport:
    """校验一个包（不抛错；问题全部以 `Issue` 形式返回）。"""
    rep = ValidationReport(robot_id=robot_id)
    pdir = PACKAGES_DIR / robot_id
    if not pdir.is_dir():
        rep.error(
            "PACKAGE_DIR_MISSING",
            f"robot-package/{robot_id}",
            f"目录不存在；已登记的包: {' / '.join(list_package_ids()) or '(空)'}",
        )
        return rep

    try:
        m = load_manifest(robot_id)
    except PackageError as exc:
        rep.error("MANIFEST_INVALID", f"robot-package/{robot_id}/manifest.yaml", str(exc))
        return rep

    if m.id != robot_id:
        rep.error(
            "PACKAGE_ID_DIR_MISMATCH",
            f"robot-package/{robot_id}/manifest.yaml",
            f"目录名是 {robot_id!r}，而 manifest.id 是 {m.id!r}（两者必须相同："
            "包的定位靠目录名，声明不一致会让「装了 A 实际是 B」）",
        )

    _check_pointers(rep, m)
    _check_kinematics_decl(rep, m)
    _check_against_config(rep, m)
    _check_against_selector(rep, m)

    if rep.ok:
        try:
            rep.resolved = load_robot(robot_id)
        except Exception as exc:
            rep.error("RESOLVE_FAILED", robot_id, f"通过契约校验但无法解析成机器人：{exc}")
    return rep


def validate_all() -> list[ValidationReport]:
    """校验全部包 + "选择器里声明了却没有包"的反向缺口。"""
    robotcfg = load_robotcfg()
    sel_ids: list[str] = []
    try:
        sel = robotcfg.load_robot_selector()
        sel_ids = list(sel.robots)
    except Exception:
        pass  # 具体错误会在各个包的 _check_against_selector 里报

    pkg_ids = list_package_ids()
    reports = [validate_package(rid) for rid in sorted(set(sel_ids) | set(pkg_ids))]

    # 反向缺口：选择器声明了一台，但没有任何包 ⇒ 三端会去加载一个不存在的包
    for rid in sel_ids:
        if rid not in pkg_ids:
            rep = ValidationReport(robot_id=rid)
            rep.error(
                "SELECTOR_WITHOUT_PACKAGE",
                f"config/robots.yaml:{rid}",
                "选择器声明了这台机器人，但 robot-package/ 下没有对应的包"
                "（「配置里有、代码里没有」—— 必须显式报告，不能静默）",
            )
            reports.append(rep)
    return reports
