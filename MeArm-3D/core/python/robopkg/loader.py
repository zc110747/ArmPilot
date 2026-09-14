"""`RobotLoader` —— 包 id → **推导后的完整视图**（spec §24）。

## 它做什么

1. 读包的 `manifest.yaml`（身份 / 能力 / **指针**）
2. 按指针去读运动学真值（`robot.yaml`）
3. **推导**那些 manifest 刻意不写的量：自由度、关节序、被动件、执行器数
4. 算内容哈希

## 为什么"推导"而不是"抄进 manifest"

`dof` / `actuator_count` 这类量**能从 `robot.yaml` 算出来**。
把它们写进 manifest = 第二份真值，而它的漂移是静默的
（改了 `robot.yaml` 的关节表，manifest 里的 4 永远不会自己变成 5）。
⇒ 所以这里推导，**期望值放在测试里**（`core/tests/test_package_contract.py`）。

## 两个 id 不是一回事（必须记住）

| 名字 | 例 | 来源 | 用途 |
|---|---|---|---|
| **包 id / 选择器 key** | `mearm-v1` | `robot-package/` 的目录名 + `config/robots.yaml` 的 key | 找包、装 Working Robot |
| **模型 id** | `mearm` | `robot.yaml → robot.id` | 模型内部标识 |

用模型 id 去反查包**必然查不到**，而"查不到就回退"是本项目明令禁止的静默行为
（见 `manifest.find_manifest_for_config`）。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .content_hash import compute_content_hash
from .errors import PackageError
from .manifest import Manifest, list_package_dirs, load_manifest, load_all_manifests
from .root import PACKAGES_DIR, load_robotcfg, repo_relative, to_abs


@dataclass(frozen=True)
class ResolvedRobot:
    """一台机器人「包 + 真值 + 推导结果」的完整视图。"""

    manifest: Manifest
    #: `robot.yaml` 的解析结果（运动学真值）
    robot: object  # robotcfg.RobotCfg（避免在类型标注处 import 以保持模块可独立测试）
    #: 选择器里的显示名（与 manifest.name 交叉校验后应当相等）
    selector_name: str

    # ---- 由 robot.yaml 推导（**不是**从 manifest 抄的）----
    model_id: str
    model_version: str | None
    units: str
    #: 有独立自由度的关节（`type == "revolute"`）—— 与前端 `isMovableJoint()`、
    #: Python `JointCfg.is_dof`、Go `JointOrder()` 是**同一条规则**，三处必须一致
    dof: int
    dof_joints: tuple[str, ...]
    #: 在 MJCF 里产生 `qpos` 的关节（含被动件；与 `dof_joints` **不是**一回事）
    qpos_joints: tuple[str, ...]
    passive_joints: tuple[str, ...]
    all_joints: tuple[str, ...]
    roles: tuple[str, ...]
    actuator_count: int
    actuator_ids: tuple[str, ...]
    has_gripper: bool

    # ---- 内容哈希 ----
    content_hash: str
    hash_file_count: int

    @property
    def id(self) -> str:
        return self.manifest.id

    @property
    def summary_line(self) -> str:
        return (
            f"{self.id} · model={self.model_id} v{self.model_version} · "
            f"dof={self.dof} qpos={len(self.qpos_joints)} act={self.actuator_count} · "
            f"ik={'✔' if self.manifest.capabilities.ik else '✘'} · hash={self.content_hash[:12]}"
        )


def list_package_ids() -> list[str]:
    """`robot-package/` 下全部包 id（= 目录名）。"""
    return [d.name for d in list_package_dirs()]


def load_robot(robot_id: str | None = None) -> ResolvedRobot:
    """按**包 id** 加载。

    `robot_id=None` ⇒ 取选择器的 `default`（**注意**：Phase 3 起"当前机器人"
    将由 `working-robot/manifest.yaml` 决定，`default` 降级为"没有运行实例时的裸运行缺省"，
    见 ADR「活动机器人是 store 状态」与 D-C 决策）。
    """
    robotcfg = load_robotcfg()
    entry = robotcfg.resolve_robot_entry(robot_id)  # 未知 id ⇒ 抛错（不回退）
    manifest = load_manifest(entry.id)
    return _resolve(manifest, entry, robotcfg)


def load_all() -> list[ResolvedRobot]:
    """全部包（按 id 排序）。任何一个坏掉都会抛错 —— **不跳过**。"""
    robotcfg = load_robotcfg()
    out: list[ResolvedRobot] = []
    for m in load_all_manifests():
        entry = robotcfg.resolve_robot_entry(m.id)
        out.append(_resolve(m, entry, robotcfg))
    return out


def _resolve(manifest: Manifest, entry: object, robotcfg: object) -> ResolvedRobot:
    robot = robotcfg.load_robot(entry.config_file)  # type: ignore[attr-defined]
    joints = list(robot.joints)
    dof_joints = tuple(j.id for j in joints if j.is_dof)
    qpos_joints = tuple(j.id for j in joints if j.has_qpos)
    passive_joints = tuple(j.id for j in joints if j.is_passive)
    roles = tuple(j.role for j in joints)
    actuators = list(robot.actuators)

    hr = compute_content_hash(manifest)
    return ResolvedRobot(
        manifest=manifest,
        robot=robot,
        selector_name=entry.name,  # type: ignore[attr-defined]
        model_id=robot.id,
        model_version=robot.model_version,
        units=robot.units,
        dof=len(dof_joints),
        dof_joints=dof_joints,
        qpos_joints=qpos_joints,
        passive_joints=passive_joints,
        all_joints=tuple(j.id for j in joints),
        roles=roles,
        actuator_count=len(actuators),
        actuator_ids=tuple(a.id for a in actuators),
        has_gripper="gripper" in roles,
        content_hash=hr.digest,
        hash_file_count=hr.file_count,
    )


def package_dir_of(robot_id: str) -> Path:
    """包目录（不存在 ⇒ 报错，并列出可选 id）。"""
    d = PACKAGES_DIR / robot_id
    if not d.is_dir():
        raise PackageError(
            f"没有名为 {robot_id!r} 的 Robot Package（{repo_relative(d)} 不存在）；"
            f"可选: {' / '.join(list_package_ids()) or '(空)'}"
        )
    return d


# ---------------------------------------------------------------------------
# 声明路径的解析（**唯一**一处把 manifest 里的相对路径变成绝对路径的地方）
# ---------------------------------------------------------------------------

#: manifest 里所有"路径型"声明（点号寻址 → 取值函数）。**单值**字段。
#:
#: ⚠️ **新增路径字段必须登记在这里**，否则 `declared_path()` 直接报错。
#: 这是与 `manifest._unknown()` 同一条纪律：漏登记的字段会静默退化成
#: "用上一个默认路径"，而那种错误的表现是"读到了另一个文件"，比崩溃难查得多。
#:
#: ★ Phase 2 补登记：`generated_by`（两处）与 `kinematics.*.entry` 本来就是**路径**，
#:   但此前没人用 `declared_path()` 去解析它们 —— 于是它们在搬迁时**不会**报错。
#:   本轮实测到了代价：`gen_model.py` 从 `simulation/mujoco/` 搬进包内后，
#:   它自己那句"生成到我旁边"会静默改写产物位置（详见 gen_model.default_out 注释）。
_DECLARED_PATHS: dict[str, object] = {
    "model.config": lambda m: m.model.config,
    "model.physics": lambda m: m.model.physics,
    "model.generated_by": lambda m: m.model.generated_by,
    "simulation.mjcf": lambda m: m.simulation.mjcf,
    "simulation.generated_by": lambda m: m.simulation.generated_by,
    "kinematics.engine.entry": lambda m: m.kinematics.engine_entry,
    "kinematics.fk.entry": lambda m: m.kinematics.fk_entry,
    "kinematics.ik.entry": lambda m: m.kinematics.ik_entry,
    "tests.cases": lambda m: m.tests.cases,
    "tests.frozen": lambda m: m.tests.frozen,
    # ★ Phase 2 步⑤：包自带的**包内测试目录**（"期望值随包"的落脚点）。
    #   登记它不是为了读它，而是为了让它**可被断言**：契约测试会核对
    #   这个目录确实存在、且里面真的有 `test_*.py` ——
    #   否则"测试随包走"会退化成"manifest 里写了一个没人看的字符串"。
    "tests.local": lambda m: m.tests.local,
}

#: **列表型**路径字段（同一个键下有多个路径）。
_DECLARED_PATH_LISTS: dict[str, object] = {
    "tests.tools": lambda m: m.tests.tools,
}


def declared_paths_keys() -> tuple[str, ...]:
    """可用于 `declared_path()` 的键（供 CLI / 测试与文档对齐）。"""
    return tuple(sorted(_DECLARED_PATHS))


def declared_path_list_keys() -> tuple[str, ...]:
    """可用于 `declared_paths()` 的键（列表型）。"""
    return tuple(sorted(_DECLARED_PATH_LISTS))


def declared_path(robot_id: str | None, key: str, *, must_exist: bool = True) -> Path:
    """取包在 manifest 里**声明**的某个路径的绝对路径。

    为什么必须有这一个函数：路径一旦散落在各处自己拼，搬迁时就得同步改 N 处，
    而**漏改的那一处不会报错** —— 它会指向一个仍然存在（但已属于另一台机器人）
    的文件。所以：**路径只声明在 manifest，解析只发生在这里。**

    `must_exist=True`（缺省）时不存在即报错 —— 这是刻意的：本项目里
    "路径写错" 应当立刻炸，而不是等到读文件那一刻。
    """
    fn = _DECLARED_PATHS.get(key)
    if fn is None:
        raise PackageError(
            f"未登记的声明路径键 {key!r}；已登记: {' / '.join(declared_paths_keys())}"
            "（新路径字段请先加进 loader._DECLARED_PATHS）"
        )
    manifest = load_manifest(package_dir_of(_require_id(robot_id)))
    rel = fn(manifest)  # type: ignore[operator]
    if rel is None:
        raise PackageError(f"{manifest.id}.{key} 未声明（manifest 里是空的）")
    p = to_abs(rel)
    if must_exist and not p.exists():
        raise PackageError(
            f"{manifest.id}.{key} 指向的路径不存在：{rel}（解析为 {p}）"
        )
    return p


def _require_id(robot_id: str | None) -> str:
    """包 id 缺省 ⇒ 取选择器 `default`（与 `load_robot(None)` 同一语义）。"""
    if robot_id is not None:
        return robot_id
    return str(load_robotcfg().load_robot_selector().default)


def declared_paths(robot_id: str | None, key: str, *, must_exist: bool = True) -> tuple[Path, ...]:
    """**列表型**声明路径（如 `tests.tools`）→ 一组绝对路径。

    与 `declared_path()` 同一条纪律，只是键下的值是一个列表。空列表**不是**错误 ——
    它表示"这个包没有这类路径"（如 SO-101 没有 `tests.frozen`）。
    """
    fn = _DECLARED_PATH_LISTS.get(key)
    if fn is None:
        raise PackageError(
            f"未登记的**列表型**声明路径键 {key!r}；已登记: "
            f"{' / '.join(declared_path_list_keys()) or '(空)'}"
            "（新字段请先加进 loader._DECLARED_PATH_LISTS）"
        )
    manifest = load_manifest(package_dir_of(_require_id(robot_id)))
    rels = fn(manifest)  # type: ignore[operator]
    out: list[Path] = []
    for rel in rels:
        p = to_abs(rel)
        if must_exist and not p.exists():
            raise PackageError(
                f"{manifest.id}.{key} 里声明的 {rel!r} 不存在（解析为 {p}）"
            )
        out.append(p)
    return tuple(out)
