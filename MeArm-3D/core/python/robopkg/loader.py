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
from .root import PACKAGES_DIR, load_robotcfg, repo_relative


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
