"""`manifest.yaml` 的加载与**严格**校验（spec §6）。

## manifest 是什么

> Robot Package 的**身份 + 能力 + 指针**。不是真值的第二份拷贝 ——
> 真值（几何/关节/限位/标定/物理量）永远在 `model.config` 指向的那份 `robot.yaml`。

## 三条纪律

1. **只放指针，不放数值**（见 `core/README.md` §二）。关节数 / 执行器数**不写**，
   由 `loader.py` 推导。
2. **缺字段报错，不兜底**。包括"字段名拼错" —— 拼错的字段会被**未知字段检查**抓住，
   否则 `tcpSite` 写成 `tcp_site` 会静默退化成默认值，而默认值恰好是 `"tcp"`，
   于是 SO-ARM101 的 TCP 会落到一个不存在于官方 MJCF 的 site 名上（运行期才炸）。
3. **`generated_by` 是"产物"的显式声明**。现状里"这份 MJCF 是生成的"这件事
   **只隐含在代码和测试里**（选择器没有这个字段），manifest 把它显式化 ——
   它决定内容哈希是否纳入该文件（产物不该进 hash，spec §19）。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml

from .errors import PackageError
from .root import PACKAGES_DIR, PROJECT_ROOT, repo_relative, to_abs

#: manifest 的格式版本。改动**不向后兼容**的结构时必须 +1。
MANIFEST_FORMAT = 1

#: 运动学实现类型
FK_TYPES = ("engine", "custom")
IK_TYPES = ("custom", "numeric", "none")

#: 能力字段（spec §26）
CAPABILITY_KEYS = ("position", "orientation", "gripper", "ik", "simulation", "hardware")

_TOP_KEYS = (
    "id",
    "name",
    "package",
    "robot",
    "model",
    "simulation",
    "kinematics",
    "capabilities",
    "tests",
)


# ---------------------------------------------------------------------------
# 严格取值小工具（缺字段/类型不符 ⇒ PackageError，绝不返回缺省）
# ---------------------------------------------------------------------------


def _mapping(raw: Any, where: str) -> Mapping[str, Any]:
    if not isinstance(raw, Mapping):
        raise PackageError(f"{where} 必须是映射（mapping）")
    return raw


def _unknown(m: Mapping[str, Any], allowed: tuple[str, ...], where: str) -> None:
    extra = [k for k in m if k not in allowed]
    if extra:
        raise PackageError(
            f"{where} 出现未知字段 {extra}；允许: {' / '.join(allowed)}"
            f"（字段名拼错是本项目最贵的一类静默错误 —— 故此处直接拒绝）"
        )


def _req_str(m: Mapping[str, Any], key: str, where: str) -> str:
    v = m.get(key)
    if not isinstance(v, str) or not v.strip():
        raise PackageError(f"{where}.{key} 必须是非空字符串（当前 = {v!r}）")
    return v


def _opt_str(m: Mapping[str, Any], key: str, where: str) -> str | None:
    v = m.get(key)
    if v is None:
        return None
    if not isinstance(v, str) or not v.strip():
        raise PackageError(f"{where}.{key} 若出现必须是非空字符串（当前 = {v!r}）")
    return v


def _req_bool(m: Mapping[str, Any], key: str, where: str) -> bool:
    v = m.get(key)
    if not isinstance(v, bool):
        raise PackageError(f"{where}.{key} 必须是布尔（当前 = {v!r}）；不接受 \"true\" 这类字符串")
    return v


def _opt_str_list(m: Mapping[str, Any], key: str, where: str) -> tuple[str, ...]:
    v = m.get(key)
    if v is None:
        return ()
    if not isinstance(v, list) or not all(isinstance(x, str) and x.strip() for x in v):
        raise PackageError(f"{where}.{key} 若是列表则每项必须是非空字符串（当前 = {v!r}）")
    return tuple(v)


# ---------------------------------------------------------------------------
# manifest 的数据形状
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ModelCfg:
    """运动学 / 标定 / 限位真值的**指针**。"""

    config: str
    physics: str | None
    #: 非空 = 该 `config` 是**生成物**（由这个脚本从上游派生）⇒ 不进内容哈希
    generated_by: str | None


@dataclass(frozen=True)
class SimulationCfg:
    """仿真模型的指针。"""

    mjcf: str | None
    #: 非空 = MJCF 是**产物**（由这个生成器从 `model.config` 派生）⇒ 不进内容哈希
    generated_by: str | None
    tcp_site: str


@dataclass(frozen=True)
class KinematicsCfg:
    #: 包内**运动学引擎**实现的仓库相对路径（TS）。
    #:
    #: 为什么它必须由 manifest 声明（而不是靠"约定目录名"自动找）：
    #: 前端 `RobotRegistry` 用 `import.meta.glob` **发现**引擎文件，而"发现到了"
    #: 与"这台机器人声明的引擎就是它"是两件事 —— 前者可以悄悄命中一个
    #: 属于别的机器人的同名文件。声明 + 对账（`test_registry_discovery_agrees_with_manifest`）
    #: 把这两件事钉在一起。
    engine_entry: str | None
    fk_type: str
    fk_entry: str | None
    ik_type: str
    ik_entry: str | None


@dataclass(frozen=True)
class Capabilities:
    position: bool
    orientation: bool
    gripper: bool
    ik: bool
    simulation: bool
    hardware: bool


@dataclass(frozen=True)
class TestsCfg:
    """**测试数据**的指针。测试**执行框架**在 Core（spec §10），不在这里。"""

    cases: str
    #: 冻结真值文件（本机 = `core/baseline/baseline-kinematics-physics.json`）；没有则 None
    frozen: str | None
    #: 该包自带的校验脚本（如"生成物与上游同步"的 `--check`）
    tools: tuple[str, ...]
    #: ★ 该包自带的**包内测试目录**（Phase 2 步⑤）；没有则 None。
    #:
    #: 判据是"这条断言换台机器人还成立吗"：不成立 ⇒ 随包走，放这里。
    #: 典型内容是**构造事实**（连杆命名、耦合结构、被动件派生出的反例窗口）。
    #: 机制（随机位形对比、格式契约…）留在 `tests/`，靠 manifest 的数据驱动。
    #:
    #: 与 `tools` 的区别：`tools` 是"可执行的校验脚本"（人/CI 手动跑），
    #: `local` 是"pytest 收集的测试代码"（`pytest.ini` 的 testpaths 覆盖到）。
    local: str | None


@dataclass(frozen=True)
class Manifest:
    id: str
    name: str
    format: int
    version: str
    robot_type: str
    model: ModelCfg
    simulation: SimulationCfg
    kinematics: KinematicsCfg
    capabilities: Capabilities
    tests: TestsCfg
    #: manifest 自己的绝对路径
    source_path: Path
    #: 包目录（= manifest 的父目录）
    package_dir: Path
    #: 参与内容哈希的**额外源**路径（生成物的上游；如官方 URDF 与网格目录）
    hash_sources: tuple[str, ...]

    @property
    def config_file(self) -> Path:
        return to_abs(self.model.config)

    @property
    def physics_file(self) -> Path | None:
        return None if self.model.physics is None else to_abs(self.model.physics)

    @property
    def mjcf_file(self) -> Path | None:
        return None if self.simulation.mjcf is None else to_abs(self.simulation.mjcf)


# ---------------------------------------------------------------------------
# 解析
# ---------------------------------------------------------------------------


def parse_manifest(raw: Any, *, source_path: Path | None = None) -> Manifest:
    """从已解析的 YAML 结构构造 `Manifest`（纯函数，便于单测）。"""
    where = str(source_path) if source_path is not None else "<manifest>"
    m = _mapping(raw, where)
    _unknown(m, _TOP_KEYS, where)

    pkg = _mapping(m.get("package"), f"{where}.package")
    _unknown(pkg, ("format", "version", "hash_sources"), f"{where}.package")
    fmt = pkg.get("format")
    if not isinstance(fmt, int) or isinstance(fmt, bool):
        raise PackageError(f"{where}.package.format 必须是整数（当前 = {fmt!r}）")
    if fmt != MANIFEST_FORMAT:
        raise PackageError(
            f"{where}.package.format = {fmt}，本工具只认 {MANIFEST_FORMAT}"
            f"（格式不兼容时必须显式升级，不许猜）"
        )

    robot = _mapping(m.get("robot"), f"{where}.robot")
    _unknown(robot, ("type",), f"{where}.robot")

    model_raw = _mapping(m.get("model"), f"{where}.model")
    _unknown(model_raw, ("config", "physics", "generated_by"), f"{where}.model")

    sim_raw = _mapping(m.get("simulation"), f"{where}.simulation")
    _unknown(sim_raw, ("mjcf", "generated_by", "tcp_site"), f"{where}.simulation")

    kin_raw = _mapping(m.get("kinematics"), f"{where}.kinematics")
    _unknown(kin_raw, ("engine", "fk", "ik"), f"{where}.kinematics")
    engine_raw = _mapping(kin_raw.get("engine"), f"{where}.kinematics.engine")
    _unknown(engine_raw, ("entry",), f"{where}.kinematics.engine")
    fk_raw = _mapping(kin_raw.get("fk"), f"{where}.kinematics.fk")
    _unknown(fk_raw, ("type", "entry"), f"{where}.kinematics.fk")
    ik_raw = _mapping(kin_raw.get("ik"), f"{where}.kinematics.ik")
    _unknown(ik_raw, ("type", "entry"), f"{where}.kinematics.ik")

    fk_type = _req_str(fk_raw, "type", f"{where}.kinematics.fk")
    ik_type = _req_str(ik_raw, "type", f"{where}.kinematics.ik")
    if fk_type not in FK_TYPES:
        raise PackageError(
            f"{where}.kinematics.fk.type = {fk_type!r}，允许: {' / '.join(FK_TYPES)}"
            "（engine = 用 Core 的通用链式 FK；custom = 包内自实现）"
        )
    if ik_type not in IK_TYPES:
        raise PackageError(
            f"{where}.kinematics.ik.type = {ik_type!r}，允许: {' / '.join(IK_TYPES)}"
            "（none = **诚实地没有**逆解，禁止伪造，spec §8/§35）"
        )

    caps_raw = _mapping(m.get("capabilities"), f"{where}.capabilities")
    _unknown(caps_raw, CAPABILITY_KEYS, f"{where}.capabilities")

    tests_raw = _mapping(m.get("tests"), f"{where}.tests")
    _unknown(tests_raw, ("cases", "frozen", "tools", "local"), f"{where}.tests")

    return Manifest(
        id=_req_str(m, "id", where),
        name=_req_str(m, "name", where),
        format=fmt,
        version=_req_str(pkg, "version", f"{where}.package"),
        robot_type=_req_str(robot, "type", f"{where}.robot"),
        model=ModelCfg(
            config=_req_str(model_raw, "config", f"{where}.model"),
            physics=_opt_str(model_raw, "physics", f"{where}.model"),
            generated_by=_opt_str(model_raw, "generated_by", f"{where}.model"),
        ),
        simulation=SimulationCfg(
            mjcf=_opt_str(sim_raw, "mjcf", f"{where}.simulation"),
            generated_by=_opt_str(sim_raw, "generated_by", f"{where}.simulation"),
            tcp_site=_req_str(sim_raw, "tcp_site", f"{where}.simulation"),
        ),
        kinematics=KinematicsCfg(
            engine_entry=_opt_str(engine_raw, "entry", f"{where}.kinematics.engine"),
            fk_type=fk_type,
            fk_entry=_opt_str(fk_raw, "entry", f"{where}.kinematics.fk"),
            ik_type=ik_type,
            ik_entry=_opt_str(ik_raw, "entry", f"{where}.kinematics.ik"),
        ),
        capabilities=Capabilities(
            **{k: _req_bool(caps_raw, k, f"{where}.capabilities") for k in CAPABILITY_KEYS}
        ),
        tests=TestsCfg(
            cases=_req_str(tests_raw, "cases", f"{where}.tests"),
            frozen=_opt_str(tests_raw, "frozen", f"{where}.tests"),
            tools=_opt_str_list(tests_raw, "tools", f"{where}.tests"),
            local=_opt_str(tests_raw, "local", f"{where}.tests"),
        ),
        source_path=(source_path or Path("<memory>")).resolve(),
        package_dir=(source_path.parent if source_path is not None else Path(".")).resolve(),
        hash_sources=_opt_str_list(pkg, "hash_sources", f"{where}.package"),
    )


def load_manifest(package_dir: Path | str) -> Manifest:
    """读 `<package_dir>/manifest.yaml`。

    未知/缺失一律报错：包目录不存在、manifest 不存在、YAML 不合法、字段不合法。
    """
    pdir = Path(package_dir)
    if not pdir.is_absolute():
        pdir = PACKAGES_DIR / pdir
    mpath = pdir / "manifest.yaml"
    if not mpath.is_file():
        raise PackageError(
            f"包 {pdir.name!r} 缺少 manifest.yaml（{mpath}）\n"
            f"  它声明「我是谁 / 我的真值在哪 / 我有什么能力」—— 没有它就不是一个 Robot Package"
        )
    try:
        raw = yaml.safe_load(mpath.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:  # pragma: no cover - 取决于 yaml 库版本
        raise PackageError(f"{mpath} 不是合法 YAML：{exc}") from exc
    return parse_manifest(raw, source_path=mpath)


def find_manifest_for_config(config_path: Path | str) -> Manifest:
    """按 `robot.yaml` 的路径反查所属包。

    ⚠️ 为什么需要它：**包的 id ≠ 模型 id**。选择器 key 是 `mearm-v1`，
    而 `robot.yaml` 里 `robot.id` 是 `mearm`。用模型 id 去反查包必然查不到，
    而"查不到就回退到第一台"是本项目明令禁止的静默行为。
    """
    target = repo_relative(to_abs(config_path))
    for m in load_all_manifests():
        if repo_relative(m.config_file) == target:
            return m
    raise PackageError(
        f"没有任何 Robot Package 声明 config = {target!r}；"
        f"已登记的: {[repo_relative(m.config_file) for m in load_all_manifests()]}"
    )


def list_package_dirs() -> list[Path]:
    """`robot-package/` 下的全部包目录（按 id 排序，结果稳定）。"""
    if not PACKAGES_DIR.is_dir():
        raise PackageError(f"robot-package/ 不存在：{PACKAGES_DIR}")
    return sorted((d for d in PACKAGES_DIR.iterdir() if d.is_dir()), key=lambda d: d.name)


def load_all_manifests() -> list[Manifest]:
    """读全部包的 manifest（按 id 排序）。任何一个坏掉都会抛错 —— **不跳过**。

    刻意不做"跳过坏包继续跑"：那会让"新加的包没被加载"表现成"这台机器人不存在"，
    而排查方向会被引到完全错误的地方。
    """
    return [load_manifest(d) for d in list_package_dirs()]
