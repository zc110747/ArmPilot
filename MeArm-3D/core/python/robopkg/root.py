"""路径锚点 —— **唯一**一处计算仓库根的地方。

为什么必须集中
--------------
现状里已经有两处各自推算仓库根：

    simulation/mujoco/robotcfg.py   PROJECT_ROOT = PKG_DIR.parents[1]
    backend/internal/robot/registry.go   RepoRoot = 选择器所在目录的上一级

两处各自假设"自己在第几层"。重构要搬目录（Phase 2），**每搬一次就有两处要同步改**，
而漏改的那处表现为"文件找不到"或者更糟 —— "找到了另一个同名文件"。

⇒ 所以 Core 侧新增的这一份只在这里算一次，并提供 `selftest()` 把
"我认为仓库根在哪"与"从已知文件反推出来的根"做一次对账（见 `test_package_contract.py`）。

Phase 2 说明
------------
`load_robotcfg()` 是本阶段的一处**临时桥**：Core 需要读 `robot.yaml`，而
`robotcfg.py` 此刻还在 `simulation/mujoco/`。搬完之后这个函数退化成一行 import，
**调用方不用改**。
"""

from __future__ import annotations

import sys
from pathlib import Path
from types import ModuleType

#: 本文件所在目录 = <root>/core/python/robopkg
PKG_DIR = Path(__file__).resolve().parent
#: <root>/core/python
PY_DIR = PKG_DIR.parent
#: <root>/core
CORE_DIR = PY_DIR.parent
#: 仓库根（= ArmPilot/MeArm-3D）
PROJECT_ROOT = CORE_DIR.parent

#: Robot Package 的根目录
PACKAGES_DIR = PROJECT_ROOT / "robot-package"
#: 模型选择器（三端共用；它自己声明"当前该加载哪台"）
SELECTOR_YAML = PROJECT_ROOT / "config" / "robots.yaml"

#: Phase 2 之前，机器人真值读取器仍在 simulation/mujoco 下
_LEGACY_SIM_DIR = PROJECT_ROOT / "simulation" / "mujoco"


def repo_relative(path: Path | str) -> str:
    """绝对路径 → 仓库相对路径（正斜杠，跨平台稳定）。

    manifest / 选择器里的路径一律是仓库相对路径，比较时必须先规范化 ——
    `pkg\\sub\\a.yaml` 与 `pkg/sub/a.yaml` 是同一个文件，
    但 `Path(...)` 在 Windows 上会给出反斜杠形式，直接比字符串会得到假差异。
    （示例刻意用**抽象路径**：写一个真实的真值路径会在随包搬迁后变成过期注释。）
    """
    p = Path(path)
    try:
        rel = p.resolve().relative_to(PROJECT_ROOT)
    except ValueError:
        return str(p).replace("\\", "/")
    return rel.as_posix()


def to_abs(rel_or_abs: str | Path) -> Path:
    """仓库相对路径 → 绝对路径（绝对路径原样返回）。"""
    p = Path(rel_or_abs)
    return p if p.is_absolute() else (PROJECT_ROOT / p)


def load_robotcfg() -> ModuleType:
    """取出 `robotcfg` 模块（**临时桥**，见模块 docstring 的 Phase 2 说明）。"""
    if str(_LEGACY_SIM_DIR) not in sys.path:
        sys.path.insert(0, str(_LEGACY_SIM_DIR))
    import robotcfg  # noqa: PLC0415  （刻意延迟 import：避免 import 期副作用）

    return robotcfg


def selftest() -> list[str]:
    """对账：返回"锚点看起来不对"的理由列表（空 = 正常）。

    判据刻意**不**用"这几个路径都存在"这种弱条件 —— 而用**反向推导**：
    从仓库根往下必须能看到 `config/`、`core/`、`robot-package/` 这三个
    重构后必须存在的锚点目录；少一个就说明 `PROJECT_ROOT` 算错了层。
    """
    problems: list[str] = []
    for name in ("config", "core", "robot-package"):
        if not (PROJECT_ROOT / name).is_dir():
            problems.append(f"{PROJECT_ROOT} 下看不到 {name}/ ⇒ 仓库根算错了层")
    if not SELECTOR_YAML.is_file():
        problems.append(f"选择器不存在：{SELECTOR_YAML}")
    return problems
