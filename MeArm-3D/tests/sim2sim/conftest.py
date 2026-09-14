# -*- coding: utf-8 -*-
"""`tests/sim2sim/` 的夹具 —— **复用** `tests/sim/conftest.py` 里的那一份定义。

## 为什么不复制一份夹具

`robot` / `physics` / `sim` / `kinematics` 这四个夹具背后是同一条纪律：
模型只有 `config/robot.yaml` 一份、MuJoCo 实例每个测试一个（防状态污染）、
前端运动学必须加载**真实源码**而不是 Python 重写版。
复制成两份迟早漂移，而漂移的表现是"两套验收各自验出不同结论"。

## 怎么复用（这里有个坑）

**不能**写 `import conftest`：本目录没有 `__init__.py`，pytest 会把本目录放进
`sys.path`，于是 `conftest` 这个裸名会解析到**本文件自己**（`tests/sim2sim/conftest.py`），
形成循环导入，报成莫名其妙的 `AttributeError`。

改为按**文件路径**显式加载 `tests/sim/conftest.py`，再把其中的夹具对象绑定到本命名空间。
pytest 收集夹具时看的就是"conftest 模块命名空间里带 `_pytestfixturefunction` 标记的对象"，
所以绑定即生效（等价于 `from myfixtures import *` 那个常见写法）。
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

_ROOT = Path(__file__).resolve().parents[2]           # tests/sim2sim -> tests -> MeArm-3D
_SIM_TESTS = _ROOT / "tests" / "sim"
_SIM_DIR = _ROOT / "simulation" / "mujoco"
_CORE_PY = _ROOT / "core" / "python"

# 本目录的测试要 import harness / fkref / robotcfg / units / ikbridge / model / robopkg
for _p in (_SIM_DIR, _SIM_TESTS, _CORE_PY):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))


def _load_shared_conftest() -> ModuleType:
    path = _SIM_TESTS / "conftest.py"
    if not path.is_file():
        raise RuntimeError(f"找不到共享夹具定义：{path}")
    spec = importlib.util.spec_from_file_location("_armpilot_sim_conftest", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法为 {path} 建立模块 spec")
    mod = importlib.util.module_from_spec(spec)
    # 先登记再执行：模块内部若有相互引用可正常解析
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


_shared = _load_shared_conftest()

# --- 夹具 -------------------------------------------------------------------
robot = _shared.robot
physics = _shared.physics
sim = _shared.sim
kinematics = _shared.kinematics

# --- 纯函数 / 常量（与 tests/sim 的用法保持一致：`from conftest import zero_pose`）---
zero_pose = _shared.zero_pose
SIM_DIR = _shared.SIM_DIR
