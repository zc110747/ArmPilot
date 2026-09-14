# -*- coding: utf-8 -*-
"""`robot-package/<id>/tests/` 的夹具 —— **复用** `tests/sim/conftest.py`（同一模式）。

## 这个目录是干什么的（Phase 2 步⑤）

`tests/sim/` 里混着两类断言：

  * **机制**（mechanism）—— "随机位形的 FK 与 MuJoCo 逐位一致"
    「限位越界必须被上层拒绝」。它对任何机器人都成立，只是**数据**不同 ⇒ 留在 Core。
  * **期望值 / 结构性事实**（expectation）—— "零位 TCP 的竖直段是
    `column + upper_arm + forearm`"「`elbow` 存绝对角、与 `shoulder` 经平行四连杆
    耦合」「被动腕把反例窗口压到 2°」。这些是**这台机器人**的构造事实 ⇒ 随包走。

判据很简单：**把一条断言放到另一台机器人上，它还是"对的说法"吗？**
不是 ⇒ 它属于包。

## 为什么夹具是**复用**而不是重写

`robot` / `physics` / `sim` / `kinematics` 背后是同一条纪律：真值只有
`robot.yaml` 一份、MuJoCo 实例每个测试一个（防状态污染）、前端运动学必须
加载**真实源码**。复制成两份迟早漂移，而漂移的表现是"两套验收各自验出不同结论"。
⇒ 与 `tests/sim2sim/conftest.py` 用**完全相同**的手法：
按文件路径显式加载共享 conftest，再把夹具对象绑定到本命名空间。

## ⚠️ 复用带来的一个必须防的坑

共享夹具里的 `robot` **写死** `mearm-v1`。所以本文件**只能**住在 `mearm-v1` 的包里 ——
一旦被原样拷到 `so-arm101/tests/`，这里的测试就会**以 SO-101 的名义测 MeArm**，
而且全绿。`test_package_identity` 专门盯这件事（见 `test_mearm_v1_structure.py`）。
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

#: `robot-package/<id>/tests/conftest.py` → `<id>`
#: 刻意**从目录名推**而不是写死字符串：拷到别的包时这份代码会立刻自曝身份。
PACKAGE_ID = Path(__file__).resolve().parents[1].name

_ROOT = Path(__file__).resolve().parents[3]          # robot-package/<id>/tests -> 仓库根
_SIM_TESTS = _ROOT / "tests" / "sim"
_SIM_DIR = _ROOT / "simulation" / "mujoco"
_CORE_PY = _ROOT / "core" / "python"

# 包内测试要 import harness / fkref / limits / robotcfg / units / ikbridge / model / robopkg
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
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


_shared = _load_shared_conftest()

# --- 夹具（共享 conftest 里那份定义）----------------------------------------
robot = _shared.robot
physics = _shared.physics
sim = _shared.sim
kinematics = _shared.kinematics

# --- 纯函数 / 常量（与 `tests/sim` 的用法一致：`from conftest import zero_pose`）---
zero_pose = _shared.zero_pose
SIM_DIR = _shared.SIM_DIR

#: 共享夹具里写死的机器人 id —— 供 `test_package_identity` 比对
SHARED_MEARM_ID = _shared.MEARM_V1
