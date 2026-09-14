# -*- coding: utf-8 -*-
"""MeArm-V1 黄金基线（`robot-package/mearm-v1/tests/cases/*.json`）的 **Python 只读门面**。

与 `frontend/tests/helpers/mearmV1Baseline.ts` 一一对应。两条验收链
（前端 vitest / Python pytest）必须读**同一批文件** —— 否则「抽象前后行为一致」
会被拆成两套互不相干的标准，而两边各自都能自己绿。

本模块**只读**、不产生任何数值：期望值一律来自 `tools/gen_mearm_v1_baseline.py`
实跑采集的落盘文件（spec §6 / §16）。

目录**不在这里拼**：取自 `robot-package/mearm-v1/manifest.yaml` 的 `tests.cases`
（经 Core 的 `robopkg.declared_path`）。Phase 2 搬迁时正因如此只需动一处。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]           # tests/sim2sim -> tests -> MeArm-3D
_CORE_PY = ROOT / "core" / "python"
if str(_CORE_PY) not in sys.path:
    sys.path.insert(0, str(_CORE_PY))

from robopkg import declared_path                    # noqa: E402

#: 本门面**只**服务 MeArm-V1：显式写 id，不读选择器 `default`
#: （否则改 default 会让黄金数据静默换成另一台机器人的快照）。
MEARM_V1 = "mearm-v1"


def baseline_dir() -> Path:
    """MeArm-V1 的用例目录 = 包在 manifest 里声明的 `tests.cases`。"""
    return declared_path(MEARM_V1, "tests.cases")

#: 冻结时使用的固定 seed（可复现性的锚点；换它等于换了一整套数据）
SEED = 20260914

FILES: dict[str, str] = {
    "joint": "joint_cases.json",
    "fk": "fk_cases.json",
    "ik": "ik_cases.json",
    "workspace": "workspace_cases.json",
}

_REQUIRED_META = ("model", "modelVersion", "robotId", "seed", "generator", "cases")


def load_doc(name: str) -> dict[str, Any]:
    """读取一份基线文件（含 `model` / `modelVersion` / `seed` 等元信息）。"""
    if name not in FILES:
        raise KeyError(f"未知基线文件 {name!r}（可用：{sorted(FILES)}）")
    path = baseline_dir() / FILES[name]
    if not path.is_file():
        raise FileNotFoundError(
            f"缺少黄金基线 {path}\n"
            "  生成：python robot-package/mearm-v1/tools/gen_mearm_v1_baseline.py\n"
            "  （需要同时具备 node 与 mujoco）"
        )
    doc = json.loads(path.read_text(encoding="utf-8"))
    missing = [k for k in _REQUIRED_META if k not in doc]
    if missing:
        raise ValueError(f"{path.name} 缺少字段 {missing}（文件被改动过？）")
    return doc


def load_cases(name: str) -> list[dict[str, Any]]:
    """只取用例数组。"""
    return list(load_doc(name)["cases"])


def tolerance(doc: dict[str, Any], key: str, default: float) -> float:
    """读基线**自带**的容差。

    刻意默认取文件里的值：容差是冻结契约的一部分，散落在测试文件里迟早与
    基线漂移。文件缺该键时用调用方给的保守缺省 —— 方向**只能是收紧**。
    """
    return float(doc.get("tolerances", {}).get(key, default))
