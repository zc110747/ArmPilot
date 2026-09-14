"""`robopkg` —— Core 的 **Robot Package** 工具链（加载 / 校验 / 内容哈希）。

对应重构 spec：§6（manifest）/ §18–§19（陈旧检测与 hash）/ §23–§25（Registry / Loader / Runtime）。

设计纪律
--------
1. **只放机制，不放机器人**。本包不得出现任何型号名（`mearm` / `so-arm101` 字样只能出现在
   注释或测试期望值里，不能出现在逻辑分支里）。
2. **缺字段就报错，绝不兜底**（与 `robotcfg.py` 同一条纪律）。静默兜底会让"包写错了"
   表现成"机器人行为诡异"。
3. **路径只在 `root.py` 算一次** —— 三处各自 `parents[N]` 是布局一改就静默错位的经典来源。
"""

from .errors import PackageError
from .manifest import Manifest, load_manifest, load_all_manifests
from .loader import (
    ResolvedRobot,
    declared_path,
    declared_path_list_keys,
    declared_paths,
    declared_paths_keys,
    list_package_ids,
    load_robot,
    package_dir_of,
)
from .content_hash import compute_content_hash, hash_paths
from .validator import Issue, ValidationReport, validate_all, validate_package

__all__ = [
    "PackageError",
    "Manifest",
    "load_manifest",
    "load_all_manifests",
    "ResolvedRobot",
    "declared_path",
    "declared_paths",
    "declared_path_list_keys",
    "declared_paths_keys",
    "list_package_ids",
    "load_robot",
    "package_dir_of",
    "compute_content_hash",
    "hash_paths",
    "Issue",
    "ValidationReport",
    "validate_all",
    "validate_package",
]
