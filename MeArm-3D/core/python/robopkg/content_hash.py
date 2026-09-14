"""Robot Package 的内容哈希（spec §19）。

## 为什么不能只比 `version`

版本是人手写的，而它**必然**会忘记跟手。改了一个限位却没升版本 ⇒
Working Robot 认为"没变" ⇒ 运行的是旧模型，而**没有任何报错**。
这与本项目最贵的一类错误（模型不对但一切都能跑）同源。

## 哈希的是什么（以及**刻意不**哈希什么）

| 进哈希 | 不进哈希 |
|---|---|
| `manifest.yaml` 自身 | `model.config` 若声明了 `generated_by`（它是产物） |
| 包目录下的全部文件 | `simulation.mjcf` 若声明了 `simulation.generated_by`（同上） |
| `model.config` / `model.physics`（未声明为产物时） | `dist/` `build/` `generated/` `node_modules/` `__pycache__/` |
| `simulation.mjcf`（未声明为产物时，如官方 MJCF） | 任何 `.git` / 缓存目录 |
| `kinematics.*.entry` / `tests.cases`（整目录）/ `tests.frozen` / `tests.tools` | |
| `package.hash_sources` 声明的上游源（如官方 URDF + 网格目录） | |

**判据是"它能不能被重新生成"**：能重新生成的（`mearm.xml` 由 `gen_model.py` 从
`robot.yaml` 派生）就不进 —— 否则生成器一旦不是字节级确定，hash 就会无意义地抖。
而不能重新生成的（官方 STL / URDF，逐字节入库）必须进 —— 它们才是"包变了"的真信号。

## 确定性要求（三条，都有测试盯）

1. **与文件系统返回顺序无关**：路径排序后依次喂入。
2. **与平台无关**：路径统一用 `/` 分隔（Windows 的 `\\` 会让同一棵树算出两个 hash）。
3. **与机器无关**：喂进去的是**仓库相对路径**，不是绝对路径
   （否则换台机器/换个盘符 hash 就变了，而 spec 要求跨平台稳定）。
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path

from .errors import PackageError
from .manifest import Manifest
from .root import PROJECT_ROOT, repo_relative, to_abs

#: 这些目录名下的内容**不进哈希**（spec §19 的"忽略 dist/build/generated"）
IGNORED_DIR_NAMES = frozenset(
    {
        "dist",
        "build",
        "generated",
        "node_modules",
        "__pycache__",
        ".git",
        ".pytest_cache",
        ".venv",
        ".workbuddy",
    }
)

#: 以这些后缀结尾的文件不进哈希（编辑器/系统垃圾，与"包内容"无关）
IGNORED_FILE_SUFFIXES = (".pyc", ".pyo", ".swp", ".tmp", ".log")


@dataclass(frozen=True)
class HashEntry:
    """哈希输入里的一条（供 CLI 摊开给人看）。"""

    rel_path: str
    size: int


@dataclass(frozen=True)
class HashResult:
    digest: str
    entries: tuple[HashEntry, ...]
    #: 被**显式排除**的"产物"路径 —— 说得出为什么没算它，而不是悄悄漏掉
    excluded_generated: tuple[str, ...]

    @property
    def file_count(self) -> int:
        return len(self.entries)

    @property
    def total_bytes(self) -> int:
        return sum(e.size for e in self.entries)


def _is_ignored(path: Path) -> bool:
    if path.name in IGNORED_DIR_NAMES:
        return True
    return path.suffix.lower() in IGNORED_FILE_SUFFIXES


def _walk_files(root: Path) -> list[Path]:
    """递归收集文件（跳过忽略目录），顺序稳定。"""
    out: list[Path] = []
    for child in sorted(root.iterdir(), key=lambda p: p.name):
        if _is_ignored(child):
            continue
        if child.is_dir():
            out.extend(_walk_files(child))
        elif child.is_file():
            out.append(child)
    return out


def expand_inputs(
    rel_paths: list[str],
    *,
    exclude: frozenset[str] = frozenset(),
) -> dict[str, Path]:
    """把"文件 + 目录"混在一起的输入**展开**成 `{仓库相对路径: 绝对路径}`。

    抽成公开函数是为了让"目录"与"文件"走**同一条**构造路径 ——
    两份实现迟早会在"目录要不要排序""要不要含空目录"上分叉。

    `exclude` 在**展开之后**才生效：这是刻意的 —— 目录扫描是无差别的，
    若只对"显式指针"做排除，包目录里那份同名产物会从另一条路重新进来，
    于是"排除"看起来生效了、其实没有。
    """
    collected: dict[str, Path] = {}
    for rel in rel_paths:
        abs_path = Path(rel) if Path(rel).is_absolute() else (PROJECT_ROOT / rel)
        if not abs_path.exists():
            raise PackageError(f"内容哈希的输入不存在：{rel} → {abs_path}")
        if abs_path.is_dir():
            for f in _walk_files(abs_path):
                if f.is_file():
                    collected[repo_relative(f)] = f
        else:
            collected[repo_relative(abs_path)] = abs_path

    for rel in exclude:
        collected.pop(repo_relative(rel), None)
    return collected


def hash_paths(rel_paths: list[str], *, exclude: frozenset[str] = frozenset()) -> tuple[str, list[HashEntry]]:
    """对一批仓库相对路径算内容哈希。返回 `(hex digest, 明细)`。"""
    collected = expand_inputs(rel_paths, exclude=exclude)
    h = hashlib.sha256()
    entries: list[HashEntry] = []
    for rel in sorted(collected):
        data = collected[rel].read_bytes()
        # 归一化：相对路径（`/` 分隔）+ 长度前缀 + 内容 —— 三者都进哈希，
        # 于是"改名"与"改内容"是两类不同的变化，都能被检出。
        h.update(rel.encode("utf-8"))
        h.update(b"\0")
        h.update(str(len(data)).encode("ascii"))
        h.update(b"\0")
        h.update(data)
        h.update(b"\0")
        entries.append(HashEntry(rel_path=rel, size=len(data)))
    return h.hexdigest(), entries


def compute_content_hash(manifest: Manifest) -> HashResult:
    """算一个包的完整内容哈希。

    ⚠️ 这里**不做任何"聪明的"省略**：只要一个声明的源不存在，就直接报错。
    "某个文件没了但 hash 照样算得出来"正是让陈旧检测静默失效的写法。
    """
    inputs: list[str] = [repo_relative(manifest.source_path)]
    #: 声明为"产物"的指针 —— 不进哈希，但要**记下来**（报告里可查）
    generated: set[str] = set()

    def add(rel: str | None, *, is_generated: bool, label: str) -> None:
        if rel is None:
            return
        abs_path = Path(rel) if Path(rel).is_absolute() else (PROJECT_ROOT / rel)
        rel_norm = repo_relative(abs_path)
        if not abs_path.exists():
            raise PackageError(f"{manifest.id}: {label} 指向的文件不存在：{rel_norm}")
        if is_generated:
            generated.add(rel_norm)
            return
        inputs.append(rel_norm)

    # model.config（robot.yaml）是**唯一真值源**，永远进哈希，绝不视为产物 ——
    # 即便 model.generated_by 被声明（它描述的是 model.urdf 的生成器，不是 model.config 的）。
    add(manifest.model.config, is_generated=False, label="model.config")
    # model.urdf 是**生成物**（由 model.generated_by 声明其生成器）：声明了就排除。
    add(manifest.model.urdf, is_generated=manifest.model.generated_by is not None, label="model.urdf")
    add(manifest.model.physics, is_generated=False, label="model.physics")
    add(
        manifest.simulation.mjcf,
        is_generated=manifest.simulation.generated_by is not None,
        label="simulation.mjcf",
    )
    add(manifest.kinematics.engine_entry, is_generated=False, label="kinematics.engine.entry")
    add(manifest.kinematics.fk_entry, is_generated=False, label="kinematics.fk.entry")
    add(manifest.kinematics.ik_entry, is_generated=False, label="kinematics.ik.entry")
    add(manifest.tests.cases, is_generated=False, label="tests.cases")
    add(manifest.tests.frozen, is_generated=False, label="tests.frozen")
    for t in manifest.tests.tools:
        add(t, is_generated=False, label="tests.tools")
    for src in manifest.hash_sources:
        add(src, is_generated=False, label="package.hash_sources")

    # 包目录自身的全部内容（Phase 2 后包是自包含的：包内任何文件都算包内容）
    if manifest.package_dir.is_dir():
        inputs.append(repo_relative(manifest.package_dir))

    # 生成物**必须**在展开后排掉：目录扫描是无差别的，否则同一份产物
    # 会从"包目录"这条路径重新进来，于是"排除"看起来生效了、其实没有。
    exclude = frozenset(generated)
    digest, entries = hash_paths(inputs, exclude=exclude)
    dropped = tuple(sorted(r for r in generated if r not in {e.rel_path for e in entries}))
    return HashResult(digest=digest, entries=tuple(entries), excluded_generated=dropped)
