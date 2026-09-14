"""`robopkg` 的命令行入口 —— `update.bat` 与人工诊断共用。

```bash
python core/python/robopkg/cli.py list
python core/python/robopkg/cli.py show mearm-v1
python core/python/robopkg/cli.py validate --all
python core/python/robopkg/cli.py hash mearm-v1
```

退出码：`0` = 通过，`1` = 有错误。**不吞错误**（脚本调用方靠退出码判断）。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# 允许 `python core/python/robopkg/cli.py …` 直接执行（不经包机制）
_PY_DIR = Path(__file__).resolve().parent.parent
if str(_PY_DIR) not in sys.path:
    sys.path.insert(0, str(_PY_DIR))

from robopkg import (  # noqa: E402
    PackageError,
    list_package_ids,
    load_all_manifests,
    load_manifest,
    load_robot,
    validate_all,
    validate_package,
)
from robopkg.content_hash import compute_content_hash  # noqa: E402
from robopkg.root import PROJECT_ROOT, selftest  # noqa: E402


def _cmd_list(_args: argparse.Namespace) -> int:
    ids = list_package_ids()
    manifests = {m.id for m in load_all_manifests()}
    print(f"robot-package/ 下 {len(ids)} 个包：")
    for rid in ids:
        m = load_manifest(rid)
        caps = m.capabilities
        flags = "".join(
            [
                "P" if caps.position else "-",
                "O" if caps.orientation else "-",
                "G" if caps.gripper else "-",
                "I" if caps.ik else "-",
                "S" if caps.simulation else "-",
                "H" if caps.hardware else "-",
            ]
        )
        mark = "" if rid in manifests else "  (manifest 缺失)"
        print(f"  {rid:<14} {m.name:<14} v{m.version:<8} caps={flags}{mark}")
    print("  caps 顺序：position / orientation / gripper / ik / simulation / hardware")
    return 0


def _cmd_show(args: argparse.Namespace) -> int:
    r = load_robot(args.robot)
    m = r.manifest
    print(f"包      : {r.id}  ({m.name} v{m.version}, format {m.format})")
    print(f"包目录  : {m.package_dir.relative_to(PROJECT_ROOT).as_posix()}")
    print(f"模型    : id={r.model_id} model={m.name} version={r.model_version} units={r.units}")
    print(f"自由度  : dof={r.dof} {list(r.dof_joints)}")
    print(f"qpos    : {len(r.qpos_joints)} {list(r.qpos_joints)}")
    print(f"被动件  : {list(r.passive_joints) or '(无)'}")
    print(f"执行器  : {r.actuator_count} {list(r.actuator_ids)}")
    print(f"真值    : config={m.model.config}")
    print(f"          physics={m.model.physics or '(无)'}")
    if m.model.generated_by:
        print(f"          ⚠️ config 是**生成物**，生成器 = {m.model.generated_by}")
    print(f"仿真    : mjcf={m.simulation.mjcf or '(无)'} tcpSite={m.simulation.tcp_site}")
    if m.simulation.generated_by:
        print(f"          ⚠️ mjcf 是**产物**，生成器 = {m.simulation.generated_by}")
    print(f"运动学  : fk={m.kinematics.fk_type} ik={m.kinematics.ik_type}")
    if m.kinematics.ik_entry:
        print(f"          ik.entry={m.kinematics.ik_entry}")
    caps = m.capabilities
    print(
        "能力    : "
        f"position={caps.position} orientation={caps.orientation} gripper={caps.gripper} "
        f"ik={caps.ik} simulation={caps.simulation} hardware={caps.hardware}"
    )
    print(f"测试    : cases={m.tests.cases} frozen={m.tests.frozen or '(无)'}")
    print(f"内容哈希: {r.content_hash}  （{r.hash_file_count} 个文件进哈希）")
    return 0


def _cmd_validate(args: argparse.Namespace) -> int:
    reports = validate_all() if args.all or not args.robots else [validate_package(r) for r in args.robots]
    bad = 0
    for rep in reports:
        print(rep.summary())
        if not rep.ok:
            bad += 1
    print()
    print(f"合计 {len(reports)} 个包：{len(reports) - bad} 通过 / {bad} 有问题")
    return 1 if bad else 0


def _cmd_hash(args: argparse.Namespace) -> int:
    m = load_manifest(args.robot)
    hr = compute_content_hash(m)
    if args.json:
        print(
            json.dumps(
                {
                    "id": m.id,
                    "version": m.version,
                    "hash": hr.digest,
                    "fileCount": hr.file_count,
                    "totalBytes": hr.total_bytes,
                    "entries": [{"path": e.rel_path, "size": e.size} for e in hr.entries],
                    "excludedGenerated": list(hr.excluded_generated),
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0
    print(f"{m.id} v{m.version}")
    print(f"hash = {hr.digest}")
    print(f"输入 {hr.file_count} 个文件 / {hr.total_bytes} 字节：")
    for e in hr.entries:
        print(f"  {e.rel_path}  ({e.size} B)")
    if hr.excluded_generated:
        print("已排除的**产物**（不进哈希）：")
        for p in hr.excluded_generated:
            print(f"  {p}")
    return 0


def _cmd_selftest(_args: argparse.Namespace) -> int:
    problems = selftest()
    if problems:
        print("✘ 路径锚点自检失败：")
        for p in problems:
            print(f"  - {p}")
        return 1
    print("✓ 路径锚点自检通过（config/ core/ robot-package/ 都能从 PROJECT_ROOT 看到）")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="robopkg",
        description="Robot Package 工具链（加载 / 校验 / 内容哈希）",
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="列出全部包（不解析模型）").set_defaults(func=_cmd_list)

    ps = sub.add_parser("show", help="摊开一台机器人的推导结果")
    ps.add_argument("robot", help="包 id（如 mearm-v1）")
    ps.set_defaults(func=_cmd_show)

    pv = sub.add_parser("validate", help="包契约校验")
    pv.add_argument("robots", nargs="*", help="包 id（省略则配合 --all）")
    pv.add_argument("--all", action="store_true", help="校验全部包 + 反向缺口")
    pv.set_defaults(func=_cmd_validate)

    ph = sub.add_parser("hash", help="算内容哈希（陈旧检测用）")
    ph.add_argument("robot", help="包 id")
    ph.add_argument("--json", action="store_true", help="输出 JSON（供脚本消费）")
    ph.set_defaults(func=_cmd_hash)

    sub.add_parser("selftest", help="路径锚点自检").set_defaults(func=_cmd_selftest)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return int(args.func(args))
    except PackageError as exc:
        print(f"✘ {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
