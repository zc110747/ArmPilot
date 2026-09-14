#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统一 Sim2Sim 回归矩阵的命令行入口。

```bash
<python> tools/run_sim2sim.py --all                     # 选择器声明的全部机器人
<python> tools/run_sim2sim.py --robot mearm-v1          # 单台
<python> tools/run_sim2sim.py --all --json out.json     # 落盘完整报告
<python> tools/run_sim2sim.py --all --n-random 0        # 只跑显式枚举用例（快）
```

退出码：0 = 全部通过；1 = 有机器人 FK 超出其**登记容差**（见 `sim2sim.FK_TOL_MM`）。

⚠️ 它只**报告**，不修改任何配置/基线：Sim2Sim 的期望值来自落盘基线
（`tests/baseline/<robot>/`），本工具负责"今天的实现能不能复现它们"。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
for _p in (PROJECT_ROOT / "simulation" / "mujoco", PROJECT_ROOT / "tests" / "sim"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from sim2sim import run_sim2sim, sim2sim_matrix  # noqa: E402
from units import ensure_utf8_stdout             # noqa: E402

ensure_utf8_stdout()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="统一 Sim2Sim 回归矩阵（runSim2Sim(robot)）")
    ap.add_argument("--robot", action="append", default=[],
                    help="机器人 id（可重复）；省略且未给 --all 时用选择器的全部")
    ap.add_argument("--all", action="store_true", help="显式要求跑选择器声明的全部机器人")
    ap.add_argument("--n-random", type=int, default=24, help="随机位形用例数（缺省 24）")
    ap.add_argument("--json", type=Path, default=None, help="把完整报告写到该文件")
    ap.add_argument("--freeze", action="store_true",
                    help="把每台机器人的报告冻结到 tests/baseline/<robot>/sim2sim.json")
    ap.add_argument("--baseline-dir", type=Path,
                    default=PROJECT_ROOT / "tests" / "baseline",
                    help="--freeze 的目标根目录（缺省 tests/baseline）")
    args = ap.parse_args(argv)

    ids = args.robot or None
    print("[sim2sim] 统一框架 runSim2Sim(robot)")
    reports = sim2sim_matrix(ids, n_random=args.n_random)

    print(f"[sim2sim] 机器人 {len(reports)} 台 · 随机用例 {args.n_random} 组")
    for r in reports:
        print(f"  {r.line()}")
        if not r.ik_supported:
            print(f"    └ {r.ik_note}")

    bad = [r for r in reports if not r.fk_ok]
    for r in bad:
        print(
            f"\n✗ {r.robot_id}: FK 残差 {r.max_fk_ref_mujoco_mm:.6e} mm "
            f"超出登记容差 {r.tolerance_mm:g} mm\n  容差依据：{r.tolerance_reason}"
        )

    if args.json:
        payload = {
            "generator": "tools/run_sim2sim.py",
            "robots": [r.as_dict() for r in reports],
        }
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        print(f"[sim2sim] 报告 → {args.json}")

    if args.freeze:
        from datetime import datetime

        stamp = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %z")
        for r in reports:
            doc = {
                "generator": "tools/run_sim2sim.py --freeze",
                "frozenAt": stamp,
                "nRandomCases": args.n_random,
                "note": (
                    "本文件是 **Sim2Sim 行为快照**：由统一框架 runSim2Sim(robot) 实跑采集，"
                    "不是手算、不是推测。判据是「今天跑能否落在同一处」（按 robotId 对应的"
                    "登记容差），结构（用例 id / 顺序 / 能力声明）必须逐位一致。"
                ),
                **r.as_dict(),
            }
            out = args.baseline_dir / r.robot_id / "sim2sim.json"
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(json.dumps(doc, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print(f"[sim2sim] 冻结 → {out.relative_to(PROJECT_ROOT)}")

    if bad:
        print(f"\n✗ {len(bad)} 台机器人超出容差")
        return 1
    print("\n✓ 全部机器人 FK 在各自登记容差内")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
