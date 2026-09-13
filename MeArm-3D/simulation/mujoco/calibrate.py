# -*- coding: utf-8 -*-
"""参数标定接口（spec §38）。

本阶段 `config/physics.yaml` 的 `calibration.calibrated` **恒为 false**：
所有质量 / 惯量 / 摩擦 / 增益都是公开值或估算值。本工具不是"自动标定器"，
而是把**标定流程**固化下来，让"哪些还是猜的"这件事有一份可执行的记录：

    --show           标定状态总览：哪些项已标定、每项该怎么测
    --template FILE  导出标定模板 JSON（实验时照着填）
    --apply FILE     校验模板并打印 **YAML 片段**

⚠️ `--apply` 刻意**不直接改文件**：`config/physics.yaml` 有一半价值在注释里
（每条数值都标了来源与理由）。用 pyyaml round-trip 写回会把注释全部抹掉 ——
那等于把"这个值为什么是这个值"一起删了，比数值本身更贵。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

PKG_DIR = Path(__file__).resolve().parent
if str(PKG_DIR) not in sys.path:
    sys.path.insert(0, str(PKG_DIR))

from robotcfg import ConfigError, load_physics, load_robot  # noqa: E402
from units import ensure_utf8_stdout  # noqa: E402

# 可标定项：键 → 说明 / 单位 / 实验方法。**与 physics.yaml 的 calibration 段一一对应**。
CALIBRATABLE: dict[str, dict[str, str]] = {
    "servo_offset": {
        "unit": "deg",
        "what": "逐关节舵机角偏置（修正安装偏心）",
        "how": "把关节摆在可达范围中点，读舵机角，与理论角相减",
    },
    "joint_scale": {
        "unit": "关节度/舵机度",
        "what": "逐关节角度增益修正（修正连杆比与机构误差）",
        "how": "扫舵机两个已知角，量对应的关节角变化，取比值",
    },
    "joint_zero": {
        "unit": "deg",
        "what": "逐关节机械零位（绝对角语义关节必需，如 elbow）",
        "how": "用相机测小臂绝对倾角，与 FK 名义值对照",
    },
    "max_velocity": {
        "unit": "rad/s",
        "what": "逐关节实测速度上限",
        "how": "给满幅阶跃，用相机/编码器测最大角速度",
    },
    "max_torque": {
        "unit": "N·m",
        "what": "逐关节实测堵转扭矩",
        "how": "顶住不动，读电流或挂砝码测力矩",
    },
    "damping": {
        "unit": "N·m·s/rad",
        "what": "逐关节实测阻尼（含减速箱粘性）",
        "how": "断电自由摆动，拟合衰减曲线",
    },
    "friction": {
        "unit": "N·m",
        "what": "逐关节实测库仑摩擦（含减速箱）",
        "how": "缓慢正反扫角，取力矩-角度回线的半宽",
    },
}


def collect_status(physics) -> tuple[bool, dict[str, Any], list[str]]:
    """返回 `(是否已标定, calibration 段, 仍未标定的键)`。"""
    cal = dict(physics.calibration)
    calibrated = bool(cal.get("calibrated", False))
    pending = [k for k in CALIBRATABLE if not cal.get(k)]
    return calibrated, cal, pending


def cmd_show(physics, robot) -> int:
    calibrated, cal, pending = collect_status(physics)
    print("=" * 78)
    print("物理参数标定状态")
    print("=" * 78)
    print(f"calibrated : {calibrated}"
          + ("   ← ★ 全部为公开值 / 估算值，不是真机标定模型" if not calibrated else ""))
    print(f"模型        : {robot.id} ({robot.name})   ← config/robot.yaml")
    print(f"物理参数    : {physics.source_path}")
    print()
    print(f"{'可标定项':<16s} {'状态':<8s} {'单位':<14s} 实验方法")
    print("-" * 78)
    for key, meta in CALIBRATABLE.items():
        filled = bool(cal.get(key))
        print(f"{key:<16s} {'已填' if filled else '未标定':<8s} {meta['unit']:<14s} {meta['how']}")
    print()
    if pending:
        print(f"仍未标定：{', '.join(pending)}")
        print("→ 这些项当前沿用 physics.yaml 里的公开值 / 估算值，")
        print("  仿真结果只应被解读为 Level 3（参数化物理），不是 Level 4~5（真机标定）。")
    return 0


def cmd_template(path: Path) -> int:
    tpl = {
        "_comment": "把实测值填进来，再跑 calibrate.py --apply <本文件>",
        "calibrated": False,
        "测量记录": {
            "日期": "",
            "方法": "",
            "备注": "填完把 calibrated 改成 true —— 只有真的测过才准改",
        },
    }
    for key, meta in CALIBRATABLE.items():
        tpl[key] = {f"<{meta['unit']}>": ""}
    path.write_text(json.dumps(tpl, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[calibrate] 模板已写出：{path}")
    for key, meta in CALIBRATABLE.items():
        print(f"    {key:<16s} {meta['what']}   （{meta['how']}）")
    return 0


def validate(data: dict[str, Any], robot) -> list[str]:
    """校验模板：键名合法、值可解析为浮点、关节 id 在 robot.yaml 里存在。"""
    errs: list[str] = []
    joint_ids = set(robot.joint_order())
    for key in data:
        if key.startswith("_") or key in ("calibrated", "测量记录"):
            continue
        if key not in CALIBRATABLE:
            errs.append(f"未知标定项 {key!r}（可选：{sorted(CALIBRATABLE)}）")
            continue
        body = data[key]
        if not isinstance(body, dict):
            errs.append(f"{key} 应为对象，实测 {type(body).__name__}")
            continue
        for jid, val in body.items():
            if jid not in joint_ids:
                errs.append(f"{key}.{jid} 不是 robot.yaml 里的可动关节"
                            f"（可选：{sorted(joint_ids)}）")
                continue
            try:
                float(val)
            except (TypeError, ValueError):
                errs.append(f"{key}.{jid} 的值 {val!r} 不是数字")
    return errs


def cmd_apply(path: Path, robot) -> int:
    data = json.loads(path.read_text(encoding="utf-8"))
    errs = validate(data, robot)
    if errs:
        print("[calibrate] 模板校验失败：")
        for e in errs:
            print(f"    ✗ {e}")
        return 1

    if not data.get("calibrated"):
        print("[calibrate] ⚠️ 模板里 calibrated 仍为 false。")
        print("            只有真的做过实验、且值来自测量时才应改成 true。")
        print("            （spec §37：不许把估算值谎称成标定结果）")

    lines = ["# ---- 把下面这段**手工**贴进 config/physics.yaml 的 calibration 段 ----",
             "# （刻意不自动写入：pyyaml round-trip 会吃掉该文件里全部注释）",
             "calibration:",
             f"  calibrated: {str(bool(data.get('calibrated'))).lower()}"]
    for key in CALIBRATABLE:
        body = data.get(key) or {}
        body = {k: v for k, v in body.items() if v not in ("", None)}
        if not body:
            lines.append(f"  {key}: {{}}")
        else:
            inner = ", ".join(f"{k}: {float(v):.6g}" for k, v in body.items())
            lines.append(f"  {key}: {{{inner}}}")
    print("\n".join(lines))
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="物理参数标定接口（spec §38）")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--show", action="store_true", help="标定状态总览（默认）")
    g.add_argument("--template", metavar="FILE", help="导出标定模板 JSON")
    g.add_argument("--apply", metavar="FILE", help="校验模板并打印 YAML 片段")
    args = p.parse_args(argv)

    ensure_utf8_stdout()
    try:
        physics = load_physics()
        robot = load_robot()
    except ConfigError as exc:
        print(f"[calibrate] 配置错误：{exc}")
        return 2

    if args.template:
        return cmd_template(Path(args.template))
    if args.apply:
        return cmd_apply(Path(args.apply), robot)
    return cmd_show(physics, robot)


if __name__ == "__main__":
    raise SystemExit(main())
