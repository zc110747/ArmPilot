# -*- coding: utf-8 -*-
"""独立 MuJoCo Viewer（spec §27）。

    python simulation/mujoco/run.py                        # 保持 HOME，只看
    python simulation/mujoco/run.py --demo                 # 跑内置演示序列
    python simulation/mujoco/run.py --pose "shoulder=30,elbow=120"
    python simulation/mujoco/run.py --headless --duration 3  # 不开 GUI，只出统计
    python simulation/mujoco/run.py --demo --record .workbuddy/sim-runs/demo.jsonl

统计行（每 `--print-interval` 秒一次）包含 spec §27 要求的全部五项：

    FPS · sim time · 关节角 · TCP(EE) XYZ · 接触数

⚠️ 时间尺度严格分层（spec §23 / §24）：
  * 物理步长固定 `physics.timestep`（1ms），**渲染帧率不决定物理步长**；
  * 渲染只在自己到点时才 `sync()`，与物理循环完全解耦；
  * 仿真时间 `t` 由**步数**累加，不用墙钟 —— 所以卡顿只会让实时倍率下降，
    不会让物理"跳步"或改变结果。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

PKG_DIR = Path(__file__).resolve().parent
if str(PKG_DIR) not in sys.path:
    sys.path.insert(0, str(PKG_DIR))

from model import MeArmSim  # noqa: E402
from record import Recorder  # noqa: E402
from robotcfg import RobotCfg  # noqa: E402
from units import ensure_utf8_stdout  # noqa: E402

DEFAULT_FPS = 60.0
DEFAULT_PRINT_INTERVAL = 0.5


# ---------------------------------------------------------------------------
# 目标脚本
# ---------------------------------------------------------------------------


def parse_pose(text: str, home: dict[str, float]) -> dict[str, float]:
    out = dict(home)
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "=" not in part:
            raise SystemExit(f"--pose 格式应为 'shoulder=30,elbow=120'，收到 {part!r}")
        k, v = (s.strip() for s in part.split("=", 1))
        if k not in home:
            raise SystemExit(f"--pose 里有未知关节 {k!r}（可选：{sorted(home)}）")
        out[k] = float(v)
    return out


def build_script(args, robot: RobotCfg) -> list[tuple[str, dict[str, float], float]]:
    """返回 `[(标签, 目标关节角, 持续秒数), ...]`。

    ⚠️ 演示序列里的角度**全部由 robot.yaml 的限位派生**，不写死数字 ——
    否则改一次硬件参数，这里就悄悄变成一份"影子真值"。
    """
    home = dict(robot.home_pose)

    if args.demo:
        bs = robot.joint("base")
        sh = robot.joint("shoulder")
        el = robot.joint("elbow")

        def pose(**kw) -> dict[str, float]:
            return {**home, **kw}

        return [
            ("HOME 悬空", home, 0.8),
            ("base 右转", pose(base=bs.limit_max * 0.5), 1.0),
            ("base 左转", pose(base=bs.limit_min * 0.5), 1.0),
            ("前伸下压（碰工作台）", pose(shoulder=sh.limit_max * 0.8,
                                          elbow=el.limit_max), 1.6),
            ("收臂", pose(shoulder=sh.limit_max * 0.2, elbow=el.limit_min), 1.2),
            ("回 HOME", home, 1.2),
        ]

    if args.pose:
        return [(f"--pose {args.pose}", parse_pose(args.pose, home), float("inf"))]
    return [("HOME 保持", home, float("inf"))]


# ---------------------------------------------------------------------------
# 统计行
# ---------------------------------------------------------------------------


def format_status(t: float, fps: float, ratio: float, state: dict, robot: RobotCfg,
                  label: str) -> str:
    ja = state["joint_angles"]
    ee = state["end_effector"]
    cols = "  ".join(f"{j[:4]:>4s}{ja[j]:8.2f}" for j in robot.joint_order())
    return (f"[t={t:8.3f}s ×{ratio:4.2f}] FPS{fps:6.1f} | {cols} | "
            f"TCP({ee['x'] * 1000:7.2f},{ee['y'] * 1000:7.2f},{ee['z'] * 1000:7.2f})mm | "
            f"接触{state['contact_count']} | {label}")


# ---------------------------------------------------------------------------
# 主循环
# ---------------------------------------------------------------------------


def run_loop(sim: MeArmSim, script, *, fps: float, print_interval: float,
             duration: float | None, headless: bool, fast: bool,
             recorder: Recorder | None) -> int:
    robot = sim.robot
    dt = float(sim.model.opt.timestep)
    batch = max(1, int(round(0.002 / dt)))          # 物理每批 2ms
    chunk = batch * dt

    viewer = None
    if not headless:
        try:
            import mujoco.viewer

            viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
        except Exception as exc:                    # noqa: BLE001
            print(f"[run] 打不开 Viewer（{exc}）→ 退回 headless 模式", flush=True)

    sim_t = 0.0
    seg_i, seg_t = 0, 0.0
    frames = 0
    fps_now = 0.0
    fps_t0 = time.perf_counter()
    last_sync = fps_t0
    last_print = -1e9
    next_t = time.perf_counter()
    wall0 = next_t
    frame_period = 1.0 / fps

    try:
        while True:
            if viewer is not None and not viewer.is_running():
                break

            label, target, seg_dur = script[seg_i]
            sim.set_target_joints(target)
            sim.step(batch)
            sim_t += chunk
            seg_t += chunk
            if seg_t >= seg_dur and seg_i < len(script) - 1:
                seg_i += 1
                seg_t = 0.0

            # --- 渲染（与物理解耦：只在到点时 sync） --------------------
            if viewer is not None:
                now = time.perf_counter()
                if now - last_sync >= frame_period:
                    viewer.sync()
                    last_sync = now
                    frames += 1

            now = time.perf_counter()
            if now - fps_t0 >= 1.0:
                fps_now = frames / (now - fps_t0) if viewer is not None else 0.0
                frames = 0
                fps_t0 = now

            if sim_t - last_print >= print_interval:
                last_print = sim_t
                state = sim.state().as_dict(robot)
                ratio = sim_t / max(now - wall0, 1e-9)
                print(format_status(sim_t, fps_now, ratio, state, robot, label),
                      flush=True)
                if recorder is not None:
                    recorder.write(state, wall_time=now - wall0)

            if duration is not None and sim_t >= duration:
                break

            # --- 实时对齐（物理侧，独立于渲染） -------------------------
            if not fast:
                next_t += chunk
                slack = next_t - time.perf_counter()
                if slack > 0:
                    time.sleep(slack)
                else:
                    next_t = time.perf_counter()
    except KeyboardInterrupt:
        print("\n[run] 收到中断，退出", flush=True)
    finally:
        if viewer is not None:
            viewer.close()

    print(f"[run] 结束：仿真 {sim_t:.3f}s，墙钟 {time.perf_counter() - wall0:.3f}s，"
          f"实时倍率 {(sim_t / max(time.perf_counter() - wall0, 1e-9)):.3f}", flush=True)
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="MeArm MuJoCo Viewer / 演示")
    p.add_argument("--demo", action="store_true", help="跑内置演示序列")
    p.add_argument("--pose", default=None, help="保持指定姿态，如 'shoulder=30,elbow=120'")
    p.add_argument("--duration", type=float, default=None, help="仿真时长（秒）后退出")
    p.add_argument("--fps", type=float, default=DEFAULT_FPS, help="渲染帧率上限（默认 60）")
    p.add_argument("--print-interval", type=float, default=DEFAULT_PRINT_INTERVAL,
                   help="统计行间隔（秒）")
    p.add_argument("--headless", action="store_true", help="不开 Viewer，仅统计")
    p.add_argument("--fast", action="store_true", help="不做实时对齐，尽快跑")
    p.add_argument("--record", default=None, help="把数据写入 JSONL / CSV（后缀决定）")
    args = p.parse_args(argv)

    ensure_utf8_stdout()
    sim = MeArmSim()
    script = build_script(args, sim.robot)

    recorder = Recorder(args.record, enabled=True) if args.record else None
    try:
        if recorder is not None:
            recorder.open()
        if args.headless:
            print(f"[run] headless · 目标脚本 {len(script)} 段 · "
                  f"字段 {recorder.fields if recorder else '—'}", flush=True)
        return run_loop(sim, script, fps=args.fps, print_interval=args.print_interval,
                        duration=args.duration, headless=args.headless, fast=args.fast,
                        recorder=recorder)
    finally:
        if recorder is not None:
            recorder.close()
            print(f"[run] 已写 {recorder.count} 条 → {recorder.path}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
