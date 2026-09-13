# -*- coding: utf-8 -*-
"""Phase 10 验收：系统级集成、可重复性与诚实声明。

覆盖 spec 的这些条目：
    §22 Test D  快速运动下的动力学稳定性（A/B/C 已在 test_gravity / test_actuator）
    §23 / §24   三个时间步分层（物理 1kHz / 控制 100Hz / 渲染 30-60FPS）**互相解耦**
    §32         Reset API 可重复
    §33         Deterministic Test（同模型+参数+初态+命令 ⇒ 一致结果）
    §34         数据记录只写 JSONL / CSV
    §37         不伪造"真实物理"：Level 声明的机器可检查形式

## 本文件的判据纪律

**"结果一致"必须按位比较**（`np.array_equal`），不能用 `allclose`。
仿真里"几乎一样"和"一样"是两种完全不同的结论：前者会在 1000 步后变成
肉眼可见的分叉，而 `allclose` 的 `atol` 又会被 `rtol` 淹没
（比较 1e2 量级的角度时 `atol=1e-6` 的真实容差 ≈ 1e-4）。
"""
from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from model import MeArmSim
from record import FIELD_SOURCE, Recorder, build_row, flatten
from robotcfg import load_physics

ROOT = Path(__file__).resolve().parents[2]
SIM_DIR = ROOT / "simulation" / "mujoco"


# ---------------------------------------------------------------------------
# 辅助
# ---------------------------------------------------------------------------


def fresh_sim(robot, physics) -> MeArmSim:
    """一个**全新**的仿真实例（不复用 `sim` 夹具：本文件要比对实例之间的一致性）。"""
    return MeArmSim(robot=robot, physics=physics)


def cmd_sequence(robot, n: int = 12) -> list[dict[str, float]]:
    """一段**确定性**的命令序列，角度全部由 robot.yaml 限位派生。"""
    order = [j.id for j in robot.movable_joints()]
    seq = []
    for k in range(n):
        frac = (k % 5) / 4.0                       # 0, .25, .5, .75, 1
        pose = {}
        for i, jid in enumerate(order):
            j = robot.joint(jid)
            lo, hi = j.limit_min, j.limit_max
            t = frac if (k + i) % 2 == 0 else 1.0 - frac
            pose[jid] = lo + t * (hi - lo)
        seq.append(pose)
    return seq


def run_sequence(sim: MeArmSim, seq, steps_per_cmd: int = 50) -> MeArmSim:
    for pose in seq:
        sim.set_target_joints(pose)
        sim.step(steps_per_cmd)
    return sim


# ---------------------------------------------------------------------------
# §23 / §24 三个时间步解耦
# ---------------------------------------------------------------------------


def test_timestep_layers_come_from_config(sim, physics):
    """物理步长**只能**来自 `physics.yaml`，且三层量级正确（1kHz / 100Hz / ~30FPS）。"""
    assert sim.model.opt.timestep == pytest.approx(physics.ts_physics, abs=0.0), (
        "模型步长必须逐位等于配置值 —— 任何四舍五入都意味着真值被抄了一份")
    assert physics.ts_physics == pytest.approx(0.001, abs=1e-12)
    assert physics.ts_control == pytest.approx(0.010, abs=1e-12)
    assert physics.ts_control > physics.ts_physics, "控制周期必须慢于物理步长"
    assert physics.ts_render >= physics.ts_physics
    # 控制周期必须是物理步长的整数倍，否则控制环会有长期相位漂移
    ratio = physics.ts_control / physics.ts_physics
    assert float(ratio) == pytest.approx(round(float(ratio)), abs=1e-9), (
        f"控制周期/物理步长 = {ratio} 不是整数 ⇒ 100Hz 控制环无法被 1kHz 物理精确整除")


def test_step_batching_does_not_change_result(robot, physics):
    """★ `step(1)` 连续调用 10 次 == `step(10)` 一次调用（按位相同）。

    这是那个**只在特定调用方式下才暴露**的静默错误的回归测试：
    控制周期累加器一旦写成局部变量，`step(1)` 每次从 0 开始、永远达不到 10ms 阈值，
    ⇒ ctrl 永不更新 ⇒ 命令完全不生效；而 `step(10)` 那条路却完全正常。
    表现为"用 settle() 测就全挂、用 step(200) 测就全过"，极难归因。
    """
    pose = {j.id: j.limit_max for j in robot.movable_joints()}

    a = fresh_sim(robot, physics)
    a.set_target_joints(pose)
    for _ in range(10):
        a.step(1)

    b = fresh_sim(robot, physics)
    b.set_target_joints(pose)
    b.step(10)

    assert np.array_equal(a.data.qpos, b.data.qpos), "分批步进与一次步进结果不同"
    assert np.array_equal(a.data.qvel, b.data.qvel)
    assert a.data.time == b.data.time


def test_control_loop_runs_at_its_own_period(robot, physics):
    """控制环按 100Hz 更新：第 1~9 个物理步里限速目标**不动**，第 10 步才走一格。

    这直接验证 §23 的分层：控制器不是每个物理步都在动，而是每 10ms 动一次。
    """
    sim = fresh_sim(robot, physics)
    home = dict(robot.home_pose)
    target = dict(home, base=robot.joint("base").limit_max)     # 一步跨到上限

    sim.set_target_joints(target)
    assert sim.ctrl_angles_deg()["base"] == pytest.approx(home["base"], abs=1e-12), (
        "刚下发命令时限速目标不应变化（还没到控制时刻）")

    for k in range(1, 10):
        sim.step(1)
        assert sim.ctrl_angles_deg()["base"] == pytest.approx(home["base"], abs=1e-12), (
            f"第 {k} 个物理步（{k}ms）限速目标就动了 ⇒ 控制环跑成了 1kHz")

    sim.step(1)                                                 # 第 10 步 = 10ms
    # `physics.servo.max_velocity` 同时给了 `deg_per_s: 573.0` 与 `rad_per_s: 10.0`。
    # 前者只是后者的**取整**（10 rad/s = 572.9578°/s），仅为人读；
    # 权威表示是 SI 的 `rad_per_s` —— 精确计算一律用它，否则会差 0.007%。
    v_rad = float(physics.servo["max_velocity"]["rad_per_s"])
    v_deg = float(physics.servo["max_velocity"]["deg_per_s"])
    assert v_deg == pytest.approx(float(np.degrees(v_rad)), rel=1e-4), (
        f"配置里 deg_per_s={v_deg} 与 rad_per_s={v_rad} 不自洽（{np.degrees(v_rad)}）")
    max_step = float(np.degrees(v_rad)) * physics.ts_control
    moved = sim.ctrl_angles_deg()["base"] - home["base"]
    assert moved == pytest.approx(max_step, rel=1e-9), (
        f"第 10 步限速目标应前进 max_velocity×ts_control = {max_step:.6f}°，实得 {moved:.6f}°")
    assert sim.ctrl_angles_deg()["base"] < target["base"], "限速目标不应一步到位"


def test_render_fps_does_not_change_physics(robot, physics):
    """★ 渲染帧率**不得**影响物理结果（spec §24 的核心断言）。

    两次运行只改 `fps`（5 与 240），物理步长与批量完全相同 ⇒ 终态必须按位相同。
    如果有人在 `run_loop` 里把批大小或步长跟 fps 挂钩（"帧率低就多步几步"是一个
    非常自然但错误的优化），这条会立刻变红。
    """
    from run import run_loop

    def run_at(fps: float) -> np.ndarray:
        sim = fresh_sim(robot, physics)
        script = [("hold-home", dict(robot.home_pose), float("inf"))]
        run_loop(sim, script, fps=fps, print_interval=1e9, duration=0.3,
                 headless=True, fast=True, recorder=None)
        return np.array(sim.data.qpos, copy=True)

    a = run_at(5.0)
    b = run_at(240.0)
    assert np.array_equal(a, b), (
        f"渲染帧率改变了物理结果：fps=5 与 fps=240 的 qpos 不同\n  {a}\n  {b}")


# ---------------------------------------------------------------------------
# §32 / §33 可重复性
# ---------------------------------------------------------------------------


def test_reset_is_bitwise_repeatable(robot, physics):
    """`reset()` 相同入参 ⇒ 逐位相同的初始状态（spec §32）。"""
    sim = fresh_sim(robot, physics)
    pose = {j.id: (j.limit_min + j.limit_max) / 2.0 for j in robot.movable_joints()}

    # 先把它搅乱：跑一段带接触的轨迹，再复位
    sim.set_target_joints({j.id: j.limit_max for j in robot.movable_joints()})
    sim.step(400)

    sim.reset(pose)
    q1 = np.array(sim.data.qpos, copy=True)
    v1 = np.array(sim.data.qvel, copy=True)
    t1 = float(sim.data.time)

    sim.step(300)
    sim.reset(pose)
    q2 = np.array(sim.data.qpos, copy=True)
    v2 = np.array(sim.data.qvel, copy=True)

    assert np.array_equal(q1, q2), "复位后的 qpos 不可重复"
    assert np.array_equal(v1, v2), "复位后的 qvel 不可重复（应恒为 0）"
    assert float(np.max(np.abs(v1))) == 0.0, "复位必须清零速度"
    assert t1 == 0.0 and float(sim.data.time) == 0.0, "复位必须清零仿真时间"
    # 控制器内部状态也必须归零，否则上一轮的目标会立刻把臂拉走
    ctrl = sim.ctrl_angles_deg()
    for jid, want in pose.items():
        assert ctrl[jid] == pytest.approx(want, abs=1e-12), (
            f"复位后 {jid} 的限速目标为 {ctrl[jid]} ≠ 指定位形 {want} ⇒ 上一轮命令仍在生效")


def test_warmstart_does_not_leak_across_resets(robot, physics):
    """★ 复位必须清掉求解器**热启动**状态，否则"跨次复位一致"不成立。

    MuJoCo 的 `data.qacc_warmstart` 会跨 `mj_step` 保留，用来加速收敛。
    复位若不清它，第二次运行的轨迹会与第一次不同 —— 而且**差异极小**，
    肉眼与 `allclose` 都看不出来，只在按位比较时暴露。
    """
    a = fresh_sim(robot, physics)
    run_sequence(a, cmd_sequence(robot, 6), steps_per_cmd=40)
    ref_q = np.array(a.data.qpos, copy=True)
    ref_v = np.array(a.data.qvel, copy=True)

    # 同一实例：复位后再跑同一段，必须与第一次逐位相同
    a.reset(dict(robot.home_pose))
    run_sequence(a, cmd_sequence(robot, 6), steps_per_cmd=40)
    assert np.array_equal(a.data.qpos, ref_q), "同一实例二次运行结果不同（热启动泄漏？）"
    assert np.array_equal(a.data.qvel, ref_v)

    # 全新实例：同样必须一致
    b = fresh_sim(robot, physics)
    run_sequence(b, cmd_sequence(robot, 6), steps_per_cmd=40)
    assert np.array_equal(b.data.qpos, ref_q), "新实例与复用实例结果不同（仍有跨次状态泄漏）"


def test_determinism_across_processes(robot):
    """★ 跨**进程**确定性：`run.py --demo` 跑两遍，数值行必须逐字相同。

    与上一条的分工：上一条证明"进程内两次运行一致"，这条证明
    "换一个进程、重头编译模型，结果仍然一致" —— 后者才会暴露
    Python 哈希种子、dict 顺序、未初始化内存之类的非确定性来源。

    只比较**数值载荷**：`FPS` 与实时倍率 `×` 是墙钟量，按定义就不该被比较，
    所以这里把它们替换成占位符再比 —— 不是"放宽标准"，而是把
    "物理是否确定"与"机器快慢"这两件无关的事切开。
    """
    def payloads() -> list[str]:
        env = {**os.environ, "PYTHONIOENCODING": "utf-8"}
        proc = subprocess.run(
            [sys.executable, str(SIM_DIR / "run.py"),
             "--headless", "--fast", "--demo", "--duration", "3.0",
             "--print-interval", "0.25"],
            cwd=str(SIM_DIR), capture_output=True, timeout=300, env=env,
        )
        assert proc.returncode == 0, proc.stderr.decode("utf-8", "replace")[-2000:]
        text = proc.stdout.decode("utf-8", "replace")
        lines = [ln for ln in text.splitlines() if ln.startswith("[t=")]
        assert lines, f"没有采到统计行，输出：\n{text[-2000:]}"
        # 抹掉两个墙钟量
        return [re.sub(r"×[\d.]+", "×?", re.sub(r"FPS\s+[\d.]+", "FPS ?", ln))
                for ln in lines]

    a = payloads()
    b = payloads()
    assert len(a) >= 5, f"采到的统计行太少（{len(a)}），判据不充分"
    for i, (x, y) in enumerate(zip(a, b)):
        assert x == y, f"第 {i} 行跨进程不一致：\n  A: {x}\n  B: {y}"
    print(f"\n[DET] 跨进程 {len(a)} 行数值载荷逐字相同（seed={load_physics().deterministic['seed']}）")


# ---------------------------------------------------------------------------
# §34 数据记录
# ---------------------------------------------------------------------------


def _collect(sim: MeArmSim, n: int, recorder: Recorder, start_wall: float = 0.0) -> list[dict]:
    rows = []
    for k in range(n):
        sim.step(10)
        st = sim.state().as_dict(sim.robot)
        recorder.write(st, wall_time=start_wall + k * 0.01)
        rows.append(st)
    return rows


def test_record_jsonl_roundtrip(sim, tmp_path):
    """JSONL 落盘后能原样读回，且**数值等于当时的状态**（不只是自洽）。"""
    path = tmp_path / "run.jsonl"
    with Recorder(path, fmt="jsonl", enabled=True) as rec:
        states = _collect(sim, 20, rec)

    assert rec.count == 20
    lines = path.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 20

    for line, st in zip(lines, states):
        row = json.loads(line)
        assert row.keys() == flatten(build_row(st, rec.fields)).keys(), (
            "记录列与配置声明的字段不一致")
        for jid, ang in st["joint_angles"].items():
            assert row[f"joint_angle.{jid}"] == pytest.approx(ang, abs=1e-12), (
                f"记录的 {jid} 角与当时状态不符")
        assert row["sim_time"] == pytest.approx(st["time"], abs=1e-12)
        assert row["contact_count"] == st["contact_count"]

    times = [json.loads(ln)["sim_time"] for ln in lines]
    assert times == sorted(times) and times[0] > 0.0, "sim_time 应严格递增且从正数开始"


def test_record_csv_header_matches_jsonl_columns(sim, tmp_path):
    """CSV 表头必须与 JSONL 的键集合完全相同 —— 两种格式不允许有信息差。"""
    import csv

    jpath, cpath = tmp_path / "a.jsonl", tmp_path / "b.csv"
    with Recorder(jpath, fmt="jsonl", enabled=True) as rj:
        _collect(sim, 8, rj)
    with Recorder(cpath, fmt="csv", enabled=True) as rc:
        _collect(sim, 8, rc)

    jkeys = list(json.loads(jpath.read_text(encoding="utf-8").splitlines()[0]))
    with cpath.open(encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        assert reader.fieldnames == jkeys, f"CSV 表头 {reader.fieldnames} != JSONL 键 {jkeys}"
        assert sum(1 for _ in reader) == 8


def test_recorder_is_config_driven_and_disabled_by_default(tmp_path):
    """`recording.enabled` 默认 false ⇒ 不产生任何文件（spec §34 默认不落盘）。"""
    cfg = load_physics().recording
    assert cfg["enabled"] is False, (
        "默认必须关闭记录 —— 验收/CI 里悄悄写盘会污染仓库并拖慢测试")
    rec = Recorder(tmp_path / "never.jsonl")
    assert rec.enabled is False
    rec.open().write({"time": 0.0})
    rec.close()
    assert not (tmp_path / "never.jsonl").exists(), "关闭状态下仍然写了文件"
    assert rec.count == 0

    fields = list(cfg["fields"])
    assert fields, "recording.fields 不能为空"
    assert set(fields) <= {"sim_time", "wall_time", "joint_angle", "joint_velocity",
                           "joint_torque", "target_joint", "end_effector", "contact_count"}, (
        f"recording.fields 里有未实现的列：{set(fields) - set(FIELD_SOURCE)}")


def test_record_rejects_unknown_field(tmp_path):
    """配置里写了没实现的字段名时必须**报错**，不能静默丢列。"""
    from record import RecordError

    rec = Recorder(tmp_path / "x.jsonl", enabled=True, fields=["sim_time", "no_such_field"])
    with pytest.raises(RecordError):
        rec.open()
        rec.write({"time": 0.0})
    rec.close()


# ---------------------------------------------------------------------------
# §22 Test D 快速运动
# ---------------------------------------------------------------------------


def test_fast_motion_stays_stable_and_within_limits(sim, robot, physics):
    """Test D：连续大幅快速命令下不发散、不出 NaN、不越物理 range。

    命令在**真机限位上**来回跳（不是"给一个超限的大数让物理层去钳"），
    这样测的是"正常使用下的最恶劣工况"，而不是"错误输入的容错"。
    """
    order = [j.id for j in robot.movable_joints()]
    seq = cmd_sequence(robot, 16)
    peak_v = 0.0
    for k, pose in enumerate(seq):
        sim.set_target_joints(pose)
        sim.step(25 if k % 2 == 0 else 60)          # 25ms / 60ms 交替，制造速度突变
        peak_v = max(peak_v, float(np.max(np.abs(sim.data.qvel))))

    q = np.array(sim.data.qpos, dtype=float)
    assert np.all(np.isfinite(q)), "qpos 出现 NaN/Inf"
    assert np.all(np.isfinite(np.array(sim.data.qvel, dtype=float))), "qvel 出现 NaN/Inf"

    # ⚠️ `jnt_range` 按**关节 id** 索引，而不是"关节在 qpos 里的第几个"。
    #    本模型恰好两者同序（4 个 hinge、无自由关节），但依赖这种巧合会在
    #    将来加入别的关节类型时静默错位 —— 所以这里按**名字**取。
    import mujoco

    for jid in order:
        k = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        assert k >= 0, f"模型里找不到关节 {jid}"
        lo, hi = (float(x) for x in sim.model.jnt_range[k])
        got = float(q[sim.angle_map.qpos_index(jid)])
        assert lo - 1e-6 <= got <= hi + 1e-6, (
            f"{jid} 越出物理 range：{got:.6f} ∉ [{lo:.6f}, {hi:.6f}]")

    vmax = float(physics.servo["max_velocity"]["rad_per_s"])
    print(f"[DYN] Test D 峰值角速度 {peak_v:.4f} rad/s（限速目标 {vmax:.4f} rad/s，"
          f"比值 {peak_v / vmax:.2f}×）")
    assert peak_v < vmax * 3.0, (
        f"峰值角速度 {peak_v:.4f} rad/s 达限速目标的 {peak_v / vmax:.1f} 倍 ⇒ 异常发散")


# ---------------------------------------------------------------------------
# §37 诚实声明（Level 3→4，不是 Level 5）
# ---------------------------------------------------------------------------


def test_level_declaration_is_machine_checkable(physics):
    """★ `calibration.calibrated` 必须为 false，且**七个可标定量全为空**。

    为什么要把"声明"也做成测试：`physics.yaml` 里的值是**公开值 + 估算值**，
    不是对本台 meArm 的实测（spec §37 明确禁止把参数化仿真说成真实模型）。
    一个能被 CI 检查的声明，比 README 里的一句话可靠得多 ——
    它保证"标定完成"这件事**不可能**被顺手改成 true 而没有真实数据支撑。
    """
    import calibrate

    assert physics.calibration["calibrated"] is False, (
        "physics.yaml 声明已标定 —— 这需要真实实验数据支撑（spec §37）")
    for key in ("servo_offset", "joint_scale", "joint_zero",
                "max_velocity", "max_torque", "damping", "friction"):
        assert physics.calibration.get(key) == {}, (
            f"calibration.{key} 非空（{physics.calibration.get(key)}）"
            "—— 有实测值却仍标 calibrated: false 是自相矛盾的")

    calibrated, _cal, pending = calibrate.collect_status(physics)
    assert calibrated is False
    assert len(pending) == len(calibrate.CALIBRATABLE), (
        f"待标定项 {len(pending)} != 可标定项 {len(calibrate.CALIBRATABLE)}")
    print(f"\n[LEVEL] 当前 = Level 3→4（参数化物理仿真）；"
          f"calibrated=false，{len(pending)} 项待真机标定")


def test_config_truth_is_not_duplicated(physics, robot):
    """`physics.yaml` **不得**出现运动学真值（spec §4「唯一参数来源」的机器化形式）。

    只要有人把连杆长度、关节轴或限位抄进 physics.yaml，这条就会变红 ——
    抄了必然漂移，而且**不会报错**（两个"真值"会各自被不同代码路径读走）。
    """
    import yaml

    raw = yaml.safe_load((ROOT / "config" / "physics.yaml").read_text(encoding="utf-8"))
    for key in ("robot", "links", "joints", "homePose", "tcp", "limits_deg"):
        assert key not in raw, (
            f"physics.yaml 里出现了运动学段 {key!r} —— 运动学真值只允许在 config/robot.yaml")
    assert physics.limits["source"] == "robot.yaml", (
        "physics.limits.source 必须显式声明真值来源（人读得懂，测试也检查得了）")

    # 关节限位的**字面量**不得出现：角度真值唯一来源是 robot.yaml
    text = (ROOT / "config" / "physics.yaml").read_text(encoding="utf-8")
    for jid in ("elbow", "shoulder"):
        for value in (robot.joint(jid).limit_min, robot.joint(jid).limit_max):
            lit = f"{value:.4f}"
            assert lit not in text, (
                f"{jid} 的限位值 {lit} 被抄进了 physics.yaml —— 请改为从 robot.yaml 读")

    # 反向确认：真值确实在 robot.yaml 里且可读
    assert robot.link("upper_arm_link").length > 0
    assert robot.joint("elbow").limit_min < robot.joint("elbow").limit_max


def test_generated_mjcf_is_in_sync_with_config(robot, physics):
    """★ 入库的 `mearm.xml` 必须与「现在用 yaml 重新生成」的结果**逐字节相同**。

    `mearm.xml` 是**派生产物**（缓存），但为了"clone 下来就能跑 pytest"也一并入库了。
    入库的派生文件有一个经典失效模式：改了 `robot.yaml` 却忘了重跑生成器 ——
    此时 MuJoCo 用的是**旧几何**，而所有 FK/IK 测试都会红得莫名其妙
    （因为它们拿新 yaml 算参考值，拿旧 XML 跑物理）。

    这条测试把这个缺口封死，失败信息直接给出修复命令。
    """
    from gen_model import build_xml

    committed = (SIM_DIR / "mearm.xml").read_text(encoding="utf-8")
    fresh = build_xml(robot, physics)
    if committed != fresh:
        import difflib

        diff = list(difflib.unified_diff(
            committed.splitlines(), fresh.splitlines(),
            fromfile="mearm.xml（入库）", tofile="gen_model.py 现算", lineterm="", n=1))
        raise AssertionError(
            "mearm.xml 已过期 —— 改配置后必须重新生成：\n"
            "    python simulation/mujoco/gen_model.py\n"
            "差异（前 30 行）：\n" + "\n".join(diff[:30]))
    assert fresh.count("<body ") == len(robot.links) + 1, (
        f"MJCF 里 {fresh.count('<body ')} 个 body != robot.yaml 的 {len(robot.links)} 连杆 + 1 场景体")


# ---------------------------------------------------------------------------
# 端到端：Go 侧协议常量与 Python 侧的一致性（跨语言的最低限度互检）
# ---------------------------------------------------------------------------


def test_python_limits_agree_with_go_error_format(robot):
    """Python `limits.py` 与 Go `protocol` 的错误文案必须同形（跨语言互检）。

    两边都会给用户看这句话，格式一旦分叉，e2e 探针与前端提示就会对不上。
    """
    from limits import validate_joints

    v = validate_joints(robot, {"shoulder": 999.0})
    assert v is not None
    assert v.message() == (
        f"ERR JOINT shoulder 999.00 "
        f"(limit {robot.joint('shoulder').limit_min:.2f}..{robot.joint('shoulder').limit_max:.2f})")
