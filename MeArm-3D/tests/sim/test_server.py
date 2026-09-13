# -*- coding: utf-8 -*-
"""Phase 6 验收：无头设备服务（`server.py`）的协议一致性。

判据全部对着 **Go 侧的协议实现** `backend/internal/protocol/protocol.go`：

  * `EncodeJR` 只保留 **1 位小数** ⇒ 文本协议的固有量化，往返偏差 ≤0.05°，
    这是**预期行为**（不是 bug），验收断言按此设容差；
  * `EncodeOKJR` 按**通道降序**输出 `S<n>=%.2f`；
  * `EncodeState` 按 `JointOrder` 位次、保留 2 位小数；
  * `ParseReply` 只识别 `ERR` / `STATE` / `OK JR` 三种形状 —— 其余一律 `ReplyOther`。

⚠️ 最后那条有个必须记住的推论：**`STATUS` 回执 Go 侧根本不解析**（落到
`ReplyOther`）。也就是说 `STATUS` 只是人工调试用的旁路，真正的状态回推通道
**只有 `STATE`**。写测试时把这点固化下来，免得日后有人拿 `STATUS` 当状态源。
"""
from __future__ import annotations

import io
import subprocess
import sys

import pytest

from conftest import SIM_DIR
from server import Emitter, MujocoDevice

# 与 robot.yaml 的 JointOrder 一致（base / shoulder / elbow / gripper）
ORDER = ("base", "shoulder", "elbow", "gripper")


# ---------------------------------------------------------------------------
# 辅助
# ---------------------------------------------------------------------------


class Capture:
    """把设备的输出收进一个 StringIO，并提供"取最后一行"的便利。"""

    def __init__(self) -> None:
        self.buf = io.StringIO()
        self.emitter = Emitter(self.buf)

    def lines(self) -> list[str]:
        return [ln for ln in self.buf.getvalue().splitlines() if ln]

    def last(self) -> str:
        ln = self.lines()
        assert ln, "设备没有任何输出"
        return ln[-1]

    def clear(self) -> None:
        self.buf.seek(0)
        self.buf.truncate(0)


@pytest.fixture
def dev():
    cap = Capture()
    d = MujocoDevice(out=cap.emitter)
    d.capture = cap            # type: ignore[attr-defined]
    return d


def last_line(dev) -> str:
    return dev.capture.last()


# ---------------------------------------------------------------------------
# 基本命令
# ---------------------------------------------------------------------------


def test_ping(dev):
    dev.handle("PING")
    assert last_line(dev) == "OK PING"


def test_status_at_home_is_four_servos_at_90(dev):
    """HOME 位 = 四个舵机恰好 90°（这是 robot.yaml 标定自洽性的直接体现）。"""
    dev.handle("STATUS")
    assert last_line(dev) == "STATUS S9=90 S7=90 S8=90 S6=90"


def test_unknown_command(dev):
    dev.handle("FOO bar")
    assert last_line(dev) == "ERR UNKNOWN FOO bar"


def test_blank_line_is_ignored(dev):
    dev.handle("   ")
    assert dev.capture.lines() == []


# ---------------------------------------------------------------------------
# JR 受理
# ---------------------------------------------------------------------------


def test_jr_ok_matches_go_encoding(dev):
    """`OK JR` 必须与 Go 的 `EncodeOKJR` 逐字节一致（通道降序 + 2 位小数）。"""
    joints = {"base": 0.0, "shoulder": 20.0, "elbow": 130.0, "gripper": 50.0}
    dev.handle("JR 0 20 130 50")

    expect = {a.channel: a.joint_to_servo(joints[a.joint_id]) for a in dev.robot.actuators}
    want = " ".join(["OK", "JR"] + [f"S{c}={expect[c]:.2f}"
                                    for c in sorted(expect, reverse=True)])
    assert last_line(dev) == want
    # 顺带把"降序"这条独立钉住（否则上面那条在只有 1 个通道时也会过）
    assert last_line(dev).split()[2:] == ["S9=90.00", "S8=48.39", "S7=117.58", "S6=90.00"]


def test_jr_rejects_joint_limit_with_go_error_text(dev):
    """越限必须回 `ERR JOINT <id> <v> (limit <min>..<max>)` —— 与 Go/固件同文案。"""
    dev.handle("JR 0 20 90 50")          # elbow 下限 108.44
    line = last_line(dev)
    assert line.startswith("ERR JOINT elbow 90.00 (limit 108.44..141.86)"), line


def test_jr_rejects_wrong_arity(dev):
    dev.handle("JR 0 20 130")
    assert last_line(dev).startswith("ERR ARG")


def test_jr_rejects_non_numeric(dev):
    dev.handle("JR a b c d")
    assert last_line(dev).startswith("ERR ARG")


def test_joint_and_servo_limits_are_equivalent(dev):
    """★ 两层限位**完全等价** —— 关节限位换算到舵机后恰好落在舵机限位边界上。

    这是标定自洽性的硬证据（spec §14 要求四方限位一致）：
    把每个关节的 limit_min/max 用 `joint_to_servo` 送过去，得到的值必须**正好等于**
    该舵机的 servo_min/max。若有人改了其中一边，这条立刻变红。

    副作用（值得知道）：正因为等价，**真配置下构造不出"关节入限但舵机出限"的位形** ——
    所以下面那条 `ERR SERVO` 测试只能靠临时收紧 servo_min 来触发。
    """
    for a in dev.robot.actuators:
        j = dev.robot.joint(a.joint_id)
        lo = a.joint_to_servo(j.limit_min)
        hi = a.joint_to_servo(j.limit_max)
        got = sorted((lo, hi))
        want = sorted((a.servo_min, a.servo_max))
        assert got == pytest.approx(want, abs=1e-6), (
            f"S{a.channel}({a.joint_id}) 关节限位 {j.limit_min}..{j.limit_max} "
            f"换算到舵机是 {got}，但舵机限位是 {want}")


def test_jr_rejects_servo_hard_limit(dev):
    """舵机层限位必须独立生效（关节层过了也要挡）。

    真配置下两层等价 ⇒ 只能**临时收紧** servo_min 来构造这个场景。
    这里改的是测试自己那份 RobotCfg（`load_robot()` 每次返回新对象），
    不会污染其他测试，也不会写回配置文件。
    """
    import dataclasses

    a = dev.robot.actuators[0]                    # base 的 S9，中位 90、限位 [30,150]
    dev.robot.actuators[0] = dataclasses.replace(
        a, servo_min=a.servo_min + 70.0)          # 30 → 100，把中位 90 也排除掉
    dev.handle("JR 0 20 130 50")
    line = last_line(dev)
    assert line.startswith(f"ERR SERVO S{a.channel}"), line
    assert "limit" in line


def test_reset_returns_servos_to_90(dev):
    dev.handle("JR 0 20 130 50")
    dev.capture.clear()
    dev.handle("RESET")
    lines = dev.capture.lines()
    assert lines[0] == "OK RESET"
    assert lines[1].startswith("OK JR ")
    for ch in (6, 7, 8, 9):
        assert f"S{ch}=90.00" in lines[1]


# ---------------------------------------------------------------------------
# ★ 核心语义：OK JR 是"意图"，STATE 才是"实际"
# ---------------------------------------------------------------------------


def test_ok_jr_is_intent_state_is_actual(dev):
    """受理回执只说明"我打算去哪"，**实际位置一步都没动**。

    这是本项目的一条铁律（MG90S 无位置回读）：`OK` / `STATUS` / 后端 `joint_state`
    全都在说意图，唯一的外部地面真值是相机。把它写成断言，防止有人日后
    图省事把 `OK JR` 的舵机角当作"实际位置"回推给前端。
    """
    home_shoulder = dev.current_joints()["shoulder"]
    dev.handle("JR 0 40 130 50")
    assert last_line(dev).startswith("OK JR")
    # 受理瞬间实际位置分毫未动
    assert dev.current_joints()["shoulder"] == pytest.approx(home_shoulder, abs=1e-9)

    # 推进 20ms：动了，但远未到目标
    dev.sim.step(20)
    mid = dev.current_joints()["shoulder"]
    assert home_shoulder < mid < 40.0 - 1.0, f"20ms 后 shoulder={mid}"

    # 收敛后贴住目标（留出重力的稳态误差 τ/kp）
    dev.sim.settle(4.0)
    assert dev.current_joints()["shoulder"] == pytest.approx(40.0, abs=0.6)


def test_state_frame_converges_then_stops(dev):
    """`STATE` 只在位置真的变化时发；收敛后必须停下来（否则永远 30Hz 刷帧）。"""
    dev.handle("JR 0 40 130 50")
    dev.sim.settle(6.0, tolerance_rad=1e-4, hold_s=0.5)
    dev.capture.clear()

    last = dev.sim.data.qpos.copy()
    dev.report_if_moved(last)                    # 静止后首帧是允许发的
    before = len(dev.capture.lines())
    for _ in range(20):
        dev.sim.step(50)                         # 再推 1s（20 × 50ms）
        last = dev.report_if_moved(last)
    assert len(dev.capture.lines()) == before, "已静止却仍在发 STATE"


def test_state_uses_absolute_elbow_angle(dev):
    """`STATE` 的 elbow 是**绝对角**（与 UI / 协议一致），不是 MuJoCo 的局部角。"""
    dev.handle("JR 0 40 125 50")
    dev.sim.settle(4.0)
    dev.report_if_moved(None)
    parts = last_line(dev).split()
    assert parts[0] == "STATE"
    vals = [float(x) for x in parts[1:]]
    assert len(vals) == 4
    joints = dict(zip(ORDER, vals))
    # 若错把局部角报出去，elbow 会变成 125-40=85 左右
    assert joints["elbow"] > 100.0, f"elbow 报成了局部角？{joints}"
    assert joints["elbow"] == pytest.approx(125.0, abs=1.0)


# ---------------------------------------------------------------------------
# 与 Go `ParseReply` 的兼容性
# ---------------------------------------------------------------------------


def classify_go_reply(line: str) -> str:
    """复刻 Go `ParseReply` 的分类（不认得的一律 OTHER）。

    注意判定顺序：先 `ERR`，再 `STATE`，最后 `OK JR` —— 与 Go 一致。
    """
    up = line.upper()
    if up.startswith("ERR"):
        return "ERR"
    if up.startswith("STATE ") or up == "STATE":
        try:
            for x in line.split()[1:]:
                float(x)
            return "STATE"
        except ValueError:
            return "OTHER"
    if up.startswith("OK JR"):
        return "OK_JR"
    return "OTHER"


def test_go_parseable_replies_are_exactly_the_three_shapes(dev):
    """正常路径的三类回执必须能被 Go 认出来：`OK JR` / `STATE` / `ERR`。"""
    cases = {
        "JR 0 20 130 50": "OK_JR",     # 受理
        "JR 0 20 90 50": "ERR",        # 关节越限
        "JR 1 2 3": "ERR",             # 参数个数错
        "ZZZ": "ERR",                  # 未知命令
    }
    for cmd, want in cases.items():
        dev.capture.clear()
        dev.handle(cmd)
        first = dev.capture.lines()[0]
        assert classify_go_reply(first) == want, f"{cmd!r} → {first!r}"


def test_status_ping_reset_are_reply_other(dev):
    """★ 登记在案的"旁路"回执：`STATUS` / `OK PING` / `OK RESET` 都是 **ReplyOther**。

    Go 的 `ParseReply` 只认 `ERR` / `STATE` / `OK JR` 三种形状，其余一律
    `ReplyOther`。也就是说：

      * `STATUS` **不是**状态回推通道 —— 真正的状态通道只有 `STATE` 主动帧；
      * `PING` 的应答 Go 也不解析（心跳只关心"有没有回音"）。

    把这三条钉住，免得日后有人拿 `STATUS` 当状态源，或者误以为
    `OK PING` 会被当作什么有意义的事件。
    """
    for cmd, first_line in (("STATUS", "STATUS S9=90 S7=90 S8=90 S6=90"),
                            ("PING", "OK PING"),
                            ("RESET", "OK RESET")):
        dev.capture.clear()
        dev.handle(cmd)
        assert dev.capture.lines()[0] == first_line
        assert classify_go_reply(first_line) == "OTHER", (
            f"{first_line!r} 竟然被 Go 识别了？请复核 protocol.go 的 ParseReply")


# ---------------------------------------------------------------------------
# 真进程冒烟（Phase 7 的 Go 侧依赖的正是这条路径）
# ---------------------------------------------------------------------------


def test_server_process_smoke():
    """像 Go 那样用管道起真进程：喂命令、读回执、靠 EOF 优雅退出。"""
    proc = subprocess.Popen(
        [sys.executable, str(SIM_DIR / "server.py"), "--no-realtime"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, encoding="utf-8", errors="replace", bufsize=1,
    )
    try:
        out, err = proc.communicate("PING\nSTATUS\nJR 0 20 130 50\nQUIT\n", timeout=120)
    finally:
        if proc.poll() is None:
            proc.kill()
    assert proc.returncode == 0, f"退出码 {proc.returncode}\nstderr:\n{err}"
    assert "OK PING" in out, out
    assert "STATUS S9=90 S7=90 S8=90 S6=90" in out, out
    assert "OK JR S9=90.00 S8=48.39 S7=117.58 S6=90.00" in out, out
