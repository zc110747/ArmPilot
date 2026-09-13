# -*- coding: utf-8 -*-
"""无头 MuJoCo 设备服务：在 stdio 上跑 **arm-device 文本协议**。

它对接 `device.Device` 的第三个实现（`backend/internal/device/mujoco.go`）。
协议与 `backend/internal/device/sim.go` 以及真实固件**逐字节一致**：

    ── 下行（Go → 本进程）────────────────────────────────────────────
      JR <j1> <j2> <j3> <grip>     关节角整帧命令（位次 = JointOrder）
      STATUS | STATE?              查询舵机角（整数）
      RESET                        全部舵机回 90°（= HOME，不是"关节全 0"）
      PING                         心跳
      QUIT | EXIT                  优雅退出

    ── 上行（本进程 → Go）────────────────────────────────────────────
      OK JR S9=.. S8=.. S7=.. S6=..   受理回执（**目标**舵机角，2 位小数）
      STATE <j1> <j2> <j3> <grip>     **实际**关节角主动上报（2 位小数）
      STATUS S9=.. S7=.. S8=.. S6=..  舵机角查询应答（整数）
      OK RESET / OK PING
      ERR JOINT <id> <v> (limit <min>..<max>)
      ERR SERVO S<n> <v> (limit <min>..<max>)
      ERR ARG <说明> / ERR UNKNOWN <原文>

于是 WebSocket / controller / protocol / 前端**全部零改动**，接入点只是
`backend/main.go` 的 device 工厂多一个 case（spec §2 / §25 / §40）。

三个时间尺度（spec §23 / §24）—— 三者互相解耦，**渲染帧率不决定物理步长**：

    physics  1000 Hz   固定 1ms 步长；唯一推进 `qpos` 的地方
    control   100 Hz   `MeArmSim.step()` 内部按 `_ctrl_accum` 节流（不是每个物理步都改 ctrl）
    report     30 Hz   只有位置**真的变了**才发 STATE

⚠️ `OK JR` 回的是"我打算去哪"（目标），`STATE` 才是"它现在在哪"（实际）。
这是本项目的一条铁律：MG90S 没有位置回读，任何 `OK`/`STATUS` 都只是意图，
唯一的外部地面真值是相机（见 docs/decisions.md D34）。
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
from collections.abc import Mapping
from pathlib import Path

PKG_DIR = Path(__file__).resolve().parent
if str(PKG_DIR) not in sys.path:
    sys.path.insert(0, str(PKG_DIR))

import numpy as np  # noqa: E402

from limits import validate_joints  # noqa: E402
from model import MeArmSim  # noqa: E402
from robotcfg import RobotCfg, load_robot  # noqa: E402
from units import ensure_utf8_stdout  # noqa: E402

DEFAULT_REPORT_HZ = 30.0
DEFAULT_PHYS_HZ = 1000.0
DEFAULT_BATCH_MS = 10.0


class Emitter:
    """线程安全地往 stdout 写整行（**立即 flush**，否则 Go 侧会卡在缓冲里）。"""

    def __init__(self, stream=None) -> None:
        self._s = stream if stream is not None else sys.stdout
        self._lock = threading.Lock()

    def line(self, text: str) -> None:
        with self._lock:
            try:
                self._s.write(text + "\n")
            except ValueError:          # 管道已关闭
                return
            self._s.flush()


class MujocoDevice:
    """把 MuJoCo 物理包成一台"能收 JR、回 OK/ERR/STATE"的机械臂。"""

    def __init__(
        self,
        *,
        robot: RobotCfg | None = None,
        sim: MeArmSim | None = None,
        report_hz: float = DEFAULT_REPORT_HZ,
        out: Emitter | None = None,
    ) -> None:
        self.robot = robot or load_robot()
        self.sim = sim if sim is not None else MeArmSim(robot=self.robot)
        self.order = self.robot.joint_order()      # base, shoulder, elbow, gripper
        self.out = out if out is not None else Emitter()
        self.report_period = 1.0 / float(report_hz)

        # 通道 → 关节（`STATUS` 的字段顺序按 sim.go：跟随 joint_order）
        self.chan_of_joint: dict[str, int] = {}
        for a in self.robot.actuators:
            self.chan_of_joint.setdefault(a.joint_id, a.channel)

        self._cmds: queue.Queue[str] = queue.Queue()
        self._stop = threading.Event()

        # 开机位 = 模型 HOME（四个舵机恰好 90°）。协议 §4 明确 RESET 不许
        # 改成"关节全 0"：关节全 0 对肘（绝对角 108..142）是不可达位姿。
        self.boot_pose: dict[str, float] = dict(self.robot.home_pose)
        self.sim.reset(self.boot_pose)

    # ------------------------------------------------------------------
    # 编码（必须与 protocol.go 的 Encode* 逐字节一致）
    # ------------------------------------------------------------------

    def encode_ok_jr(self, servo: Mapping[int, float]) -> str:
        """`OK JR S9=.. S8=.. S7=.. S6=..` —— 通道**降序**（对齐 EncodeOKJR）。"""
        chans = sorted(servo, reverse=True)
        return "OK JR " + " ".join(f"S{c}={servo[c]:.2f}" for c in chans)

    def encode_status(self, servo: Mapping[int, float]) -> str:
        """`STATUS S9=.. S7=.. S8=.. S6=..` —— 顺序按 joint_order，整数（对齐 sim.go）。"""
        chans = [self.chan_of_joint[j] for j in self.order if j in self.chan_of_joint]
        return "STATUS " + " ".join(f"S{c}={int(round(servo[c]))}" for c in chans
                                    if c in servo)

    def encode_state(self, joints: Mapping[str, float]) -> str:
        """`STATE <j1> <j2> <j3> <grip>`（对齐 EncodeState，2 位小数）。"""
        return "STATE " + " ".join(f"{joints[j]:.2f}" for j in self.order)

    # ------------------------------------------------------------------
    # 换算
    # ------------------------------------------------------------------

    def joints_to_servo(self, joints: Mapping[str, float]) -> dict[int, float]:
        out: dict[int, float] = {}
        for a in self.robot.actuators:
            theta = joints.get(a.joint_id)
            if theta is None:
                continue
            out[a.channel] = a.joint_to_servo(float(theta))
        return out

    def current_joints(self) -> dict[str, float]:
        """当前**实际**关节角（度，绝对语义）—— 由 `qpos` 反解而来。"""
        return self.sim.joint_angles_deg()

    def current_servo(self) -> dict[int, float]:
        return self.joints_to_servo(self.current_joints())

    # ------------------------------------------------------------------
    # 命令
    # ------------------------------------------------------------------

    def handle(self, raw: str) -> None:
        line = raw.strip()
        if not line:
            return
        upper = line.upper()

        if upper.startswith("JR"):
            self._handle_jr(line)
        elif upper in ("STATUS", "STATE?"):
            self.out.line(self.encode_status(self.current_servo()))
        elif upper == "RESET":
            self.sim.set_target_joints(self.boot_pose)
            self.out.line("OK RESET")
            self.out.line(self.encode_ok_jr(self.joints_to_servo(self.boot_pose)))
        elif upper == "PING":
            self.out.line("OK PING")
        elif upper in ("QUIT", "EXIT"):
            self.stop()
        else:
            self.out.line(f"ERR UNKNOWN {line}")

    def _handle_jr(self, line: str) -> None:
        fields = line.split()[1:]
        if len(fields) != len(self.order):
            self.out.line(
                f"ERR ARG JR 需要 {len(self.order)} 个关节角，收到 {len(fields)} 个")
            return
        try:
            joints = {j: float(v) for j, v in zip(self.order, fields)}
        except ValueError as exc:
            self.out.line(f"ERR ARG {exc}")
            return

        # ① 关节限位 —— 与 Go controller / 真机固件读的是**同一份** robot.yaml
        v = validate_joints(self.robot, joints)
        if v is not None:
            self.out.line(v.message())
            return

        # ② 舵机角换算 + 舵机硬限位
        servo = self.joints_to_servo(joints)
        for a in self.robot.actuators:
            s = servo.get(a.channel)
            if s is None:
                continue
            if s < a.servo_min - 1e-6 or s > a.servo_max + 1e-6:
                self.out.line(f"ERR SERVO S{a.channel} {s:.2f} "
                              f"(limit {a.servo_min:.2f}..{a.servo_max:.2f})")
                return

        # ③ 受理：只写**目标**。实际位置永远由物理决定 ——
        #    绝不出现 `qpos[...] = target` 这种"直接摆位"（spec §12）。
        self.sim.set_target_joints(joints)
        self.out.line(self.encode_ok_jr(servo))

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------

    def _read_stdin(self) -> None:
        try:
            for line in sys.stdin:
                self._cmds.put(line)
        except (OSError, ValueError):
            pass
        self._stop.set()          # stdin 关闭 ⇒ 优雅退出

    def stop(self) -> None:
        self._stop.set()

    def pump_once(self, batch: int) -> bool:
        """处理待办命令 + 推进 `batch` 个物理步。返回是否应继续。

        ⚠️ 命令必须先 drain **再**判 stop。写成"先判 stop"会丢命令：
        stdin 被管道喂完立刻 EOF 时，`_stop` 会在队列被消费前就置位。
        """
        while True:
            try:
                raw = self._cmds.get_nowait()
            except queue.Empty:
                break
            self.handle(raw)

        if self._stop.is_set():
            return False
        self.sim.step(batch)
        return True

    def report_if_moved(self, last_qpos):
        """位置变化超过阈值才发一帧 STATE；返回最新 qpos 快照。

        ⚠️ 不能用严格相等：MuJoCo 在"接近静止"时 qpos 的末位仍会抖，
        严格相等会让状态帧以 30Hz 无限期地发下去。sim.go 的等价判据是
        "target 与 actual 是否还有 delta"，这里用位置阈值对齐同一语义。

        阈值取 1e-5 rad（≈0.00057°）—— 远小于串口协议 0.01° 的量化步长，
        因此"真有位移"一定会被报出来，不会漏帧。

        ⚠️ **判据是「相对上次发射的累计位移」，不是「单次调用的增量」**：
        快照只在**发射**时更新（未发射时原样返回入参），所以一条以 1e-3 rad/s
        缓慢爬行的臂会每 ~10ms 发一帧，而一条以 1e-7 rad/s 爬行的臂每 ~100s 发一帧。
        这是有意为之 —— 若改成"单次增量 > 1e-5 才发"，慢速运动就会**彻底静默**。
        推论（写测试时必须知道）：`settle(tolerance_rad=T)` 管的是**速度**，
        要求 1s 内不发帧就得 `T × 1s < 1e-5`；见
        tests/sim/test_server.py::test_state_frame_converges_then_stops。
        """
        q = np.array(self.sim.data.qpos, copy=True)
        if last_qpos is None or float(np.max(np.abs(q - last_qpos))) > 1e-5:
            self.out.line(self.encode_state(self.current_joints()))
            return q
        return last_qpos

    def run(
        self,
        *,
        phys_hz: float = DEFAULT_PHYS_HZ,
        batch_ms: float = DEFAULT_BATCH_MS,
        realtime: bool = True,
    ) -> None:
        dt = 1.0 / float(phys_hz)
        batch = max(1, int(round((batch_ms / 1000.0) / dt)))
        chunk_s = batch * dt
        chunk_ms = chunk_s * 1000.0

        # 每 chunk 发一帧 STATE 就够 30Hz：chunk 默认 10ms ⇒ 最多 100Hz，
        # 再用 report_period 节流到 report_hz。
        chunks_per_report = max(1, int(round(self.report_period / chunk_s)))

        reader = threading.Thread(target=self._read_stdin, daemon=True,
                                  name="mujoco-stdin")
        reader.start()

        next_t = time.perf_counter()
        since_report = 0
        last_qpos = None
        while True:
            if not self.pump_once(batch):
                break
            since_report += 1
            if since_report >= chunks_per_report:
                since_report = 0
                last_qpos = self.report_if_moved(last_qpos)
            if realtime:
                next_t += chunk_s
                slack = next_t - time.perf_counter()
                if slack > 0:
                    # ⚠️ 按 chunk（默认 10ms）睡一次，而不是每个物理步睡 1ms。
                    #    Windows 的 sleep 粒度约 1~2ms，逐步 sleep 会让仿真
                    #    比真实时间慢一个数量级 —— 那样"实时"就名存实亡了。
                    time.sleep(slack)
                else:
                    next_t = time.perf_counter()      # 落后了就重新对齐，不追债


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="MuJoCo 设备服务（stdio 文本协议）")
    p.add_argument("--xml", default=None, help="MJCF 路径（默认 simulation/mujoco/mearm.xml）")
    p.add_argument("--report-hz", type=float, default=DEFAULT_REPORT_HZ,
                   help="STATE 上报频率（默认 30）")
    p.add_argument("--phys-hz", type=float, default=DEFAULT_PHYS_HZ,
                   help="物理步频率（默认 1000）")
    p.add_argument("--batch-ms", type=float, default=DEFAULT_BATCH_MS,
                   help="每次 sleep 前连续推进的物理时长（默认 10ms）")
    p.add_argument("--no-realtime", action="store_true",
                   help="不做实时对齐（尽快跑，供自动化测试使用）")
    args = p.parse_args(argv)

    ensure_utf8_stdout()
    try:
        sys.stdout.reconfigure(newline="\n", line_buffering=True)   # type: ignore[attr-defined]
    except (AttributeError, ValueError):
        pass

    dev = MujocoDevice(report_hz=args.report_hz)
    dev.run(phys_hz=args.phys_hz, batch_ms=args.batch_ms,
            realtime=not args.no_realtime)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
