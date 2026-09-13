"""pytest 公共夹具（spec §35）。

运行方式（必须在**装了 mujoco / pyyaml 的解释器**下）::

    cd MeArm-3D
    <python> -m pytest tests/sim -q

本项目的 Python 解释器建议用隔离环境：
    ~/.workbuddy/binaries/python/envs/default/Scripts/python.exe
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]          # tests/sim -> tests -> MeArm-3D
SIM_DIR = ROOT / "simulation" / "mujoco"
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

from robotcfg import PhysicsCfg, RobotCfg, load_physics, load_robot  # noqa: E402
from units import ensure_utf8_stdout  # noqa: E402

# Windows 控制台默认 GBK，测试里打印中文会乱码
ensure_utf8_stdout()


@pytest.fixture(scope="session")
def robot() -> RobotCfg:
    """config/robot.yaml（运动学真值）—— 全测试共用，只读。"""
    return load_robot()


@pytest.fixture(scope="session")
def physics() -> PhysicsCfg:
    """config/physics.yaml（物理参数）—— 全测试共用，只读。"""
    return load_physics()


@pytest.fixture
def sim(robot: RobotCfg, physics: PhysicsCfg):
    """每个测试一个**干净**的仿真实例。

    刻意不用 session 作用域：仿真是有状态的（时间、qvel、warmstart），
    跨测试复用会让"前一个测试留下的速度"污染后一个测试的判据。
    模型编译约 50ms，20 个测试也只多花 1s，不值得为省这点时间冒污染风险。
    """
    from model import MeArmSim

    s = MeArmSim(robot=robot, physics=physics)
    yield s


@pytest.fixture(scope="session")
def kinematics():
    """前端真实运动学（`ik.ts` / `fk.ts`）的进程级桥 —— spec §21 的独立裁判。

    session 作用域：Vite 冷启动 + 模块加载约 1s，且模型元信息只需取一次。
    桥脚本本身是"一次性 CLI"，所以每次 `solve()` 仍会起一个独立 node 进程。

    ⚠️ 刻意**不做 skip 降级**：node 或桥不可用时让 IK 用例**失败**并给出修复指引。
    跳过会让"Phase 9 IK 验收"在报告里显示成绿色而实际从未执行 —— 那是伪造通过。
    """
    from ikbridge import KinematicsBridge

    bridge = KinematicsBridge()
    yield bridge
    bridge.close()


def zero_pose(robot: RobotCfg) -> dict[str, float]:
    """全零位形（关节空间原点）。"""
    return {jid: 0.0 for jid in robot.joint_order()}
