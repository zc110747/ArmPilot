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
CORE_PY = ROOT / "core" / "python"
for _p in (SIM_DIR, CORE_PY):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from robotcfg import (  # noqa: E402
    PhysicsCfg,
    RobotCfg,
    load_physics,
    load_robot_by_id,
    resolve_robot_entry,
)
from units import ensure_utf8_stdout  # noqa: E402

# Windows 控制台默认 GBK，测试里打印中文会乱码
ensure_utf8_stdout()

#: 本目录下的测试全部只针对 **MeArm-V1**（Golden Baseline）。
#:
#: ⚠️ 刻意**显式写出** id 而不是靠选择器的 `default`：与前端 22 处
#: `loadRobotModel('mearm-v1')` 是同一条纪律 —— 依赖缺省，将来改了 `default`
#: 就会**静默换掉被测对象**，而所有断言照样能绿（它们只是换了另一台机器人去比）。
MEARM_V1 = "mearm-v1"


@pytest.fixture(scope="session")
def robot() -> RobotCfg:
    """MeArm-V1 的 robot.yaml（运动学真值）—— 全测试共用，只读。"""
    return load_robot_by_id(MEARM_V1)


@pytest.fixture(scope="session")
def physics() -> PhysicsCfg:
    """MeArm-V1 的 physics.yaml（物理参数）—— 全测试共用，只读。"""
    return load_physics(resolve_robot_entry(MEARM_V1).physics_file)


@pytest.fixture
def sim(robot: RobotCfg, physics: PhysicsCfg):
    """每个测试一个**干净**的仿真实例。

    刻意不用 session 作用域：仿真是有状态的（时间、qvel、warmstart），
    跨测试复用会让"前一个测试留下的速度"污染后一个测试的判据。
    模型编译约 50ms，20 个测试也只多花 1s，不值得为省这点时间冒污染风险。
    """
    from model import RobotSim

    s = RobotSim(robot=robot, physics=physics)
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

    bridge = KinematicsBridge(robot_id=MEARM_V1)
    yield bridge
    bridge.close()


def zero_pose(robot: RobotCfg) -> dict[str, float]:
    """全零位形（关节空间原点）。"""
    return {jid: 0.0 for jid in robot.joint_order()}


# ---------------------------------------------------------------------------
# 多机器人轨（2026-09-14）
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def selector_robot_ids() -> list[str]:
    """`config/robots.yaml` 声明的**全部**机器人 id（按声明顺序）。

    ⚠️ 与上面的 `MEARM_V1` 常量刻意相反：夹具是"只要 MeArm"，
    而这里是"选择器有什么就跑什么" —— 统一 Sim2Sim 的回归矩阵必须
    **自动**覆盖新增机器人，而不是等人去改测试文件。
    """
    from robotcfg import load_robot_selector

    return list(load_robot_selector().robots.keys())


@pytest.fixture(scope="session")
def bridge_factory():
    """`robot_id → KinematicsBridge` 的会话级工厂（会话末统一关闭）。

    每个 id 一个桥实例、只建一次：Vite 冷启动 + 模块加载约 1s，
    每台机器人三次调用（info / fk / solve）若各自重建就得付三次。
    """
    from ikbridge import KinematicsBridge

    created: list = []

    def make(robot_id: str) -> object:
        bridge = KinematicsBridge(robot_id=robot_id)
        created.append(bridge)
        return bridge

    yield make
    for bridge in created:
        bridge.close()
