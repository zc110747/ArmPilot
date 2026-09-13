"""驱动**前端真实运动学**的进程级桥（spec §21）。

## 它解决的信任问题

Phase 9 要证明的是「项目自己的 IK 解出来的关节角，在 MuJoCo 里落到哪」。
如果验收程序用 Python **重写**一份 IK，那证明的只是"我又写了一遍、而且自洽" ——
证明不了 `frontend/src/robot/kinematics/ik.ts` 是对的。

所以这里通过 `frontend/tests/tools/kinematics-bridge.mjs` 加载**同一份 TS 源码**
（用 Vite 的 SSR 加载器，因为它有 `@config/robot.yaml?raw` 这种 Node 不认的语法）。
本模块只负责"起进程、递 JSON、收 JSON"，**不解释任何运动学语义** ——
一旦它开始自己算几何，独立判据就没了。
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any, Sequence

ROOT = Path(__file__).resolve().parents[2]           # tests/sim -> tests -> MeArm-3D
BRIDGE_SCRIPT = ROOT / "frontend" / "tests" / "tools" / "kinematics-bridge.mjs"
FRONTEND_DIR = ROOT / "frontend"


class BridgeError(RuntimeError):
    """桥本身跑不起来（与"IK 解不出来"是两回事，必须区分开报）。"""


def find_node() -> str:
    """定位 node 可执行文件。

    ⚠️ 刻意**不写死本机绝对路径** —— 项目其它前端脚本（`npm run test:e2e`）
    也是靠 PATH 找 node 的，这里保持一致；需要覆盖时用环境变量 `ARMPILOT_NODE`。
    """
    override = os.environ.get("ARMPILOT_NODE")
    if override:
        if not Path(override).is_file():
            raise BridgeError(f"ARMPILOT_NODE 指向的文件不存在：{override}")
        return override
    found = shutil.which("node")
    if not found:
        raise BridgeError(
            "PATH 里找不到 node，无法运行前端运动学桥。\n"
            "  Phase 9 的 IK 验收必须加载前端真实的 ik.ts，不能退化成 Python 重写版。\n"
            "  修复：把 node 加入 PATH，或设 ARMPILOT_NODE=<node 可执行文件绝对路径>。"
        )
    return found


class KinematicsBridge:
    """`frontend/tests/tools/kinematics-bridge.mjs` 的 Python 门面。

    生命周期：第一次调用时惰性启动（Vite 冷启动约 1s），随后每次调用复用同一批
    已加载的模块 —— 但**每次仍是一个独立 node 进程**：桥脚本本身是"一次性 CLI"，
    它跑完就退出。这样比常驻进程简单得多，代价是每次多 ~1s。
    """

    def __init__(
        self,
        *,
        node: str | None = None,
        script: Path | None = None,
        frontend_dir: Path | None = None,
        timeout: float = 300.0,
    ) -> None:
        self.node = node or find_node()
        self.script = Path(script or BRIDGE_SCRIPT)
        self.frontend_dir = Path(frontend_dir or FRONTEND_DIR)
        self.timeout = float(timeout)
        if not self.script.is_file():
            raise BridgeError(f"找不到运动学桥脚本：{self.script}")
        self._tmp = Path(tempfile.mkdtemp(prefix="armpilot-ikbridge-"))
        self._n = 0
        self._model_info: dict[str, Any] | None = None

    # -- 内部 ---------------------------------------------------------------

    def _invoke(self, *, info: bool, cases: Sequence[dict] | None) -> dict[str, Any]:
        self._n += 1
        req_path = self._tmp / f"req-{self._n}.json"
        out_path = self._tmp / f"res-{self._n}.json"
        args = [self.node, str(self.script), "--out", str(out_path)]
        if info:
            args.append("--info")
        else:
            req_path.write_text(
                json.dumps({"cases": list(cases or [])}, ensure_ascii=False),
                encoding="utf-8",
            )
            args += ["--in", str(req_path)]

        env = dict(os.environ)
        env["PYTHONIOENCODING"] = "utf-8"
        try:
            proc = subprocess.run(
                args,
                cwd=str(self.frontend_dir),
                capture_output=True,
                timeout=self.timeout,
                env=env,
            )
        except subprocess.TimeoutExpired as exc:
            raise BridgeError(f"运动学桥超时（{self.timeout:g}s）: {' '.join(args)}") from exc

        stdout = proc.stdout.decode("utf-8", "replace").strip()
        stderr = proc.stderr.decode("utf-8", "replace").strip()
        if proc.returncode != 0 or not out_path.is_file():
            raise BridgeError(
                f"运动学桥退出码 {proc.returncode}，未产出结果文件。\n"
                f"  命令: {' '.join(args)}\n  stdout: {stdout[-2000:]}\n  stderr: {stderr[-4000:]}"
            )
        data = json.loads(out_path.read_text(encoding="utf-8"))
        if not data.get("ok"):
            raise BridgeError(f"运动学桥返回失败：{json.dumps(data, ensure_ascii=False)[:2000]}")
        return data

    # -- 公开 ---------------------------------------------------------------

    def model_info(self) -> dict[str, Any]:
        """前端**实际加载到的** `RobotModel` 元信息（关节 / 限位 / 轴 / 耦合 / TCP / 2R 几何）。

        用途不是"当作真值"，而是**校验桥本身有没有加载错配置**：
        `test_bridge_model_matches_robot_yaml` 会把它与 `config/robot.yaml` 逐项对照。
        这一步不过，后面所有 IK 数字都没有意义。
        """
        if self._model_info is None:
            self._model_info = self._invoke(info=True, cases=None)["model"]
        return self._model_info

    def solve(self, cases: Sequence[dict]) -> list[dict[str, Any]]:
        """批量求解。`cases` 里每项至少要有 `target: [x, y, z]`（mm）。"""
        return list(self._invoke(info=False, cases=cases)["results"])

    def solve_one(self, target: Sequence[float], **opts: Any) -> dict[str, Any]:
        """单点求解。`opts` 透传 `near` / `seed` / `prefer`。"""
        res = self.solve([{"id": 0, "target": [float(v) for v in target], **opts}])
        if len(res) != 1:
            raise BridgeError(f"期望 1 条结果，收到 {len(res)} 条")
        return res[0]

    def close(self) -> None:
        """删掉临时目录（请求/响应 JSON）。失败不影响测试结论。"""
        shutil.rmtree(self._tmp, ignore_errors=True)

    def __enter__(self) -> "KinematicsBridge":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
