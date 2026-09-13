# -*- coding: utf-8 -*-
"""仿真数据记录（spec §34）：只写 JSONL / CSV，**不引入数据库**。

字段清单来自 `config/physics.yaml` 的 `recording.fields`，不在代码里重复一遍 ——
"配置化"这条在本项目是硬约束：任何一处把字段名写死，就多了一个漂移点。

    sim_time        仿真时间（秒）—— 由物理步数决定，与墙钟无关
    wall_time       墙钟时间（秒，perf_counter）—— 用来核对实时倍率
    joint_angle     关节角（度，**绝对语义**，与 UI / 协议一致）
    joint_velocity  关节角速度（rad/s）
    joint_torque    执行器广义力（N·m）
    target_joint    上层**请求**的目标关节角（度，绝对语义）。
                    与 `joint_angle` 的差 = 跟踪误差（含舵机限速欠债 + 位置环稳态误差）。
                    ⚠️ 若要单独看"限速"这一项，用 `MeArmSim.ctrl_angles_deg()` ——
                    那是限速后真正写进 `ctrl` 的目标，与本字段不同。
    end_effector    TCP 世界坐标（米）
    contact_count   当前接触对数
"""

from __future__ import annotations

import csv
import json
import time
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any

from robotcfg import load_physics  # noqa: E402  (调用方负责把包目录加进 sys.path)


class RecordError(RuntimeError):
    """记录器使用错误。"""


def _plain(value: Any) -> Any:
    """把 numpy 标量 / 数组转成可 JSON 序列化的原生类型。"""
    if isinstance(value, Mapping):
        return {k: _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    if hasattr(value, "tolist"):
        return _plain(value.tolist())
    if hasattr(value, "item"):
        return value.item()
    return value


def flatten(row: Mapping[str, Any]) -> dict[str, Any]:
    """把嵌套 dict 展平成 `a.b` 形式的单层键（CSV 需要）。"""
    out: dict[str, Any] = {}
    for k, v in row.items():
        if isinstance(v, Mapping):
            for kk, vv in v.items():
                out[f"{k}.{kk}"] = _plain(vv)
        else:
            out[k] = _plain(v)
    return out


# `recording.fields` 里的字段名 → `SimState.as_dict()` 里的键。
# `None` 表示该列由记录器自己产生（墙钟时间）。
FIELD_SOURCE: dict[str, str | None] = {
    "sim_time": "time",
    "wall_time": None,
    "joint_angle": "joint_angles",
    "joint_velocity": "joint_velocities",
    "joint_torque": "joint_torques",
    "target_joint": "target_joints",
    "end_effector": "end_effector",
    "contact_count": "contact_count",
}


def build_row(state: Mapping[str, Any], fields: Iterable[str],
              wall_time: float | None = None) -> dict[str, Any]:
    """按 `fields` 从 `SimState.as_dict()` 的结果里挑出要记录的键。

    ⚠️ 配置里的字段名（`recording.fields`）与 `SimState.as_dict()` 的键**不同名**：
    配置用单数（`joint_angle`）表示"这一类量记一列"，而状态 schema 用复数
    （`joint_angles`）表示"一个关节角映射"。这里用一张**显式映射表**把两者接起来，
    而不是去改配置或改状态 —— 前者会让配置文件读起来别扭，后者会破坏
    §19 统一状态 schema 的稳定性（那个 schema 还喂给 Web UI）。
    """
    src = dict(state)
    wall = time.perf_counter() if wall_time is None else wall_time
    row: dict[str, Any] = {}
    for f in fields:
        if f not in FIELD_SOURCE:
            raise RecordError(
                f"recording.fields 里有未知字段 {f!r}\n可选：{sorted(FIELD_SOURCE)}")
        key = FIELD_SOURCE[f]
        if key is None:                      # 由记录器自己产生（墙钟）
            row[f] = wall
        elif key in src:
            row[f] = src[key]
        else:
            raise RecordError(f"字段 {f!r} 需要状态里的 {key!r}，但 as_dict() 没提供")
    return row


class Recorder:
    """一个仿真会话的记录器（上下文管理器，异常退出也会关文件）。"""

    def __init__(
        self,
        path: Path | str,
        *,
        fmt: str | None = None,
        fields: Iterable[str] | None = None,
        enabled: bool | None = None,
    ) -> None:
        cfg = load_physics().recording
        self.fmt = (fmt or cfg.get("format") or "jsonl").lower()
        if self.fmt not in ("jsonl", "csv"):
            raise RecordError(f"不支持的记录格式 {self.fmt!r}（只做 jsonl / csv）")
        self.fields = list(fields if fields is not None else (cfg.get("fields") or []))
        if not self.fields:
            raise RecordError("recording.fields 为空 —— 拒绝产生没有列的数据文件")
        self.enabled = bool(cfg.get("enabled", False)) if enabled is None else bool(enabled)
        self.path = Path(path)
        self._fh = None
        self._writer = None
        self._header: list[str] | None = None
        self.count = 0

    # -- 生命周期 -------------------------------------------------------

    def open(self) -> "Recorder":
        if not self.enabled:
            return self
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = self.path.open("a" if self.path.suffix == ".csv" else "w",
                                  encoding="utf-8", newline="")
        if self.fmt == "csv":
            self._writer = csv.DictWriter(self._fh, fieldnames=[])  # 表头按首行决定
        return self

    def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None
            self._writer = None

    def __enter__(self) -> "Recorder":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    # -- 写入 -----------------------------------------------------------

    def write(self, state: Mapping[str, Any], *, wall_time: float | None = None) -> None:
        if not self.enabled or self._fh is None:
            return
        row = flatten(build_row(state, self.fields, wall_time))
        if self.fmt == "jsonl":
            self._fh.write(json.dumps(row, ensure_ascii=False) + "\n")
        else:
            if self._header is None:
                self._header = list(row)
                self._writer = csv.DictWriter(self._fh, fieldnames=self._header)
                self._writer.writeheader()
            self._writer.writerow(row)
        self._fh.flush()
        self.count += 1
