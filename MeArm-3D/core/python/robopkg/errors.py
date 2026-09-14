"""`robopkg` 的错误类型。

刻意**不**继承 `robotcfg.ConfigError` —— 两者是不同层的事实错误：
配置错误 = "这台机器人的真值文件写错了"；
包错误 = "这台机器人的**封装**（manifest / 指针 / 能力声明）写错了"。
混成一个类型会让"哪一层错了"在日志里消失。
"""

from __future__ import annotations


class PackageError(RuntimeError):
    """Robot Package 的 manifest / 指针 / 能力声明不合法。"""
