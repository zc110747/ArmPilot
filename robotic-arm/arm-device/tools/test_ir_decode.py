#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NEC 红外解码逻辑自测 (无硬件)
===========================
把 bsp/ir.c 里的状态机 1:1 移植到 Python，构造 8 个原工程遥控码对应的
NEC 波形（边沿+tick 时长），喂给状态机，断言解码出的 32 位 raw 与原码一致。
这样在没有红外接收头的情况下也能验证解码算法正确。

tick = 4us (与固件 Timer0 预分频64 一致)。
"""
import sys

# ---- 固件 bsp/ir.c 状态机阈值 (单位: tick) ----
T_START_MARK_LO, T_START_MARK_HI = 1600, 2900
T_START_SPC_LO,  T_START_SPC_HI  = 800, 1500
T_BIT1_THRESH = 350

MARK = 141      # 562.5us 载波突发
SPACE0 = 141    # 逻辑0 间隔 (总 1.125ms)
SPACE1 = 422    # 逻辑1 间隔 (总 2.25ms)
START_MARK = 2250   # 9ms
START_SPC  = 1125   # 4.5ms


def bytes_to_bits(bs):
    """4 字节, 每字节 LSB-first, 拼成 32 位 bit 序列 (bit0 = 最先发送)."""
    bits = []
    for byte in bs:
        for i in range(8):
            bits.append((byte >> i) & 1)
    return bits


def build_edges(bits):
    """返回边沿列表 [(level_after_edge, dt_ticks), ...].
    level: 0=LOW(载波), 1=HIGH(空闲). dt = 刚结束的那段时长.
    解码在下降沿用 dt(间隔) 判 bit; 9ms/4.5ms 由上升沿/下降沿的 dt 判定。"""
    edges = [(0, START_MARK)]      # 起始下降沿 (9ms 突发开始, dt=空闲段, state0 忽略)
    edges.append((1, START_MARK))  # 上升沿: dt=9ms 突发 -> state1
    edges.append((0, START_SPC))   # 下降沿: dt=4.5ms 起始间隔 -> state2
    for b in bits:
        edges.append((1, MARK))                        # 上升沿: dt=bit 突发(141, 忽略)
        edges.append((0, SPACE1 if b else SPACE0))     # 下降沿: dt=bit 间隔 -> 判 bit
    edges.append((1, MARK))          # 停止位上升沿
    return edges


def decode(edges):
    state = 0
    ir_data = 0
    ir_bits = 0
    ir_code = None
    for lvl, dt in edges:
        if lvl == 1:
            if state == 0:
                if T_START_MARK_LO <= dt <= T_START_MARK_HI:
                    state = 1
        else:
            if state == 1:
                if T_START_SPC_LO <= dt <= T_START_SPC_HI:
                    state = 2; ir_bits = 0; ir_data = 0
                else:
                    state = 0
            elif state == 2:
                b = 1 if dt > T_BIT1_THRESH else 0
                ir_data |= (b << ir_bits)
                ir_bits += 1
                if ir_bits >= 32:
                    b0 = ir_data & 0xFF
                    b1 = (ir_data >> 8) & 0xFF
                    b2 = (ir_data >> 16) & 0xFF
                    b3 = (ir_data >> 24) & 0xFF
                    if (~b0 & 0xFF) == b1 and (~b2 & 0xFF) == b3:
                        ir_code = (b0 << 24) | (b1 << 16) | (b2 << 8) | b3
                    state = 0
    return ir_code


def main():
    codes = [0xF708FF00, 0xA55AFF00, 0xB946FF00, 0xEA15FF00,
             0xE718FF00, 0xAD52FF00, 0xBB44FF00, 0xBC43FF00]
    ok = 0
    for code in codes:
        b0 = (code >> 24) & 0xFF
        b1 = (code >> 16) & 0xFF
        b2 = (code >> 8) & 0xFF
        b3 = code & 0xFF
        assert (~b0 & 0xFF) == b1 and (~b2 & 0xFF) == b3, f"码 {code:08X} 反码不成立"
        bits = bytes_to_bits([b0, b1, b2, b3])
        got = decode(build_edges(bits))
        if got == code:
            print(f"  [PASS] {code:08X} -> decode {got:08X}")
            ok += 1
        else:
            g = f"{got:08X}" if got is not None else "None"
            print(f"  [FAIL] {code:08X} -> decode {g}")
    print(f"\nNEC 解码自测: {ok}/{len(codes)} PASS")
    return 0 if ok == len(codes) else 1


if __name__ == "__main__":
    sys.exit(main())
