#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
meArm 机械臂 上位机自动验证脚本 (PC 侧)
========================================
打开 COM4 (9600 8N1)，向裸机 AVR 固件下发多套串口指令，
读取 COM4 回显，对每条指令做 pass/fail 校验并打印报告。

用法:
    pip install pyserial
    python tools/host_verify.py            # 默认 COM4 / 9600
    python tools/host_verify.py COM3 115200

说明:
    固件开机打印启动横幅 + STATUS；本脚本先 RESET 建立已知状态，
    再逐条下发测试用例。角度在固件内会被强制范围钳位并斜坡运动，
    因此 SET 回显的是"生效(钳位后)角度"，STATUS 回显的是实时角度。
"""

import sys
import time
import re

try:
    import serial
except ImportError:
    sys.exit("需要 pyserial: 请先 `pip install pyserial`")

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 9600
RAMP_WAIT = 1.6  # 斜坡到位等待(s)，RAMP_STEP=3deg/20ms -> 120deg~0.8s，留余量


def open_port(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
        time.sleep(2.5)  # 等 optiboot 超时 + 固件 boot 稳定 (COM4 @ Uno)
        return ser
    except Exception as e:
        print(f"[FATAL] 无法打开 {port}: {e}")
        return None


def drain(ser, idle=0.15, max_wait=2.0):
    """读取直到空闲 idle 秒无新数据，返回全部文本。"""
    buf = ""
    last = time.time()
    while time.time() - last < max_wait:
        n = ser.in_waiting
        if n:
            data = ser.read(n).decode(errors="replace")
            buf += data
            last = time.time()
        else:
            if time.time() - last > idle:
                break
        time.sleep(0.02)
    return buf


def send(ser, cmd, wait=0.5):
    ser.write((cmd + "\r").encode())
    time.sleep(wait)
    return drain(ser)


def send_retry(ser, cmd, expect, tries=4, wait=0.4):
    """发送命令并在回显中找到 expect；用于开机初期 bootloader 窗口可能吞掉
    首条命令的情况，重试直到命中或次数用尽。"""
    for _ in range(tries):
        txt = send(ser, cmd, wait)
        if expect in txt:
            return txt
        time.sleep(0.25)
    return txt


def parse_status(text):
    """从 STATUS 行解析 S6..S9 的角度与模式: 返回 dict id->(angle,mode)。"""
    out = {}
    for m in re.finditer(r"S(\d)=(\d+)\(([AH])\)", text):
        out[int(m.group(1))] = (int(m.group(2)), m.group(3))
    return out


def get_status(ser, retries=2):
    """发送 STATUS 并解析; 首帧可能被开机横幅干扰, 失败重试。"""
    for _ in range(retries):
        txt = send(ser, "STATUS", 0.5)
        st = parse_status(txt)
        if st:
            return st
        time.sleep(0.2)
    return {}


results = []  # (name, passed, detail)


def check(name, cond, detail=""):
    results.append((name, bool(cond), detail))
    mark = "PASS" if cond else "FAIL"
    print(f"  [{mark}] {name}" + (f"  -- {detail}" if detail else ""))


def main():
    ser = open_port(PORT, BAUD)
    if ser is None:
        return 2
    print(f"* 已连接 {PORT} @ {BAUD}")

    # 清空开机横幅缓冲, 建立已知状态
    drain(ser, 0.4)

    # 关闭硬件摇杆/红外, 保证后续串口用例确定性 (避免未接硬件时浮动 ADC 干扰)
    # 首条命令可能落在 optiboot 窗口内被吞, 用重试确保命中
    txt = send_retry(ser, "JOYHW OFF", "OK JOYHW OFF")
    check("JOYHW OFF", "OK JOYHW OFF" in txt, txt.strip().replace("\r", " | "))
    txt = send_retry(ser, "IRHW OFF", "OK IRHW OFF")
    check("IRHW OFF", "OK IRHW OFF" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "ADC", 0.3)
    check("ADC 输出格式", "ADC A0=" in txt, txt.strip().replace("\r", " | "))

    send(ser, "RESET", 0.6)
    drain(ser, 0.3)

    # 1) RESET 后全 90
    st = get_status(ser)
    check("RESET -> all 90", all(st.get(i, (None,))[0] == 90 for i in (6, 7, 8, 9)),
          f"parsed={st}")

    # 2) 单控各舵机到合法角
    for cmd, sid, ang in [("SET 9 120", 9, 120), ("SET 8 30", 8, 30),
                          ("SET 7 150", 7, 150), ("SET 6 100", 6, 100)]:
        txt = send(ser, cmd, 0.4)
        ok = (f"OK SET S{sid}={ang}" in txt)
        check(cmd, ok, txt.strip().replace("\r", " | "))
        time.sleep(RAMP_WAIT)
        stx = get_status(ser)
        check(f"{cmd} 斜坡到位 S{sid}={ang}", stx.get(sid, (None,))[0] == ang,
              f"live={stx.get(sid)}")

    # 3) 范围钳位: 超出上限应被钳到上限
    txt = send(ser, "SET 9 200", 0.4)
    check("SET 9 200 -> 钳位150", "OK SET S9=150" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "SET 8 0", 0.4)
    check("SET 8 0 -> 钳位20", "OK SET S8=20" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "SET 7 0", 0.4)
    check("SET 7 0 -> 钳位80", "OK SET S7=80" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "SET 6 200", 0.4)
    check("SET 6 200 -> 钳位125", "OK SET S6=125" in txt, txt.strip().replace("\r", " | "))

    # 4) 非法 id
    txt = send(ser, "SET 5 90", 0.4)
    check("SET 5 90 -> BAD_ID", "ERR BAD_ID" in txt, txt.strip().replace("\r", " | "))

    # 5) 左右舵机允许同时工作 (用户: 左右舵允许同时工作)
    txt = send(ser, "SET 7 100 8 50", 0.4)
    check("SET 7 100 8 50 -> 左右同时成功", "OK SET S7=100 S8=50" in txt,
          txt.strip().replace("\r", " | "))
    # 6) 超过 3 个电机
    txt = send(ser, "SET 9 90 8 50 7 100 6 80", 0.4)
    check("4电机 -> TOO_MANY", "ERR TOO_MANY" in txt, txt.strip().replace("\r", " | "))
    # 7) 合法组合(不含左右同时): base+grip
    txt = send(ser, "SET 9 90 6 80", 0.4)
    check("SET 9 90 6 80 -> 合法组合", "OK SET S9=90 S6=80" in txt, txt.strip().replace("\r", " | "))

    # 8) 简写 S<id>=<angle>
    txt = send(ser, "S7=90", 0.4)
    check("S7=90 简写", "OK SET S7=90" in txt, txt.strip().replace("\r", " | "))

    # 9) AUTO 自变化 + STOP 冻结
    send(ser, "RESET", 0.6); drain(ser, 0.3)
    txt = send(ser, "AUTO 9", 0.4)
    check("AUTO 9 接受", "OK AUTO S9" in txt, txt.strip().replace("\r", " | "))
    st_a = get_status(ser)
    check("AUTO 后 S9 模式=A", st_a.get(9, (None, None))[1] == "A", f"{st_a.get(9)}")
    time.sleep(RAMP_WAIT)
    st_b = get_status(ser)
    check("AUTO 后 S9 角度已自变化(≠90)", st_b.get(9, (90,))[0] != 90, f"live={st_b.get(9)}")
    txt = send(ser, "STOP 9", 0.4)
    check("STOP 9 冻结", "OK STOP S9" in txt, txt.strip().replace("\r", " | "))
    st_c = get_status(ser)
    check("STOP 后 S9 模式=H", st_c.get(9, (None, None))[1] == "H", f"{st_c.get(9)}")
    frozen = st_c.get(9, (None,))[0]
    time.sleep(0.6)
    st_d = get_status(ser)
    check("STOP 后角度保持", st_d.get(9, (None,))[0] == frozen, f"{frozen} -> {st_d.get(9)}")

    # 10) JOY 单轴步进 (逻辑同原 Arduino 摇杆 handleJoystickControl)
    #     方向阈值保持一致; 但步进改为按偏移比例 (raw=900 -> 5 度/次, 远快于原 +/-1)
    send(ser, "RESET", 0.6); drain(ser, 0.3)
    # base(9): raw>800 -> -step ; raw<200 -> +step
    txt = send(ser, "JOY 9 900", 0.3)
    check("JOY 9 900 -> S9=85 (fast)", "OK JOY S9=85" in txt, txt.strip().replace("\r", " | "))
    st = get_status(ser)
    check("JOY 9 900 后 S9=85", st.get(9, (None,))[0] == 85, f"{st.get(9)}")
    txt = send(ser, "JOY 9 100", 0.3)
    check("JOY 9 100 -> S9=90", "OK JOY S9=90" in txt, txt.strip().replace("\r", " | "))
    # left(8): raw<200 -> -step ; raw>800 -> +step (反向)
    txt = send(ser, "JOY 8 100", 0.3)
    check("JOY 8 100 -> S8=85 (fast)", "OK JOY S8=85" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "JOY 8 900", 0.3)
    check("JOY 8 900 -> S8=90", "OK JOY S8=90" in txt, txt.strip().replace("\r", " | "))

    # 11) JOY 四轴整帧 (JOY <raw9> <raw8> <raw6> <raw7>)
    send(ser, "RESET", 0.6); drain(ser, 0.3)
    txt = send(ser, "JOY 900 100 900 900", 0.3)
    ok = all(f"S{i}=85" in txt for i in (9, 8, 6, 7))
    check("JOY 整帧 各-5 (fast)", ok, txt.strip().replace("\r", " | "))
    st = get_status(ser)
    check("JOY 整帧 后四轴=85", all(st.get(i, (None,))[0] == 85 for i in (9, 8, 7, 6)), f"{st}")

    # 12) IR 按键 ±2 (复刻原 Arduino IRremote 映射)
    send(ser, "RESET", 0.6); drain(ser, 0.3)
    txt = send(ser, "IR F708FF00", 0.3)   # 左  base(9)+2 -> 92
    check("IR F708FF00 左 S9=92", "OK IR 左 S9=92" in txt, txt.strip().replace("\r", " | "))
    st = get_status(ser)
    check("IR 后 S9=92", st.get(9, (None,))[0] == 92, f"{st.get(9)}")
    txt = send(ser, "IR A55AFF00", 0.3)   # 右  base(9)-2 -> 90
    check("IR A55AFF00 右 S9=90", "OK IR 右 S9=90" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "IR E718FF00", 0.3)   # 上  right(7)+2 -> 92
    check("IR E718FF00 上 S7=92", "OK IR 上 S7=92" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "IR AD52FF00", 0.3)   # 下  right(7)-2 -> 90
    check("IR AD52FF00 下 S7=90", "OK IR 下 S7=90" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "IR BB44FF00", 0.3)   # 数字4 grip(6)+2 -> 92
    check("IR BB44FF00 数字4 S6=92", "OK IR 数字4 S6=92" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "IR BC43FF00", 0.3)   # 数字6 grip(6)-2 -> 90
    check("IR BC43FF00 数字6 S6=90", "OK IR 数字6 S6=90" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "IR DEADBEEF", 0.3)   # 未知码
    check("IR 未知码 -> UNKNOWN", "ERR IR UNKNOWN" in txt, txt.strip().replace("\r", " | "))

    # 12.1) JOYHW / IRHW 开关
    txt = send(ser, "JOYHW ON", 0.3)
    check("JOYHW ON", "OK JOYHW ON" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "JOYHW OFF", 0.3)
    check("JOYHW OFF", "OK JOYHW OFF" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "IRHW ON", 0.3)
    check("IRHW ON", "OK IRHW ON" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "IRHW OFF", 0.3)
    check("IRHW OFF", "OK IRHW OFF" in txt, txt.strip().replace("\r", " | "))

    # 13) IR 动作序列引擎 (按钮 1/3/7/9 各一套 ~15s 循环, 5 键/摇杆停止)
    send(ser, "RESET", 0.6); drain(ser, 0.3)
    # 13.1 SEQ 1 启动
    txt = send(ser, "SEQ 1", 0.3)
    check("SEQ 1 启动", "OK IRSEQ 1 start" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "SEQ ?", 0.3)
    check("SEQ ? -> running 1", "running 1" in txt, txt.strip().replace("\r", " | "))
    # 13.2 运行中切换另一套 (SEQ 3) -> 从头执行
    time.sleep(1.4)
    txt = send(ser, "SEQ 3", 0.3)
    check("SEQ 3 (运行中切换) -> 重启", "OK IRSEQ 3 start" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "SEQ ?", 0.3)
    check("SEQ ? -> running 3 (已切换)", "running 3" in txt, txt.strip().replace("\r", " | "))
    # 13.3 IR 硬件码触发同一套 (按钮1 0xC13E01FE)
    send(ser, "SEQ STOP", 0.3); drain(ser, 0.3)
    txt = send(ser, "IR C13E01FE", 0.3)
    check("IR C13E01FE (按钮1) -> SEQ 1 启动", "OK IRSEQ 1 start" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "SEQ ?", 0.3)
    check("IR 按钮1 后 running 1", "running 1" in txt, txt.strip().replace("\r", " | "))
    # 13.4 IR 按钮5 (0xC53A05FA) 停止循环
    txt = send(ser, "IR C53A05FA", 0.3)
    check("IR C53A05FA (按钮5) -> 停止", "OK IRSEQ 1 stop" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "SEQ ?", 0.3)
    check("IR 按钮5 后 idle", "idle" in txt, txt.strip().replace("\r", " | "))
    # 13.5 摇杆指令(JOY)停止循环 (req 3)
    txt = send(ser, "SEQ 7", 0.3)
    check("SEQ 7 启动", "OK IRSEQ 7 start" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "JOY 900 100 900 900", 0.3)
    check("JOY 指令 -> 停止循环", "OK IRSEQ 7 stop" in txt, txt.strip().replace("\r", " | "))
    txt = send(ser, "SEQ ?", 0.3)
    check("JOY 停止后 idle", "idle" in txt, txt.strip().replace("\r", " | "))
    # 13.6 非法 SEQ 参数
    txt = send(ser, "SEQ 2", 0.3)
    check("SEQ 2 -> 非法", "ERR SEQ" in txt, txt.strip().replace("\r", " | "))
    # 13.7 未知 IR 码 (按钮1 不在, 但序列码组外) 不影响 idle
    send(ser, "SEQ STOP", 0.3); drain(ser, 0.3)

    # 收尾复位
    send(ser, "SEQ STOP", 0.3)
    send(ser, "RESET", 0.6)
    ser.close()

    # 报告
    passed = sum(1 for _, ok, _ in results if ok)
    total = len(results)
    print("\n" + "=" * 56)
    print(f"验证结果: {passed}/{total} PASS")
    print("=" * 56)
    for name, ok, detail in results:
        if not ok:
            print(f"  FAIL: {name}  ({detail})")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
