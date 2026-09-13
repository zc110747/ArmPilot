#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
_cam_stability.py —— 相机稳定性探针（静止场景，不驱动机械臂）

为什么需要它
------------
ADR D39 已确认：**相机整体位移 6px 是反解退化的头号杀手**（枢轴偏 → 尺度 s 偏 -7.2%
→ 假偏置 → 全批污染）。但"相机稳不稳"不能靠肉眼判断 —— 需要量化：

  1. 连抓 N 帧（臂完全静止，只留相机自身噪声 + 自动曝光/对焦微调）
  2. 逐帧做**互相关**估整体位移（±8px 搜索）
  3. 统计 max |位移| 与残差

判读（沿用 D39）：
  * max |位移| == 0 且残差 ≈ 噪声底   → 相机静止，可以跑验收
  * max |位移| >= 2px 且位移后残差显著下降 → 相机仍在漂，本批作废
  * 位移 == 0 但残差很高               → 场景内容变了（臂动了 / 台面被改）

用法
----
    python tools/cam_stability.py            # 抓 10 帧
    python tools/cam_stability.py 15         # 抓 15 帧
"""
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '.workbuddy' / 'analysis' / 'cam_stability'
OUT.mkdir(parents=True, exist_ok=True)

CAM = os.environ.get('MEARM_CAM', 'Integrated Camera')
W, H = 1280, 720
N = int(sys.argv[1]) if len(sys.argv) > 1 else 10


def grab(path):
    """抓一帧，失败自动重试 yuyv422（与 verify_serial_e2e.mjs 同策略）。"""
    base = ['-hide_banner', '-loglevel', 'error',
            '-f', 'dshow', '-rtbufsize', '100M',
            '-video_size', f'{W}x{H}', '-i', f'video={CAM}',
            '-frames:v', '1', '-q:v', '2', '-update', '1', '-y', str(path)]
    for args in (base, base[:7] + ['-pixel_format', 'yuyv422'] + base[7:]):
        r = subprocess.run(['ffmpeg'] + args, capture_output=True, text=True)
        if r.returncode == 0 and path.exists() and path.stat().st_size > 1024:
            return True
    print(f'  ! 抓帧失败: {r.stderr.strip().splitlines()[-1:] or r.returncode}')
    return False


def best_shift(a, b, rng=8):
    """b 相对 a 的最优整数位移 -> (残差, dx, dy)。a/b 为 float32 灰度。"""
    y0, y1, x0, x1 = 200, 520, 380, 900     # 取中部一块（含臂 + 底座纹理）
    A = a[y0:y1, x0:x1]
    best = (float('inf'), 0, 0)
    for dy in range(-rng, rng + 1):
        for dx in range(-rng, rng + 1):
            B = b[y0 + dy:y1 + dy, x0 + dx:x1 + dx]
            if B.shape != A.shape:
                continue
            err = float(np.abs(A - B).mean())
            if err < best[0]:
                best = (err, dx, dy)
    return best


print(f'==== 相机稳定性探针 · {N} 帧静止 ====')
print(f'# 设备 {CAM} · {W}x{H}')
print(f'# 输出 {OUT}')

files = []
for i in range(N):
    f = OUT / f'st_{i:02d}.jpg'
    if grab(f):
        files.append(f)
        print(f'  [{i:02d}] ok  {f.stat().st_size} bytes')
    else:
        print(f'  [{i:02d}] FAILED')

if len(files) < 2:
    print('帧数不足，无法评估。')
    sys.exit(2)

imgs = [np.asarray(Image.open(f).convert('L'), dtype=np.float32) for f in files]

# 参考帧 = 第 0 帧
ref = imgs[0]
print('\n-- 逐帧 vs 首帧（互相关 ±8px）--')
print(f'{"帧":>4} {"dx":>5} {"dy":>5} {"位移前残差":>11} {"位移后残差":>11} {"降幅":>8}')
stats = []
for i in range(1, len(imgs)):
    before = float(np.abs(ref - imgs[i]).mean())
    after, dx, dy = best_shift(ref, imgs[i])
    drop = (before - after) / before * 100 if before > 0 else 0.0
    stats.append((dx, dy, after))
    print(f'{i:>4} {dx:>5} {dy:>5} {before:>11.3f} {after:>11.3f} {drop:>7.1f}%')

dxs = np.array([s[0] for s in stats])
dys = np.array([s[1] for s in stats])
maxshift = max(int(np.abs(dxs).max()), int(np.abs(dys).max()))

# 相邻帧互相关（捕捉"缓慢漂移"，vs 首帧会被累积掩盖）
print('\n-- 相邻帧互相关（捕捉缓慢漂移）--')
adj = []
for i in range(1, len(imgs)):
    _, dx, dy = best_shift(imgs[i - 1], imgs[i])
    adj.append((dx, dy))
for i, (dx, dy) in enumerate(adj):
    print(f'  {i:02d}->{i + 1:02d}  dx={dx:>3}  dy={dy:>3}')

adj_max = max(max(abs(d[0]), abs(d[1])) for d in adj) if adj else 0
adj_sum = (int(sum(d[0] for d in adj)), int(sum(d[1] for d in adj)))

# 亮度稳定性（自动曝光漂移也污染反解）
means = [float(im.mean()) for im in imgs]
meds = [float(np.median(im)) for im in imgs]
print('\n-- 亮度稳定性 --')
print(f'  整图均值 min/max = {min(means):.1f} / {max(means):.1f}  (波动 {max(means) - min(means):.2f})')
print(f'  整图中位 min/max = {min(meds):.1f} / {max(meds):.1f}  (波动 {max(meds) - min(meds):.2f})')

# 判定
print('\n==== 判定 ====')
if maxshift == 0 and adj_max == 0:
    print(f'✅ 相机静止：{N} 帧内整体位移恒为 0（vs 首帧与相邻帧均 0）')
    print('   → 可以跑真机验收')
elif maxshift <= 1:
    print(f'⚠️  轻微抖动：vs 首帧最大位移 {maxshift}px（相邻帧 {adj_max}px）')
    print('   → 在 D39 的 6px 阈值内，可以跑；但若反解重复性 >1.5° 说明实际更糟')
else:
    print(f'❌ 相机仍在漂：vs 首帧最大位移 {maxshift}px（相邻帧 {adj_max}px，累积 {adj_sum}）')
    print('   → 按 D39，本批数据作废。先固定相机机身 + 扎住 USB 线缆。')

# 落一张可视化：首帧与末帧的差异图
if len(imgs) >= 2:
    d = np.abs(ref - imgs[-1])
    vis = np.clip(d * 2, 0, 255).astype(np.uint8)
    Image.fromarray(vis).save(OUT / 'diff_first_last.png')
    print(f'\n# 首末帧差异图 -> {OUT / "diff_first_last.png"}（画出完整轮廓 = 整体平移）')
