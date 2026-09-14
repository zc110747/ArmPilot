# SO-ARM101 官方模型 · 来源与完整性记录

> 本目录内容 **逐字节原样复制自上游**，未做任何修改。
> 对应 spec §二十五·2（模型来源）与 §二十一 Phase 1（"不要修改官方模型内容"）。

## 1. 来源

| 项 | 值 |
|---|---|
| 仓库 | https://github.com/TheRobotStudio/SO-ARM100 |
| 上游路径 | `Simulation/SO101/` |
| 固定 commit | `eecbe3e0a9ebb23e25ad7b2759b03884c6660903` （`refs/heads/main`，2026-09-14 抓取） |
| 机器人 | SO-ARM101（SO-101） |
| 变体 | **`so101_new_calib`**（新标定版本；spec §二明确要求优先使用 new calibration） |
| 生成链 | Onshape CAD → `onshape-to-robot` → URDF / MJCF（官方 README 声明） |
| 许可 | **Apache License 2.0**（见同目录 `LICENSE`，11357 字节） |

上游自带说明文档已原样保留为 `README.md`（即上游 `Simulation/SO101/README.md`）。

## 2. 本目录布局

```text
assets/models/so-arm101/official/
├── so101_new_calib.urdf     官方 URDF（结构 / joint / axis / limit / inertial 参考）
├── so101_new_calib.xml      官方 MuJoCo MJCF（物理 / actuator / collision / 仿真）
├── README.md                上游 Simulation/SO101/README.md（原样）
├── LICENSE                  Apache-2.0 全文（原样）
├── SOURCE.md                本文件
└── assets/                  13 个二进制 STL（网格）
    ├── base_motor_holder_so101_v1.stl
    ├── base_so101_v2.stl
    ├── motor_holder_so101_base_v1.stl
    ├── motor_holder_so101_wrist_v1.stl
    ├── moving_jaw_so101_v1.stl
    ├── rotation_pitch_so101_v1.stl
    ├── sts3215_03a_no_horn_v1.stl
    ├── sts3215_03a_v1.stl
    ├── under_arm_so101_v1.stl
    ├── upper_arm_so101_v1.stl
    ├── waveshare_mounting_plate_so101_v2.stl
    ├── wrist_roll_follower_so101_v1.stl
    └── wrist_roll_pitch_so101_v2.stl
```

★ **`assets/` 子目录名不可改**：官方 MJCF 声明 `compiler meshdir="assets"`，
官方 URDF 的 mesh 路径为 `assets/xxx.stl`。保持官方布局 ⇒ **两个官方文件都无需任何路径适配**（零修改）。

## 3. 完整性（sha256）

### 3.1 定义文件

| 文件 | 字节 | sha256 |
|---|---:|---|
| `so101_new_calib.urdf` | 16231 | `3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c` |
| `so101_new_calib.xml` | 13921 | `d75253eb568e8a7214db9c631ab7bed4217f608a26f7276ebe9a7636cac82580` |

### 3.2 网格（二进制 STL，全部满足 `len == 84 + 50×tris`）

| 文件 | 字节 | 三角形 | sha256 |
|---|---:|---:|---|
| `assets/base_motor_holder_so101_v1.stl` | 1877084 | 37540 | `8cd2f241037ea377af1191fffe0dd9d9006beea6dcc48543660ed41647072424` |
| `assets/base_so101_v2.stl` | 471584 | 9430 | `bb12b7026575e1f70ccc7240051f9d943553bf34e5128537de6cd86fae33924d` |
| `assets/motor_holder_so101_base_v1.stl` | 1129384 | 22586 | `31242ae6fb59d8b15c66617b88ad8e9bded62d57c35d11c0c43a70d2f4caa95b` |
| `assets/motor_holder_so101_wrist_v1.stl` | 1052184 | 21042 | `887f92e6013cb64ea3a1ab8675e92da1e0beacfd5e001f972523540545e08011` |
| `assets/moving_jaw_so101_v1.stl` | 1413584 | 28270 | `785a9dded2f474bc1d869e0d3dae398a3dcd9c0c345640040472210d2861fa9d` |
| `assets/rotation_pitch_so101_v1.stl` | 883684 | 17672 | `9be900cc2a2bf718102841ef82ef8d2873842427648092c8ed2ca1e2ef4ffa34` |
| `assets/sts3215_03a_no_horn_v1.stl` | 865884 | 17316 | `75ef3781b752e4065891aea855e34dc161a38a549549cd0970cedd07eae6f887` |
| `assets/sts3215_03a_v1.stl` | 954084 | 19080 | `a37c871fb502483ab96c256baf457d36f2e97afc9205313d9c5ab275ef941cd0` |
| `assets/under_arm_so101_v1.stl` | 1975884 | 39516 | `d01d1f2de365651dcad9d6669e94ff87ff7652b5bb2d10752a66a456a86dbc71` |
| `assets/upper_arm_so101_v1.stl` | 1303484 | 26068 | `475056e03a17e71919b82fd88ab9a0b898ab50164f2a7943652a6b2941bb2d4f` |
| `assets/waveshare_mounting_plate_so101_v2.stl` | 62784 | 1254 | `e197e24005a07d01bbc06a8c42311664eaeda415bf859f68fa247884d0f1a6e9` |
| `assets/wrist_roll_follower_so101_v1.stl` | 1439884 | 28796 | `4b17b410a12d64ec39554abc3e8054d8a97384b2dc4a8d95a5ecb2a93670f5f4` |
| `assets/wrist_roll_pitch_so101_v2.stl` | 2699784 | 53994 | `6c7ec5525b4d8b9e397a30ab4bb0037156a5d5f38a4adf2c7d943d6c56eda5ae` |

**合计 13 个网格：16,129,292 字节（≈15.38 MiB），322,564 三角形。**

> 复现方式：`sha256sum so101_new_calib.urdf so101_new_calib.xml assets/*.stl`（在 git-bash 下）。
> ⚠️ 本机 `find` / `sort` 会解析到 Windows `System32` 同名程序（`sort -u` 静默返回空、`find` 报参数错误）
> ⇒ 清单类命令一律用 `awk '!seen[$0]++'` 或专用工具。

## 4. 适配记录（spec §二十一「如果必须做适配」）

> 格式：`original / modified / reason`

**无。** 本目录 14 个文件全部与上游逐字节一致。

原因：官方 MJCF 用 `meshdir="assets"`、官方 URDF 用 `assets/xxx.stl`，
本目录**原样复刻上游相对布局** ⇒ 官方模型自带的相对路径直接成立，无需改写。

### 4.1 勘误（Phase 0 → Phase 1）

Phase 0 分析文档曾把 13 个 STL 记为 "ASCII STL" 并据此保留了一个"必要时转二进制"的决策点（D4）。
P1 实测后确认**官方资源本就是二进制 STL**（13 个文件长度全部满足 `84 + 50×n`，头部 80 字节为空）
⇒ **D4 自动消解，无需任何转换**。已回填勘误到 `docs/architecture/so-arm101-phase0.md` 第 4 节与 §7·D4。

### 4.2 ★ 官方 URDF 与 MJCF 的 TCP 帧不一致（**必须显式声明**）

这是接入过程中发现的**唯一一处两份官方文件互相矛盾**的地方，属 spec §十六
「FK 与 MuJoCo 不一致时优先查 coordinate frame / joint origin / zero position」的情形。

同一个 TCP 点（平移逐位相同，均为 `[-0.0079, -0.000218121, -0.0981274]` 相对 `gripper_link`），
两文件的**姿态约定相差 90°（绕该点局部 y 轴）**：

| 文件 | 定义 | 等效旋转 |
|---|---|---|
| `so101_new_calib.urdf` · `<joint name="gripper_frame_joint">` | `origin rpy="0 3.14159 0"` | `Ry(π)` |
| `so101_new_calib.xml` · `<site name="gripperframe">` | `quat="0.707107 0 0.707107 ~0"` | `Ry(π/2)` |

**本项目的裁决：以 MJCF `gripperframe` 为准**（ArmPilot 的 MuJoCo 侧读的就是这个 site，
且 LeRobot 官方仿真亦以之为末端帧）。依据与实测（7 组位姿，`qpos` 弧度制）：

| TCP 约定 | 最大位置误差 | 最大姿态误差 |
|---|---:|---:|
| 取 URDF `gripper_frame_joint`（`Ry(π)`） | 0.0025 mm | **90.0004°** |
| **取 MJCF `gripperframe`（`Ry(π/2)`）← 采用** | **0.0025 mm** | **0.0007°** |

⇒ 采用 MJCF 约定后，**URDF 派生的 FK 与 MuJoCo 一致到 2.5 µm / 0.0007°**，无需任何"调参让它对上"。

★ 两种取法的**位置完全相同**（同一个原点），故 spec §十四 的主指标 `position_error` 不受此裁决影响；
受影响的只有姿态。此差异**不修改任何官方文件**，而由 ArmPilot 的 `RobotDefinition` 显式声明 TCP 帧。

## 5. 零修改可用性验证（Phase 1 实测）

| 验证 | 结果 |
|---|---|
| MuJoCo 直接加载 `so101_new_calib.xml` | ✅ `MjModel.from_xml_path()` 成功（mujoco **3.13.0**） |
| 规模 | `nq=6 nv=6 nu=6 nbody=8 ngeom=30 nsite=2 nmesh=13` |
| 13 个 mesh | ✅ 全部按官方 `meshdir="assets"` 自动载入，无需改路径 |
| 关节 | 6 个全部 `type=hinge`，名与范围见下表 |
| 执行器 | 6 个 `<position>`，`ctrlrange` ≡ 关节范围，`gear=1` ⇒ **关节空间位置伺服，无 offset/scale** |
| 末端 | ✅ 官方 `gripperframe` site 天然存在（`site_id=1`）⇒ **TCP 无需自造** |

关节范围（官方 MJCF，弧度→度）：

| 关节 | rad | deg |
|---|---|---|
| `shoulder_pan` | ±1.919862 | **±110.0000** |
| `shoulder_lift` | ±1.745329 | **±100.0000** |
| `elbow_flex` | ±1.690000 | ±96.8299 |
| `wrist_flex` | ±1.658063 | **±95.0000** |
| `wrist_roll` | −2.743847 / +2.841206 | −157.2109 / **+162.7891** |
| `gripper` | −0.174533 / +1.745329 | −10.0000 / **+100.0000** |

官方 MJCF 在 `qpos = 0` 时的 `gripperframe` 世界位姿（ArmPilot 的 `homePose` 参照）：

```text
pos = [0.39136190, -0.00001126, 0.22646875] m  =  [391.3619, -0.0113, 226.4687] mm
```

## 6. 使用方式

| 用途 | 文件 | 说明 |
|---|---|---|
| 结构与运动学参考（link / joint / axis / limit / inertial / visual） | `so101_new_calib.urdf` | 供 ArmPilot 派生 `RobotDefinition` 与 FK |
| MuJoCo 仿真（physics / actuator / collision / dynamics） | `so101_new_calib.xml` | **由 MuJoCo 直接加载，零修改** |
| 三维渲染网格 | `assets/*.stl` | 前端按 URDF 的 visual `<origin>` / `<geometry>` 摆放 |

★ 本阶段**不要求** URDF 与 MJCF 在 ArmPilot 内自动互转（spec §十）。
