# `core/` —— ArmPilot 的共享机制（Robot-agnostic）

> 本目录对应重构 spec 的 **Core** 概念。三概念对应关系（spec §3 / §54）：
>
> | 概念 | 位置 | 一句话 | 换机器人时 |
> |---|---|---|---|
> | **Core** | `core/`（+ 现存的 `frontend/` `backend/` `simulation/` 共享部分） | 共享**机制** | 不用改 |
> | **Robot Package** | `robot-package/<id>/` | 机器人**差异**（含自带测试） | 只动这一个包 |
> | **Working Robot** | `working-robot/` | 当前机器人的**运行实例** | 整体替换 |
>
> **`Core ≠ MeArm`、`Core ≠ SO-101`；`Robot Package ≠ 一套完整 ArmPilot`；
> `Working Robot ≠ 第二套源码`。**

---

## 一、什么进 Core，什么不进

判据只有一条：**换一台机器人，这个文件需要改吗？**

- 不需要 → Core
- 需要 → 它属于某个 Robot Package

正面清单（进 Core）：加载器 / 注册表 / 运行时 / 运动学**接口**与**通用算法** /
包校验器 / 测试执行框架 / 内容哈希 / 构建脚本 / 传输与协议基础设施 / UI。

反面清单（**禁止**进 Core，spec §38 / §8）：

- ❌ 任何 `if robot == "mearm-v1"` / `if robot == "so-arm101"` 型特判
- ❌ 某台机器人的 FK/IK 算法、标定值、限位、MJCF、网格、纹理
- ❌ 某台机器人的测试**数据**（测试**执行框架**才进 Core，spec §10）
- ❌ 复制一份"看起来通用"的 MeArm 实现（那是把机器人实现塞回 Core）

**唯一的例外**是 Adapter（spec §39）：当"包的实现"与"Core 的接口"形状不一致时，
允许在**包的边界处**放转换器 —— 但 Adapter 的职责是**隔离差异**，
不是把包内逻辑重新搬进 Core。

---

## 二、铁律：manifest 与选择器**只放指针，不放数值**

`robot-package/<id>/manifest.yaml` 与 `config/robots.yaml` 允许出现：
文件路径、MJCF 里 site 的名字、布尔能力声明、包自身的封装版本。

**禁止**出现：尺寸 / 角度 / 限位 / 标定 / 物理量 / TCP 偏移 / 关节数 / 执行器数。
理由不是洁癖，而是**抄一份就是第二份真值**，而它与原件的漂移是**静默的**。

> ⚠️ 与 spec §6 示例的一处**有意偏离**：spec 的示例 manifest 里写了
> `dof: 4` 与 `actuator_count: 4`。本项目**不写**这两个数 —— 它们是
> `model.config` 指向的那份 `robot.yaml` 的**推导结果**，写进 manifest 就是第二份真值。
> 改为由 `core/python/robopkg/loader.py` **推导**，期望值放在**测试**里
> （`core/tests/test_package_contract.py`）。判据强度不降反升：
> 抄来的数不会自己发现自己过期，推导出来的会。

---

## 三、当前目录（Phase 1 已落地部分）

```text
core/
├── README.md                  ← 本文件
├── python/
│   ├── robopkg/               ← Core 的 Python 侧：包加载 / 校验 / 内容哈希 / CLI
│   │   ├── root.py            路径锚点（**唯一**一处算 PROJECT_ROOT）
│   │   ├── errors.py
│   │   ├── manifest.py        manifest 的加载与严格校验（缺字段就报错，不兜底）
│   │   ├── loader.py          RobotLoader：id → 推导后的完整视图（dof / 关节序 / 能力）
│   │   ├── content_hash.py    内容哈希（哈希的是**源**，产物不进）
│   │   ├── validator.py       PackageValidator
│   │   └── cli.py             `list / show / validate / hash`
│   └── (Phase 2 起接收 simulation/ 的机器人无关模块)
├── tests/
│   └── test_package_contract.py
├── frontend/                  ← Phase 2 起接收 frontend/src/robot 的通用部分（现为空档）
└── backend/                   ← Phase 2 起接收 backend/ 的通用部分（现为空档）
```

> `core/frontend/` 与 `core/backend/` 目录**有意暂不创建** —— 空目录会被 git 丢弃，
> 且"看起来搬完了"比"还差什么"更危险。搬迁在 Phase 2 逐子系统进行，
> 每步跑冻结校验（`freeze_baseline.py`）保证语义哈希不变。

---

## 四、诊断入口

```bash
# 列出全部包（不解析模型，故某台包坏掉也能列出来）
python core/python/robopkg/cli.py list

# 摊开一台机器人的推导结果（dof / 关节序 / 能力 / 内容哈希）
python core/python/robopkg/cli.py show mearm-v1

# 包契约校验：manifest ↔ 选择器 ↔ 指针存在性 ↔ 能力自洽
python core/python/robopkg/cli.py validate --all

# 只算内容哈希（update.bat 的陈旧判据用它）
python core/python/robopkg/cli.py hash so-arm101
```

---

## 五、能力声明的语义（`capabilities`）

`capabilities` 是**包对 Core 的承诺**，Core 据此决定开放哪些功能，不做猜测。
它**必须**与前端引擎的 `KinematicsCapability` 逐字段一致 ——
由 `frontend/tests/unit/robotManifest.test.ts` 跨端核对（Python 读不到 TS）。

| 字段 | 含义 | 约束 |
|---|---|---|
| `ik` | 提供逆运动学求解 | `true` ⟺ `kinematics.ik.type != "none"` |
| `position` | 可把末端送到指定**位置**（需要 IK） | `ik: true ⇒ position: true` |
| `orientation` | 支持**姿态**目标 | `ik: false ⇒ orientation: false`（没有逆解就不可能有姿态逆解） |
| `gripper` | 有夹爪自由度 | 由 `robot.yaml` 的关节表交叉核对 |
| `simulation` | 有可用 MJCF | `simulation.mjcf` 必须存在 |
| `hardware` | **本项目**里有真机链路 | 与"官方支持"无关，是"我们接没接" |

> `ik: false` 时**禁止**伪造逆解（spec §8/§35）：返回明确的
> `NOT_IMPLEMENTED` 失败码，而不是一个未经验证的解。SO-ARM101 就走这条路。
