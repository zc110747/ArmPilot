# 运动学变更提案（KINEMATIC CHANGE PROPOSAL）

> 配套文档：`docs/STEP_KINEMATICS_VALIDATION.md`（Phase 0 证据）
> 真值文件：`robot-package/mearm-v1/model/robot.yaml`（**本提案未修改它，一个字节都没改**）
> 日期：2026-09-15

---

## 1. 提案摘要

| 项目 | 结论 |
|---|---|
| **运动学结构是否需要变更** | ❌ **不需要**（无 `KINEMATIC_STRUCTURE_CONFLICT`） |
| **运动学参数是否需要变更** | ❌ **不需要**，且**不可由 STEP 推导**（理由见 §3） |
| **外观（3D 显示几何）是否需要变更** | ✅ **建议变更**（这是本次唯一的正向提案） |
| **冻结基线是否需要重新生成** | ❌ 不需要（真值零改动） |
| **是否阻塞外观阶段** | ❌ 不阻塞 |

一句话：**STEP 用来换外观；运动学一个数字都不动。**

---

## 2. 为什么运动学不需要改（结构层）

Phase 0 用 CAD 几何逐项核验了 `robot.yaml` 的结构声明，**全部通过**
（详见 `STEP_KINEMATICS_VALIDATION.md` §4.1）：

| `robot.yaml` 声明 | CAD 证据 | 结果 |
|---|---|---|
| `base.axis = [0,0,1]` | 底座舵机轴竖直 | ✅ |
| `shoulder.axis = [0,1,0]` | 肩舵机轴沿 Y | ✅ |
| `elbow.axis = [0,1,0]` | 肘舵机轴沿 Y | ✅ |
| `elbow.coupling = {shoulder, gain:-1}` | **肩轴 ∥ 肘轴**（0.00°，间距 107.39） | ✅ **结构实证** |
| `tool` 为 passive、爪锁水平 | 腕轴与肩轴平行 | ✅ |
| `gripper.axis = [1,0,0]` | 夹爪铰轴 ⟂ 前臂平面 | ✅ |
| 全部为转动副 | 全部为圆柱面，无平移/螺旋 | ✅ |

其中**最有价值的一条**：`elbow.coupling.gain = -1` 此前是**2026-09-13 照片反解**
的结论（拟合残差约 19%，yaml 注释自陈「若按实测残差硬拟合，等效增益 ≈ −0.81」）。
现在 CAD 给出**结构证据**：肩轴与肘轴**严格平行**、间距 107.39 mm ——
这是平行四连杆（parallelogram linkage）的定义性几何特征。

> ⇒ **建议在 `robot.yaml` 的注释里补一句「已由 CAD 几何实证」**（注释不是运动学量，
> 改动**不触发**冻结基线告警）。这是**可选**的文档性改进，不是必须项。

---

## 3. 为什么连杆长度**不能**从 STEP 反推

这是本次最容易犯错的一点，必须写清楚。

### 3.1 根本原因：STEP 是「单姿态快照」

| 需要的东西 | STEP 提供的东西 |
|---|---|
| 零点位（θ=0）下沿各杆 +Z 的伸展 `length` | **某一个具体姿态**下各关节轴的空间位置 |

两者只有在「建模姿态恰好是零点位」时才等价。而 **§4.4 已证明它不是零点位**：

- `robot.yaml` 零点位：`shoulder=0` ⇒ 大臂竖直；`elbow=112.62°` ⇒ 小臂前倾下 22.6°；
  两轴应相差 **22.6°**。
- CAD 实测：肩轴与肘轴夹角 **0.00°**。

⇒ 建模姿态下大臂与小臂**共线**（伸直的折叠态之一），与零点位不同。

### 3.2 错误做法（**明令禁止**）

| ❌ 错误做法 | 后果 |
|---|---|
| 把 `upper_arm` 侧板的 mesh 包围盒 z 长度当 `length` | spec §9 明令禁止；且包围盒含端盖/横撑/倒角，系统性偏大 |
| 把「CAD 里肩轴到肘轴的间距 107.39」直接写成 `upper_arm_link.length` | 把快照姿态当零位尺寸，**立即破坏 FK/IK 与冻结基线** |
| 用「底板到肩轴 106 mm」覆盖 `column_link.length = 60` | 该 106 mm 是**含舵机体 29.8 + 转盘板 + 立柱板的总高**，语义完全不同 |

### 3.3 如果将来确实要精测杆长，唯一可行的路径

必须拿到**能确定零点位的**补充数据，例如：

1. **让 STEP 的建模姿态可复现**：记录该姿态下 4 个舵机的角度读数；
   有了「姿态角 + 该姿态下的轴位置」才能反算零位长度。
2. 或者按既有方法：**实拍 + 标尺 + 正交取景**（`docs/hardware-measurement.md`），
   把 FK 骨架拟合到照片上解 `length`（当前 `length` 就是这么来的）。

**两条路都不在本次 STEP 任务的范围内。** 本次**不推导、不修改**任何 `length`。

---

## 4. 外观变更提案（本次唯一的正向动作）

### 4.1 现状

`robot.yaml` 的 `links[].geometry` / `details` 目前是**按开源 meArm 实物外观
参数化拼装**的近似件（蓝色倒角薄板 + 4 个黑色舵机的示意几何），
文件自述见 `robot.yaml` 第 43–45 行。

### 4.2 提案

用 STEP 的**真实 CAD 几何**替换参数化近似件，**仅限 `geometry` / `details`**：

| 允许改 | 禁止改 |
|---|---|
| `links[].geometry.*` | `links[].length` |
| `links[].details.*` | `joints[].*`（含 `axis` / `limit` / `coupling`） |
| `appearance.*`（渲染参数） | `actuators[].*` |
| `robot.tcp` | `robot.homePose` |

### 4.3 为什么这一步是安全的

`robot.yaml` 自己写明了这条边界（第 45 行）：

> **改 geometry 只影响外观，绝不影响 FK / IK / 标定。**

且冻结基线的判据是**语义核心哈希**（ADR D55），其白名单只含
`id/name/units/tcp/homePose` 与运动学/物理量 —— **`geometry` / `details` 不在其中**。
所以改外观**不会**让 `tests/sim/test_baseline_frozen.py` 报红。

> ⚠️ 但**必须**注意：`tcp` 在语义核心白名单里。因此 **§4.2 表里把 `robot.tcp` 列在
> 「允许改」一侧是笔误风险** —— 若外观阶段有任何理由触动 TCP，**一定先停下来问用户**。
> 本提案的实际立场是：**`robot.tcp` 不动**。

### 4.4 外观阶段的推荐做法（Phase 1 待办，本次未执行）

1. 把 STEP 的 111 个 occurrence 按**关节归属**分组（哪块板属于 `base_link` /
   `column_link` / `upper_arm_link` / `forearm_link` / `tool_link` / `jaw_link`）。
2. 每组导出一份轻量网格（STL/GLB），或保留现有的参数化件但**按 CAD 实测尺寸校正
   `size` / `position`**（后者改动更小、更安全）。
3. ⚠️ **不得**按视觉外观重新划分 `links`（spec §14–17 与用户此前明确要求）：
   显示分组**必须**沿用现有的 6 个 link 边界，只是每个 link 内的几何换成真实形状。
4. 跑完整验收七件套，确认 `geometry` 改动**没有**触碰任何运动学哈希。

---

## 5. 遗留待办（不阻塞外观，但必须在任何运动学改动前闭合）

| # | 待办 | 严重度 | 闭合方式 |
|---|---|---|---|
| T1 | 关节 `+θ` 旋向符号无法由 STEP 判定 | Level 2 | 保持 `robot.yaml` 现有符号（实拍反解），或补录 STEP 建模姿态角 |
| T2 | 连杆 `length` ×4 不可由 STEP 反推 | Level 2 | 实拍精测，或补录建模姿态角（§3.3） |
| T3 | 两根平行竖直轴（间距 14.78）的身份 | Level 2 | Phase 1 做连杆追踪：看哪块板把哪根轴连到哪根轴 |
| T4 | 夹爪铰轴销（R≈3.2）被提取阈值漏掉 | Level 1 | 阈值降到 3.0 复扫即可 |
| T5 | 摇杆总成与运动无关 | — | 按用户说明忽略；不进入任何分组 |

> T3 的两种解释（传动件 / 未建模自由度）**都不改变 5 关节拓扑**，
> 所以**不影响**本提案的有效性。若后续证明是一个真实的新自由度，
> 那是一次**产品决策**（要不要加关节），需另开任务，不能由几何推断代做。

---

## 6. 最终裁定

```
KINEMATIC_STRUCTURE_CONFLICT  = NO
KINEMATIC_PARAMETER_CHANGE    = NOT_REQUIRED  (and NOT_DERIVABLE from STEP)
APPEARANCE_CHANGE             = RECOMMENDED
TRUTH_FILES_MODIFIED          = NONE
BLOCKING_ISSUES               = NONE
```

**下一步建议**：进入 Phase 1（外观替换），严格限定在 `geometry` / `details`
两个字段内；任何触及 `length` / `axis` / `limit` / `coupling` / `actuators` /
`homePose` / `tcp` 的改动**必须先停下来向用户确认**。
