# STEP 装配体分析（含一次误判的复盘）

> 输入：`robot-package/mearm-v1/3d-structure/mearm3Dasm.STEP`（AP203，ISO-10303-21）
> 工具：`core/tools/step_report.py`（OCCT / Open CASCADE）
> 日期：2026-09-15

---

## 0. 摘要

**装配体是完整的**，111 个 occurrence 全部解析出几何：4 个 SG90 舵机、33 块结构板、
摇杆控制盒、Arduino Uno、若干标准件。可直接用作 **3D 外观**数据源。

本文档同时记录了**一次必须记住的误判**：早期我用自研正则解析器读这个文件，
得出「装配体只有层级没有几何、46 个 PRODUCT_DEFINITION 全悬空」的结论并写进了文档。
该结论**是错的**，根因是**解析器**而非**数据**。复盘见 §2。

---

## 1. OCCT 解析结果（可信基线）

```
nodes=111  leaves=103  with_geometry=111  servos=4
```

顶层：

```
mearm3Dasm                                     sz=[165.1, 153.5, 225.6]  com=[-31.96, 15.37, 231.77]
  mearm avec arduino                           sz=[165.1, 153.5, 225.6]
```

54 个唯一产品名，111 个 occurrence：

| 类别 | 名称 | 数量 |
|---|---|---|
| 结构板 | `1`–`31`、`A`、`B` | 33 |
| 舵机 | `microservoSG90` | **4** |
| 舵盘 | `Tower pro micro servo SG90 HORN` | 3 |
| 电子 | `arduinoUno_Detailed` / `board` / `potentiometer___` | 7 |
| 摇杆盒 | `Joystick assembly*` / `Thumb Joystick*` / `joystickhead*` / `tactileswitch-short` / `body___` | 9 |
| 标准件 | M3 螺丝 ×33、M3 螺母 ×10 | 43 |
| 底盘脚 | `pieds sous la platine` | 4 |

---

## 2. 误判复盘（重要）

### 2.1 我当时的错误结论

- 「241 个 occurrence，**0 个**解析出几何」
- 「46 个 `PRODUCT_DEFINITION` 一个都到不了 `ADVANCED_BREP_SHAPE_REPRESENTATION`」
- 「几何与层级是断开的两半」
- ⇒ 写入 `PHASE_0_RESULT = BLOCKED`

### 2.2 真实原因：自研正则解析器的**三处静默丢弃**

| # | 丢弃机制 | 说明 |
|---|---|---|
| 1 | **复杂实例语法未处理** | AP203 允许 `#1385 =( TYPE_A(...) TYPE_B(...) );` —— 一条实体带多个子类型、**无单一类型名**。按 `#id = TYPE(...)` 逐行匹配时整条**静默跳过**。丢掉的恰是 `GEOMETRIC_REPRESENTATION_CONTEXT` 等**根上下文**，于是所有几何引用悬空。 |
| 2 | **嵌套 bound 的引用遍历不全** | 引用在 `EDGE_LOOP((#a,#b))` 里是 list，在 `FACE_OUTER_BOUND(#c,.T.)` 里是 str。只处理 str 就会漏掉整条 bound 链。 |
| 3 | **非 ASCII 名称编码回退不完整** | 文件里有法语名（`pieds sous la platine`、`mearm avec arduino`），UTF-8/GBK 交替，回退逻辑遗漏分支后又静默丢弃。 |

修了 #1 之后计数对齐了（154708 条实体、0 悬空），**但几何依然是 0** —— 说明 #2/#3 仍在吞数据。
**这时我没有继续加补丁，而是换了参考实现**，这是对的。

### 2.3 教训（应当长期遵守）

> **不要用自研解析器去读复杂标准格式**（STEP / IGES / DWG / DICOM…）。
> 当自研工具给出**否定性结论**（「文件坏了」「数据缺」）时，
> 必须先用**参考实现**复核 —— 否则会把「工具的能力边界」误报成「数据的缺陷」。
>
> 用户当时的判断（「不要自己写解析器，使用完整的开源项目，感觉是解析的有问题」）
> **是正确的**，OCCT 复核结果是 111/111 vs 我的 0/111。

### 2.4 工具去向

| 工具 | 状态 |
|---|---|
| `core/tools/step_report.py` | ✅ **保留**，OCCT 版，本次及后续唯一使用 |
| `core/tools/step_occt_tree.py` / `step_joint_axes.py` | 🔶 阶段性探测脚本，已被 `step_report.py` 吸收，可删 |
| `core/tools/parse_step_assembly.py` | ❌ **已废弃**，产出过错误结论 |
| `core/tools/step_world_geometry.py` | ❌ **已废弃**，产出过错误结论 |

---

## 3. 4 个 SG90 舵机及其输出轴

SG90 的**输出轴圆柱面**就是该关节的机械旋转轴（R = 6.20 mm，每个舵机上有 3 个共轴圆柱面）：

| 编号 | bbox 尺寸 (mm) | 中心 | 轴方向 `d` | 轴上一点 `p` |
|---|---|---|---|---|
| 0 | `[12.4, 29.8, 33.7]` | `[-14.73, 13.70, 160.40]` | `[0, -1, 0]` | `[-14.731, 25.400, 155.975]` |
| 1 | `[33.7, 12.4, 29.8]` | `[  4.28, -13.17, 64.80]` | `[0, 0, 1]` | `[  8.705, -13.165, 53.100]` |
| 2 | `[33.7, 12.4, 29.8]` | `[  9.27,   0.75, 64.80]` | `[0, 0, 1]` | `[ 13.693,   0.746, 53.100]` |
| 3 | `[31.6, 29.8, 33.2]` | `[-87.09, 47.49, 229.37]` | `[0, 1, 0]` | `[-89.986, 35.788, 232.587]` |

**轴间关系**：

| 轴对 | 夹角 | 垂直距离 |
|---|---|---|
| 1 ↔ 2 | **0.00°** | **14.78 mm** |
| 0 ↔ 3 | **0.00°** | **107.39 mm** |
| 0 ↔ 1 | 90.00° | 105.51 mm |
| 0 ↔ 2 | 90.00° | 106.73 mm |
| 1 ↔ 3 | 89.98° | 110.16 mm |
| 2 ↔ 3 | 89.98° | 109.44 mm |

两条**严格平行**的轴（0 ↔ 3，间距 107.39）是**平行四连杆**的几何签名 ——
这为 `robot.yaml` 里 `elbow.coupling = {shoulder, gain: -1}` 提供了 CAD 层面的结构证据。

详细比对与判定见 **`docs/STEP_KINEMATICS_VALIDATION.md`**。

---

## 4. 附件坐标约定

| 项 | 值 |
|---|---|
| 单位 | mm |
| 坐标系 | Z-up（`export_report.json` 记为 `z_up_x_forward`） |
| 整体包围盒 | `sz = [165.1, 153.5, 225.6]` |

⚠️ 该**不是** `robot.yaml` 的关节坐标系 —— 两者原点与朝向都不同。
比对时只使用**不变量**（轴向、夹角、平行关系），不使用绝对坐标。

---

## 附：复现

```bash
PYTHONIOENCODING=utf-8 \
  "C:/Users/lx176/.workbuddy/binaries/python/envs/default/Scripts/python.exe" \
  core/tools/step_report.py \
  robot-package/mearm-v1/3d-structure/mearm3Dasm.STEP \
  --json .workbuddy/captures/step_report.json
# 预期: nodes=111 leaves=103 with_geometry=111 servos=4
```
