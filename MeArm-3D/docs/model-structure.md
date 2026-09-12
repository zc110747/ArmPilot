# 模型结构（Model Structure）

> 权威来源：`config/robot.yaml`（唯一数据源）+ `frontend/src/robot/model/Link.ts`（几何类型定义）
> 本文说明「显示的机械臂」是怎么被参数化拼出来的，以及**显示层与运动学层的边界在哪里**。

---

## 1. 一句话原则

> **`link.length` / `joint.axis` / `joint.limit` 决定机器臂怎么动；`link.geometry` / `link.details`
> 只决定它长什么样。两者严格解耦，改外观不会动到 FK / IK / 标定一个数字。**

由 `frontend/tests/unit/linkGeometry.test.ts` 断言：把整份 `geometry` / `details` 全部抹掉换成
`{ type: 'none' }`，`endEffectorPosition()` 的返回值**逐位不变**。

---

## 2. 对象树（Scene Graph）

```
RobotGroup                       root（= 根连杆的近端坐标系）
└── JointGroup:base              ← 一个关节 = 一个 THREE.Group，只改它的 quaternion
    ├── LinkObject:column_link   ← 该关节的子连杆（geometry + details 全部挂这里）
    │   ├── plate / box / cylinder / sphere / servo …
    │   └── JointGroup:shoulder
    │       ├── LinkObject:upper_arm_link
    │       │   └── JointGroup:elbow
    │       │       ├── LinkObject:forearm_link
    │       │       │   └── JointGroup:tool（固定关节）
    │       │       │       ├── LinkObject:tool_link
    │       │       │       │   ├── JointGroup:gripper
    │       │       │       │   └── gripperPalm（两片爪，绕 X 铰轴对称开合）
    │       │       │       └── tcpMarker
    │       │       └── …（jointAxisHelper / jointOriginHelper）
```

关节运动**只体现为 `JointGroup.quaternion` 的变化**，任何时候都不去改 Mesh 的 `position`。
详见 `docs/decisions.md` D10 与 `frontend/src/components/RobotScene/buildRobotObject3D.ts` 头注释。

---

## 3. 几何类型（`LINK_GEOMETRY`）

全部在 `config/robot.yaml` 里声明，渲染层只做「参数 → 网格」的翻译。

| `type` | 参数 | 渲染为 | 典型用途 |
|--------|------|--------|----------|
| `none` | — | 不出几何 | 叶连杆（如 `jaw_link` 的本体） |
| `box` | `size` | `BoxGeometry`（直角） | 无倒角的连接件 |
| `plate` | `size`, `cornerRadius?` | `RoundedBoxGeometry` | **低多边形工程外观主力件**：底盘 / 立柱侧板 / 臂板 / 爪片 |
| `cylinder` | `radius`, `height`, `radialSegments?` | `CylinderGeometry`（默认轴 +Y） | 轴销 / 端盖 / 螺栓 |
| `sphere` | `radius` | `SphereGeometry` | 球头（备用） |
| `servo` | `size?`, `shaftLength?`, `ears?`, `shaftColor?` | 壳体 + 安装耳 + 金属输出轴 + 圆舵盘 | 4 个舵机 |

所有类型都支持可选的 `position` / `rotation` / `color`。

- `position` 省略时，**连杆本体**默认落在「连杆中段」`(0, 0, length/2)`；
  **`details` 附加件**默认落在本坐标系原点 `(0, 0, 0)`（附加件最合理的缺省）。
- `rotation` 是 intrinsic XYZ 欧拉角（degree），与项目统一约定一致。

### `plate` 的倒角

`cornerRadius` 省略时取 `DEFAULT_PLATE_CORNER_RADIUS`（1.6 mm）；
渲染时按 `min(size) / 2 - 0.01` **自动钳位**，避免半径大于半边长导致几何自交。
半径 < 0.05 mm 时退化为 `BoxGeometry`。

### `servo` 的本体约定（重要）

> **局部 `+Z` = 输出轴方向，原点 = 壳体中心。**

因此：

| 想要输出轴朝向 | 写法 |
|----------------|------|
| 世界/坐标系 `+Z`（向上） | 不写 `rotation` |
| `+Y` | `rotation: [-90, 0, 0]` |
| `+X` | `rotation: [0, 90, 0]` |

尺寸缺省 `DEFAULT_SERVO_SIZE = [22.8, 12.2, 22.5]`（SG90 / MG90S 量级，**仅默认值**）。

舵盘刻意做成**圆盘**而不是带臂舵盘：带臂件在关节旋转时会产生方向错觉
（舵机挂在父连杆上，而舵臂应随子连杆转）。结构正确性优先于装饰（见 §6）。

---

## 4. 机构布置（对齐开源 meArm 实物）

| 连杆 | 显示件 | 说明 |
|------|--------|------|
| `base_link` | 底盘倒角板 + 四角螺栓 + **S9 底座舵机**（平躺居中，轴 `+Z`） | 链根，不随任何关节运动 |
| `column_link` | 转盘 + 两片立柱侧板（夹住 S9 舵机体）+ **S8 肩舵机**（顶部，轴 `+Y`） | 随 `base` 绕 Z 旋转 |
| `upper_arm_link` | 两片大臂侧板 + 肩/肘轴端盖 + 端部横撑 + **S7 肘舵机**（末端，轴 `+Y`） | 随 `shoulder` 摆动 |
| `forearm_link` | 两片小臂侧板 + 肘轴端盖 + 腕部横撑 | 随 `elbow` 摆动 |
| `tool_link` | 腕座 + **S6 夹取舵机**（轴 `+X`，与夹爪铰轴同向）+ 爪铰轴 | 固定连杆（无自由度） |
| `jaw_link` | `geometry.size = [宽, 厚, 长]` 仅提供**爪片尺寸** | 叶关节，不参与末端定位 |

**舵机挂载原则**：驱动某个子关节的舵机，画在**它的父连杆**上。例如 S7 驱动 `elbow`，
就画在 `upper_arm_link`（= `elbow` 的 `parentLink`）上，因此在 `shoulder` 转动时跟着走、
在 `elbow` 转动时保持不动——与真实机构一致。

**夹爪特例**：`jaw_link` 不产生独立几何，由渲染层把它特化成「两片爪绕 X 铰轴对称开合」，
每片各转 `±θ/2`。符号规则：左爪（`y < 0`）取 `+θ/2`、右爪（`y > 0`）取 `−θ/2`，
使两爪**向外张开**；写反会导致两爪越过中线互穿。
`frontend/tests/unit/linkGeometry.test.ts` 用「相对掌心中线的坐标」断言这一点（与整机姿态无关）。

---

## 5. 调试可视化（spec §20）

| 元素 | 挂载点 | 开关（store） | 默认 |
|------|--------|---------------|------|
| 关节轴（实体细杆 + 锥头） | 各 `JointGroup` | `showJointAxes` | 开 |
| 关节原点小球 | 各 `JointGroup` | `showJointOrigins` | 关 |
| TCP 球 + 三色十字 | `tcp.joint` 的 `JointGroup` | `showTcp` | 开 |
| 世界坐标轴 | 场景根 | `showWorldAxes` | 开 |
| 机器人坐标轴 | 机器人根 | `showRobotAxes` | 关 |

> 关节轴用 `CylinderGeometry` 而非 `THREE.Line`：WebGL 下 `LineBasicMaterial.linewidth`
> 恒为 1px，几乎看不见。

---

## 6. 改外观的标准流程

1. 只改 `config/robot.yaml` 里目标连杆的 `geometry` / `details`；
2. `cd frontend && npx tsc -b && npx vitest run`（43 项应全绿，其中 13 项是几何回归）；
3. 起开发服务器，`node tests/e2e/screenshot.mjs http://127.0.0.1:5273/ 9345 out.png 1600 1100 "j1,j2,j3,grip"`
   落一张图做视觉确认（第 7 个参数可选，用于摆姿势）；
4. `node tests/e2e/ui-smoke.mjs` 确认 FK↔3D 仍 < 0.1 mm。

**禁止**：为了让外观更好看去改 `link.length`（那会改变机构尺寸与 TCP）。

---

## 7. 与后续阶段的边界

- **Phase 5 IK** 只读 `length` / `axis` / `limit`，与本文无关。
- 若将来换成 GLB/GLTF 模型：替换 `createGeometryObject()` 的实现即可，
  `RobotModel` 与运动学层不用动（`geometry` 字段本就声明为"仅显示"）。
