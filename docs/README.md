# 平台级文档

本目录存放 **ArmPilot 平台级**（跨子项目）的文档。子项目内部文档见各自的 `README.md` 与 `docs/`。

---

## 开发提示词记录（Prompter）

项目按「**先出实现计划并确认 → 实现 → 验收 → 同步文档**」的节奏推进，每一阶段的开发任务书全文留档，
用于复现过程与追溯「当时为什么这么定」：

| 文档 | 阶段 | 覆盖内容 |
|---|---|---|
| [`MeArm_3D_Prompter_01.md`](MeArm_3D_Prompter_01.md) | 第一阶段 | 建立虚拟 mARM 并实现与真实 mARM 的一一对应控制；明确**不做** AI / 机器学习 / 视觉识别 / 数据集 / 训练 |
| [`MeArm_3D_Prompter_02.md`](MeArm_3D_Prompter_02.md) | 3D 建模轨 | 机械臂 3D 模型建立 · 各部件参数化 · 关节层级关系 · 关节旋转控制 |
| [`MeArm_3D_Prompter_03.md`](MeArm_3D_Prompter_03.md) | MuJoCo 轨 | 在 `MeArm-3D` 上接入 **MuJoCo 真实物理仿真**（MJCF 生成 / 三层时间步 / 与既有 Web·Go 链路零改动对接） |

> 这三个文件是**任务书**（输入），不是成果文档（输出）。
> 成果文档请看：[`../MeArm-3D/README.md`](../MeArm-3D/README.md)（数字孪生全貌与阶段验收）、
> [`../MeArm-Device/README.md`](../MeArm-Device/README.md)（固件）、
> [`../MeArm-RemoteControl/README.md`](../MeArm-RemoteControl/README.md)（串口网关）。

---

## 文档落点约定

| 内容 | 位置 |
|---|---|
| 平台总览 / 三项目关系 / 技术栈 / 完成情况 | [`../README.md`](../README.md) |
| 平台级铁律与 skill 路由 | [`../.workbuddy/memory/MEMORY.md`](../.workbuddy/memory/MEMORY.md) |
| 设计决策 ADR | [`../MeArm-3D/docs/decisions.md`](../MeArm-3D/docs/decisions.md)（**最新在最上**） |
| 坐标系 / 单位 / 运动学链 | [`../MeArm-3D/docs/coordinate-system.md`](../MeArm-3D/docs/coordinate-system.md) |
| 模型结构（几何 vs 运动学边界） | [`../MeArm-3D/docs/model-structure.md`](../MeArm-3D/docs/model-structure.md) |
| 真机实测记录与不确定度 | [`../MeArm-3D/docs/hardware-measurement.md`](../MeArm-3D/docs/hardware-measurement.md) |
| 串口 / WebSocket 协议基线 | [`../MeArm-3D/protocol/serial-v1.md`](../MeArm-3D/protocol/serial-v1.md) |
| 工作日志 | 各子项目 `.workbuddy/memory/YYYY-MM-DD.md` |
