---
name: 步兵云台 yaw 轴系统辨识与控制调优任务
description: 任务 1.2 的当前阶段、剩余阶段与关键思路；所有实测数据与拟合结果都写在 docs/zh-cn/gimbal-yaw-system-identification.md，记忆不重复记录数值
type: project
---

**目标**：自行设计方案，对步兵云台 yaw 轴做系统辨识与控制调优，并对比调优前后效果。

**设备**：狗洞全向轮步兵 C 车。日常配置 `deformable-infantry-omni-c.yaml`，辨识配置 `deformable-infantry-omni-c-sysid.yaml`。

**分支**：当前在 `dev/gimbal-yaw-sysid-baseline`。主干是 `dev/gimbal-yaw-sysid-data`。远端是 fork（Shingura/RMCS）。推送必须用 SSH：`git push git@github.com:Shingura/RMCS.git <分支>`。

**对应文档**：/docs/zh-cn/gimbal-yaw-system-identification.md，任务详情可自行参阅。

## 当前进度与剩余任务

**模型辨识已完成，通过交叉验证**，模型参数取两组数据的平均值。

**测试手段**：在云台控制器内部自建阶跃发生器（摇杆推满只能产生斜坡，且推动时间污染上升时间）。触发沿用扫频的开关顺序，左中右上开始，序列 48 s 后自停。当前四档两遍，幅值 0.1/0.3/0.6/1.0 rad。

**剩余任务**：从 baseline 尖端重建 `-pid` 与 `-adrc` 分支分别调优；写分析脚本算五项指标并填进对比表；调优后重复同一套序列测量并对比。调优第一步是加输出限幅（`yaw_velocity_output_min`/`yaw_velocity_output_max`，建议先取 ±10 N·m），该参数无需改代码。

## 关键信息

- 这台车有两个 pitch：云台 pitch 控制枪管俯仰，底盘 pitch（`active_suspension_pitch_*`）属于底盘主动悬挂，控制车身起立。
- 分支切出点是 `dev/gimbal-yaw-sysid-data`（含配置、数据与文档）。`dev/gimbal-yaw-sysid-baseline` 由此切出，调参分支再从 baseline 尖端切出，保证三者共享阶跃测试代码。
- 调优针对日常配置 `deformable-infantry-omni-c.yaml`，该文件已被改造成测试配置（底盘、射击、自瞄已停用，pitch 零点已改为 7093）。
- 采集从程序启动就开始，有效数据只有目标角度非空的那一段，时间轴见文档。
- 前馈增益 `yaw_velocity_ff_gain`、`yaw_acceleration_ff_gain` 只在自瞄接管时生效，手动与阶跃测试下不参与运算。
- 控制器速度环用的是 IMU 转速（世界系），电机转速是相对底盘的。分析跟随误差以 IMU 那一路为准，两者之差反映车身晃动。

**用户背景**：熟悉 RMCS 的 topic、yaml 与 DR16，独立完成过 gm6020 与龙门架任务，对实际兵种调试不熟悉。
