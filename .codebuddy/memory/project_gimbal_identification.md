---
name: 步兵云台 yaw 轴系统辨识与控制调优任务
description: 任务 1.2 的当前阶段、剩余阶段与关键思路；所有实测数据与拟合结果都写在 docs/zh-cn/gimbal-yaw-system-identification.md，记忆不重复记录数值
type: project
---

**目标**：自行设计方案，对步兵云台 yaw 轴做系统辨识与控制调优，并对比调优前后效果。

**设备**：狗洞全向轮步兵 C 车。日常配置 `deformable-infantry-omni-c.yaml`，辨识配置 `deformable-infantry-omni-c-sysid.yaml`。

**分支**：当前在 `dev/gimbal-yaw-sysid-data`（数据采集），主分支 `dev/gimbal-yaw-sysid`。远端是 fork（Shingura/RMCS）。容器内没有 gh，HTTPS 无凭证，推送必须用 SSH：`git push git@github.com:Shingura/RMCS.git <分支>`。

**对应文档**：/docs/zh-cn/gimbal-yaw-system-identification.md，任务详情可自行参阅。

## 当前进度与剩余任务

**模型辨识已完成，通过交叉验证**，模型参数取两组数据的平均值，下一步进入控制器调优与前后对比。

具体进度可参阅文档。

已得到的模型可阅读文档 `五、拟合结果`，接下来的任务详情可阅读 `六、控制器调优与对比`。

## 一些操作要点

- 这台车有两个 pitch：云台 pitch 控制枪管俯仰，底盘 pitch（`active_suspension_pitch_*`）属于底盘主动悬挂，控制车身起立。

**用户背景**：熟悉 RMCS 的 topic、yaml 与 DR16，独立完成过 gm6020 与龙门架任务，对实际兵种调试不熟悉。
