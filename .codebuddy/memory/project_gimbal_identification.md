---
name: 步兵云台 yaw 轴系统辨识与控制调优任务
description: 任务 1.2 的目标、硬件细节、工具链、分支与配置、已完成与待办事项
type: project
---

**目标**：自行设计方案，对步兵云台 yaw 轴做系统辨识与控制调优，并对比调优前后效果。

**设备**：狗洞全向轮步兵 C 车，配置 `deformable-infantry-omni-c.yaml`。

**分支**：`dev/gimbal-yaw-sysid`（主任务）与 `dev/gimbal-yaw-sysid-data`（数据收集，当前所在）。两者基于 `origin/merge/deformable`。用户后续要推送到 fork（Shingura/RMCS）。gh 已安装（2.45.0），用户已登录。

**硬件关键事实**：yaw 轴电机是 `LkMotor::Type::kMG4010Ei10`，减速比 10，CAN 2，零点取 `yaw_motor_zero_point`（C 车 38910）。测试车部分电机更换过，yaml 参数只作初值参考。

**工具链（复用战队现成的，不自己造）**：
- 采集：`rmcs_core::controller::identification::SweptFrequencyController`，配置 target 为 `/gimbal/yaw`，参数 `start_freq/end_freq/duration/amplitude/logarithmic/pid/setpoint/position_*/velocity_*/dc_offset`。CSV 写到部署容器 `/tmp/swept_frequency_controller_<target>_sweep_<count>.csv`，含列 `update_count, elapsed_s, pid_output, sweep_output, {target}/control_torque, {target}/torque, {target}/velocity, {target}/angle`。
- 触发方式：左开关 MIDDLE、右开关从 MIDDLE 拨到 UP 开始扫频；双下开关结束并落盘。
- 拟合：`.script/identification/fit_sweep_graybox.py <csv> --target /gimbal/yaw --input-signal torque --fixed-gravity-gain 0 --fixed-gravity-phase 0`。yaw 是竖直轴无重力矩，必须固定重力项走三参数路径。
- 取回：`scp "remote:/tmp/<文件名>.csv" ./`（`remote` 是 ssh 主机别名）。

**已踩的坑**：框架只允许一个组件注册同一路输出（`output_map.emplace` 会抛 Duplicate output 并 FATAL）。云台控制器与扫频组件都要写 `/gimbal/yaw/control_torque`，辨识时必须注释掉 `gimbal_controller`，并另加一个只做位置保持的 SweptFrequencyController 接管 pitch，否则 pitch 会自由下垂。

**已完成**：两个分支已建；新增辨识配置 `deformable-infantry-omni-c-sysid.yaml`；文档 `docs/zh-cn/gimbal-yaw-system-identification.md` 已写好，含线下操作流程与结果待填区。

**待办**：commit 并 push 到 fork 以便笔记本同步；线下采集数据；拟合；调优与对比；填文档第六七节。

**注意**：`.codebuddy` 目录在 merge/deformable 系分支上原本不存在，已用 `git checkout develop -- .codebuddy` 恢复。这两个分支上需要重新提交 `.codebuddy` 才能跨设备同步记忆。

**用户背景**：熟悉 RMCS 的 topic、yaml 参数与 DR16 使用，独立完成过 gm6020 与龙门架任务。对实际兵种调试不熟悉。
