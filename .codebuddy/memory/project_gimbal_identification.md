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

术语歧义（曾导致误解）：这台车上有两个 pitch。云台 pitch（`/gimbal/pitch`）控制发射枪管的俯仰；底盘 pitch（`active_suspension_pitch_*` 等参数）属于底盘主动悬挂，控制车身起立姿态。用户说「pitch 参数不适用」时指的是底盘那个，起因是悬挂电机维修过，与云台无关。讨论 pitch 时先分清是云台还是底盘。

云台 pitch 保持（SweptFrequencyController，sweep=false）的调参经验（实测）：
- `position_kp` 过大会持续抖振并发出响声，35.0 明显抖，降到 5.0 后安静停下
- `position_ki` 非零会持续累积，把 pitch 推过目标并顶到机械限位（表现为抬到最顶后一直响）
- `setpoint` 决定停留的机械位置：0.0 对应底部，需设成抬起后的角度，这台车用 0.3
- 该组件没有角度软限位，只有 PID 输出限幅与积分限幅（配置项 `position_output_max` 等）

**辨识配置当前状态**：`deformable-infantry-omni-c-sysid.yaml` 启用 6 个组件——硬件、yaw 扫频、pitch 保持、referee_status、DeformableChassis、ChassisPowerController。底盘的起立/轮/关节/悬挂、射击系统、自瞄、UI 全部停用，车保持趴着。后三个组件是依赖链要求（硬件命令组件需要 `/referee/chassis/output_status` 与 `/chassis/supercap/charge_power_limit`），它们不下发力矩。

**新增的组件能力**：给 SweptFrequencyController 加了 `setpoint_from_current` 参数，首次使能时取当前角度为 setpoint，实现手掰到哪里停在哪里。两个组件都启用。已构建通过。

**PID 调参结论（实测）**：
- pitch：position_kp 5.0、position_ki 0、velocity_kp 2.0、dc_offset 3.5（重力前馈，2.0 不够）
- yaw：position_kp 5.0、position_ki 0、velocity_kp 1.0
- 噪音来自 `velocity_kp`（放大速度反馈噪声），与 `position_kp` 无关；velocity_kp 降到 1.0 后消失
- 等效刚度 = position_kp × velocity_kp，决定回中速度与稳态偏差；要提高刚度就提 position_kp，不动 velocity_kp

**已完成**：两个分支已建；辨识配置与文档已写好；组件精简与依赖链已理清；手掰保持已实现；pitch 与 yaw 的 PID 已调到安静可用；数据采集与拟合已完成，结果已写入文档第六节。

**已采集数据**：`data/swept_frequency_controller__gimbal_yaw_sweep_19750.csv`，amplitude 2.0，1000 Hz，60 秒，60001 行，0.1 到 10 Hz 对数扫频。解环绕后角度总范围 30.7 度（低频段 19.7 度、高频段 4 度），力矩最大 2.11 N·m 未饱和。

**第一次辨识结果（2026-10-10）**：用 `fit_sweep_graybox.py --window-length 101` 拟合，\(J = 0.128\) kg·m²、\(B = 0.319\) N·m·s/rad、\(F_c = 0.373\) N·m，\(R^2 = 0.913\)。换用指令力矩再拟合得 \(J = 0.140\)、\(B = 0.387\)、\(F_c = 0.235\)，量级一致。派生量：时间常数 0.40 秒，截止频率约 0.40 Hz。默认窗口 31 时 \(R^2\) 只有 0.75，窗口 101 才到 0.91，报告结果必须注明窗口。

**下一步建议**：
1. 交叉验证：把 `amplitude` 改成 1.0 再录一组，确认 \(J\) 与 \(B\) 稳定。目前只有一组数据。
2. 先只改前馈上机试：`yaw_acceleration_ff_gain` 设 0.13、`yaw_velocity_ff_gain` 设 0.32（现有 0.18 和 0.13），观察阶跃响应。
3. 速度环：按 10 Hz 穿越频率应为 kp≈8.0、ki≈0.020；现有 `yaw_velocity_kp` 13.0 对应约 16 Hz，与 ki 0.02 不一致，可一并试。
4. 测基线与调优后的对比数据，填文档第六节的对比表与曲线，补 `docs/zh-cn/assets/` 下的截图。
5. 待确认：力矩、角度、速度的口径是否都在减速后的输出轴。口径不同则数值需按比例换算。

**采集经验**：幅度要够大。amplitude 0.3 时 yaw 只动 0.176 度，肉眼看不见且无法辨识；1.0 时低频段 5 度；2.0 时低频段 19.7 度、总范围 30.7 度，数据可用。判断依据是解环绕后的角度峰峰值，建议低频段达到 10 度以上。

**注意**：`.codebuddy` 目录在 merge/deformable 系分支上原本不存在，已用 `git checkout develop -- .codebuddy` 恢复。这两个分支上需要重新提交 `.codebuddy` 才能跨设备同步记忆。

**用户背景**：熟悉 RMCS 的 topic、yaml 参数与 DR16 使用，独立完成过 gm6020 与龙门架任务。对实际兵种调试不熟悉。
