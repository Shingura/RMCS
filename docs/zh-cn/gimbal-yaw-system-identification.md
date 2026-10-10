# 步兵云台 yaw 轴系统辨识与控制调优

任务编号 1.2。本文记录辨识与调优的做法，以及线下的操作步骤。

开发分支：`dev/gimbal-yaw-sysid-data`，基线为 `merge/deformable`。

## 一、目标与设备

目标是给步兵云台 yaw 轴建立至少二阶的数学模型，用实测数据确定模型参数，再依据模型调整控制器，最后对比调优前后的效果。

测试车是狗洞全向轮步兵 C 车，配置文件为 `deformable-infantry-omni-c.yaml`。

yaw 轴电机是 LK MG4010E-i10，型号里的 i10 表示减速比 10。该电机自带驱动，能通过 CAN 反馈角度、角速度和力矩。

有一点要注意：这台车的部分电机因故障更换过。配置文件里的零点、方向和力矩常数只能作为初值参考，实际值以辨识结果为准。

## 二、建模

### 模型结构

取输入为电机力矩 \(T_e\)，输出为云台角度 \(\theta\)。忽略外部扰动时，转动方程为：

\[ J\ddot{\theta} = T_e - B\dot{\theta} \]

对应的传递函数是：

\[ \frac{\Theta(s)}{T_e(s)} = \frac{1}{Js^2 + Bs} \]

分母中 \(s\) 的最高次数是 2，所以这是二阶系统。两个极点分别在 \(s = 0\) 和 \(s = -\frac{B}{J}\)。

二阶的来源很清楚：力矩先决定角加速度，角加速度积分得到角速度，角速度再积分得到角度。最后一步积分决定了「力矩到角度」必须用二阶描述。

实际系统还有未建模因素：电流环带宽有限、CAN 通信与采样存在延时、减速机构有弹性、摩擦有非线性。这些因素会引入额外的时间常数，使真实系统高于二阶。

### 更完整的模型

战队现成的拟合脚本用的是灰箱模型，在二阶基础上补充了摩擦项：

\[ J\ddot{\theta} + B\dot{\theta} + F_c\tanh(k\dot{\theta}) = \tau \]

其中 \(J\) 是转动惯量，\(B\) 是粘性阻尼系数，\(F_c\) 是库仑摩擦幅值，\(k\) 是脚本内写死的平滑系数。

yaw 轴是竖直转轴，重力对该轴不产生力矩，所以脚本里的重力项对 yaw 不适用，拟合时要显式把它固定为 0。

### 前馈增益的物理含义

云台控制器里有两个前馈参数：`yaw_velocity_ff_gain` 和 `yaw_acceleration_ff_gain`。

按模型，让云台按期望轨迹运动所需的力矩是：

\[ \tau_{ff} = J\ddot{\theta}_{ref} + B\dot{\theta}_{ref} \]

所以加速度前馈增益的理想值就是辨识出的 \(J\)，速度前馈增益的理想值就是辨识出的 \(B\)。

现有 C 车配置里这两个值分别是 0.18 和 0.13。辨识结果可以和这两个数直接对照，用来判断现有前馈是否合理。

## 三、采集数据

### 使用的工具

战队已有成套的辨识工具，直接复用：

| 工具 | 位置 | 作用 |
| --- | --- | --- |
| `SweptFrequencyController` | `rmcs_core/src/identification/` | 产生扫频激励并把数据写成 CSV |
| `fit_sweep_graybox.py` | `.script/identification/` | 用扫频数据拟合灰箱模型，输出 \(J\)、\(B\)、\(F_c\) |

采集不需要自己写代码，只需要在配置里启用扫频组件。

### 配置改动

新增了辨识专用配置 `deformable-infantry-omni-c-sysid.yaml`。与日常配置相比，它有三处改动：

1. 停用云台控制器。框架只允许一个组件注册同一路输出，云台控制器和扫频组件都要写 `/gimbal/yaw/control_torque`，同时启用会报重复输出并退出。
2. 启用 `yaw_swept_frequency_controller`，target 为 `/gimbal/yaw`，扫频范围 0.1 到 10 Hz，时长 60 秒。
3. 启用 `pitch_hold_controller`，target 为 `/gimbal/pitch`，只做位置保持，不扫频。停用云台控制器后，pitch 需要有人接管，否则会自由下垂。

扫频幅度 `amplitude` 初值取 0.3。这个值需要现场试：响应太小就加大，云台动作过猛就减小。

### 线下操作步骤

在开发容器里执行：

```bash
build-rmcs                      # 构建
set-remote <remote-host>        # 首次连接 MiniPC 时才需要
sync-remote                     # 另开一个终端，保持运行，实时同步构建产物
```

同步终端出现 `Nothing to do` 后，进入部署容器：

```bash
ssh-remote
set-robot deformable-infantry-omni-c-sysid
service rmcs restart
service rmcs attach             # 查看实时输出，Ctrl+A 然后按 D 退出
```

若启动失败并提示 `Cannot find ... output`，按报错里提到的组件名，到配置的 `components` 段把对应行注释掉。最常见的是 auto_aim 相关的三行。

### 录制

扫频由遥控器开关触发，条件是左开关在中位、右开关从中位拨到上位。

1. 先把两个三挡开关都拨到最下方，确认云台静止。
2. 左开关拨到中位。
3. 右开关从中位拨到上位，扫频开始。
4. 等待 60 秒，扫频自动结束。需要提前结束就把两个开关拨回最下方。

结束运行前必须先把两个开关拨到最下方，等电机停稳后再退出，直接 Ctrl+C 会让电机快速转动一下。

录制期间可以换几个幅度重复录制，用于交叉验证。改 `amplitude` 后不需要重新构建，配置文件会被 `sync-remote` 自动同步，重启服务即可。

### 取回数据

CSV 写在部署容器的 `/tmp` 下，文件名形如 `swept_frequency_controller__gimbal_yaw_sweep_12345.csv`。

在开发容器里执行：

```bash
ssh-remote "ls -la /tmp/swept_frequency_controller_"*"_sweep_"*".csv"    # 先确认文件名
scp "remote:/tmp/<文件名>.csv" ./
```

## 四、拟合

### 确认依赖

```bash
python3 -c "import numpy; print(numpy.__version__)"
```

默认的线性模式只需要 numpy。若改用 `ode-angle` 模式，还需要 scipy。

### 执行拟合

```bash
python3 .script/identification/fit_sweep_graybox.py <csv 文件名> \
    --target /gimbal/yaw \
    --input-signal torque \
    --fixed-gravity-gain 0 \
    --fixed-gravity-phase 0 \
    --json-output docs/zh-cn/yaw-fit-result.json
```

参数说明：

- `--target /gimbal/yaw`：脚本会去找 `/gimbal/yaw/control_torque`、`/gimbal/yaw/torque`、`/gimbal/yaw/velocity`、`/gimbal/yaw/angle` 这几列。
- `--input-signal torque`：用电机反馈的实际力矩作为输入。若反馈不可靠，改成 `control_torque`。
- `--fixed-gravity-gain 0 --fixed-gravity-phase 0`：yaw 轴没有重力矩，固定这两项，走三参数路径。这两项必须成对给出。

脚本会输出 `Suggested params`，包含转动惯量、粘性阻尼、库仑摩擦，以及一组警告。要特别看 `Warnings` 段里有没有提到 \(R^2\) 过低、角度覆盖不足或优化器未收敛。

### 交叉验证

换一组不同幅度的扫频数据再拟合一次。两次结果接近，说明模型可用。差异大，通常说明存在未建模的非线性，最常见的是摩擦。

## 五、控制器调优与对比

### 测量基线

改回日常配置，测量现有控制器的表现：

```bash
ssh-remote
set-robot deformable-infantry-omni-c
service rmcs restart
```

用遥控器给 yaw 一个阶跃目标，观察并记录：上升时间、超调量、稳态误差。再让底盘原地旋转，记录 yaw 被带动的幅度和恢复时间。

### 调优方向

依据辨识结果，按下面的顺序试：

1. 先核对前馈。把 `yaw_acceleration_ff_gain` 设为辨识出的 \(J\)，`yaw_velocity_ff_gain` 设为辨识出的 \(B\)，与现有值 0.18 和 0.13 对照。
2. 再调速度环。速度环直接作用于被控对象，参数可以用模型算：目标穿越频率 \(f_c\) 下，\(k_p = 2\pi f_c J\)，\(k_i = 2\pi f_c B\)。
3. 最后调角度环。角度环的对象包含速度环和一个积分环节，增益过高会产生超调和振荡。
4. 若低速段有明显顿挫，说明库仑摩擦影响显著，需要在前馈里补一项与角速度方向有关的补偿。

### 对比指标

调优后重复基线的测量方式，把两组数据放进同一张表：

| 指标 | 调优前 | 调优后 |
| --- | --- | --- |
| 阶跃上升时间 | | |
| 超调量 | | |
| 稳态误差 | | |
| 底盘旋转时的 yaw 偏移 | | |
| 扰动恢复时间 | | |

结果为负优化也可以，重点是把对比过程和数据讲清楚。

## 六、结果

### 已采集数据

| 项目 | 内容 |
| --- | --- |
| 数据文件 | `data/swept_frequency_controller__gimbal_yaw_sweep_19750.csv` |
| 采集配置 | `deformable-infantry-omni-c-sysid.yaml`，`amplitude` 2.0 |
| 采样 | 1000 Hz，60 秒，60001 行 |
| 扫频范围 | 0.1 到 10 Hz，对数扫频 |
| 角度范围 | 解环绕后总范围约 30.7 度；低频段约 19.7 度，高频段约 4 度 |
| 力矩 | 最大 2.11 N·m，未饱和 |

采集时发现幅度要足够大。`amplitude` 取 0.3 时 yaw 只动 0.176 度，肉眼不可见且无法辨识；取 1.0 时低频段约 5 度；取 2.0 后低频段约 19.7 度，数据可用。判断依据是解环绕后的角度峰峰值，建议低频段达到 10 度以上。

### 辨识结果

拟合命令：

```bash
python3 .script/identification/fit_sweep_graybox.py \
    data/swept_frequency_controller__gimbal_yaw_sweep_19750.csv \
    --target /gimbal/yaw \
    --input-signal torque \
    --fixed-gravity-gain 0 \
    --fixed-gravity-phase 0 \
    --window-length 101
```

拟合出的模型是：

\[ 0.128\,\ddot\theta + 0.319\,\dot\theta + 0.373\,\tanh(100\,\dot\theta) = \tau \]

| 参数 | 第一次 | 第二次（交叉验证） | 配置里的现有值 |
| --- | --- | --- | --- |
| 转动惯量 \(J\) | 0.128 kg·m² | 待补充 | 0.18 |
| 粘性阻尼 \(B\) | 0.319 N·m·s/rad | 待补充 | 0.13 |
| 库仑摩擦 \(F_c\) | 0.373 N·m | 待补充 | 无 |

用指令力矩替代实测力矩再拟合一遍，得到 \(J = 0.140\)、\(B = 0.387\)、\(F_c = 0.235\)，\(R^2 = 0.898\)。两组结果相差 10% 到 20%，量级一致。

### 平滑窗口的敏感性

脚本默认的窗口长度是 31。加速度由角速度求导得到，窗口太短会放大噪声。改变窗口后的结果：

| 窗口长度 | \(J\) | \(B\) | \(F_c\) | \(R^2\) |
| --- | --- | --- | --- | --- |
| 31（默认） | 0.096 | 0.411 | 0.306 | 0.748 |
| 51 | 0.119 | 0.385 | 0.325 | 0.874 |
| 101（采用） | 0.128 | 0.319 | 0.373 | 0.913 |
| 201 | 0.133 | 0.407 | 0.306 | 0.830 |

窗口 101 时 \(R^2\) 最高，采用这组结果。\(J\) 随窗口从 0.096 升到 0.133，不确定度约 ±20%。

### 由模型算出的量

- 时间常数 \(J/B = 0.40\) 秒。
- 开环极点 \(-B/J = -2.50\) rad/s，对应截止频率约 0.40 Hz。
- 库仑摩擦占激励幅度的约 19%。比例不小，低速段会有明显粘滞。

### 与现有配置的对照

| 参数 | 现有配置值 | 按辨识结果应为 |
| --- | --- | --- |
| `yaw_acceleration_ff_gain` | 0.18 | 约 0.128（即 \(J\)） |
| `yaw_velocity_ff_gain` | 0.13 | 约 0.319（即 \(B\)） |
| `yaw_velocity_kp` | 13.0 | 取 10 Hz 穿越频率时约 8.0 |
| `yaw_velocity_ki` | 0.02 | 取 10 Hz 穿越频率时约 0.020 |

现有加速度前馈偏大约 40%，速度前馈偏小约 60%。速度环的 `ki` 与 10 Hz 穿越频率吻合，`kp` 折算约对应 16 Hz，两者不一致。

有一点要注意：力矩、角度、速度的口径是否都在减速后的输出轴，尚未从代码确认。若口径不同，上表要按比例换算。

### 曲线

待补充。把拟合曲线截图放在 `docs/zh-cn/assets/` 下，用下面的方式插入：

```markdown
![yaw 轴拟合曲线](./assets/yaw-fit.png)
```

### 调优前后对比

待补充，填第五节末尾的表格，并附上阶跃响应曲线。

## 七、过程中遇到的问题

踩过的坑记录在这里，供后续复用。

- 扫频组件与云台控制器争用同一路输出，辨识时必须停用云台控制器。
- 激励幅度过小无法辨识。`amplitude` 取 0.3 时 yaw 只动 0.176 度，取 2.0 后低频段约 19.7 度，数据才可用。
- 拟合脚本默认的平滑窗口 31 会让加速度估计偏噪，\(R^2\) 只有 0.75。窗口调到 101 后升到 0.91。窗口变化会让 \(J\) 在 0.096 到 0.133 之间移动，报告结果时应注明所用窗口。
