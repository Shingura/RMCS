# RMCS 战队 C++ 代码规范

本规范从 `merge/deformable` 分支的官方硬件与控制器源码中总结而来。写新代码时遵守，改旧代码时对齐。

## 命名

- 文件名与类名严格对应。控制器按子系统分目录：`controller/gimbal`、`controller/chassis`、`controller/shooting`。
- 类名用大驼峰。缩写只首字母大写：`Bmi088`、`Dr16`、`Gm6020`、`LkMotor`。
- 常量用 `kCamelCase`。成员变量用 `lower_case_`。私有成员函数也带尾下划线。
- 命名空间至少三级：`rmcs_core::hardware`、`rmcs_core::controller::gimbal`。结尾写 `} // namespace ...`。

## 组件结构

- 继承列表每个基类一行。显式写 `~X() override = default;`。
- 生命周期顺序：`before_updating()` 做硬件同步，`update()` 做状态解算，`command_update()` 做下发。
- 板级回调拆成独立结构体，主类用 `std::unique_ptr` 组合。
- 输入输出聚合成 `struct Input` 与 `struct Output`。伙伴组件命名为 `Command`，成员为 `command_`。
- 函数用尾置返回类型：`auto update() -> void override`。

## 参数与信号

- 必填参数用 `get_parameter`。可选参数用 `get_parameter_or`，并在成员声明处给默认值。
- PID 用 `configure_pid(prefix, calc)`，在成员初始化列表中构造。
- 开关类参数用 `bool`，不用字符串枚举。
- 非必需输入写 `register_input(name, iface, false)`，用前检查 `ready()`。
- 信号名按子系统分层：`/gimbal/...`、`/chassis/...`、`/remote/...`。
- 无效输出统一写 `kNaN`，与零指令区分。

## 语言与格式

- 遵守仓库根的 `.clang-format`：LLVM 基，列宽 100，缩进 4，访问修饰符偏移 -4，逗号前置换行。
- 注释用中文，只写「为什么」，不复述代码。
- 日志用英文，带数值与单位。主循环里不放周期性 INFO。
- 角度统一用弧度，常量写成 `deg * std::numbers::pi / 180.0`。
- 用 `const auto` 与 Eigen 表达向量位姿。周期从 `update_rate` 推导，不硬编码。
- `plugins.xml` 按 hardware 与 controller 分组登记。

## 我们旧代码待修正的项

1. 跑一遍 `clang-format`。
2. 拆出板级回调，补齐生命周期函数与虚析构。
3. 全部参数改为带默认值的形式。
4. 无效值改用 `kNaN`，信号名补上子系统前缀。
5. 去掉周期性 INFO，统一命名与注释语言。
