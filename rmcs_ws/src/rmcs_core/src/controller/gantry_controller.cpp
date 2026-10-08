// 龙门架发射架控制器：把 DR16 摇杆指令换算成三个电机的目标转速
//
// 输入：
//   - /remote/joystick/left  左摇杆，取 y 分量控制 pitch
//   - /remote/joystick/right 右摇杆，取 x 分量控制 yaw
//   - /remote/switch/left    遥控器左开关状态
//   - /remote/switch/right   遥控器右开关状态
//   - /pitch/left/angle      左侧 pitch 电机角度
//   - /pitch/right/angle     右侧 pitch 电机角度
//
// 输出：
//   - /pitch/left/control_velocity  左侧 pitch 电机目标转速
//   - /pitch/right/control_velocity 右侧 pitch 电机目标转速
//   - /yaw/control_velocity         yaw 电机目标转速
//
// 停机条件：
//   遥控器两个开关同时拨到最下方时，忽略摇杆指令，三个目标转速全部置零。
//   该判断每个周期重新计算，任一开关离开最下方即恢复控制。
//   结束运行前先拨双下，等电机停稳后再退出进程。
//
// 同步原理：
//   上电后等待若干周期，等电机反馈到达，记录两侧角度作为各自的相对零点。
//   之后比较两侧相对零点的位移之差，把差值乘以同步系数，反向叠加到两侧的
//   转速指令上。上电时架子处于水平状态，两侧位移之差为零，同步控制维持
//   当前姿态，不会把架子推歪。

#include <cmath>

#include <eigen3/Eigen/Dense>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/logging.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller {

class GantryJoystick : public rmcs_executor::Component, public rclcpp::Node
{
    public:
        GantryJoystick()
            : Node{get_component_name(), // 从 yaml 文件里读取配置并构造 ROS 节点
                rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
            {
                // 读取两路摇杆
                register_input("/remote/joystick/left", joystick_left_);
                register_input("/remote/joystick/right", joystick_right_);

                // 读取遥控器左右两个三挡开关的状态
                register_input("/remote/switch/left", switch_left_);
                register_input("/remote/switch/right", switch_right_);

                // 读取两侧 pitch 电机的角度
                register_input("/pitch/left/angle", pitch_left_angle_);
                register_input("/pitch/right/angle", pitch_right_angle_);

                // 输出三个电机的目标转速
                register_output("/pitch/left/control_velocity", pitch_left_control_velocity_);
                register_output("/pitch/right/control_velocity", pitch_right_control_velocity_);
                register_output("/yaw/control_velocity", yaw_control_velocity_);

                dead_zone_ = get_parameter("dead_zone").as_double();
                pitch_max_velocity_ = get_parameter("pitch_max_velocity").as_double();
                yaw_max_velocity_ = get_parameter("yaw_max_velocity").as_double();
                sync_gain_ = get_parameter("sync_gain").as_double();
                max_sync_error_ = get_parameter("max_sync_error").as_double();
                zero_point_delay_cycles_ = static_cast<int>(get_parameter("zero_point_delay_cycles").as_int());
            }

        void update() override
        {
            // 两个开关都拨到最下方时停机
            const bool should_stop =
                *switch_left_ == rmcs_msgs::Switch::DOWN && *switch_right_ == rmcs_msgs::Switch::DOWN;

            if (should_stop) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "遥控器双下，三个电机目标转速置零");

                *pitch_left_control_velocity_ = 0.0;
                *pitch_right_control_velocity_ = 0.0;
                *yaw_control_velocity_ = 0.0;
                return;
            }

            // 摇杆指令：去死区后按最大转速换算成目标转速
            const double pitch_command = apply_dead_zone(joystick_left_->y()) * pitch_max_velocity_;
            const double yaw_command = apply_dead_zone(joystick_right_->x()) * yaw_max_velocity_;

            *yaw_control_velocity_ = yaw_command;

            // 等待电机反馈到达后再取相对零点，此前 pitch 不输出
            if (!zero_point_ready_) {
                if (++startup_cycles_ >= zero_point_delay_cycles_) {
                    pitch_left_zero_ = *pitch_left_angle_;
                    pitch_right_zero_ = *pitch_right_angle_;
                    zero_point_ready_ = true;
                }

                *pitch_left_control_velocity_ = 0.0;
                *pitch_right_control_velocity_ = 0.0;
                return;
            }

            // 两侧相对位移之差，即同步误差
            const double sync_error =
                (*pitch_left_angle_ - pitch_left_zero_) - (*pitch_right_angle_ - pitch_right_zero_);

            // 同步误差超阈值时停止 pitch 输出，误差过大通常意味着一侧被卡住
            if (std::abs(sync_error) > max_sync_error_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 500,
                    "sync_error=%.3f 超过阈值 %.3f，停止 pitch 输出", sync_error, max_sync_error_);

                *pitch_left_control_velocity_ = 0.0;
                *pitch_right_control_velocity_ = 0.0;
                return;
            }

            // 同步补偿：领先的一侧减速，落后的一侧加速
            const double correction = sync_gain_ * sync_error;
            *pitch_left_control_velocity_ = pitch_command - correction;
            *pitch_right_control_velocity_ = pitch_command + correction;

            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 500,
                "sync_error=%+.3f  velocity: pitch_left=%+7.3f  pitch_right=%+7.3f  yaw=%+7.3f",
                sync_error, *pitch_left_control_velocity_, *pitch_right_control_velocity_,
                *yaw_control_velocity_);
        }

    private:
        // 低于死区的摇杆量归零，高于死区的部分重新映射到满量程，避免越过死区时输出跳变
        double apply_dead_zone(double value) const
        {
            if (std::abs(value) <= dead_zone_) return 0.0;
            return (value - std::copysign(dead_zone_, value)) / (1.0 - dead_zone_);
        }

        // 摇杆死区
        double dead_zone_;
        double pitch_max_velocity_;
        double yaw_max_velocity_;
        // 同步补偿参数，过大时震荡，过小时补偿不明显
        double sync_gain_;
        // 最大同步误差
        double max_sync_error_;

        int zero_point_delay_cycles_;
        int startup_cycles_ = 0;
        bool zero_point_ready_ = false;
        double pitch_left_zero_ = 0.0;
        double pitch_right_zero_ = 0.0;

        InputInterface<Eigen::Vector2d> joystick_left_;
        InputInterface<Eigen::Vector2d> joystick_right_;
        InputInterface<rmcs_msgs::Switch> switch_left_;
        InputInterface<rmcs_msgs::Switch> switch_right_;
        InputInterface<double> pitch_left_angle_;
        InputInterface<double> pitch_right_angle_;

        OutputInterface<double> pitch_left_control_velocity_;
        OutputInterface<double> pitch_right_control_velocity_;
        OutputInterface<double> yaw_control_velocity_;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::GantryJoystick, rmcs_executor::Component)
