#include <string>
#include <eigen3/Eigen/Dense>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/logging.hpp>


#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller {

class GM6020TestController : public rmcs_executor::Component, public rclcpp::Node
{
    public:
        GM6020TestController()
            : Node{get_component_name(),
                rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
            {
                // 读取右摇杆数据
                register_input("/remote/joystick/right", joystick_right_);

                // 统一输出目标，防止不同模式下冲突
                register_output("/motor/control_target", control_target_);

                // 读取设定的模式（控制速度 / 目标角度）
                mode_ = get_parameter("mode").as_string();
                // 读取角度模式下摇杆推满时的最大爬升速度
                max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();
                // 读取设定的最大速度
                max_velocity_ = get_parameter("max_velocity").as_double();
            }
        
        // 控制量和摇杆推移程度成正比
        void update() override
        {
            // 速度用右摇杆 y 方向控制
            if (mode_ == "velocity") { *control_target_ = joystick_right_ -> y() * max_velocity_; }
            // 角度用右摇杆 y 方向控制（注意不是实时角度，而是正推加一点，负推减一点，松开时停在当前位置）
            else 
            {
                target_angle_ += joystick_right_ -> y() * max_angular_velocity_ / 1000.0;
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 500,
            "y=%f  max_ang=%f   target=%f",
                    joystick_right_->y(), max_angular_velocity_, target_angle_);
                *control_target_ = target_angle_;
            }
        }

    
    private:
        std::string mode_;
        double max_velocity_;
        double max_angular_velocity_;
        double target_angle_;

        InputInterface<Eigen::Vector2d> joystick_right_;
        OutputInterface<double> control_target_;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::GM6020TestController, rmcs_executor::Component)