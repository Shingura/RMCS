// 龙门架发射架硬件组件：C 板 + M2006 电机（配 C610 电调）× 3 + DR16 遥控器
//
// 三个电机的分工：
//   - pitch_left、pitch_right：竖直安装，同步驱动左右两根杆升降，共同决定 pitch
//   - yaw：水平安装，改变 yaw
//
// 电机对象按名字前缀自动注册以下信号：
//   - /pitch/left/angle、/pitch/left/velocity、/pitch/left/torque、/pitch/left/max_torque
//   - /pitch/right/angle、/pitch/right/velocity、/pitch/right/torque、/pitch/right/max_torque
//   - /yaw/angle、/yaw/velocity、/yaw/torque、/yaw/max_torque
//   - 指令侧：/pitch/left/control_torque、/pitch/right/control_torque、/yaw/control_torque
//
// 本组件另外注册三个滤波后的速度输出，供速度环使用：
//   - /pitch/left/velocity_filtered、/pitch/right/velocity_filtered、/yaw/velocity_filtered

// C++ 标准库
#include <memory>

// 使用 C 板开发
#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/logging.hpp>

#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"

#include "filter/low_pass_filter.hpp"

namespace rmcs_core::hardware {

class Gantry : public rmcs_executor::Component, public rclcpp::Node, public librmcs::board::CBoard::Callback
{
    public:
        Gantry() 
            : Node{get_component_name(), // 从 yaml 文件里读取配置并构造 ROS 节点
                rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
            , command_component_{create_partner_component<MotorCommand>(get_component_name() + "_command", *this)} // partner_component 负责在最后发送指令到电机
            // 「左 / 右」是人面对龙门架时，人的左 / 右。
            , pitch_left_motor_{*this, *command_component_, "/pitch/left"}   // 生成左侧 pitch 电机对象
            , pitch_right_motor_{*this, *command_component_, "/pitch/right"} // 生成右侧 pitch 电机对象
            , yaw_motor_{*this, *command_component_, "/yaw"}                 // 生成 yaw 电机对象
            , dr16_{} // 生成 DR16 对象
            {
                // 从 yaml 中读取三个电机的 ID（pitch 左为 1, pitch 右为 2, yaw 为 3），并启用多圈角度累积
                // 多圈累积用于记录跨越多个电机圈的行程，上电时从当前位置开始累加
                pitch_left_motor_.configure(
                    device::DjiMotor::Config{
                        device::DjiMotor::Type::kM2006,
                        static_cast<std::uint8_t>(get_parameter("pitch_left_motor_id").as_int())}
                        .enable_multi_turn_angle());

                pitch_right_motor_.configure(
                    device::DjiMotor::Config{
                        device::DjiMotor::Type::kM2006,
                        static_cast<std::uint8_t>(get_parameter("pitch_right_motor_id").as_int())}
                        .enable_multi_turn_angle());

                yaw_motor_.configure(
                    device::DjiMotor::Config{
                        device::DjiMotor::Type::kM2006,
                        static_cast<std::uint8_t>(get_parameter("yaw_motor_id").as_int())}
                        .enable_multi_turn_angle());

                // 注册 DR16 遥控器
                remote_control_ = std::make_unique<device::RemoteControl>(*this);
                remote_control_ -> register_dr16(&dr16_);

                // 注册 C 板
                board_ = std::make_unique<librmcs::board::CBoard>(*this, get_parameter("board_serial").as_string());

                // 注册滤波后的速度输出
                register_output("/pitch/left/velocity_filtered", pitch_left_velocity_filtered_);
                register_output("/pitch/right/velocity_filtered", pitch_right_velocity_filtered_);
                register_output("/yaw/velocity_filtered", yaw_velocity_filtered_);
            }
        
        // 更新电机、遥控器状态
        void update() override
        {
            // 从 can_receive_callback() 获得的 CAN 帧中解出电机角度、速度和力矩并发布
            pitch_left_motor_.update_status();
            pitch_right_motor_.update_status();
            yaw_motor_.update_status();

            // 解出电机速度后立即输入到低通滤波器中，得到滤波后的速度
            *pitch_left_velocity_filtered_ = pitch_left_velocity_filter_.update(pitch_left_motor_.velocity());
            *pitch_right_velocity_filtered_ = pitch_right_velocity_filter_.update(pitch_right_motor_.velocity());
            *yaw_velocity_filtered_ = yaw_velocity_filter_.update(yaw_motor_.velocity());

            dr16_.update_status();       // 从 uart_receive_callback() 获得的 UART 帧中解出 DR16 指令
            remote_control_ -> update(); // 代发 DR16 指令
        }
    
    private:
        // 在 PID 计算完成后调用 command_update() 向电机发送 CAN 帧
        class MotorCommand : public rmcs_executor::Component
        {
            public:
                explicit MotorCommand(Gantry& hardware) : hardware_(hardware) {}
                void update() override { hardware_.command_update(); }

            private:
                Gantry& hardware_;
        };

        void command_update()
        {
            auto builder = board_ -> start_transmit();

            // 三个电机的 ID 都在 1 到 4 之间时共用同一帧，标识符为 0x200（参考 C610 电调文档）
            // 每个电机按自己的 ID 装进对应的四分之一槽位，剩余槽位填 0
            device::CanPacket8 packet{};
            packet << pitch_left_motor_ << pitch_right_motor_ << yaw_motor_;

            // 终端输出打印三个电机的目标力矩，力矩顺序与电机一一对应
            RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "send_id=0x%lX  torque: pitch_left=%+7.3f  pitch_right=%+7.3f  yaw=%+7.3f",
            (unsigned long)pitch_left_motor_.send_id(),
            pitch_left_motor_.control_torque(),
            pitch_right_motor_.control_torque(),
            yaw_motor_.control_torque());

            builder.can_transmit(Spec::kCans.kCan1, 
            {
                .can_id = pitch_left_motor_.send_id(),  // 标识符，表明电流控制
                .can_data = packet.as_bytes()           // 三个电机的目标力矩换算成的电流值
            });
        }

        void can_receive_callback(const Spec::Can & can, const View::Can & data) override
        {
            if (can != Spec::kCans.kCan1) return; // 丢弃不来自 CAN1 的帧

            // 匹配并存储对应电机 ID 的数据
            pitch_left_motor_.match_then_store_status(data.can_id, data.can_data);
            pitch_right_motor_.match_then_store_status(data.can_id, data.can_data);
            yaw_motor_.match_then_store_status(data.can_id, data.can_data);
        }

        void uart_receive_callback(const Spec::Uart & uart, const View::Uart & data) override
        {
            if (uart != Spec::kUarts.kDbus) return;                                                         // 丢弃不来自 DBUS 的帧
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());   // 储存遥控器发送的 UART 数据
        } 

        std::unique_ptr<librmcs::board::CBoard> board_;

        std::shared_ptr<MotorCommand> command_component_;

        device::DjiMotor pitch_left_motor_;
        device::DjiMotor pitch_right_motor_;
        device::DjiMotor yaw_motor_;
        device::Dr16 dr16_;
        std::unique_ptr<device::RemoteControl> remote_control_;

        // 设置低通滤波器截止频率为 50Hz，采样频率为 1000Hz
        filter::LowPassFilter<1> pitch_left_velocity_filter_{50.0, 1000.0};
        filter::LowPassFilter<1> pitch_right_velocity_filter_{50.0, 1000.0};
        filter::LowPassFilter<1> yaw_velocity_filter_{50.0, 1000.0};
        OutputInterface<double> pitch_left_velocity_filtered_;
        OutputInterface<double> pitch_right_velocity_filtered_;
        OutputInterface<double> yaw_velocity_filtered_;
};


} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Gantry, rmcs_executor::Component)
