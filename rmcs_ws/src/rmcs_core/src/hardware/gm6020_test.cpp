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

class GM6020Test : public rmcs_executor::Component, public rclcpp::Node, public librmcs::board::CBoard::Callback
{
    public:
        GM6020Test() 
            : Node{get_component_name(), // 从 yaml 文件里读取配置并构造 ROS 节点
                rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
            , command_component_{create_partner_component<MotorCommand>(get_component_name() + "_command", *this)} // partner_component 负责在最后发送指令到电机
            , motor_{*this, *command_component_, "/motor"} // 生成 GM6020 电机对象
            , dr16_{} // 生成 DR16 对象（这里是已经封装好的 DR16 的数据解算器，注意与下面的 remote_control 区分）
            {
                motor_.configure(
                    device::DjiMotor::Config{
                        device::DjiMotor::Type::kGM6020, 
                        static_cast<std::uint8_t>(get_parameter("motor_id").as_int())}  // 从 yaml 中读取电机 ID
                        .enable_multi_turn_angle());   // 启动多圆累积

                // 注册 DR16 遥控器（这里的 remote_control 更像是一个 hub，负责仲裁多路遥控器并代发数据到 channel 中，本例中只有 DR16 这一个遥控器）
                remote_control_ = std::make_unique<device::RemoteControl>(*this);
                remote_control_ -> register_dr16(&dr16_); // 可以理解为把 DR16 挂载到这个 hub 上

                // 注册 C 板
                board_ = std::make_unique<librmcs::board::CBoard>(*this, get_parameter("board_serial").as_string());

                // 注册滤波后的速度输出
                register_output("/motor/velocity_filtered", velocity_filtered_);
            }
        
        // 更新电机、遥控器状态
        void update() override
        {
            motor_.update_status();                                                         // 从 can_receive_callback() 获得的 CAN 帧中解出电机角度、速度和力矩并发布
            *velocity_filtered_ = velocity_filter_.update(motor_.velocity());         // 解出电机速度后立即输入到低通滤波器中，得到滤波后的速度
            dr16_.update_status();                                                          // 从 uart_receive_callback() 获得的 UART 帧中解出 DR16 指令
            remote_control_ -> update();                                                    // 代发 DR16 指令
        }
    
    private:
        // MotorCommand 作为 partner_component，唯一的作用就是在最后（PID 计算完成后）调用 command_update() 向电机发送 CAN 帧
        class MotorCommand : public rmcs_executor::Component
        {
            public:
                explicit MotorCommand(GM6020Test& hardware) : hardware_(hardware) {}
                void update() override { hardware_.command_update(); }

            private:
                GM6020Test& hardware_;
        };

        void command_update()
        {
            auto builder = board_ -> start_transmit();

            builder.can_transmit(Spec::kCans.kCan2, 
            {
                .can_id = motor_.send_id(),                                // 标识符，表明电流控制（参考 GM6020 文档）
                .can_data = device::CanPacket8
                    {
                    motor_.generate_command(),              // 读取目标力矩，换算成电流发送给 1 号电机
                    device::CanPacket8::PaddingQuarter{},   // 其余电机设置为 0
                    device::CanPacket8::PaddingQuarter{},
                    device::CanPacket8::PaddingQuarter{},
                    }.as_bytes()
            });
        }

        void can_receive_callback(const Spec::Can & can, const View::Can & data) override
        {
            if (can != Spec::kCans.kCan2) return;                                               // 丢弃不来自 CAN2 的帧
            motor_.match_then_store_status(data.can_id, data.can_data);        // 留下电机 ID 匹配的数据
        }

        void uart_receive_callback(const Spec::Uart & uart, const View::Uart & data) override
        {
            if (uart != Spec::kUarts.kDbus) return;                                                         // 丢弃不来自 DBUS 的帧
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());   // 储存遥控器发送的 UART 数据
        } 

        std::unique_ptr<librmcs::board::CBoard> board_;

        std::shared_ptr<MotorCommand> command_component_;

        device::DjiMotor motor_;
        device::Dr16 dr16_;
        std::unique_ptr<device::RemoteControl> remote_control_;

        // 设置低通滤波器截止频率为 100Hz，采样频率为 1000Hz
        filter::LowPassFilter<1> velocity_filter_{100.0, 1000.0};
        OutputInterface<double> velocity_filtered_;
};


} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::GM6020Test, rmcs_executor::Component)