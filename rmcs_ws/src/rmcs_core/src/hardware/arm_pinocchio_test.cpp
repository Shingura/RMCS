#include "hardware/device/dr16.hpp"
#include "hardware/device/encorder.hpp"
#include "hardware/device/lk_motor.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <librmcs/board/rmcs_board_lite.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <std_msgs/msg/int32.hpp>

namespace rmcs_core::hardware {

// Six joints and the joint-2 absolute encoder, all on one RMCS Board Lite.
//
// CAN assignment on the board:
//   kCan0: joint 1..3 motors (0x141, 0x142, 0x143)
//   kCan1: joint 4..6 motors (0x144, 0x145, 0x141)
//   kCan2: joint 2 absolute encoder (0x200)
class ArmPinocchioTest final
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    static constexpr std::size_t num_axes = 6;

    class Command final : public rmcs_executor::Component {
    public:
        explicit Command(ArmPinocchioTest& hardware)
            : hardware_(hardware) {
            register_input("/arm/enable_flag", enabled_);
            register_input("/arm/hardware_ready", ready_);
        }

        void update() override {
            bool enabled = *enabled_ && *ready_ && hardware_.feedback_fresh();
            hardware_.command(enabled);
        }

    private:
        ArmPinocchioTest& hardware_;
        InputInterface<bool> enabled_;
        InputInterface<bool> ready_;
    };

    // Debug-only component: reads the torque command each motor driver consumes and prints it in
    // the motor frame, at most once per second.
    //
    // The inputs cannot live in the hardware component, because the hardware provides the motor
    // feedback that arm_config needs; an input there would close the cycle
    // hardware -> arm_config -> arm_solver -> hardware.
    // They cannot live in Command either, because LKMotor already registers them on it.
    // This component has no outputs, so it never affects the update order.
    class TorqueWatcher final : public rmcs_executor::Component {
    public:
        explicit TorqueWatcher(ArmPinocchioTest& hardware)
            : hardware_(hardware) {
            for (std::size_t i = 0; i < num_axes; ++i)
                register_input(
                    "/arm/joint_" + std::to_string(i + 1) + "/motor/control_torque",
                    commanded_torque_[i]);
        }

        void update() override {
            const auto t0 = hardware_.motor_torque(0, *commanded_torque_[0]);
            const auto t1 = hardware_.motor_torque(1, *commanded_torque_[1]);
            const auto t2 = hardware_.motor_torque(2, *commanded_torque_[2]);
            const auto t3 = hardware_.motor_torque(3, *commanded_torque_[3]);
            const auto t4 = hardware_.motor_torque(4, *commanded_torque_[4]);
            const auto t5 = hardware_.motor_torque(5, *commanded_torque_[5]);
            RCLCPP_INFO_THROTTLE(
                hardware_.get_logger(), *hardware_.get_clock(), 1000,
                "motor torque: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] N*m", t0, t1, t2, t3, t4, t5);
        }

    private:
        ArmPinocchioTest& hardware_;
        std::array<InputInterface<double>, num_axes> commanded_torque_;
    };

    class ArmBoard final : public librmcs::board::RmcsBoardLite::Callback {
    public:
        ArmBoard(ArmPinocchioTest& hardware, const std::string& serial)
            : hardware_(hardware) {
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(
                *this, serial,
                librmcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true));
        }

        ~ArmBoard() override {
            try {
                transmit(false);
                transmit(false);
            } catch (...) {
                // Board disconnection must not throw from destruction.
            }
        }

        void transmit(bool enabled) {
            auto tx = board_->start_transmit();
            // Alternate three joints each tick: every joint is commanded at 500 Hz.
            for (std::size_t i = phase_; i < num_axes; i += 2) {
                const auto& can = motor_can(i);
                auto packet = enabled ? hardware_.joints_[i].generate_torque_command()
                                        : device::LKMotor::lk_zero_torque_command();
                tx.can_transmit(can, {.can_id = can_ids_[i], .can_data = packet.as_bytes()});
            }
            phase_ = 1 - phase_;
        }

    private:
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission
                || data.can_data.size() != 8)
                return;

            // Check the encoder frame before the motor command-byte filter below:
            // the first byte of an encoder frame carries angle data, not a command code.
            if (can == encoder_can() && data.can_id == joint2_encoder_can_id_) {
                hardware_.joint2_encoder_.store_status(data.can_data);
                hardware_.last_feedback_[num_axes].store(Clock::now(), std::memory_order_release);
                return;
            }

            const auto command = std::to_integer<std::uint8_t>(data.can_data.front());
            if (command != 0x9C && command != 0xA4 && command != 0xA1 && command != 0xAD)
                return;
            for (std::size_t i = 0; i < num_axes; ++i) {
                const auto& expected_can = motor_can(i);
                if (can == expected_can && data.can_id == can_ids_[i]) {
                    hardware_.joints_[i].store_status(data.can_data);
                    hardware_.last_feedback_[i].store(Clock::now(), std::memory_order_release);
                    return;
                }
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            // The DR16 frame has no header; hand the whole payload to the parser.
            if (uart == Spec::kUarts.kDbus)
                hardware_.dr16_.store_status(data.uart_data);
        }

        static constexpr const Spec::Can& motor_can(std::size_t joint_index) {
            return joint_index < 3 ? Spec::kCans.kCan0 : Spec::kCans.kCan1;
        }

        static constexpr const Spec::Can& encoder_can() { return Spec::kCans.kCan2; }

        static constexpr std::array<std::uint32_t, num_axes> can_ids_{0x141, 0x142, 0x143,
                                                                      0x144, 0x145, 0x141};
        static constexpr std::uint32_t joint2_encoder_can_id_ = 0x200;

        ArmPinocchioTest& hardware_;
        std::size_t phase_{0};
        // Destroy the transport before the callback state it accesses.
        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    };

public:
    ArmPinocchioTest()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this))
        , torque_watcher_(create_partner_component<TorqueWatcher>(
              get_component_name() + "_torque_watcher", *this))
        , joints_{{{*this, *command_, "/arm/joint_1/motor"}, {*this, *command_, "/arm/joint_2/motor"}, {*this, *command_, "/arm/joint_3/motor"}, {*this, *command_, "/arm/joint_4/motor"}, {*this, *command_, "/arm/joint_5/motor"}, {*this, *command_, "/arm/joint_6/motor"}}}
        , joint2_encoder_(*this, "/arm/joint_2/encoder") {
        using namespace device;
        register_output("/arm/hardware_ready", all_feedback_fresh_, false);
        register_output("/remote/switch/left", remote_switch_left_, rmcs_msgs::Switch::UNKNOWN);
        register_output("/remote/switch/right", remote_switch_right_, rmcs_msgs::Switch::UNKNOWN);

        // One-shot report of raw encoder readings, used for joint zero calibration.
        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });

        const auto timeout = get_parameter("feedback_timeout_ms").as_int();
        if (timeout <= 0)
            throw std::invalid_argument("feedback_timeout_ms must be positive");
        feedback_timeout_ = std::chrono::milliseconds(timeout);
        for (auto& time : last_feedback_)
            time.store(TimePoint::min(), std::memory_order_relaxed);

        // Must match the reverse() flags below: joints 1, 2 and 6 are reversed.
        motor_direction_ = {-1.0, -1.0, 1.0, 1.0, 1.0, -1.0};

        const auto zero = [this](int joint) {
            return static_cast<int>(
                get_parameter("joint" + std::to_string(joint) + "_zero_point").as_int());
        };
        joints_[0].configure(
            LKMotorConfig{LKMotorType::MG5010E_i36V3}.reverse().set_encoder_zero_point(zero(1)));
        joints_[1].configure(
            LKMotorConfig{LKMotorType::MF7015V210T}.reverse().set_gear_ratio(42.0));
        joints_[2].configure(
            LKMotorConfig{LKMotorType::MG6012_i36}.set_encoder_zero_point(zero(3)));
        joints_[3].configure(
            LKMotorConfig{LKMotorType::MG4010E_i36V3}.set_encoder_zero_point(zero(4)));
        joints_[4].configure(
            LKMotorConfig{LKMotorType::MG5010E_i10V3}.set_encoder_zero_point(zero(5)));
        joints_[5].configure(
            LKMotorConfig{LKMotorType::MHF6015}.reverse().set_encoder_zero_point(zero(6)));
        joint2_encoder_.configure(
            EncoderConfig{EncoderType::KTH7823}.set_encoder_zero_point(zero(2)));

        if (get_parameter("connect_hardware").as_bool()) {
            const auto arm_serial = get_parameter("board_serial_arm_board").as_string();
            if (arm_serial.empty())
                throw std::invalid_argument("Set the arm board serial number");
            arm_board_ = std::make_unique<ArmBoard>(*this, arm_serial);
        } else {
            RCLCPP_INFO(
                get_logger(),
                "Hardware disconnected: model/graph verification only; enable is unavailable");
        }
    }

    void update() override {
        for (auto& joint : joints_)
            joint.update();
        joint2_encoder_.update();

        dr16_.update_status();
        *remote_switch_left_ = dr16_.switch_left();
        *remote_switch_right_ = dr16_.switch_right();

        *all_feedback_fresh_ = feedback_fresh();
    }

private:
    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            get_logger(), ".joint_1=%f,.joint_2=%f,.joint_3=%f,.joint_4=%f,.joint_5=%f,.joint_6=%f",
            joints_[0].get_angle(), joint2_encoder_.get_angle(), joints_[2].get_angle(),
            joints_[3].get_angle(), joints_[4].get_angle(), joints_[5].get_angle());
        RCLCPP_INFO(
            get_logger(), ".joint_1=%d,.joint_2=%d,.joint_3=%d,.joint_4=%d,.joint_5=%d,.joint_6=%d",
            joints_[0].get_raw_angle(), joint2_encoder_.get_raw_angle(), joints_[2].get_raw_angle(),
            joints_[3].get_raw_angle(), joints_[4].get_raw_angle(), joints_[5].get_raw_angle());
    }

    bool feedback_fresh() const {
        const auto now = Clock::now();
        for (const auto& feedback : last_feedback_) {
            const auto time = feedback.load(std::memory_order_acquire);
            if (time == TimePoint::min() || now - time > feedback_timeout_)
                return false;
        }
        return true;
    }

    void command(bool enabled) {
        if (arm_board_)
            arm_board_->transmit(enabled);
    }

    // Torque the driver sends for joint i: direction applied, clamped to the motor limit.
    // A NaN command stays NaN, so a disabled controller remains visible in the log.
    double motor_torque(std::size_t i, double command) {
        const double limit = joints_[i].get_max_torque();
        return std::clamp(motor_direction_[i] * command, -limit, limit);
    }

    std::shared_ptr<Command> command_;
    std::shared_ptr<TorqueWatcher> torque_watcher_;
    std::array<device::LKMotor, num_axes> joints_;
    std::array<double, num_axes> motor_direction_{};
    device::Encoder joint2_encoder_;
    device::Dr16 dr16_;
    OutputInterface<bool> all_feedback_fresh_;
    OutputInterface<rmcs_msgs::Switch> remote_switch_left_;
    OutputInterface<rmcs_msgs::Switch> remote_switch_right_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;
    std::chrono::milliseconds feedback_timeout_{100};
    std::array<std::atomic<TimePoint>, num_axes + 1> last_feedback_;
    std::unique_ptr<ArmBoard> arm_board_;
};

} // namespace rmcs_core::hardware

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::ArmPinocchioTest, rmcs_executor::Component)
