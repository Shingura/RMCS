#include "hardware/device/encorder.hpp"
#include "hardware/device/lk_motor.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <librmcs/board/c_board.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

// Six joints and the joint-2 absolute encoder
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

    class ArmBoard final : public librmcs::board::CBoard::Callback {
    public:
        ArmBoard(ArmPinocchioTest& hardware, const std::string& serial)
            : hardware_(hardware) {
            board_ = std::make_unique<librmcs::board::CBoard>(
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
                const auto& can = i < 3 ? Spec::kCans.kCan1 : Spec::kCans.kCan2;
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
            const auto command = std::to_integer<std::uint8_t>(data.can_data.front());
            if (command != 0x9C && command != 0xA4 && command != 0xA1 && command != 0xAD)
                return;
            for (std::size_t i = 0; i < num_axes; ++i) {
                const auto& expected_can = i < 3 ? Spec::kCans.kCan1 : Spec::kCans.kCan2;
                if (can == expected_can && data.can_id == can_ids_[i]) {
                    hardware_.joints_[i].store_status(data.can_data);
                    hardware_.last_feedback_[i].store(Clock::now(), std::memory_order_release);
                    return;
                }
            }
        }

        static constexpr std::array<std::uint32_t, num_axes> can_ids_{0x141, 0x142, 0x143,
                                                                      0x144, 0x145, 0x141};
        ArmPinocchioTest& hardware_;
        std::size_t phase_{0};
        // Destroy the transport before the callback state it accesses.
        std::unique_ptr<librmcs::board::CBoard> board_;
    };

    class EncoderBoard final : public librmcs::board::CBoard::Callback {
    public:
        EncoderBoard(ArmPinocchioTest& hardware, const std::string& serial)
            : hardware_(hardware) {
            board_ = std::make_unique<librmcs::board::CBoard>(
                *this, serial,
                librmcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true));
        }

    private:
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission
                || data.can_data.size() != 8)
                return;
            if (can == Spec::kCans.kCan1 && data.can_id == 0x200) {
                hardware_.joint2_encoder_.store_status(data.can_data);
                hardware_.last_feedback_[num_axes].store(Clock::now(), std::memory_order_release);
            }
        }

        ArmPinocchioTest& hardware_;
        std::unique_ptr<librmcs::board::CBoard> board_;
    };

public:
    ArmPinocchioTest()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this))
        , joints_{{{*this, *command_, "/arm/joint_1/motor"}, {*this, *command_, "/arm/joint_2/motor"}, {*this, *command_, "/arm/joint_3/motor"}, {*this, *command_, "/arm/joint_4/motor"}, {*this, *command_, "/arm/joint_5/motor"}, {*this, *command_, "/arm/joint_6/motor"}}}
        , joint2_encoder_(*this, "/arm/joint_2/encoder") {
        using namespace device;
        register_output("/arm/hardware_ready", ready_, false);
        const auto timeout = get_parameter("feedback_timeout_ms").as_int();
        if (timeout <= 0)
            throw std::invalid_argument("feedback_timeout_ms must be positive");
        feedback_timeout_ = std::chrono::milliseconds(timeout);
        for (auto& time : last_feedback_)
            time.store(TimePoint::min(), std::memory_order_relaxed);

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
            const auto encoder_serial = get_parameter("board_serial_encoder_board").as_string();
            if (arm_serial.empty() || encoder_serial.empty() || arm_serial == encoder_serial)
                throw std::invalid_argument("Set distinct arm and encoder board serial numbers");
            arm_board_ = std::make_unique<ArmBoard>(*this, arm_serial);
            encoder_board_ = std::make_unique<EncoderBoard>(*this, encoder_serial);
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
        *ready_ = feedback_fresh();
    }

private:
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

    std::shared_ptr<Command> command_;
    std::array<device::LKMotor, num_axes> joints_;
    device::Encoder joint2_encoder_;
    OutputInterface<bool> ready_;
    std::chrono::milliseconds feedback_timeout_{100};
    std::array<std::atomic<TimePoint>, num_axes + 1> last_feedback_;
    std::unique_ptr<ArmBoard> arm_board_;
    std::unique_ptr<EncoderBoard> encoder_board_;
};

} // namespace rmcs_core::hardware

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::ArmPinocchioTest, rmcs_executor::Component)
