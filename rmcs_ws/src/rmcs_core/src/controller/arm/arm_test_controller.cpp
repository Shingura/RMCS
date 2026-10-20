#include <array>
#include <cstddef>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::arm {

class ArmTestController final
    : public rmcs_executor::Component
    , public rclcpp::Node {
    static constexpr std::size_t num_axes = 6;

public:
    ArmTestController()
        : Node(
        get_component_name(),
        rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
            register_output("/arm/enable_flag", enabled_, false);

            for (std::size_t i = 0; i < num_axes; ++i) {
                const auto prefix = "/arm/joint_" + std::to_string(i + 1);
                register_input(prefix + "/theta", theta_[i]);
                register_output(prefix + "/target_theta", target_[i], 0.0);
            }
    }

    void update() override {
        *enabled_ = true;
        for (std::size_t i = 0; i < num_axes; ++i)
            *target_[i] = *theta_[i];
    }

private:
    OutputInterface<bool> enabled_;
    std::array<InputInterface<double>, num_axes> theta_;
    std::array<OutputInterface<double>, num_axes> target_;
};

} // namespace rmcs_core::controller::arm

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmTestController, rmcs_executor::Component)
