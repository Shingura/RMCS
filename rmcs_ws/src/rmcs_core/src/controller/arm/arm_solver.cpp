#include "controller/pid/pid_calculator.hpp"
#include "std_msgs/msg/string.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <exception>
#include <memory>
#include <numbers>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_executor/component.hpp>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>
namespace rmcs_core::controller::arm {

class ArmSolver final
    : public rmcs_executor::Component
    , public rclcpp::Node {

    static constexpr std::size_t num_axis = 6;
    static constexpr int joint1_index      = 0;
    static constexpr int joint4_index      = 3;
    using TorqueVec                       = Eigen::Array<double, num_axis, 1>;

    struct PinocchioModel {
        pinocchio::Model pinocchio_model;
        std::array<pinocchio::JointIndex, num_axis> joint_ids{};
    };

public:
    explicit ArmSolver()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , joint_angle_pid_controller{
              pid::PidCalculator(6000.0, 0.0, 0.0), // joint_1
              pid::PidCalculator(33.0, 0.0, 0.0), // joint_2
              pid::PidCalculator(1100.0, 0.0, 50.0), // joint_3
              pid::PidCalculator(3000.0, 0.0, 4.0), // joint_4
              pid::PidCalculator(750.0, 0.0, 10.0), // joint_5
              pid::PidCalculator(100.0, 0.0, 4.0), // joint_6
          }
        , joint_vel_pid_controller{
              pid::PidCalculator(0.1, 0.0, 0.0), // joint_1
              pid::PidCalculator(15.1, 0.0, 0.00), // joint_2
              pid::PidCalculator(1.1, 0.0, 0.0), // joint_3
              pid::PidCalculator(0.125, 0.0, 0.00), // joint_4
              pid::PidCalculator(0.11, 0.0, 0.004), // joint_5
              pid::PidCalculator(0.0900, 0.0, 0.004), // joint_6
          } {
        for (std::size_t i = 0; i < num_axis; ++i) {
            const std::string joint_prefix = "/arm/joint_" + std::to_string(i + 1);
            register_input(joint_prefix + "/theta", joint_theta[i]);
            register_input(joint_prefix + "/lower_limit", joint_lower_limit[i]);
            register_input(joint_prefix + "/upper_limit", joint_upper_limit[i]);
            register_input(joint_prefix + "/velocity", joint_velocity[i]);
            register_input(joint_prefix + "/friction", joint_friction[i]);

            register_output(joint_prefix + "/motor/control_torque", joint_control_torque[i], NAN);
        }
        for (std::size_t i = 0; i < num_axis; ++i) {
            const std::string link_prefix = "/arm/link_" + std::to_string(i + 1);
            register_input(link_prefix + "/mass", link_mass[i]);
            register_input(link_prefix + "/com", link_com[i]);
            register_input(link_prefix + "/length", link_length[i]);
        }

        urdf_subscription = this->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).transient_local().reliable(),
            [this](const std_msgs::msg::String::ConstSharedPtr& msg) {
                load_pinocchio_model(msg->data);
            });

        register_input("urdf_loaded", is_loaded);
        register_input("/arm/joint_4/position", joint4_position);
        joint1_velocity_target_ = this->get_parameter("joint1_velocity_target").as_double();
        joint4_velocity_target_ = this->get_parameter("joint4_velocity_target").as_double();

        const auto list = this->get_parameter("controller_list").as_string_array();
        load_controller_list(list);
    }

    void update() override {
        const auto model = latest_received_model_.load(std::memory_order_acquire);

        static constexpr int kStartupHoldCycles = 1000;
        TorqueVec tau_cmd;
        tau_cmd.setZero();
        static int startup_cycle_count_ = 0;
        if (startup_cycle_count_ < kStartupHoldCycles) {
            ++startup_cycle_count_;
            tau_cmd = calculate_zero_torque();
            for (std::size_t i = 0; i < num_axis; ++i) {
                *joint_control_torque[i] = tau_cmd[i];
            }
            return;
        }
        if (!*is_loaded || !model) {
            tau_cmd = calculate_zero_torque();
            for (std::size_t i = 0; i < num_axis; ++i) {
                *joint_control_torque[i] = tau_cmd[i];
            }
            return;
        }

        if (in_use_model_ != model) {
            const auto& pinocchio_model = model->pinocchio_model;

            in_use_data_ = std::make_unique<pinocchio::Data>(pinocchio_model);
            joint_position_.setZero(pinocchio_model.nq);

            for (std::size_t i = 0; i < num_axis; ++i) {
                const auto joint_id = model->joint_ids[i];
                q_index_[i]         = static_cast<Eigen::Index>(pinocchio_model.idx_qs[joint_id]);
                v_index_[i]         = static_cast<Eigen::Index>(pinocchio_model.idx_vs[joint_id]);
            }

            in_use_model_ = model;
        }

        for (auto fn : controller_list) {
            tau_cmd += (this->*fn)();
        }
        tau_cmd[joint1_index] += calculate_joint1_velocity_pid();
        tau_cmd[joint4_index] += calculate_joint4_velocity_pid();

        tau_cmd[5] = 0;

        for (std::size_t i = 0; i < num_axis; ++i) {
            *joint_control_torque[i] = tau_cmd[i];
        }
    }

private:
    /// @brief /robot_description 的订阅回调，从 URDF 的 XML 建模
    void load_pinocchio_model(const std::string& urdf_xml) {
        auto building_model = std::make_shared<PinocchioModel>();

        try {
            pinocchio::urdf::buildModelFromXML(urdf_xml, building_model->pinocchio_model);

            auto& model = building_model->pinocchio_model;

            if ((model.nq != static_cast<int>(num_axis))
                || (model.nv != static_cast<int>(num_axis))) {
                throw std::runtime_error(
                    "Expected a " + std::to_string(num_axis) + "-DOF model, but got nq="
                    + std::to_string(model.nq) + ", nv=" + std::to_string(model.nv));
            }

            for (std::size_t i = 0; i < num_axis; ++i) {
                const std::string joint_name = "joint_" + std::to_string(i + 1);
                const auto joint_id          = model.getJointId(joint_name);

                if (joint_id == 0) {
                    throw std::runtime_error("Pinocchio joint not found: " + joint_name);
                }

                if (model.nqs[joint_id] != 1 || model.nvs[joint_id] != 1) {
                    throw std::runtime_error("Pinocchio joint is not one-DOF: " + joint_name);
                }

                building_model->joint_ids[i] = joint_id;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Pinocchio joint[%zu]: name=%s, id=%zu, q_index=%d, v_index=%d", i,
                    joint_name.c_str(), joint_id, model.idx_qs[joint_id], model.idx_vs[joint_id]);
            }

            model.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.81);

            RCLCPP_INFO(
                get_logger(), "Pinocchio model loaded: nq=%d, nv=%d, joints=%d", model.nq, model.nv,
                model.njoints);

            latest_received_model_.store(building_model, std::memory_order_release);
        } catch (const std::exception& err) {
            RCLCPP_ERROR(this->get_logger(), "Pinocchio URDF parse failed: %s", err.what());
            return;
        }
    }
    Eigen::Array<double, 6, 1> calculate_pid() {
        const auto normalize_angle = [](double angle) {
            while (angle > std::numbers::pi) {
                angle -= 2.0 * std::numbers::pi;
            }
            while (angle < -std::numbers::pi) {
                angle += 2.0 * std::numbers::pi;
            }
            return angle;
        };
        auto clamp_target_theta = [this](int idx, double target_theta) {
            const double lower_limit    = *joint_lower_limit(idx);
            const double upper_limit    = *joint_upper_limit(idx);
            const double clamped_target = std::clamp(target_theta, lower_limit, upper_limit);

            return clamped_target;
        };

        Eigen::Array<double, 6, 1> tau_pid;
        tau_pid.setZero();

        for (std::size_t i = 0; i < 6; ++i) {
            const int idx = static_cast<int>(i);
            if (idx == joint1_index || idx == joint4_index) {
                continue;
            }

            const double current_theta = *joint_theta(idx);
            const double target_theta  = clamp_target_theta(idx, *joint_target_theta(idx));
            const double current_vel   = *joint_velocity(idx);

            const double angle_error = normalize_angle(target_theta - current_theta);
            const double target_vel  = joint_angle_pid_controller(idx).update(angle_error);
            const double vel_error   = target_vel - current_vel;

            tau_pid(idx) = joint_vel_pid_controller(idx).update(vel_error);
        }

        return tau_pid;
    }
    double calculate_joint1_velocity_pid() {
        const double current_vel = *joint_velocity(joint1_index);
        const double vel_error   = joint1_velocity_target_ - current_vel;
        return joint_vel_pid_controller(joint1_index).update(vel_error);
    }
    double calculate_joint4_velocity_pid() {
        const double current_vel = *joint_velocity(joint4_index);
        const double vel_error   = joint4_velocity_target_ - current_vel;
        return joint_vel_pid_controller(joint4_index).update(vel_error);
    }
    Eigen::Array<double, 6, 1> calculate_friction_compensation() {
        // tau_f = b * dq + tau_c * tanh(dq / v0);
        //@YuuuuQingChi:I manually set the friction compensation of joint1, joint5, and joint6 to 0
        // because I felt the feel was not good.
        //@YuuuuQingChi:At the same time, I also do not consider viscous friction.

        // tau_f = tau_c * tanh(dq / v0)
        // joint1, joint5, joint6 = 0
        // no viscous friction

        Eigen::Array<double, 6, 1> joint_vel;
        joint_vel << *joint_velocity(0), *joint_velocity(1), *joint_velocity(2), *joint_velocity(3),
            *joint_velocity(4), *joint_velocity(5);

        Eigen::Array<double, 6, 1> speed_threshold;
        speed_threshold.setConstant(0.6);
        speed_threshold(1) = 0.2;

        speed_threshold(2) = 0.5;

        Eigen::Array<double, 6, 1> tau_c;
        tau_c << *joint_friction(0), *joint_friction(1), *joint_friction(2), *joint_friction(3),
            *joint_friction(4), *joint_friction(5);

        Eigen::Array<double, 6, 1> tau_f = tau_c * (joint_vel / speed_threshold).tanh();

        tau_f(1) = 0.0;
        tau_f(2) = 0.0;

        tau_f(3) = 0.0;
        tau_f(4) = 0.0;
        tau_f(5) = 0.0;
        tau_f    = -1.0 * tau_f;

        return tau_f;
    };

    TorqueVec calculate_gravity_compensation_with_pinocchio() {
        TorqueVec tau_g;
        tau_g.setZero();

        if (!in_use_model_ || !in_use_data_)
            return tau_g;

        const auto& pinocchio_model = in_use_model_->pinocchio_model;

        for (std::size_t i = 0; i < num_axis; ++i) {
            joint_position_(q_index_[i]) = *joint_theta[i];
        }

        const Eigen::VectorXd& gravity =
            pinocchio::computeGeneralizedGravity(pinocchio_model, *in_use_data_, joint_position_);

        for (std::size_t i = 0; i < num_axis; ++i) {
            tau_g(i) = gravity(v_index_[i]);
        }

        return tau_g;
    }

    Eigen::Array<double, 6, 1> calculate_gravity_compensation() {
        // The gravity compensation theoretical calculation for joint1 and joint6 is 0.
        static constexpr double g       = 9.81;
        static constexpr double reverse = -1.0;
        double theta_1                  = -(*joint_theta(1));
        double theta_2                  = -*joint_theta(2) + std::numbers::pi / 2.0;
        double theta_4                  = -*joint_theta(3);
        double theta_3                  = -*joint_theta(4);

        const double mass_1 = *link_mass(1);
        const double mass_2 = (*link_mass(3) + *link_mass(2));
        const double mass_3 = (*link_mass(4) + *link_mass(5));

        const double l_1m = link_com(1)->y();
        const double l_2m = ((link_com(2)->y() * (*link_mass(2)))
                             + ((joint4_position->y() + link_com(3)->z()) * (*link_mass(3))))
                          / ((*link_mass(2) + *link_mass(3)));
        constexpr double l_3m = 0.16;

        const double l1 = *link_length(1);
        const double l2 = *link_length(2);
        double s12      = sin(theta_1 + theta_2);

        const double x     = sin(theta_3) * cos(theta_4);
        const double phi   = std::asin(std::clamp(x, -1.0, 1.0));
        const double denom = std::sqrt(std::max(0.0, 1.0 - x * x));

        const double k_5 = (denom > 0.0) ? ((cos(theta_3) * cos(theta_4)) / denom) : 0.0;

        double k = l_3m * sin(theta_1 + theta_2 + phi);

        double joint_5_tau_g = (-k * mass_3 * k_5) * g;
        double joint_4_tau_g = (-mass_3 * l_3m * sin(theta_3) * sin(theta_4) * s12) * g;
        double joint_3_tau_g = (-l_2m * s12 * mass_2 - (l2 * s12 + k) * mass_3) * g;

        double k_2          = -l_1m * sin(theta_1) * mass_1;
        double j_2          = -l1 * sin(theta_1) - l_2m * s12;
        double i_2          = -(l1 * sin(theta_1) + l2 * s12 + k);
        double joint2_tau_g = (k_2 + j_2 * mass_2 + i_2 * mass_3) * g;

        Eigen::Array<double, 6, 1> tau_g;
        tau_g.setZero();

        tau_g(1) = reverse * joint2_tau_g;
        tau_g(2) =  reverse * joint_3_tau_g;
        tau_g(3) = joint_4_tau_g;
        tau_g(4) = reverse * joint_5_tau_g;
        return tau_g;
    };
    Eigen::Array<double, 6, 1> calculate_zero_torque() {
        Eigen::Array<double, 6, 1> tau;
        tau.fill(NAN);
        return tau;
    }
    using controller_type = Eigen::Array<double, 6, 1> (ArmSolver::*)();

    static constexpr std::array<std::tuple<std::string_view, controller_type>, 5> term_table_{
        {{"gravity", &ArmSolver::calculate_gravity_compensation},
         {"gravity_pinocchio", &ArmSolver::calculate_gravity_compensation_with_pinocchio},
         {"pid", &ArmSolver::calculate_pid},
         {"friction", &ArmSolver::calculate_friction_compensation},
         {"zero_torque", &ArmSolver::calculate_zero_torque}}
    };

    std::vector<controller_type> controller_list;

    void load_controller_list(const std::vector<std::string>& list) {
        controller_list.clear();
        controller_list.reserve(list.size());
        std::unordered_set<std::string_view> record;
        record.reserve(list.size());

        for (const auto& l : list) {
            const std::string_view sv{l};

            if (!record.insert(sv).second)
                continue;

            bool matched = false;
            for (const auto& [name, fn] : term_table_) {
                if (sv == name) {
                    controller_list.push_back(fn);
                    matched = true;
                    break;
                }
            }

            if (!matched) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Unknown enable_terms entry: '%s' (allowed: gravity, gravity_pinocchio, pid, "
                    "friction, zero_torque)",
                    l.c_str());
            }
        }
    }

    std::atomic<std::shared_ptr<const PinocchioModel>> latest_received_model_{nullptr};
    std::shared_ptr<const PinocchioModel> in_use_model_;
    std::unique_ptr<pinocchio::Data> in_use_data_;

    Eigen::VectorXd joint_position_;
    std::array<Eigen::Index, num_axis> q_index_{};
    std::array<Eigen::Index, num_axis> v_index_{};

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_subscription;

    Eigen::Array<pid::PidCalculator, 6, 1> joint_angle_pid_controller;
    Eigen::Array<pid::PidCalculator, 6, 1> joint_vel_pid_controller;

    Eigen::Array<OutputInterface<double>, 6, 1> joint_control_torque;

    Eigen::Array<InputInterface<double>, 6, 1> joint_target_theta;
    Eigen::Array<InputInterface<double>, 6, 1> joint_lower_limit;
    Eigen::Array<InputInterface<double>, 6, 1> joint_upper_limit;
    Eigen::Array<InputInterface<double>, 6, 1> joint_friction;
    Eigen::Array<InputInterface<double>, 6, 1> joint_velocity;
    Eigen::Array<InputInterface<double>, 6, 1> joint_theta;
    Eigen::Array<InputInterface<double>, 6, 1> link_mass;
    Eigen::Array<InputInterface<double>, 6, 1> link_length;
    Eigen::Array<InputInterface<Eigen::Vector3d>, 6, 1> link_com;

    InputInterface<Eigen::Vector3d> joint4_position;
    InputInterface<bool> is_arm_enable;
    OutputInterface<double> gripper_control_torque;
    double joint1_velocity_target_{0.0};
    double joint4_velocity_target_{0.0};

    InputInterface<bool> is_loaded;
};

} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmSolver, rmcs_executor::Component)
