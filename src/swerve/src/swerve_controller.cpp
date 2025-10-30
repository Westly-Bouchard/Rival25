#include "../include/swerve/swerve_controller.hpp"

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

using namespace std;

namespace swerve {

double absMin(double one, double two) {
    if (abs(one) < abs(two)) {
        return one;
    }

    return two;
}

SwerveController::SwerveController() {
    azTargets = {0.0, 0.0, 0.0, 0.0};
    driveTargets = {0.0, 0.0, 0.0, 0.0};
}

controller_interface::CallbackReturn SwerveController::on_init() {
    for (const auto& name : names) {
        auto_declare<double>("azimuth_offsets." + name, 0.0);
    }

    auto_declare<double>("wheel_radius", 0.0);
    auto_declare<double>("az_gear_ratio", 0.0);
    auto_declare<double>("drive_gear_ratio", 0.0);

    auto_declare<double>("center_dist", 0.0);

    auto node = get_node();

    cmd_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&SwerveController::cmdCallback, this, std::placeholders::_1));

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_configure(
    const rclcpp_lifecycle::State& /* previous_state */) {
    auto node = get_node();

    for (const auto& name : names) {
        double azOffset;
        node->get_parameter("azimuth_offsets." + name, azOffset);

        azimuthOffsets.emplace_back(azOffset);
    }

    node->get_parameter("wheel_radius", wheelRadius);
    node->get_parameter("az_gear_ratio", azimuthGearRatio);
    node->get_parameter("drive_gear_ratio", driveGearRatio);
    node->get_parameter("center_dist", centerDistance);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SwerveController::command_interface_configuration()
    const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : names) {
        config.names.emplace_back(name + "Azimuth/az_target_position");
        config.names.emplace_back(name + "Drive/drive_target_velocity");
    }

    return config;
}

controller_interface::InterfaceConfiguration SwerveController::state_interface_configuration()
    const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : names) {
        config.names.emplace_back(name + "Azimuth/azimuth_position");
        config.names.emplace_back(name + "Azimuth/az_motor_pos");
    }

    return config;
}

controller_interface::CallbackReturn SwerveController::on_activate(
    const rclcpp_lifecycle::State& /* previous_state */) {
    geometry_msgs::msg::Twist zero;
    cmd_buff.writeFromNonRT(zero);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SwerveController::update(const rclcpp::Time& /* time */,
                                                           const rclcpp::Duration& /* duration */) {
    // Get most recent input to the controller
    const auto& cmd = cmd_buff.readFromRT();

    // Kinematics calculations
    double vX = cmd->linear.x - (cmd->angular.z * centerDistance);
    double vY = cmd->linear.y + (cmd->angular.z * -1 * centerDistance);

    driveTargets.at(0) = sqrt((vX * vX) + (vY * vY));
    azTargets.at(0) = atan2(vY, vX);

    vX = cmd->linear.x - (cmd->angular.z * centerDistance);
    vY = cmd->linear.y + (cmd->angular.z * centerDistance);

    driveTargets.at(1) = sqrt((vX * vX) + (vY * vY));
    azTargets.at(1) = atan2(vY, vX);

    vX = cmd->linear.x - (cmd->angular.z * -1 * centerDistance);
    vY = cmd->linear.y + (cmd->angular.z * centerDistance);

    driveTargets.at(2) = sqrt((vX * vX) + (vY * vY));
    azTargets.at(2) = atan2(vY, vX);

    vX = cmd->linear.x - (cmd->angular.z * -1 * centerDistance);
    vY = cmd->linear.y + (cmd->angular.z * -1 * centerDistance);

    driveTargets.at(3) = sqrt((vX * vX) + (vY * vY));
    azTargets.at(3) = atan2(vY, vX);

    for (int i = 0; i < 4; i++) {
        // Azimuth conversion math
        double azCurrentPos = state_interfaces_.at(i * 2).get_value();

        double azTargetPos = fmod((azTargets.at(i) / (2 * M_PI)) + azimuthOffsets.at(i), 1.0);

        double delta = -1 * (azTargetPos - azCurrentPos) * azimuthGearRatio;

        if (delta > 0.5) {
            delta -= 1.0;
        } else if (delta < -0.5) {
            delta += 1.0;
        }

        // bool err = 0;

        command_interfaces_.at(i * 2).set_value(state_interfaces_.at((i * 2) + 1).get_value() +
                                                delta);

        // Drive conversion math
        // Need to convert from m/s to turns per second

        // RCLCPP_INFO(get_node()->get_logger(), "Drive target: %f", driveTargets.at(i));

        int multiplier = i == 0 || i == 3 ? -1 : 1;

        command_interfaces_.at((i * 2) + 1)
            .set_value(multiplier * driveTargets.at(i) * driveGearRatio / (2 * M_PI * wheelRadius));

        // if (err) {
        //     return controller_interface::return_type::ERROR;
        // }
    }

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn SwerveController::on_deactivate(
    const rclcpp_lifecycle::State& /* previous_state */) {
    // Again, not sure what's supposed to happen here
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_cleanup(
    const rclcpp_lifecycle::State& /* previous_state */) {
    // Who even knows
    return controller_interface::CallbackReturn::SUCCESS;
}

void SwerveController::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_buff.writeFromNonRT(*msg);

    // RCLCPP_INFO(get_node()->get_logger(), "Received message");
    // RCLCPP_INFO(get_node()->get_logger(), "\tvX: %f", msg->linear.x);
    // RCLCPP_INFO(get_node()->get_logger(), "\tvY: %f", msg->linear.y);
    // RCLCPP_INFO(get_node()->get_logger(), "\twZ: %f", msg->angular.z);
}

};  // namespace swerve

// Plugin lib export stuff
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(swerve::SwerveController, controller_interface::ControllerInterface)