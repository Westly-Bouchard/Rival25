#include "../include/swerve/swerve_controller.hpp"

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

using namespace std;

namespace swerve {

SwerveController::SwerveController() {}

controller_interface::CallbackReturn SwerveController::on_init() {
    for (const auto& name : names) {
        auto_declare<vector<double>>("module_positions." + name, {0.0, 0.0});
        auto_declare<double>("azimuth_offsets." + name, 0.0);
    }

    auto_declare<double>("wheel_radius", 0.0);
    auto_declare<double>("az_gear_ratio", 0.0);
    auto_declare<double>("drive_gear_ratio", 0.0);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
    auto node = get_node();

    for (const auto& name : names) {
        vector<double> pos;
        node->get_parameter("module_positions." + name, pos);

        modulePositions.emplace_back(pos.at(0), pos.at(1));

        double azOffset;
        node->get_parameter("azimuth_offsets." + name, azOffset);

        azimuthOffsets.emplace_back(azOffset);
    }

    node->get_parameter("wheel_radius", wheelRadius);
    node->get_parameter("az_gear_ratio", azimuthGearRatio);
    node->get_parameter("drive_gear_ratio", driveGearRatio);

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
    const rclcpp_lifecycle::State& previous_state) {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SwerveController::update(const rclcpp::Time& time,
                                                           const rclcpp::Duration& duration) {
    for (int i = 0; i < 4; i++) {
        command_interfaces_[i * 2].set_value(0.0);
    }
    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn SwerveController::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_cleanup(
    const rclcpp_lifecycle::State& previous_state) {
    return controller_interface::CallbackReturn::SUCCESS;
}

};  // namespace swerve

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(swerve::SwerveController, controller_interface::ControllerInterface)