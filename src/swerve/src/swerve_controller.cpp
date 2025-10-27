#include "../include/swerve/swerve_controller.hpp"

namespace swerve {

SwerveController::SwerveController() {}

controller_interface::CallbackReturn SwerveController::on_init() {}

controller_interface::CallbackReturn SwerveController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {}

controller_interface::CallbackReturn SwerveController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {}

controller_interface::CallbackReturn SwerveController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {}

controller_interface::CallbackReturn SwerveController::on_cleanup   (
    const rclcpp_lifecycle::State &previous_state) {}

};  // namespace swerve