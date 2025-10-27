#ifndef SWERVE_CONTROLLER_HPP
#define SWERVE_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

namespace swerve {

class SwerveController : public controller_interface::ControllerInterface {
   public:
    SwerveController();

    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(const rclcpp::Time& time,
                                             const rclcpp::Duration& duration) override;

   private:
    const std::vector<std::string> names = {"FR", "FL", "BL", "BR"};

    std::vector<std::pair<double, double>> modulePositions;

    std::vector<double> azimuthOffsets;

    double wheelRadius;  // meters

    double azimuthGearRatio;

    double driveGearRatio;
};
};  // namespace swerve

#endif