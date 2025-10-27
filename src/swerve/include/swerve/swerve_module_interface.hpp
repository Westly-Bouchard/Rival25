#ifndef SWERVE_MODULE_INTERFACE_HPP
#define SWERVE_MODULE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
// #include <odesc_driver/axis_rx_interface.hpp>
// #include <odesc_driver/axis_trx_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "../../../odesc_driver/include/odesc_driver/axis_rx_interface.hpp"
#include "../../../odesc_driver/include/odesc_driver/axis_trx_interface.hpp"

namespace swerve {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SwerveModuleHWI : public hardware_interface::SystemInterface {
   public:
    SwerveModuleHWI();

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time& time,
                                         const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time,
                                          const rclcpp::Duration& period) override;

   private:
    odesc_driver::AxisRxInterface* azimuthEncoder;
    odesc_driver::AxisTrxInterface* azimuthMotor;
    odesc_driver::AxisTrxInterface* driveMotor;

    std::string moduleName;

    double azPosition;

    double azTargetPosition;
    double driveTargetSpeed;
};
};  // namespace swerve

#endif  // SWERVE_MODULE_INTERFACE_HPP