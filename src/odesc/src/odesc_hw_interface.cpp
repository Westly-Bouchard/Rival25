#include "../include/odesc/odesc_hw_interface.hpp"

namespace odesc {

    ODescHWInterface::ODescHWInterface() {}

    CallbackReturn ODescHWInterface::on_init(const hardware_interface::HardwareInfo& info) {
        if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
    }


    CallbackReturn ODescHWInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {}

    CallbackReturn ODescHWInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {}

    CallbackReturn ODescHWInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state) {}

    CallbackReturn ODescHWInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state) {}


    CallbackReturn ODescHWInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state) {}

    CallbackReturn ODescHWInterface::on_error(const rclcpp_lifecycle::State& previous_state) {}


    std::vector<hardware_interface::StateInterface> ODescHWInterface::export_state_interfaces() {}

    std::vector<hardware_interface::CommandInterface> ODescHWInterface::export_command_interfaces() {}
    

    hardware_interface::return_type ODescHWInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {}

    hardware_interface::return_type ODescHWInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {}

};