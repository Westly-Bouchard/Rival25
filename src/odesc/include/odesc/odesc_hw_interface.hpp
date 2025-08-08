#ifndef ODESC_HW_INTERFACE_HPP
#define ODESC_HW_INTERFACE_HPP

#include <hardware_interface/actuator_interface.hpp>

#include <rclcpp/rclcpp.hpp>

namespace odesc {

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class ODescHWInterface : public hardware_interface::ActuatorInterface {
        public:

        ODescHWInterface();

        CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
        
        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    };

};

#endif // ODESC_HW_INTERFACE_HPP