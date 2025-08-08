#ifndef ODESC_HW_INTERFACE_HPP
#define ODESC_HW_INTERFACE_HPP

#include <hardware_interface/actuator_interface.hpp>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <thread>

namespace odesc {

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class ODescHwInterface : public hardware_interface::ActuatorInterface {
        public:

        ODescHwInterface();

        /**
         * @brief Parses parameters and joint information from urdf
         * 
         * @param info information from <ros2_control> tag in urdf
         * @return CallbackReturn SUCCESS or ERROR
         */
        CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        /**
         * @brief 
         * 
         * @param previous_state 
         * @return CallbackReturn 
         */
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

        private:

        CallbackReturn error(std::string msg);

        void rx_thread_func();

        // Interface parameters
        int can_id;
        std::string can_interface;

        // Private variables
        int bus_fd;
        std::atomic<bool> rx_running;
        std::thread rx_thread;
    };

};

#endif // ODESC_HW_INTERFACE_HPP