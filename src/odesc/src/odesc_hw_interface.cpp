#include "odesc/odesc_hw_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <sstream>

namespace odesc {
    using namespace std;

    ODescHwInterface::ODescHwInterface() {}

    CallbackReturn ODescHwInterface::on_init(const hardware_interface::HardwareInfo& info) {
        if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        command = numeric_limits<double>::quiet_NaN();
        position = 0.0;
        velocity = 0.0;
        effort = 0.0;

        can_id = stoi(info_.hardware_parameters["can_id"]);

        if (can_id < 0 || can_id > 63) {
            stringstream msg;
            msg << "Can id must be non-negative and less than 63. Id is currently: " << can_id;

            return error(msg.str());
        }

        // Default interface is 'can0'
        can_interface = "can0";
        if (info_.hardware_parameters.find("can_interface") != info_.hardware_parameters.end()) {

            string interface = info_.hardware_parameters["can_interface"];

            if (interface.length() >= static_cast<string::size_type>(IFNAMSIZ)) {
                stringstream msg;
                msg << "Can interface name cannot be longer than " << IFNAMSIZ << " characters";
                
                return error(msg.str());
            }

            can_interface = info_.hardware_parameters["can_interface"];
        }

        command_mode = info_.hardware_parameters["command_mode"];

        return CallbackReturn::SUCCESS;
    }


    CallbackReturn ODescHwInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
        // Obtain file descriptor
        bus_fd = socket(PF_CAN, static_cast<int32_t>(SOCK_RAW), CAN_RAW);

        if (bus_fd < 0) {
            return error("Failed to open can socket");
        }

        // Set interface name
        struct ifreq ifr;
        (void)strncpy(&ifr.ifr_name[0U], can_interface.c_str(), can_interface.length() + 1U);
        if (ioctl(bus_fd, static_cast<uint32_t>(SIOCGIFINDEX), &ifr) != 0) {
            return error("Failed to set can socket name via ioctl()");
        }

        // Create address and bind socket to it
        struct sockaddr_can addr;
        addr.can_family = static_cast<decltype(addr.can_family)>(AF_CAN);
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(bus_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
            return error("Failed to bind can socket");
        }

        // Disable local message loopback
        if (setsockopt(bus_fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, 0, sizeof(int)) != 0) {
            return error("Failed to disable local loopback");
        }

        // Set filtering to only receive frames from the correct id
        struct can_filter rfilter;
        rfilter.can_id = can_id;
        rfilter.can_mask = CAN_SFF_MASK;

        if (setsockopt(bus_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) != 0) {
            return error("Failed to configure can filters");
        }

        // Set up receive thread
        rx_running = true;
        rx_thread = thread(&ODescHwInterface::rx_thread_func, this);

        // TODO: Set AxisState to IDLE before ending the loop (just to make sure)
        // Also potentially set configuration parameters
        // Set controller mode (position, velocity, torque)

        return CallbackReturn::SUCCESS;

    }

    CallbackReturn ODescHwInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) { return CallbackReturn::SUCCESS; }

    CallbackReturn ODescHwInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) { return CallbackReturn::SUCCESS; }

    CallbackReturn ODescHwInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
        // Shut down rx thread
        rx_running = false;
        if (rx_thread.joinable()) {
            rx_thread.join();
        }

        // Close socket
        if (bus_fd >= 0) {
            close(bus_fd);
            bus_fd = -1;
        }

        return CallbackReturn::SUCCESS;
    }


    CallbackReturn ODescHwInterface::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) { return CallbackReturn::SUCCESS; }

    CallbackReturn ODescHwInterface::on_error(const rclcpp_lifecycle::State& /*previous_state*/) { return CallbackReturn::SUCCESS; }


    vector<hardware_interface::StateInterface> ODescHwInterface::export_state_interfaces() {
        vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &position)
        );
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &velocity)
        );
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &effort)
        );

        return state_interfaces;
    }

    vector<hardware_interface::CommandInterface> ODescHwInterface::export_command_interfaces() {
        vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &command)
        );
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &command)
        );
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &command)
        );

        return command_interfaces;
    }
    

    hardware_interface::return_type ODescHwInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) { return hardware_interface::return_type::OK; }

    hardware_interface::return_type ODescHwInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) { return hardware_interface::return_type::OK; }


    CallbackReturn ODescHwInterface::error(string msg) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("ODescHwInterface"), msg);
        return CallbackReturn::ERROR;
    }

    void ODescHwInterface::rx_thread_func() {
        while (rx_running) {
            // Read frame
        }
    }

};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(odesc::ODescHwInterface, hardware_interface::ActuatorInterface)