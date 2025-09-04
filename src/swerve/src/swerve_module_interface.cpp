#include "../include/swerve/swerve_module_interface.hpp"

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace odesc_driver;
using namespace std;

namespace swerve {

SwerveModuleHWI::SwerveModuleHWI() {}

CallbackReturn SwerveModuleHWI::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    int azimuthEncoderId = stoi(info_.hardware_parameters["azimuth_encoder_id"]);
    int azimuthMotorId = stoi(info_.hardware_parameters["azimuth_motor_id"]);
    int driveMotorId = stoi(info_.hardware_parameters["drive_motor_id"]);

    string canInterface = "can0";

    if (info_.hardware_parameters.find("can_interface") != info_.hardware_parameters.end()) {
        canInterface = info_.hardware_parameters["can_interface"];
    }

    azimuthEncoder = new AxisRxInterface();
    azimuthMotor = new AxisTrxInterface();
    driveMotor = new AxisTrxInterface();

    vector<optional<string>> errors;

    errors.push_back(azimuthEncoder->setParams(azimuthEncoderId, canInterface));
    errors.push_back(azimuthMotor->setParams(azimuthMotorId, canInterface));
    errors.push_back(driveMotor->setParams(driveMotorId, canInterface));

    bool configError = false;

    for (const auto& result : errors) {
        if (result != nullopt) {
            RCLCPP_ERROR_STREAM(get_logger(), result.value());
            configError = true;
        }
    }

    if (configError) {
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveModuleHWI::on_configure(const rclcpp_lifecycle::State& /*previous state*/) {
    vector<optional<string>> errors;

    errors.push_back(azimuthEncoder->connect());
    errors.push_back(azimuthMotor->connect());
    errors.push_back(driveMotor->connect());

    bool hadError = false;

    for (const optional<string>& result : errors) {
        if (result != nullopt) {
            RCLCPP_ERROR_STREAM(get_logger(), result.value());
            hadError = true;
        }
    }

    if (hadError) {
        return CallbackReturn::ERROR;
    } else {
        return CallbackReturn::SUCCESS;
    }
}

CallbackReturn SwerveModuleHWI::on_cleanup(const rclcpp_lifecycle::State& /*previous state*/) {
    azimuthEncoder->shutdown();
    azimuthMotor->shutdown();
    driveMotor->shutdown();

    // Right now there's no path in the shutdown function that can error so this is fine
    return CallbackReturn::SUCCESS;
}

vector<hardware_interface::StateInterface> SwerveModuleHWI::export_state_interfaces() {
    vector<hardware_interface::StateInterface> interfaces;
    // TODO: Figure out what to put in the first argument here
    interfaces.emplace_back("azimuth", "azimuth_position", &azPosition);

    return interfaces;
}

vector<hardware_interface::CommandInterface> SwerveModuleHWI::export_command_interfaces() {
    vector<hardware_interface::CommandInterface> interfaces;

    interfaces.emplace_back("azimuth", "az_target_position", &azTargetPosition);
    interfaces.emplace_back("drive", "drive_target_velocity", &driveTargetSpeed);

    return interfaces;
}

CallbackReturn SwerveModuleHWI::on_activate(const rclcpp_lifecycle::State& /*previous state*/) {
    // TODO: Create enum for axis states
    azimuthMotor->setAxisRequestedState(8);
    driveMotor->setAxisRequestedState(8);

    // Technically these method calls can fail, but the only failure path comes from being passed an
    // invalid state, which we aren't doing here. For now, this is fine.
    return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveModuleHWI::on_deactivate(const rclcpp_lifecycle::State& /*previous state*/) {
    driveMotor->setAxisRequestedState(1);
    azimuthMotor->setAxisRequestedState(1);

    return CallbackReturn::SUCCESS;
}

/*
For now, these twwo functions will just do the same thing as deactivate, because I'm still not
entirely sure what they're actually supposed to do
*/

CallbackReturn SwerveModuleHWI::on_shutdown(const rclcpp_lifecycle::State& /*previous state*/) {
    driveMotor->setAxisRequestedState(1);
    azimuthMotor->setAxisRequestedState(1);

    return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveModuleHWI::on_error(const rclcpp_lifecycle::State& /*previous state*/) {
    driveMotor->setAxisRequestedState(1);
    azimuthMotor->setAxisRequestedState(1);

    return CallbackReturn::SUCCESS;
}

/* Periodic IO Functions */

hardware_interface::return_type SwerveModuleHWI::read(const rclcpp::Time& /*time*/,
                                                      const rclcpp::Duration& /*period*/) {
    azPosition = azimuthEncoder->getEncoderCountCPR() / pow(2, 14);

    // TODO: Need to check for axis errors here and eventually handle reading other data from the
    // module

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SwerveModuleHWI::write(const rclcpp::Time& /*time*/,
                                                       const rclcpp::Duration& /*period*/) {
    // azimuthMotor->setInputPos(azTargetPosition);
    // driveMotor->setInputVel(driveTargetSpeed);

    // RCLCPP_INFO(get_logger(), "AZ target tosition is: %f", azTargetPosition);
    // RCLCPP_INFO(get_logger(), "Drive target speed is: %f", driveTargetSpeed);

    // TODO: Error checking?

    return hardware_interface::return_type::OK;
}

};  // namespace swerve

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(swerve::SwerveModuleHWI, hardware_interface::SystemInterface)