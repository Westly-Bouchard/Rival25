#ifndef ODESC_DRIVER_TRANSMITTERS_HPP
#define ODESC_DRIVER_TRANSMITTERS_HPP

#include <linux/can/raw.h>

#include <cstdint>
#include <cstring>
#include <format>
#include <functional>
#include <optional>
#include <string>

#include "util.hpp"

using frame = struct can_frame;

namespace odesc_driver {

class SetAxisRequestedStateTransmitter {
   public:
    SetAxisRequestedStateTransmitter(std::function<std::optional<std::string>(uint64_t, int)> wf)
        : writeFrame(std::move(wf)) {}

    std::optional<std::string> setAxisRequestedState(int state) {
        if (state < 0 || state > 13 || state == 5) {
            return std::make_optional<std::string>(std::format("Invalid axis state: {}", state));
        }

        return writeFrame(static_cast<uint32_t>(state), CmdType::SET_AXIS_REQUESTED_STATE);
    }

   private:
    std::function<std::optional<std::string>(uint64_t, int)> writeFrame;
};

class SetInputPosTransmitter {
   public:
    SetInputPosTransmitter(std::function<std::optional<std::string>(uint64_t, int)> wf)
        : writeFrame(std::move(wf)) {}

    std::optional<std::string> setInputPos(float pos) {
        uint64_t data = 0;

        int16_t vel_ff = 0;
        int16_t torque_ff = 0;

        memcpy(((uint8_t*) &data) + 0, &pos, sizeof(pos));
        memcpy(((uint8_t*) &data) + 4, &vel_ff, sizeof(vel_ff));
        memcpy(((uint8_t*) &data) + 6, &torque_ff, sizeof(torque_ff));
        return writeFrame(data, CmdType::SET_INPUT_POS);
    }

   private:
    std::function<std::optional<std::string>(uint64_t, int)> writeFrame;
};

class SetInputVelTransmitter {
   public:
    SetInputVelTransmitter(std::function<std::optional<std::string>(uint64_t, int)> wf)
        : writeFrame(std::move(wf)) {}

    std::optional<std::string> setInputVel(float vel) {
        uint64_t data = 0;

        int32_t torque_ff = 0;

        memcpy(((uint8_t*) &data) + 0, &vel, sizeof(vel));
        memcpy(((uint8_t*) &data) + 4, &torque_ff, sizeof(torque_ff));

        return writeFrame(data, CmdType::SET_INPUT_VEL);
    }

   private:
    std::function<std::optional<std::string>(uint64_t, int)> writeFrame;
};

class SetLinearCountTransmitter {
   public:
    SetLinearCountTransmitter(std::function<std::optional<std::string>(uint64_t, int)> wf)
        : writeFrame(std::move(wf)) {}

    std::optional<std::string> setLinearCount(int count) {
        uint64_t data = 0;

        memcpy(((uint8_t*) &data) + 0, &count, sizeof(count));
        return writeFrame(data, CmdType::SET_LINEAR_COUNT);
    }

   private:
    std::function<std::optional<std::string>(uint64_t, int)> writeFrame;
};
}  // namespace odesc_driver

#endif  // ODESC_DRIVER_TRANSMITTERS_HPP