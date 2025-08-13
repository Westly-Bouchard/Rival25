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

        return writeFrame(static_cast<uint32_t>(state), MsgType::SET_AXIS_REQUESTED_STATE);
    }

   private:
    std::function<std::optional<std::string>(uint64_t, int)> writeFrame;
};

class SetInputPosTransmitter {
   public:
    SetInputPosTransmitter(std::function<std::optional<std::string>(uint64_t, int)> wf)
        : writeFrame(std::move(wf)) {}

    std::optional<std::string> setInputPos(float pos) {
        uint64_t data;
        memcpy(&data, &pos, sizeof(pos));
        return writeFrame(data, MsgType::SET_INPUT_POS);
    }

   private:
    std::function<std::optional<std::string>(uint64_t, int)> writeFrame;
};
}  // namespace odesc_driver

#endif  // ODESC_DRIVER_TRANSMITTERS_HPP