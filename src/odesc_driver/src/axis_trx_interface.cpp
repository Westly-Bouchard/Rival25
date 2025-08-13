#include "../include/odesc_driver/axis_trx_interface.hpp"

#include <cstring>

using namespace std;

namespace odesc_driver {

AxisTrxInterface::AxisTrxInterface()
    : AxisRxInterface(),
      SetAxisRequestedStateTransmitter(std::bind(&AxisTrxInterface::writeFrame, this,
                                                 std::placeholders::_1, std::placeholders::_2)),
      SetInputPosTransmitter(std::bind(&AxisTrxInterface::writeFrame, this, std::placeholders::_1,
                                       std::placeholders::_2)) {}

optional<string> AxisTrxInterface::writeFrame(uint64_t data, int msgType) {
    if (!connected) {
        return withError("Error writing can frame to bus, must be connected before write");
    }

    frame f;

    f.can_id = id << 5 | msgType;

    f.len = 8;

    memcpy(f.data, &data, sizeof(data));

    if (write(bus_fd, &f, sizeof(f)) < 0) {
        return withError("Error writing can frame to bus, write call failed");
    }

    return nullopt;
}
}  // namespace odesc_driver