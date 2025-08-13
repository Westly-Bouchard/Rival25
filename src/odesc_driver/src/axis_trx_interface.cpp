#include "../include/odesc_driver/axis_trx_interface.hpp"

#include <cstring>

using namespace std;

namespace odesc_driver {

AxisTrxInterface::AxisTrxInterface() : AxisRxInterface() {}

optional<string> AxisTrxInterface::writeFrame(uint64_t data, int msgType) {
    if (!connected) {
        return withError("Error writing can frame to bus, must be connected before write");
    }

    frame f;

    f.can_id = bus_fd << 5 | msgType;

    f.len = 8;

    memcpy(f.data, &data, sizeof(data));

    if (write(bus_fd, &f, sizeof(f)) < sizeof(f)) {
        return withError("Error writing can frame to bus, write call failed");
    }
}
}  // namespace odesc_driver