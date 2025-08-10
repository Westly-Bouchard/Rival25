#include "../include/odesc_driver/axis_encoder_only_interface.hpp"

#include <unistd.h>
#include <cstring>

using namespace std;

namespace odesc_driver {

AxisEncoderOnlyInterface::AxisEncoderOnlyInterface() : AxisRxInterface() {}

void AxisEncoderOnlyInterface::rx_thread_func() {
    frame f;
    while (rx_running) {
        f = readFrame();

        uint8_t msg_type = static_cast<uint8_t>(f.can_id & 0x1F);
        uint64_t data;
        memcpy(&data, f.data, sizeof(uint64_t));

        if (msg_type == 1) {
            updateHeartbeatData(data);
            node_publish_func(true, false);
        } else if (msg_type == 9) {
            updateEncoderEstimateData(data);
            node_publish_func(false, true);
        }
    }
}

void AxisEncoderOnlyInterface::node_publish_func(bool, bool) {}

};  // namespace odesc_driver