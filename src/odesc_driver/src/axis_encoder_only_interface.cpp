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

        switch (msg_type) {
            case MsgType::HEARTBEAT:
                updateHeartbeatData(data);
                break;
            case MsgType::GET_ENCODER_ESTIMATES:
                updateEncoderEstimateData(data);
                break;
        }

        node_publish_func(static_cast<MsgType>(msg_type));
    }
}

void AxisEncoderOnlyInterface::node_publish_func(MsgType) {}

};  // namespace odesc_driver