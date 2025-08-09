#include "../include/odesc_driver/axis_encoder_only_interface.hpp"

#include <unistd.h>

using namespace std;

namespace odesc_driver {

AxisEncoderOnlyInterface::AxisEncoderOnlyInterface() : AxisRxInterface() {}

void AxisEncoderOnlyInterface::rx_thread_func() {
    frame f;
    while (rx_running) {
        read(bus_fd, &f, sizeof(frame));
        auto iter = messages.find(f.can_id);
        if (iter != messages.end()) {
            const dbcppp::IMessage* msg = iter->second;
            string type = msg->Name().substr(msg->Name().find('_') + 1, msg->Name().length());
            if (type == "Get_Encoder_Estimates") {
                updateEncoderEstimateData(msg, f);
                node_publish_func(false, true);
                continue;
            }

            if (type == "Heartbeat") {
                updateHeartbeatData(msg, f);
                node_publish_func(true, false);
                continue;
            }
        }
    }
}

void AxisEncoderOnlyInterface::node_publish_func(bool, bool) {}

};  // namespace odesc_driver