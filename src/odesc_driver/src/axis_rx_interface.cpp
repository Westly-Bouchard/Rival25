#include "../include/odesc_driver/axis_rx_interface.hpp"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <fstream>

using namespace std;

namespace odesc_driver {

AxisRxInterface::AxisRxInterface() : id(0), interface("can0") {}

optional<string> AxisRxInterface::setParams(int newId, string newInterface) {
    if (newId < 0 || newId > 63) {
        return retWithError("Can id must be between 0 and 63 inclusive");
    }

    if (newInterface.length() > IF_NAMESIZE) {
        return retWithError("Can interface must be less than 16 characters long");
    }

    id = newId;
    interface = newInterface;

    return nullopt;
}

optional<string> AxisRxInterface::connect() {
    // Obtain file descriptor
    bus_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (bus_fd < 0) {
        return retWithError("Failed to open socket");
    }

    // Set interface name
    struct ifreq ifr;
    strncpy(&ifr.ifr_name[0], interface.c_str(), interface.length() + 1);
    if (ioctl(bus_fd, SIOCGIFINDEX, &ifr) < 0) {
        return retWithError("Failed to set can socket name via ioctl()");
    }

    // Create address and bind to it
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(bus_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        return retWithError("Failed to bind can socket");
    }

    // Disable local message loopback
    int loopback = 0;
    if (setsockopt(bus_fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0) {
        return retWithError("Failed to disable local loopback");
    }

    // Set filtering to only receive frames from the correct id
    struct can_filter rfilter;
    rfilter.can_id = id << 5;
    rfilter.can_mask = idMask;
    if (setsockopt(bus_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
        return retWithError("Failed to configure can filters");
    }

    // Set up receive thread
    rx_running = true;
    rx_thread = thread(&AxisRxInterface::rx_thread_func, this);

    return nullopt;
}

optional<string> AxisRxInterface::shutdown() {
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

    return nullopt;
}

optional<string> AxisRxInterface::getError() {
    return error;
}

void AxisRxInterface::rx_thread_func() {
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

void AxisRxInterface::node_publish_func(MsgType) {}

frame AxisRxInterface::readFrame() {
    frame f;

    read(bus_fd, &f, sizeof(frame));

    return f;
}

optional<string> AxisRxInterface::retWithError(string msg) {
    error = make_optional<string>(msg);
    return error;
}

};  // namespace odesc_driver