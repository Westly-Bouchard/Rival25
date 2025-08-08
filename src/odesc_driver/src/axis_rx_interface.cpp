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

AxisRxInterface::AxisRxInterface(int id, string interface) : id(id), interface(interface) {
    if (id < 0 || id > 63) {
        retWithError("Id must be between 0 and 63 inclusive");
        return;
    }

    if (interface.length() >= IF_NAMESIZE) {
        retWithError(string("Interface length must be less th an %i characters", IF_NAMESIZE));
        return;
    }

    // Parse dbc
    ifstream idbc("../dbc/odrive-cansimple.dbc");
    net = dbcppp::INetwork::LoadDBCFromIs(idbc);
    if (net.get() == nullptr) {
        retWithError("Failed to load dbc file");
        return;
    }
    for (const dbcppp::IMessage& msg : net->Messages()) {
        messages.insert(std::make_pair(msg.Id(), &msg));
    }
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
    if (bind(bus_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
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

frame AxisRxInterface::readFrame() {
    frame f;

    int nbytes = read(bus_fd, &f, sizeof(frame));

    return f;
}

optional<string> AxisRxInterface::retWithError(string msg) {
    error = make_optional<string>(msg);
    return error;
}

};  // namespace odesc_driver