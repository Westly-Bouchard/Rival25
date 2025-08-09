#ifndef ODESC_DRIVER_AXIS_RX_INTERFACE_HPP
#define ODESC_DRIVER_AXIS_RX_INTERFACE_HPP

#include <dbcppp/CApi.h>
#include <dbcppp/Network.h>
#include <linux/can/raw.h>

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

#include "mixins/receivers.hpp"

using frame = struct can_frame;

namespace odesc_driver {

class AxisRxInterface : public HeartbeatReceiver, public GetEncoderEstimatesReceiver {
   public:
    /**
     * @brief Construct a new Axis Rx Interface object
     *
     * @param id Id of the axis on the can bus
     * @param interface Name of interface (default is "can0")
     */
    AxisRxInterface(int id, std::string interface = "can0");

    /**
     * @brief Attempt to connect to the can bus
     *
     * @return std::optional<std::string> nullopt if successful, string with
     * error msg if unsuccessful
     */
    std::optional<std::string> connect();

    /**
     * @brief Shut down connection and clean up
     *
     * @return std::optional<std::string> nullopt if successful, string with
     * error msg if unsuccessful
     */
    std::optional<std::string> shutdown();

   protected:
    /**
     * @brief Function that runs the receiving thread
     *
     */
    virtual void rx_thread_func() = 0;

    /**
     * @brief Helper function to read a frame from the bus
     *
     * @return frame can_frame
     */
    frame readFrame();

    /**
     * @brief Helper function to simplify error reporting
     *
     * @param msg error message to report
     * @return std::optional<std::string> will never return nullopt
     */
    std::optional<std::string> retWithError(std::string msg);

    // Bus specific data
    const int id;
    const std::string interface;
    int bus_fd;
    const int idMask = 0x7c0;

    // Axis state data
    std::optional<std::string> error;

    // Rx thread data
    std::atomic<bool> rx_running;
    std::thread rx_thread;

    // Dbc parsing data
    std::unique_ptr<dbcppp::INetwork> net;
    std::unordered_map<uint64_t, const dbcppp::IMessage *> messages;
};

};  // namespace odesc_driver

#endif  // ODESC_DRIVER_AXIS_RX_INTERFACE_HPP