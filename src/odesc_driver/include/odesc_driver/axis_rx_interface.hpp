#ifndef ODESC_DRIVER_AXIS_RX_INTERFACE_HPP
#define ODESC_DRIVER_AXIS_RX_INTERFACE_HPP

#include <linux/can/raw.h>

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

#include "mixins/receivers.hpp"
#include "mixins/util.hpp"

using frame = struct can_frame;

namespace odesc_driver {

class AxisRxInterface : public HeartbeatReceiver,
                        public GetEncoderEstimatesReceiver,
                        public GetEncoderCountReceiver {
   public:
    /**
     * @brief Construct a new Axis Rx Interface object
     *
     * @param id Id of the axis on the can bus
     * @param interface Name of interface (default is "can0")
     */
    AxisRxInterface();

    std::optional<std::string> setParams(int newId, std::string newInterface = "can0");

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

    /**
     * @brief Get the Error object
     *
     * @return std::optional<std::string>
     */
    std::optional<std::string> getError();

   protected:
    /**
     * @brief Function that runs the receiving thread
     *
     */
    void rx_thread_func();

    /**
     * @brief To be implemented in a child class that is a node, called in the rx thread to trigger
     * publishes to topics
     *
     * @param type type of message to publish
     */
    virtual void node_publish_func(MsgType type);

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
    int id;
    std::string interface;
    int bus_fd;
    const int idMask = 0x7c0;

    // Axis state data
    std::optional<std::string> error;

    // Rx thread data
    std::atomic<bool> rx_running;
    std::thread rx_thread;
};

};  // namespace odesc_driver

#endif  // ODESC_DRIVER_AXIS_RX_INTERFACE_HPP