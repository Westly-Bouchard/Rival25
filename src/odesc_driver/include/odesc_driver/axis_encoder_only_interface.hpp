#ifndef ODESC_DRIVER_AXIS_ENCODER_ONLY_INTERFACE
#define ODESC_DRIVER_AXIS_ENCODER_ONLY_INTERFACE

#include "axis_rx_interface.hpp"

namespace odesc_driver {

class AxisEncoderOnlyInterface : public AxisRxInterface {
   public:
    AxisEncoderOnlyInterface(int id, std::string interface = "can0");

   private:
    void rx_thread_func() override;

    virtual void node_publish_func(bool heartbeat, bool encoder);
};
};  // namespace odesc_driver

#endif  // ODESC_DRIVER_AXIS_ENCODER_ONLY_INTERFACE