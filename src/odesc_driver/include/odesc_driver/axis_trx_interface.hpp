#ifndef ODESC_DRIVER_AXIS_TRX_INTERFACE_HPP
#define ODESC_DRIVER_AXIS_TRX_INTERFACE_HPP

#include <linux/can/raw.h>

#include <optional>
#include <string>

#include "axis_rx_interface.hpp"
#include "mixins/transmitters.hpp"

using frame = struct can_frame;

namespace odesc_driver {
class AxisTrxInterface : public AxisRxInterface,
                         public SetAxisRequestedStateTransmitter,
                         public SetInputPosTransmitter,
                         public SetInputVelTransmitter,
                         public SetLinearCountTransmitter {
   public:
    AxisTrxInterface();

    std::optional<std::string> writeFrame(uint64_t data, int msgType);

   private:
};
};  // namespace odesc_driver

#endif  // ODESC_DRIVER_AXIS_TRX_INTERFACE_HPP