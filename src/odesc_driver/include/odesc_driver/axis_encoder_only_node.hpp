#ifndef ODESC_DRIVER_AXIS_ENCODER_ONLY_NODE_HPP
#define ODESC_DRIVER_AXIS_ENCODER_ONLY_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include "axis_rx_interface.hpp"
#include "mixins/axis_node_publisher.hpp"
#include "mixins/util.hpp"

namespace odesc_driver {

class AxisEncoderOnlyNode : public rclcpp::Node,
                            public AxisRxInterface,
                            public AxisNodePublisher {
   public:
    AxisEncoderOnlyNode();

   protected:
    void node_publish_func(MsgType type) override;
};
};  // namespace odesc_driver

#endif  // ODESC_DRIVER_AXIS_ENCODER_ONLY_NODE_HPP