#ifndef ODESC_DRIVER_AXIS_ENCODER_ONLY_NODE_HPP
#define ODESC_DRIVER_AXIS_ENCODER_ONLY_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include "axis_encoder_only_interface.hpp"
#include "mixins/axis_node_publisher.hpp"

namespace odesc_driver {

class AxisEncoderOnlyNode : public rclcpp::Node,
                            public AxisEncoderOnlyInterface,
                            public AxisNodePublisher {
   public:
    AxisEncoderOnlyNode();

   protected:
    void node_publish_func(bool heartbeat, bool encoder) override;
};
};  // namespace odesc_driver

#endif  // ODESC_DRIVER_AXIS_ENCODER_ONLY_NODE_HPP