#ifndef ODESC_DRIVER_AXIS_NODE_HPP
#define ODESC_DRIVER_AXIS_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "axis_trx_interface.hpp"
#include "mixins/axis_node_publisher.hpp"

namespace odesc_driver {

class AxisNode : public rclcpp::Node, public AxisTrxInterface, public AxisNodePublisher {
   public:
    AxisNode();

   private:
    void node_publish_func(MsgType type) override;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr axisRequestedStateSub;
    void axisRequestedStateCallback(std_msgs::msg::Int32::UniquePtr state);
};
};  // namespace odesc_driver

#endif  // ODESC_DRIVER_AXIS_NODE_HPP