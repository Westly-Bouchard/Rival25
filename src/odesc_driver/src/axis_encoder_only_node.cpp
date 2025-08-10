#include "../include/odesc_driver/axis_encoder_only_node.hpp"

#include <optional>

using namespace std;

namespace odesc_driver {

AxisEncoderOnlyNode::AxisEncoderOnlyNode()
    : Node("odesc_encoder_only_axis_node"),
      AxisEncoderOnlyInterface(),
      AxisNodePublisher(static_cast<rclcpp::Node&>(*this)) {
    this->declare_parameter("can_id", 0);
    this->declare_parameter("can_interface", "can0");

    auto paramsResult = setParams(this->get_parameter("can_id").as_int(),
                                  this->get_parameter("can_interface").as_string());

    if (paramsResult != nullopt) {
        RCLCPP_ERROR_STREAM(this->get_logger(), paramsResult.value());
        return;
    }

    auto connectResult = connect();

    if (connectResult != nullopt) {
        RCLCPP_ERROR_STREAM(this->get_logger(), connectResult.value());
    }
}

void AxisEncoderOnlyNode::node_publish_func(MsgType type) {
    switch (type) {
        case MsgType::HEARTBEAT:
            publishAxisState(getAxisCurrentState());
            break;
        case MsgType::GET_ENCODER_ESTIMATES:
            publishEncoderEstimates(getEncoderPosEstimate(), getEncoderVelEstimate());
            break;
    }
}
};  // namespace odesc_driver

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<odesc_driver::AxisEncoderOnlyNode>();
    rclcpp::spin(node);
    node->shutdown();
    rclcpp::shutdown();
    return 0;
}