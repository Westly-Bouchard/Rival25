#include "../include/odesc_driver/axis_node.hpp"

#include <format>
#include <optional>

using namespace std;

namespace odesc_driver {

AxisNode::AxisNode()
    : Node("odesc_axis_node"),
      AxisTrxInterface(),
      AxisNodePublisher(static_cast<rclcpp::Node&>(*this)) {
    this->declare_parameter("can_id", 0);
    this->declare_parameter("can_interface", "can0");

    auto paramsResult = setParams(this->get_parameter("can_id").as_int(),
                                  this->get_parameter("can_interface").as_string());

    if (paramsResult != nullopt) {
        RCLCPP_ERROR_STREAM(this->get_logger(), paramsResult.value());
        return;
    }

    createPublishers(id);

    auto connectResult = connect();

    if (connectResult != nullopt) {
        RCLCPP_ERROR_STREAM(this->get_logger(), connectResult.value());
    }

    RCLCPP_INFO(this->get_logger(), "Successfully connected to axis %i", id);

    axisRequestedStateSub = create_subscription<std_msgs::msg::Int32>(
        format("odesc{}/axisRequestedState", id), 10,
        bind(&AxisNode::axisRequestedStateCallback, this, placeholders::_1));
}

void AxisNode::node_publish_func(MsgType type) {
    switch (type) {
        case MsgType::HEARTBEAT:
            publishAxisState(getAxisCurrentState());
            break;
        case MsgType::GET_ENCODER_ESTIMATES:
            publishEncoderEstimates(getEncoderPosEstimate(), getEncoderVelEstimate());
            break;
        case MsgType::GET_ENCODER_COUNT:
            publishEncoderCount(getEncoderCountCPR());
            break;
    }
}

void AxisNode::axisRequestedStateCallback(std_msgs::msg::Int32::UniquePtr msg) {
    optional<string> result = setAxisRequestedState(msg->data);

    if (result != nullopt) {
        RCLCPP_ERROR_STREAM(this->get_logger(), result.value());
    }
}

};  // namespace odesc_driver

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<odesc_driver::AxisNode>();
    if (node->getError() != nullopt) {
        rclcpp::shutdown();
        return 0;
    }
    rclcpp::spin(node);
    node->shutdown();
    rclcpp::shutdown();
    return 0;
}