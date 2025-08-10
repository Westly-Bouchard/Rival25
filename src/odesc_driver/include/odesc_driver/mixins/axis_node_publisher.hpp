#ifndef ODESC_DRIVER_AXIS_NODE_PUBLISHER_HPP
#define ODESC_DRIVER_AXIS_NODE_PUBLISHER_HPP

#include <format>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

namespace odesc_driver {

class AxisNodePublisher {
   public:
    AxisNodePublisher(rclcpp::Node& node) : node(node) {}

    AxisNodePublisher(const AxisNodePublisher&) = delete;

   protected:
    void createPublishers(int id) {
        axisStatePub =
            node.create_publisher<std_msgs::msg::Int32>(std::format("odescAxis{}/state", id), 10);

        encoderPosPub = node.create_publisher<std_msgs::msg::Float64>(
            std::format("odescAxis{}/encoderPos", id), 10);
        encoderVelPub = node.create_publisher<std_msgs::msg::Float64>(
            std::format("odescAxis{}/encoderVel", id), 10);

        encoderCountPub = node.create_publisher<std_msgs::msg::Int32>(
            std::format("odescAxis{}/encoderCountCPR"), 10);
    }

    void publishAxisState(int state) {
        auto msg = std_msgs::msg::Int32();
        msg.data = state;
        axisStatePub->publish(msg);
    }

    void publishEncoderEstimates(double pos, double vel) {
        auto posMsg = std_msgs::msg::Float64();
        posMsg.data = pos;
        encoderPosPub->publish(posMsg);

        auto velMsg = std_msgs::msg::Float64();
        velMsg.data = vel;
        encoderVelPub->publish(velMsg);
    }

    void publishEncoderCount(int count) {
        auto msg = std_msgs::msg::Int32();
        msg.data = count;
        encoderCountPub->publish(msg);
    }

   private:
    rclcpp::Node& node;

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> axisStatePub;

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> encoderPosPub;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> encoderVelPub;

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> encoderCountPub;
};
};  // namespace odesc_driver

#endif  // ODESC_DRIVER_AXIS_NODE_PUBLISHER_HPP