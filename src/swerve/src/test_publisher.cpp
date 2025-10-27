#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class TestPublisher : public rclcpp::Node {
   public:
    TestPublisher() : Node("swerve_test_publisher") {
        joySub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&TestPublisher::joyCallback, this, std::placeholders::_1));

        velPub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_velocity_controller/commands", 10);
        posPub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_position_controller/commands", 10);

        RCLCPP_INFO(this->get_logger(), "SET UP COMPLETE");
    }

   private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velPub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr posPub;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "RECEIVED MESSAGE");
        auto velMsg = std_msgs::msg::Float64MultiArray();
        auto posMsg = std_msgs::msg::Float64MultiArray();

        // velMsg.data = (msg->axes[5] - 1) * 10;
        // posMsg.data = msg->axes[0] * 1.5;

        velMsg.data.push_back((msg->axes[5] - 1) * 40);
        posMsg.data.push_back(msg->axes[0] * 1.5);

        // RCLCPP_INFO(this->get_logger(), "SENDING COMMANDS: %f, %f", velMsg.data, posMsg.data);

        velPub->publish(velMsg);
        posPub->publish(posMsg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPublisher>());
    rclcpp::shutdown();
    return 0;
}