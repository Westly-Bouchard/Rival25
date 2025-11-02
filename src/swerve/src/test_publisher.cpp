#include <cmath>
#include <cstdlib>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class TestPublisher : public rclcpp::Node {
   public:
    TestPublisher() : Node("swerve_test_publisher") {
        count = 0;
        joySub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&TestPublisher::joyCallback, this, std::placeholders::_1));

        cmdPub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "SET UP COMPLETE");
    }

   private:
    int count;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPub;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->buttons[10 == 1]) {
            count++;

            if (count > 100) {
                int result = system("shutdown -h now");

                count = 0;
            }
        }
        auto cmdMsg = geometry_msgs::msg::Twist();

        if (fabs(msg->axes[0]) > 0.1) cmdMsg.linear.set__y(msg->axes[0] * 0.75);
        if (fabs(msg->axes[1]) > 0.1) cmdMsg.linear.set__x(msg->axes[1] * 0.75);
        if (fabs(msg->axes[2]) > 0.1) cmdMsg.angular.set__z(msg->axes[2]);

        cmdPub->publish(cmdMsg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPublisher>());
    rclcpp::shutdown();
    return 0;
}