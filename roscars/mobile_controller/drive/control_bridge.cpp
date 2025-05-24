// File: drive/control_bridge.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <functional>

using std::placeholders::_1;

class ControlBridge : public rclcpp::Node {
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

public:
  ControlBridge()
  : Node("control_bridge")
  {
    // reliable QoS 설정
    rclcpp::QoS reliable_qos(10);
    reliable_qos.reliable();

    pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", reliable_qos);
    RCLCPP_INFO(get_logger(), "[Init] Publisher to /cmd_vel");

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/robot/control_cmd", reliable_qos,
      std::bind(&ControlBridge::onControlCmd, this, _1));
    RCLCPP_INFO(get_logger(), "[Init] Subscription to /robot/control_cmd");
  }

private:
  void onControlCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(),
      "[Bridge RX] lin=%.2f, ang=%.2f", msg->linear.x, msg->angular.z);
    pub_->publish(*msg);
    RCLCPP_INFO(get_logger(),
      "[Bridge TX] published lin=%.2f, ang=%.2f", msg->linear.x, msg->angular.z);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlBridge>());
  rclcpp::shutdown();
  return 0;
}
