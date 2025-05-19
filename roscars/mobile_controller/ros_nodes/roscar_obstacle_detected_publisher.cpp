#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/obstacle_detected.hpp"

class ObstacleDetectedPublisher : public rclcpp::Node
{
public:
  ObstacleDetectedPublisher()
  : Node("obstacle_detected_publisher")
  {
    publisher_ = this->create_publisher<shared_interfaces::msg::ObstacleDetected>(
      "/roscar/obstacle/detected", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(300),
      std::bind(&ObstacleDetectedPublisher::publish_detected, this));
  }

private:
  void publish_detected()
  {
    auto msg = shared_interfaces::msg::ObstacleDetected();
    msg.roscar_id = 1;      // TODO
    msg.distance  = 0.5f;   // TODO (m 단위)
    msg.direction = 90.0f;  // TODO (deg)
    msg.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    publisher_->publish(msg);
  }

  rclcpp::Publisher<shared_interfaces::msg::ObstacleDetected>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectedPublisher>());
  rclcpp::shutdown();
  return 0;
}
