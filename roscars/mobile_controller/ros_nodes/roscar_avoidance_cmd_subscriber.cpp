#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/obstacle_avoidance_cmd.hpp"

class AvoidanceCmdSubscriber : public rclcpp::Node
{
public:
  AvoidanceCmdSubscriber()
  : Node("avoidance_cmd_subscriber")
  {
    subscription_ = this->create_subscription<shared_interfaces::msg::ObstacleAvoidanceCmd>(
      "/roscar/avoidance/cmd", 10,
      std::bind(&AvoidanceCmdSubscriber::on_avoid, this, std::placeholders::_1));
  }

private:
  void on_avoid(const shared_interfaces::msg::ObstacleAvoidanceCmd::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "[AvoidCmd] id=%u, dir=%.2f, speed=%.2f, stamp=(%u,%u)",
      msg->roscar_id, msg->direction, msg->speed,
      msg->stamp.sec, msg->stamp.nanosec);
    // TODO: 회피 제어 로직 추가
  }

  rclcpp::Subscription<shared_interfaces::msg::ObstacleAvoidanceCmd>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AvoidanceCmdSubscriber>());
  rclcpp::shutdown();
  return 0;
}
