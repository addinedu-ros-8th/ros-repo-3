#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/charge_command.hpp"

class ChargeCommandSubscriber : public rclcpp::Node
{
public:
  ChargeCommandSubscriber()
  : Node("charge_command_subscriber")
  {
    subscription_ = this->create_subscription<shared_interfaces::msg::ChargeCommand>(
      "/roscar/status/charge", 10,
      std::bind(&ChargeCommandSubscriber::on_charge_cmd, this, std::placeholders::_1));
  }

private:
  void on_charge_cmd(const shared_interfaces::msg::ChargeCommand::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "[ChargeCmd] id=%u, x=%.2f, y=%.2f, θ=%.2f, stamp=(%u,%u)",
      msg->roscar_id, msg->goal_x, msg->goal_y, msg->theta,
      msg->stamp.sec, msg->stamp.nanosec);
    // TODO: 충전 위치 이동 로직 추가
  }

  rclcpp::Subscription<shared_interfaces::msg::ChargeCommand>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChargeCommandSubscriber>());
  rclcpp::shutdown();
  return 0;
}
