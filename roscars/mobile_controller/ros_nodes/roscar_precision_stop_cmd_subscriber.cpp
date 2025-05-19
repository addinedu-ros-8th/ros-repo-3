#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/precision_stop_cmd.hpp"

class PrecisionStopCmdSubscriber : public rclcpp::Node
{
public:
  PrecisionStopCmdSubscriber()
  : Node("precision_stop_cmd_subscriber")
  {
    subscription_ = this->create_subscription<shared_interfaces::msg::PrecisionStopCmd>(
      "/roscar/precision_stop/cmd", 10,
      std::bind(&PrecisionStopCmdSubscriber::on_stop_cmd, this, std::placeholders::_1));
  }

private:
  void on_stop_cmd(const shared_interfaces::msg::PrecisionStopCmd::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "[PrecStopCmd] id=%u, x=%.2f, y=%.2f, stamp=(%u,%u)",
      msg->roscar_id, msg->target_x, msg->target_y,
      msg->stamp.sec, msg->stamp.nanosec);
    // TODO: 정밀 정지 로직 추가
  }

  rclcpp::Subscription<shared_interfaces::msg::PrecisionStopCmd>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrecisionStopCmdSubscriber>());
  rclcpp::shutdown();
  return 0;
}
