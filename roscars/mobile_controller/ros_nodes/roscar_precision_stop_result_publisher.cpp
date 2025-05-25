#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/precision_stop_result.hpp"

class PrecisionStopResultPublisher : public rclcpp::Node
{
public:
  PrecisionStopResultPublisher()
  : Node("precision_stop_result_publisher")
  {
    publisher_ = this->create_publisher<shared_interfaces::msg::PrecisionStopResult>(
      "/roscar/precision_stop/result", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PrecisionStopResultPublisher::publish_result, this));
  }

private:
  void publish_result()
  {
    auto msg = shared_interfaces::msg::PrecisionStopResult();
    msg.roscar_id = 1;      // TODO
    msg.success   = true;   // TODO
    msg.deviation = 0.05f;  // TODO (m 단위)
    msg.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    publisher_->publish(msg);
  }

  rclcpp::Publisher<shared_interfaces::msg::PrecisionStopResult>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrecisionStopResultPublisher>());
  rclcpp::shutdown();
  return 0;
}
