#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/obstacle_response.hpp"

class ObstacleResponseSubscriber : public rclcpp::Node
{
public:
  ObstacleResponseSubscriber()
  : Node("obstacle_response_subscriber")
  {
    subscription_ = this->create_subscription<shared_interfaces::msg::ObstacleResponse>(
      "/roscar/obstacle/response", 10,
      std::bind(&ObstacleResponseSubscriber::on_response, this, std::placeholders::_1));
  }

private:
  void on_response(const shared_interfaces::msg::ObstacleResponse::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "[ObsResp] id=%u, cmd=0x%02X, stamp=(%u,%u)",
      msg->roscar_id, msg->command,
      msg->stamp.sec, msg->stamp.nanosec);
    // TODO: 정지/회피 처리 로직 추가
  }

  rclcpp::Subscription<shared_interfaces::msg::ObstacleResponse>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleResponseSubscriber>());
  rclcpp::shutdown();
  return 0;
}
