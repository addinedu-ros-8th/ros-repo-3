#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/dashboard_status.hpp"

class DashboardStatusSubscriber : public rclcpp::Node
{
public:
  DashboardStatusSubscriber()
  : Node("dashboard_status_subscriber")
  {
    subscription_ = this->create_subscription<shared_interfaces::msg::DashboardStatus>(
      "/dashboard/status/update", 10,
      std::bind(&DashboardStatusSubscriber::on_status_update, this, std::placeholders::_1));
  }

private:
  void on_status_update(const shared_interfaces::msg::DashboardStatus::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "[Dashboard] id=%u, task=%u, x=%.2f, y=%.2f, stamp=(%u,%u)",
      msg->roscar_id, msg->task_id, msg->pose_x, msg->pose_y,
      msg->stamp.sec, msg->stamp.nanosec);
    // TODO: 대시보드 갱신 로직 추가
  }

  rclcpp::Subscription<shared_interfaces::msg::DashboardStatus>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DashboardStatusSubscriber>());
  rclcpp::shutdown();
  return 0;
}
