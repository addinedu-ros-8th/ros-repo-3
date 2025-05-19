#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/navigation_goal.hpp"

class NavigationGoalSubscriber : public rclcpp::Node
{
public:
  NavigationGoalSubscriber()
  : Node("navigation_goal_subscriber")
  {
    subscription_ = this->create_subscription<shared_interfaces::msg::NavigationGoal>(
      "/roscar/navigation/goal", 10,
      std::bind(&NavigationGoalSubscriber::on_nav_goal, this, std::placeholders::_1));
  }

private:
  void on_nav_goal(const shared_interfaces::msg::NavigationGoal::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "[NavGoal] id=%u, x=%.2f, y=%.2f, θ=%.2f, stamp=(%u,%u)",
      msg->roscar_id, msg->goal_x, msg->goal_y, msg->theta,
      msg->stamp.sec, msg->stamp.nanosec);
    // TODO: 경로 이동 로직 추가
  }

  rclcpp::Subscription<shared_interfaces::msg::NavigationGoal>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationGoalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
