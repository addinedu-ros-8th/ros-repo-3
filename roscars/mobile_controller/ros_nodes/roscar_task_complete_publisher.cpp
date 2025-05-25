#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/task_complete.hpp"

class TaskCompletePublisher : public rclcpp::Node
{
public:
  TaskCompletePublisher()
  : Node("task_complete_publisher")
  {
    publisher_ = this->create_publisher<shared_interfaces::msg::TaskComplete>(
      "/roscar/task/complete", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TaskCompletePublisher::publish_complete, this));
  }

private:
  void publish_complete()
  {
    auto msg = shared_interfaces::msg::TaskComplete();
    msg.roscar_id = 1;    // TODO
    msg.task_id   = 42;   // TODO
    msg.is_success   = true; // TODO
    msg.result_code = "hello"; // TODO
    msg.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    publisher_->publish(msg);
  }

  rclcpp::Publisher<shared_interfaces::msg::TaskComplete>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskCompletePublisher>());
  rclcpp::shutdown();
  return 0;
}
