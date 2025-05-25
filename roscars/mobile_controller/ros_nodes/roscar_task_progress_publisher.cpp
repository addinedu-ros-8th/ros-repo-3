#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/task_progress.hpp"

class TaskProgressPublisher : public rclcpp::Node
{
public:
  TaskProgressPublisher()
  : Node("task_progress_publisher")
  {
    publisher_ = this->create_publisher<shared_interfaces::msg::TaskProgress>(
      "/roscar/task/progress", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&TaskProgressPublisher::publish_progress, this));
  }

private:
  void publish_progress()
  {
    auto msg = shared_interfaces::msg::TaskProgress();
    msg.roscar_id = 1;    // TODO
    msg.task_id   = 42;   // TODO
    msg.progress_percent  = 50;   // TODO
    msg.phase_description = "In Progress"; // TODO
    msg.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    publisher_->publish(msg);
  }

  rclcpp::Publisher<shared_interfaces::msg::TaskProgress>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskProgressPublisher>());
  rclcpp::shutdown();
  return 0;
}
