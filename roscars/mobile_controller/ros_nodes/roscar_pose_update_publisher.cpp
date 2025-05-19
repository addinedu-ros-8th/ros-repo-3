#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/roscar_pose_update.hpp"

class PoseUpdatePublisher : public rclcpp::Node
{
public:
  PoseUpdatePublisher()
  : Node("pose_update_publisher")
  {
    publisher_ = this->create_publisher<shared_interfaces::msg::RoscarPoseUpdate>(
      "/roscar/pose/update", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PoseUpdatePublisher::publish_pose, this));
  }

private:
  void publish_pose()
  {
    auto msg = shared_interfaces::msg::RoscarPoseUpdate();
    msg.roscar_id = 1;            // TODO: 실제 ID로 교체
    msg.pos_x = 0.0f;            // TODO: 실제 좌표로 교체
    msg.pos_y = 0.0f;
    msg.heading_theta = 0.0f;
    msg.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    publisher_->publish(msg);
  }

  rclcpp::Publisher<shared_interfaces::msg::RoscarPoseUpdate>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseUpdatePublisher>());
  rclcpp::shutdown();
  return 0;
}
