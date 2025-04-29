// 파일 위치: mobile_controller/ros_nodes/robot_status_publisher.cpp

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RobotStatusPublisher : public rclcpp::Node
{
public:
    RobotStatusPublisher()
    : Node("robot_status_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/robot_status", 10);

        // 1초마다 publish
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RobotStatusPublisher::publish_status, this)
        );
    }

private:
    void publish_status()
    {
        auto message = std_msgs::msg::String();
        message.data = "robot_id: pinky, ip: 192.168.0.44";

        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
