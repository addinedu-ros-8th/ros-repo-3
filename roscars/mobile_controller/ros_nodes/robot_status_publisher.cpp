#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/robot_status.hpp"

using std::placeholders::_1;
using RobotStatus = shared_interfaces::msg::RobotStatus;

class RobotStatusPublisher : public rclcpp::Node
{
public:
    RobotStatusPublisher() : Node("robot_status_publisher")
    {
        publisher_ = this->create_publisher<RobotStatus>("/robot_status_response", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&RobotStatusPublisher::publish_status, this)
        );
    }

private:
    void publish_status()
    {
        auto msg = RobotStatus();
        msg.roscar_id = 1;
        msg.roscar_name = "robot_001";
        msg.battery_percentage = 88;
        msg.operational_status = "ACTIVE";
        msg.roscar_ip_v4 = "192.168.0.101";

        RCLCPP_INFO(this->get_logger(), "Publishing RobotStatus: %s", msg.roscar_name.c_str());
        publisher_->publish(msg);
    }

    rclcpp::Publisher<RobotStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
