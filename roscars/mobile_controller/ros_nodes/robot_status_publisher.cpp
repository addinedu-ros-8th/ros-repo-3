#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/robot_status.hpp"
#include "mobile_controller/utils.hpp"  // 추가: utils 사용
#include <fstream>

using RobotStatus = shared_interfaces::msg::RobotStatus;
using namespace std::chrono_literals;
using namespace mobile_controller;

class RobotStatusPublisher : public rclcpp::Node
{
public:
    RobotStatusPublisher() : Node("robot_status_publisher")
    {
        publisher_ = this->create_publisher<RobotStatus>("/robot_status_response", 10);
        timer_ = this->create_wall_timer(
            3s,
            std::bind(&RobotStatusPublisher::publish_status, this)
        );
    }
    std::string get_ap_ssid()
    {
        std::ifstream hostapd_file("/etc/hostapd/hostapd.conf");
        std::string line;
        while (std::getline(hostapd_file, line))
        {
            if (line.find("ssid=") == 0)
            {
                return line.substr(5);  // "ssid=" 이후 부분만
            }
        }
        return "UNKNOWN_SSID";
    }

private:
    
    void publish_status()
    {
        auto msg = RobotStatus();
        msg.roscar_id = 1;
        msg.roscar_name = get_ap_ssid();  // ← 여기 수정
        msg.battery_percentage = 100;
        msg.operational_status = "ACTIVE";
        msg.roscar_ip_v4 = get_ip_address("wlan0");

        RCLCPP_INFO(this->get_logger(), "Publishing RobotStatus: name=%s, ip=%s",
                    msg.roscar_name.c_str(), msg.roscar_ip_v4.c_str());
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
