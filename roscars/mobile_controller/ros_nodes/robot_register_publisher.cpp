#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/robot_register.hpp"
#include "mobile_controller/utils.hpp"
#include <fstream>
#include <iostream>

using RobotRegister = shared_interfaces::msg::RobotRegister;
using namespace std::chrono_literals;
using namespace mobile_controller;

namespace robot_register_publisher {

class RobotRegisterPublisher : public rclcpp::Node
{
public:
    RobotRegisterPublisher() : Node("robot_register_publisher")
    {
        // SSID를 기반으로 토픽 네임스페이스 설정
        std::string ssid = get_ap_ssid();
        std::string topic_name = "/" + ssid + "/robot/register";

        // 로깅을 통해 topic_name 확인
        RCLCPP_INFO(this->get_logger(), "Using topic: %s", topic_name.c_str());

        publisher_ = this->create_publisher<RobotRegister>(topic_name, 10);
        timer_ = this->create_wall_timer(
            3s,
            std::bind(&RobotRegisterPublisher::publish_status, this)
        );
    }

    std::string get_ap_ssid()
    {
        std::ifstream hostapd_file("/etc/hostapd/hostapd.conf");
        std::string line;
        bool found_ssid = false;
        while (std::getline(hostapd_file, line))
        {
            if (line.find("ssid=") == 0)
            {
                found_ssid = true;
                RCLCPP_INFO(this->get_logger(), "Found SSID: %s", line.substr(5).c_str());
                return line.substr(5);
            }
        }

        // SSID를 찾지 못한 경우 로깅을 추가하고 기본값 반환
        RCLCPP_WARN(this->get_logger(), "SSID not found, returning default value.");
        return "UNKNOWN_SSID";
    }

private:
    void publish_status()
    {
        auto msg = RobotRegister();
        msg.roscar_name = get_ap_ssid();  // SSID 가져오기
        msg.battery_percentage = 100;
        msg.operational_status = "STANDBY";
        msg.roscar_ip_v4 = get_ip_address("wlan0");

        RCLCPP_INFO(this->get_logger(), "Publishing RobotRegister: name=%s, ip=%s",
                    msg.roscar_name.c_str(), msg.roscar_ip_v4.c_str());
        publisher_->publish(msg);
    }

    rclcpp::Publisher<RobotRegister>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_register_publisher

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_register_publisher::RobotRegisterPublisher>());
    rclcpp::shutdown();
    return 0;
}
