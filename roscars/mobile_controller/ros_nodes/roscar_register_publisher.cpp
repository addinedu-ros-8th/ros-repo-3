#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/roscar_register.hpp"
#include "mobile_controller/utils.hpp"
#include <fstream>
#include <iostream>
#include <cstdlib>  // 환경 변수 접근을 위한 헤더

using RoscarRegister = shared_interfaces::msg::RoscarRegister;
using namespace std::chrono_literals;
using namespace mobile_controller;

namespace roscar_register_publisher {

class RoscarRegisterPublisher : public rclcpp::Node
{
public:
    RoscarRegisterPublisher() : Node("roscar_register_publisher")
    {
        // SSID를 기반으로 토픽 네임스페이스 설정
        std::string ssid = get_ap_ssid();
        std::string topic_name = "/" + ssid + "/roscar/register";

        // 로깅을 통해 topic_name 확인
        RCLCPP_INFO(this->get_logger(), "Using topic: %s", topic_name.c_str());

        publisher_ = this->create_publisher<RoscarRegister>(topic_name, 10);
        timer_ = this->create_wall_timer(
            3s,
            std::bind(&RoscarRegisterPublisher::publish_status, this)
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

    unsigned char get_domain_id()
    {
        const char* domain_id = std::getenv("ROS_DOMAIN_ID");  // ROS_DOMAIN_ID 환경 변수
        if (domain_id)
        {
            try
            {
                RCLCPP_INFO(this->get_logger(), "Found ROS_DOMAIN_ID: %s", domain_id);
                return static_cast<unsigned char>(std::stoi(domain_id));  // 문자열을 unsigned char로 변환
            }
            catch (const std::invalid_argument& e)
            {
                RCLCPP_WARN(this->get_logger(), "Invalid ROS_DOMAIN_ID format, using default.");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "ROS_DOMAIN_ID not found, returning default value.");
        }
        return 0;  // 기본값 0
    }

private:
    void publish_status()
    {
        auto msg = RoscarRegister();
        msg.roscar_name = get_ap_ssid();  // SSID 가져오기
        msg.battery_percentage = 100;
        msg.operational_status = "STANDBY";
        msg.roscar_ip_v4 = get_ip_address("wlan0");
        msg.from_domain_id = get_domain_id();  // 도메인 ID 가져오기
        msg.to_domain_id = 25;

        RCLCPP_INFO(this->get_logger(), "Publishing roscarRegister: name=%s, ip=%s, from_domain_id=%u",
                    msg.roscar_name.c_str(), msg.roscar_ip_v4.c_str(), msg.from_domain_id);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<RoscarRegister>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace roscar_register_publisher

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<roscar_register_publisher::RoscarRegisterPublisher>());
    rclcpp::shutdown();
    return 0;
}
