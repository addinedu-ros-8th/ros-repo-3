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
        // 공통 등록 토픽 사용
        std::string topic_name = "/roscar/register";

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
        while (std::getline(hostapd_file, line))
        {
            if (line.find("ssid=") == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Found SSID: %s", line.substr(5).c_str());
                return line.substr(5);
            }
        }

        RCLCPP_WARN(this->get_logger(), "SSID not found, returning default value.");
        return "UNKNOWN_SSID";
    }

    unsigned char get_domain_id()
    {
        const char* domain_id = std::getenv("ROS_DOMAIN_ID");
        if (domain_id)
        {
            try
            {
                RCLCPP_INFO(this->get_logger(), "Found ROS_DOMAIN_ID: %s", domain_id);
                return static_cast<unsigned char>(std::stoi(domain_id));
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
        return 0;
    }

private:
    void publish_status()
    {
        auto msg = RoscarRegister();
        msg.roscar_name = get_ap_ssid();
        msg.battery_percentage = 100;
        msg.operational_status = "STANDBY";
        msg.roscar_ip_v4 = get_ip_address("wlan0");
        msg.from_domain_id = get_domain_id();
        msg.to_domain_id = 26;

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
