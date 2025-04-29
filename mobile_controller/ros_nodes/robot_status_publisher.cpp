#include <memory>
#include <fstream>
#include <string>
#include <iostream>
#include <cstdio>
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

        // AP 모드에서 SSID를 가져오는 함수 호출
        std::string robot_id = get_ap_ssid();
        
        // 로봇의 IP 주소를 가져오는 함수 호출
        std::string ip = get_ip_address();

        // 메시지 데이터 구성
        message.data = "robot_id: " + robot_id + ", ip: " + ip;

        // 로그 출력
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

        // 메시지 발행
        publisher_->publish(message);
    }

    // AP 모드에서 SSID를 가져오는 함수
    std::string get_ap_ssid(const std::string& config_path = "/etc/hostapd/hostapd.conf")
    {
        std::ifstream file(config_path);
        std::string line;
        std::string ssid = "unknown";

        if (!file.is_open()) {
            return ssid;
        }

        while (std::getline(file, line)) {
            if (line.find("ssid=") == 0) {
                ssid = line.substr(5);  // "ssid=" 이후 문자열 추출
                break;
            }
        }

        return ssid;
    }

    // 로봇의 IP 주소를 가져오는 함수
    std::string get_ip_address()
    {
        FILE* fp = popen("hostname -I", "r");
        if (fp == nullptr) {
            return "unknown";
        }

        char buffer[128];
        std::string ip_address = "";
        while (fgets(buffer, sizeof(buffer), fp) != nullptr) {
            ip_address += buffer;
        }
        fclose(fp);

        // IP 주소 앞뒤 공백 제거
        ip_address.erase(0, ip_address.find_first_not_of(" \t\r\n"));
        ip_address.erase(ip_address.find_last_not_of(" \t\r\n") + 1);

        return ip_address.empty() ? "unknown" : ip_address;
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
