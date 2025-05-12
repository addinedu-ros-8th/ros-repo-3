#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/battery_status.hpp"  // ✅ 명세 맞게 메시지 확인 필요
#include "battery.hpp"
#include <iostream>
#include <fstream>
#include <regex>

std::string get_ap_ssid() {
    std::ifstream hostapd_file("/etc/hostapd/hostapd.conf");
    std::string line;
    while (std::getline(hostapd_file, line)) {
        if (line.find("ssid=") == 0) {
            return line.substr(5);
        }
    }
    return "UNKNOWN_SSID";
}

// 숫자 ID를 SSID에서 추출 (예: pinky_07db → 7)
uint8_t extract_robot_id(const std::string& ssid) {
    std::regex re("\\d+");
    std::smatch match;
    if (std::regex_search(ssid, match, re)) {
        return static_cast<uint8_t>(std::stoi(match.str()));
    }
    return 0;
}

class BatteryStatusPublisher : public rclcpp::Node {
public:
    BatteryStatusPublisher() : Node("battery_status_publisher"), battery_() {
        std::string ssid = get_ap_ssid();
        std::string topic_name = "/" + ssid + "/roscar/status/battery";  // ✅ topic path 수정

        publisher_ = this->create_publisher<shared_interfaces::msg::BatteryStatus>(topic_name, 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&BatteryStatusPublisher::publish_status, this));
    }

private:
    void publish_status() {
        std::string ssid = get_ap_ssid();
        uint8_t robot_id = extract_robot_id(ssid);  // SSID에서 ID 추출
        float battery_percent = battery_.get_battery();  // 0.0 ~ 100.0

        shared_interfaces::msg::BatteryStatus msg;
        msg.robot_id = robot_id;
        msg.battery_percent = battery_percent;
        msg.is_charging = false;  // ✅ 센서 연동 시 갱신
        msg.stamp = this->get_clock()->now();  // 현재 타임스탬프 설정

        RCLCPP_INFO(this->get_logger(), "[%s] 배터리 잔량: %.1f%%", ssid.c_str(), battery_percent);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<shared_interfaces::msg::BatteryStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    Battery battery_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryStatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
