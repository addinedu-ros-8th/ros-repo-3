#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/battery_status.hpp"
#include "battery.hpp"
#include <iostream>
#include <fstream>
#include "rclcpp/qos.hpp"

using std::placeholders::_1;

std::string get_ap_ssid() {
    std::ifstream hostapd_file("/etc/hostapd/hostapd.conf");
    std::string line;
    while (std::getline(hostapd_file, line)) {
        if (line.find("ssid=") == 0) {
            return line.substr(5);  // "ssid=" 이후 문자열
        }
    }
    return "UNKNOWN_SSID";
}

class BatteryStatusPublisher : public rclcpp::Node {
public:
    BatteryStatusPublisher() : Node("battery_status_publisher"), battery_() {
        std::string ssid = get_ap_ssid();
        std::string topic_name = "/" + ssid + "/roscar/status/battery";

        // ✅ QoS 설정
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_profile.keep_last(10);
        qos_profile.reliable();

        publisher_ = this->create_publisher<shared_interfaces::msg::BatteryStatus>(
            topic_name,
            qos_profile
        );

        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&BatteryStatusPublisher::publish_status, this)
        );

        RCLCPP_INFO(this->get_logger(), "BatteryStatusPublisher initialized for topic: %s", topic_name.c_str());
    }

private:
    void publish_status() {
        std::string ssid = get_ap_ssid();
        float battery_percent = battery_.get_battery();

        shared_interfaces::msg::BatteryStatus msg;
        msg.robot_name = ssid;  // ✅ SSID 직접 포함
        msg.battery_percent = battery_percent;
        msg.is_charging = false;  // 필요시 실제 센서 연결
        msg.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();

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
