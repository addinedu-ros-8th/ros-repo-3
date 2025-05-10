#include <chrono>  // std::chrono를 사용하기 위해 추가
#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/battery_status.hpp"
#include "battery.hpp"
#include <iostream>
#include <fstream>

std::string get_ap_ssid() {
    std::ifstream hostapd_file("/etc/hostapd/hostapd.conf");
    std::string line;
    while (std::getline(hostapd_file, line)) {
        if (line.find("ssid=") == 0) {
            return line.substr(5);  // "ssid=" 이후 부분만 반환
        }
    }
    return "UNKNOWN_SSID";
}

class BatteryStatusPublisher : public rclcpp::Node {
public:
    BatteryStatusPublisher() : Node("battery_status_publisher"), battery_() {
        // SSID를 기반으로 네임스페이스 생성
        std::string ssid = get_ap_ssid();
        std::string topic_name = "/" + ssid + "/roscar/status/battery";
        
        publisher_ = this->create_publisher<shared_interfaces::msg::BatteryStatus>(topic_name, 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&BatteryStatusPublisher::publish_status, this));
    }

private:
    void publish_status() {
        float percentage_float = battery_.get_battery();
        int percentage = static_cast<int>(roundf(percentage_float));  // float → int 변환

        std::string ssid = get_ap_ssid();

        shared_interfaces::msg::BatteryStatus msg;
        msg.percentage = percentage;
        msg.roscar_name = ssid;

        RCLCPP_INFO(this->get_logger(), "Battery Status → %d%% (SSID: %s)", msg.percentage, ssid.c_str());
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
