#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/ultra_status.hpp"
#include <cmath>
#include <gpiod.h>
#include <fstream>
#include <string>

using namespace std::chrono_literals;

namespace ultra_publisher {

class UltraPublisher : public rclcpp::Node {
public:
    UltraPublisher()
        : Node("ultra_publisher")
    {
        // SSID를 기반으로 네임스페이스 적용
        std::string ssid = get_ap_ssid();
        std::string topic_name = "/" + ssid + "/roscar/sensor/ultra";

        // 로깅을 통해 topic_name 확인
        RCLCPP_INFO(this->get_logger(), "Using topic name: %s", topic_name.c_str());

        publisher_ = this->create_publisher<shared_interfaces::msg::UltraStatus>(topic_name, 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&UltraPublisher::read_and_publish, this));

        chip = gpiod_chip_open_by_name("gpiochip4"); // GPIO 23, 24 = BCM 기준 chip 4
        if (!chip) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open gpiochip4");
            rclcpp::shutdown();
            return;
        }

        trig_line = gpiod_chip_get_line(chip, 23); // TRIG = GPIO 23
        echo_line = gpiod_chip_get_line(chip, 24); // ECHO = GPIO 24

        gpiod_line_request_output(trig_line, "ultra_trig", 0);
        gpiod_line_request_input(echo_line, "ultra_echo");
    }

    ~UltraPublisher() {
        gpiod_line_release(trig_line);
        gpiod_line_release(echo_line);
        gpiod_chip_close(chip);
    }

private:
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

    void read_and_publish() {
        // 10us high trigger pulse
        gpiod_line_set_value(trig_line, 1);
        rclcpp::sleep_for(10us);
        gpiod_line_set_value(trig_line, 0);

        // Wait for echo start
        auto start = std::chrono::high_resolution_clock::now();
        while (gpiod_line_get_value(echo_line) == 0) {
            start = std::chrono::high_resolution_clock::now();
        }

        // Wait for echo end
        auto end = start;
        while (gpiod_line_get_value(echo_line) == 1) {
            end = std::chrono::high_resolution_clock::now();
        }

        std::chrono::duration<float> duration = end - start;
        float distance = (duration.count() * 34300.0) / 2.0; // cm 단위

        // 소수점 2자리까지 반올림하여 저장
        distance = std::round(distance * 100.0) / 100.0;  // 소수점 2자리 반올림

        // 메시지로 저장
        auto msg = shared_interfaces::msg::UltraStatus();
        msg.distance = distance;
        
        // SSID를 msg.roscar_name에 저장
        msg.roscar_name = get_ap_ssid(); // SSID 정보 저장

        // 데이터를 publish하기 전에 출력시 소수점 2자리로 포맷팅
        RCLCPP_INFO(this->get_logger(), "Distance: %.2f cm, SSID: %s", msg.distance, msg.roscar_name.c_str());

        publisher_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<shared_interfaces::msg::UltraStatus>::SharedPtr publisher_;

    struct gpiod_chip *chip;
    struct gpiod_line *trig_line;
    struct gpiod_line *echo_line;
};

}  // namespace ultra_publisher

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ultra_publisher::UltraPublisher>());
    rclcpp::shutdown();
    return 0;
}
