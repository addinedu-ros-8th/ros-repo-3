#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/battery_status.hpp"
#include "mobile_controller/utils.hpp"

#include <fstream>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <vector>
#include <cstdint>

using BatteryStatusMsg = shared_interfaces::msg::BatteryStatus;
using namespace std::chrono_literals;
using namespace mobile_controller;

class Battery {
public:
    Battery(int min_value = 2396, int max_value = 3215)
        : min_value(min_value), max_value(max_value) {

        if ((file = open(bus, O_RDWR)) < 0) {
            std::cerr << "I2C 버스 열기 실패\n";
            exit(1);
        }

        int addr = 0x48; // ADS1115 기본 주소
        if (ioctl(file, I2C_SLAVE, addr) < 0) {
            std::cerr << "ADS1115 디바이스 연결 실패\n";
            exit(1);
        }
    }

    ~Battery() {
        close(file);
    }

    float calculate_percentage(int value) {
        if (value < min_value)
            return 0.0f;
        else if (value > max_value)
            return 100.0f;
        else
            return static_cast<float>(value - min_value) / (max_value - min_value) * 100.0f;
    }

    float get_battery(int sample_count = 10) {
        std::vector<int> values;
        for (int i = 0; i < sample_count; i++) {
            values.push_back(read_adc_value());
            usleep(100000);  // 0.1초 대기
        }

        int sum = 0;
        for (int v : values) sum += v;
        int avg = sum / sample_count;

        return calculate_percentage(avg);
    }

private:
    int file;
    const char *bus = "/dev/i2c-1";
    int min_value, max_value;

    int read_adc_value() {
        uint8_t config[3] = {0x01, 0xC3, 0x83};  // 설정 레지스터
        write(file, config, 3);

        uint8_t pointer[1] = {0x00};
        write(file, pointer, 1);
        usleep(100000);

        uint8_t data[2];
        read(file, data, 2);

        return (data[0] << 8) | data[1];
    }
};

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
    BatteryStatusPublisher() : Node("battery_status_publisher") {
        // SSID를 기반으로 네임스페이스 생성
        std::string ssid = get_ap_ssid();
        std::string topic_name = "/" + ssid + "/robot/status/battery";
        
        publisher_ = this->create_publisher<BatteryStatusMsg>(topic_name, 10);
        timer_ = this->create_wall_timer(3s, std::bind(&BatteryStatusPublisher::publish_status, this));
    }

private:
    void publish_status() {
        Battery battery;
        float percentage_float = battery.get_battery();
        int percentage = static_cast<int>(roundf(percentage_float));  // float → int 변환

        std::string ssid = get_ap_ssid();

        BatteryStatusMsg msg;
        msg.percentage = percentage;
        msg.roscar_name = ssid;

        RCLCPP_INFO(this->get_logger(), "Battery Status → %d%% (SSID: %s)", msg.percentage, ssid.c_str());
        publisher_->publish(msg);
    }

    rclcpp::Publisher<BatteryStatusMsg>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryStatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
