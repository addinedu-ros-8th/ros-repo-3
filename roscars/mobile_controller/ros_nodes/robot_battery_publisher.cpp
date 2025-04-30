#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"  // 배터리 퍼센트를 나타내는 메시지
#include <fstream>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <vector>
#include <cstdint>  // uint8_t, uint16_t 타입을 사용하기 위한 헤더

using namespace std::chrono_literals;

class Battery {
public:
    Battery(int min_value = 2396, int max_value = 3215)
        : min_value(min_value), max_value(max_value) {

        // I2C bus 열기
        if ((file = open(bus, O_RDWR)) < 0) {
            std::cerr << "I2C bus open failed\n";
            exit(1);
        }

        // ADS1115 주소 설정
        int addr = 0x48; // 기본 주소
        if (ioctl(file, I2C_SLAVE, addr) < 0) {
            std::cerr << "Failed to connect to the device\n";
            exit(1);
        }
    }

    ~Battery() {
        // 자원 해제
        close(file);
    }

    float calculate_percentage(int value) {
        if (value < min_value)
            return 0.0f;
        else if (value > max_value)
            return 100.0f;
        else
            return (float)(value - min_value) / (max_value - min_value) * 100.0f;
    }

    float get_battery(int sample_count = 10) {
        std::vector<int> values;

        for (int i = 0; i < sample_count; i++) {
            int value = read_adc_value();
            values.push_back(value);
            usleep(100000);  // 0.1초 대기
        }

        // 평균값 계산
        int sum = 0;
        for (int v : values) {
            sum += v;
        }
        int avg_value = sum / values.size();

        // 퍼센트 계산
        return calculate_percentage(avg_value);
    }

private:
    int file;
    const char *bus = "/dev/i2c-1";
    int min_value, max_value;

    int read_adc_value() {
        // ADS1115에 읽기 명령 전송 (변환 시작)
        uint8_t config[3] = {0x01, 0xC3, 0x83};  // AIN0, 4.096V, 128SPS, single-shot
        write(file, config, 3);

        // 변환 대기 후 레지스터 포인터로 읽기
        uint8_t pointer[1] = {0x00};  // 데이터 레지스터 포인터
        write(file, pointer, 1);
        usleep(100000);  // 변환 대기

        uint8_t data[2];
        read(file, data, 2);

        // 16비트 데이터를 결합하여 반환
        int16_t raw = (data[0] << 8) | data[1];
        return raw;
    }
};

class BatteryStatusPublisher : public rclcpp::Node
{
public:
    BatteryStatusPublisher() : Node("battery_status_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/robot/status/battery", 10);
        timer_ = this->create_wall_timer(
            3s,
            std::bind(&BatteryStatusPublisher::publish_status, this)
        );
    }

private:
    void publish_status()
    {
        // 배터리 잔량 계산
        Battery battery;
        float battery_percentage = battery.get_battery();

        auto msg = std_msgs::msg::Float32();
        msg.data = battery_percentage;

        RCLCPP_INFO(this->get_logger(), "Publishing Battery Status: battery=%f%%", msg.data);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryStatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
