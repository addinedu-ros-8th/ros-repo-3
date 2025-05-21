#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/ultra_status.hpp"
#include <fstream>
#include "ultra_sensor.hpp"

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
        
        // C++14 이전 환경에서도 동작할 수 있도록 수정
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&UltraPublisher::read_and_publish, this));

        ultra_sensor_ = std::make_shared<UltraSensor>();
    }

private:
    std::string get_ap_ssid() {
        std::ifstream hostapd_file("/etc/hostapd/hostapd.conf");
        std::string line;
        while (std::getline(hostapd_file, line)) {
            if (line.find("ssid=") == 0) {
                return line.substr(5); // SSID 추출
            }
        }
        return "UNKNOWN_SSID"; // SSID를 찾지 못했을 경우
    }

    void read_and_publish() {
        // 초음파 센서로부터 거리 읽기
        float distance = ultra_sensor_->read_distance();

        // 메시지로 저장
        auto msg = shared_interfaces::msg::UltraStatus();
        msg.distance = distance;
        
        // SSID를 msg.roscar_name에 저장
        msg.roscar_name = get_ap_ssid(); // SSID 정보 저장

        // 데이터를 publish하기 전에 출력시 소수점 2자리로 포맷팅
        RCLCPP_INFO(this->get_logger(), "Distance: %.2f cm, SSID: %s", msg.distance, msg.roscar_name.c_str());

        publisher_->publish(msg); // 데이터 전송
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<shared_interfaces::msg::UltraStatus>::SharedPtr publisher_;
    std::shared_ptr<UltraSensor> ultra_sensor_;
};

}  // namespace ultra_publisher

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ultra_publisher::UltraPublisher>());
    rclcpp::shutdown();
    return 0;
}