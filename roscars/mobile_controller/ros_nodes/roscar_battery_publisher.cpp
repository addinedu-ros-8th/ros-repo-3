#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/battery_status.hpp"
#include "battery.hpp"
#include "rclcpp/qos.hpp"

class BatteryStatusPublisher : public rclcpp::Node {
public:
    BatteryStatusPublisher()
    : Node("battery_status_publisher")
    , battery_()
    {
        std::string topic_name = "/roscar/status/battery";

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
        const int SAMPLE_COUNT = 5;
        float current_percent = battery_.get_battery(SAMPLE_COUNT);

        // previous_percent 와 charging_state 를 파일 스코프(static)로 유지
        static float last_percent = current_percent;
        static bool charging_state = false;
        static int consec_up = 0;
        static int consec_down = 0;

        // 히스테리시스 임계치 (0.2% 상승 → charging, 0.2% 하락 → not charging)
        const float UP_THRESH   = 0.2f;
        const float DOWN_THRESH = -0.2f;
        // 연속 판정 횟수 (예: 2회 연속 조건 만족 시 상태 전환)
        const int   REQ_COUNT   = 2;

        float diff = current_percent - last_percent;
        last_percent = current_percent;

        if (diff > UP_THRESH) {
            consec_up++;
            consec_down = 0;
        } else if (diff < DOWN_THRESH) {
            consec_down++;
            consec_up = 0;
        } else {
            // 변화량이 임계치 사이에 있으면 카운터 리셋하지 않고 유지하거나, 
            // 완전히 리셋하고 싶으면 주석 해제:
            // consec_up = consec_down = 0;
        }

        // 연속 조건 만족 시에만 charging_state 토글
        if (consec_up >= REQ_COUNT) {
            charging_state = true;
            consec_up = 0;  // 또는 유지해도 무방
        } else if (consec_down >= REQ_COUNT) {
            charging_state = false;
            consec_down = 0;
        }

        // 메시지 구성
        shared_interfaces::msg::BatteryStatus msg;
        msg.battery_percent = current_percent;
        msg.is_charging    = charging_state;
        msg.stamp           = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        RCLCPP_INFO(this->get_logger(),
            "Battery: %.1f%% | Charging: %s  (Δ=%.2f, ups=%d, downs=%d)",
            current_percent,
            charging_state ? "true" : "false",
            diff,
            consec_up,
            consec_down
        );

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
