// ros_nodes/roscar_pressure_relay.cpp

#include <memory>
#include <string>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

// ─────────────────────────────────────
// 호스트 AP SSID를 읽어오는 함수
// ─────────────────────────────────────
std::string get_ap_ssid() {
  std::ifstream hostapd_file("/etc/hostapd/hostapd.conf");
  std::string line;
  while (std::getline(hostapd_file, line)) {
    if (line.rfind("ssid=", 0) == 0) {
      return line.substr(5);  // "ssid=" 이후 문자열
    }
  }
  return "UNKNOWN_SSID";
}

// ─────────────────────────────────────
// PressureRelay 클래스
// ─────────────────────────────────────
class PressureRelay : public rclcpp::Node {
public:
  explicit PressureRelay(const std::string & ssid)
  : Node("pressure_relay")
  {
    subscribe_topic_ = "/isOn";
    publish_topic_ = "/" + ssid + "/isOn";

    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      subscribe_topic_, 10,
      std::bind(&PressureRelay::topic_callback, this, std::placeholders::_1)
    );

    publisher_ = this->create_publisher<std_msgs::msg::Int32>(
      publish_topic_, 10
    );

    RCLCPP_INFO(this->get_logger(),
      "[Relay 시작] 수신: '%s' → 발행: '%s'",
      subscribe_topic_.c_str(), publish_topic_.c_str());
  }

private:
  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
      "[중계] 수신한 값: %d → %s", msg->data, publish_topic_.c_str());
    publisher_->publish(*msg);
  }

  std::string subscribe_topic_;
  std::string publish_topic_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};

// ─────────────────────────────────────
// main()
// ─────────────────────────────────────
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  std::string ssid = get_ap_ssid();
  RCLCPP_INFO(rclcpp::get_logger("main"), "SSID 감지됨: '%s'", ssid.c_str());

  auto node = std::make_shared<PressureRelay>(ssid);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
