#include <chrono>
#include <fstream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// /etc/hostapd/hostapd.conf에서 ssid 값을 읽어옵니다.
static std::string get_ap_ssid() {
    std::ifstream hostapd_file("/etc/hostapd/hostapd.conf");
    std::string line;
    while (std::getline(hostapd_file, line)) {
        if (line.rfind("ssid=", 0) == 0) {
            return line.substr(5);
        }
    }
    return "UNKNOWN_SSID";
}

class SSIDPublisher : public rclcpp::Node {
public:
  SSIDPublisher()
  : Node("ssid_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/roscar/ssid", 10);

    timer_ = this->create_wall_timer(
      1s,
      std::bind(&SSIDPublisher::publish_ssid, this)
    );

    RCLCPP_INFO(this->get_logger(), "SSIDPublisher initialized, publishing to /roscar/ssid");
  }

private:
  void publish_ssid() {
    auto msg = std_msgs::msg::String();
    msg.data = get_ap_ssid();
    RCLCPP_INFO(this->get_logger(), "Publishing SSID: %s", msg.data.c_str());
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SSIDPublisher>());
  rclcpp::shutdown();
  return 0;
}