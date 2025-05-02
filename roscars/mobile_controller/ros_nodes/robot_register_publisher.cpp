#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/robot_register.hpp"
#include "mobile_controller/utils.hpp"
#include <fstream>

using RobotRegister = shared_interfaces::msg::RobotRegister;
using namespace std::chrono_literals;
using namespace mobile_controller;

class RobotRegisterPublisher : public rclcpp::Node
{
public:
    // 클래스 이름과 동일한 생성자 이름으로 수정
    RobotRegisterPublisher() : Node("robot_register_publisher")
    {
        publisher_ = this->create_publisher<RobotRegister>("/robot/register", 10);
        timer_ = this->create_wall_timer(
            3s,
            std::bind(&RobotRegisterPublisher::publish_status, this)
        );
    }

    std::string get_ap_ssid()
    {
        std::ifstream hostapd_file("/etc/hostapd/hostapd.conf");
        std::string line;
        while (std::getline(hostapd_file, line))
        {
            if (line.find("ssid=") == 0)
            {
                return line.substr(5);
            }
        }
        return "UNKNOWN_SSID";
    }

private:
    void publish_status()
    {
        auto msg = RobotRegister();
        msg.roscar_id = 1;
        msg.roscar_name = get_ap_ssid();
        msg.battery_percentage = 100;
        msg.operational_status = "STANDBY";
        msg.roscar_ip_v4 = get_ip_address("wlan0");

        RCLCPP_INFO(this->get_logger(), "Publishing RobotRegister: name=%s, ip=%s",
                    msg.roscar_name.c_str(), msg.roscar_ip_v4.c_str());
        publisher_->publish(msg);
    }

    rclcpp::Publisher<RobotRegister>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotRegisterPublisher>());
    rclcpp::shutdown();
    return 0;
}
