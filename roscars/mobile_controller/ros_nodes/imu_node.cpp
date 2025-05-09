#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/imu_status.hpp"
#include "imu_sensor.hpp"

#include <fstream>
#include <string>

std::string get_ap_ssid() {
    std::ifstream file("/etc/hostapd/hostapd.conf");
    std::string line;
    while (std::getline(file, line)) {
        if (line.find("ssid=") == 0) {
            return line.substr(5);
        }
    }
    return "UNKNOWN_SSID";
}

class ICM20948Publisher : public rclcpp::Node {
public:
    ICM20948Publisher() : Node("imu_publisher"), imu_("/dev/i2c-1", 0x68) {
        publisher_ = this->create_publisher<shared_interfaces::msg::ImuStatus>("/robot/sensor/imu", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ICM20948Publisher::publish_imu, this));

        // if (!imu_.initialize()) {
        //     RCLCPP_FATAL(this->get_logger(), "IMU 초기화 실패");
        //     std::exit(1);
        // }
    }

private:
    ICM20948 imu_;
    rclcpp::Publisher<shared_interfaces::msg::ImuStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_imu() {
        float ax, ay, az, gx, gy, gz, mx, my, mz;
        imu_.read_sensor(ax, ay, az, gx, gy, gz, mx, my, mz);

        shared_interfaces::msg::ImuStatus msg;
        msg.accel_x = ax;
        msg.accel_y = ay;
        msg.accel_z = az;

        msg.gyro_x = gx;
        msg.gyro_y = gy;
        msg.gyro_z = gz;

        msg.mag_x = mx;
        msg.mag_y = my;
        msg.mag_z = mz;

        msg.roscar_name = get_ap_ssid();

        publisher_->publish(msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICM20948Publisher>());
    rclcpp::shutdown();
    return 0;
}
