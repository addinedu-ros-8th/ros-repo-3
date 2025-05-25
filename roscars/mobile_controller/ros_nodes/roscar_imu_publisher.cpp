// File: ros_nodes/roscar_imu_publisher.cpp

#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/imu_status.hpp"
#include "imu_sensor.hpp"

using namespace std::chrono_literals;

class ICM20948Publisher : public rclcpp::Node {
  rclcpp::Publisher<shared_interfaces::msg::ImuStatus>::SharedPtr pub_;
  ICM20948 imu_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  ICM20948Publisher()
  : Node("imu_publisher"), imu_("/dev/i2c-1", 0x68)
  {
    pub_ = create_publisher<shared_interfaces::msg::ImuStatus>("/roscar/sensor/imu", 10);
    imu_.initialize();
    timer_ = create_wall_timer(100ms, std::bind(&ICM20948Publisher::tick, this));
  }

  void tick() {
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    imu_.read_sensor(ax, ay, az, gx, gy, gz, mx, my, mz);

    auto msg = shared_interfaces::msg::ImuStatus();
    msg.roscar_name = "roscar";
    msg.accel_x = ax; msg.accel_y = ay; msg.accel_z = az;
    msg.gyro_x  = gx; msg.gyro_y  = gy; msg.gyro_z  = gz;
    msg.mag_x   = mx; msg.mag_y   = my; msg.mag_z   = mz;
    pub_->publish(msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ICM20948Publisher>());
  rclcpp::shutdown();
  return 0;
}
