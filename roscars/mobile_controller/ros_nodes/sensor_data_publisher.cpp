#include <rclcpp/rclcpp.hpp>
#include <shared_interfaces/msg/sensor_data.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sl_lidar.h>
#include <sl_lidar_driver.h>

#include "imu_sensor.hpp"
#include "ultra_sensor.hpp"

#include <string>
#include <fstream>
#include <json/json.h>

#define COUNT_OF(x) (sizeof(x) / sizeof((x)[0]))

using namespace sl;

class SensorDataPublisher : public rclcpp::Node
{
public:
  SensorDataPublisher()
  : Node("sensor_data_publisher"),
    imu_("/dev/i2c-1", 0x68),
    ultra_sensor_(std::make_shared<UltraSensor>())
  {
    ssid_ = get_ap_ssid();
    std::string topic = "/" + ssid_ + "/roscar/sensor_data";

    publisher_ = this->create_publisher<shared_interfaces::msg::SensorData>(topic, 10);
    init_lidar();
    imu_.initialize();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&SensorDataPublisher::publish_sensor_data, this)
    );

    RCLCPP_INFO(this->get_logger(), "✅ SensorData 퍼블리셔 시작됨 (topic: %s)", topic.c_str());
  }

private:
  std::string ssid_;
  rclcpp::Publisher<shared_interfaces::msg::SensorData>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  ICM20948 imu_;
  std::shared_ptr<UltraSensor> ultra_sensor_;

  ILidarDriver* drv_ = nullptr;
  sl_lidar_response_measurement_node_hq_t nodes_[8192];
  size_t node_count_ = COUNT_OF(nodes_);

  std::string get_ap_ssid() {
    std::ifstream file("/etc/hostapd/hostapd.conf");
    std::string line;
    while (std::getline(file, line)) {
      if (line.find("ssid=") == 0)
        return line.substr(5);
    }
    return "unknown_ssid";
  }

  void init_lidar() {
    drv_ = *createLidarDriver();
    auto channel = *createSerialPortChannel("/dev/ttyAMA0", 460800);
    if (SL_IS_FAIL(drv_->connect(channel))) {
      RCLCPP_ERROR(this->get_logger(), "❌ Lidar 연결 실패");
      return;
    }
    drv_->startScan(false, true);
    RCLCPP_INFO(this->get_logger(), "✅ Lidar 초기화 완료");
  }

  void publish_sensor_data() {
    if (!drv_) return;

    auto msg = shared_interfaces::msg::SensorData();
    msg.robot_id = ssid_;
    msg.stamp = this->now();

    // ✅ LiDAR JSON
    node_count_ = COUNT_OF(nodes_);
    if (SL_IS_OK(drv_->grabScanDataHq(nodes_, node_count_))) {
      drv_->ascendScanData(nodes_, node_count_);
      Json::Value lidar_json;
      Json::Value ranges(Json::arrayValue);
      for (size_t i = 0; i < node_count_; ++i) {
        float distance = (float)nodes_[i].dist_mm_q2 / 4.0f / 1000;
        if (distance == 0.0f) distance = -1.0f;
        ranges.append(distance);
      }
      lidar_json["ranges"] = ranges;
      msg.lidar_raw = Json::writeString(Json::StreamWriterBuilder(), lidar_json);
    }

    // ✅ IMU JSON
    try {
      float ax, ay, az, gx, gy, gz, mx, my, mz;
      imu_.read_sensor(ax, ay, az, gx, gy, gz, mx, my, mz);
      Json::Value imu_json;
      imu_json["accel"] = Json::arrayValue;
      imu_json["gyro"] = Json::arrayValue;
      imu_json["mag"] = Json::arrayValue;

      imu_json["accel"].append(ax); imu_json["accel"].append(ay); imu_json["accel"].append(az);
      imu_json["gyro"].append(gx); imu_json["gyro"].append(gy); imu_json["gyro"].append(gz);
      imu_json["mag"].append(mx); imu_json["mag"].append(my); imu_json["mag"].append(mz);

      msg.imu_data = Json::writeString(Json::StreamWriterBuilder(), imu_json);
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "IMU 오류: %s", e.what());
      msg.imu_data = "{}";
    }

    // ✅ 초음파 JSON
    try {
      float dist = ultra_sensor_->read_distance();
      Json::Value ultra_json;
      ultra_json["front"] = dist;
      msg.ultrasonic_data = Json::writeString(Json::StreamWriterBuilder(), ultra_json);
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "초음파 오류: %s", e.what());
      msg.ultrasonic_data = "{}";
    }

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "SensorData 발행됨");
  }
};

// ✅ main 함수 추가
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorDataPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
