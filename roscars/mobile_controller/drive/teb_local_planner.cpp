// File: drive/teb_local_planner.cpp

#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/imu_status.hpp"
#include "shared_interfaces/msg/lidar_scan.hpp"
#include "shared_interfaces/msg/ultra_status.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

class TebLocalPlanner : public rclcpp::Node {
  rclcpp::Subscription<shared_interfaces::msg::ImuStatus>::SharedPtr  imu_sub_;
  rclcpp::Subscription<shared_interfaces::msg::LidarScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<shared_interfaces::msg::UltraStatus>::SharedPtr ultra_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr               path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr           cmd_pub_;
  rclcpp::TimerBase::SharedPtr                                      timer_;

  shared_interfaces::msg::LidarScan::SharedPtr scan_;
  float yaw_{0.0f};
  float ultra_d_{INFINITY};
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  size_t idx_{0};

public:
  explicit TebLocalPlanner()
  : Node("teb_local_planner")
  {
    imu_sub_ = this->create_subscription<shared_interfaces::msg::ImuStatus>(
      "/roscar/sensor/imu",
      rclcpp::SensorDataQoS(),
      [this](const shared_interfaces::msg::ImuStatus::SharedPtr m) {
        yaw_ += m->gyro_z * 0.1f;
        if (yaw_ > M_PI)  yaw_ -= 2*M_PI;
        if (yaw_ < -M_PI) yaw_ += 2*M_PI;
      });

    lidar_sub_ = this->create_subscription<shared_interfaces::msg::LidarScan>(
      "/roscar/sensor/lidar",
      rclcpp::SensorDataQoS(),
      [this](const shared_interfaces::msg::LidarScan::SharedPtr m) {
        scan_ = m;
      });

    ultra_sub_ = this->create_subscription<shared_interfaces::msg::UltraStatus>(
      "/roscar/sensor/ultra",
      rclcpp::SensorDataQoS(),
      [this](const shared_interfaces::msg::UltraStatus::SharedPtr m) {
        ultra_d_ = m->distance;
      });

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/global_path",
      10,
      [this](const nav_msgs::msg::Path::SharedPtr p) {
        path_ = p->poses;
        idx_ = 0;
        RCLCPP_INFO(get_logger(), "[Path] received %zu poses", path_.size());
      });

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/robot/control_cmd", 10);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&TebLocalPlanner::loop, this));

    RCLCPP_INFO(get_logger(), "TebLocalPlanner initialized.");
  }

private:
  void loop() {
    geometry_msgs::msg::Twist cmd;

    if (idx_ >= path_.size() || !scan_) {
      cmd_pub_->publish(cmd);
      return;
    }

    // 1) 초음파 회피
    if (ultra_d_ < 0.02f) {
      cmd.angular.z = 0.5;
      cmd_pub_->publish(cmd);
      return;
    }

    // 2) LiDAR 회피용 파라미터
    constexpr float LIDAR_DETECT_THRESHOLD = 0.06f;   // 감지 거리: 12cm
    constexpr float LIDAR_DETECT_ANGLE     = M_PI / 4; // 전방 ±45°

    const auto &ranges      = scan_->ranges;
    const float angle_inc  = scan_->angle_increment;
    size_t N                = ranges.size();
    size_t center           = N / 2;
    size_t sector_idx       = static_cast<size_t>(LIDAR_DETECT_ANGLE / angle_inc);

    float front_min = INFINITY;
    size_t start = (center + N - sector_idx) % N;
    size_t end   = (center + sector_idx) % N;
    for (size_t i = start; ; i = (i + 1) % N) {
      front_min = std::min(front_min, ranges[i]);
      if (i == end) break;
    }

    if (front_min < LIDAR_DETECT_THRESHOLD) {
      // 장애물 가까울수록 빠르게 회전
      float ratio = (LIDAR_DETECT_THRESHOLD - front_min) / LIDAR_DETECT_THRESHOLD;
      cmd.angular.z = (front_min_dir(ranges, center, sector_idx) > 0 ? 1.0f : -1.0f)
                      * (0.3f + 0.7f * ratio);
      cmd_pub_->publish(cmd);
      return;
    }

    // 3) 목표 추종
    const auto &p = path_[idx_].pose.position;
    double ang_goal = std::atan2(p.y, p.x);
    double err      = std::atan2(std::sin(ang_goal - yaw_), std::cos(ang_goal - yaw_));

    if (std::fabs(err) > 0.1) {
      cmd.angular.z = err;
    } else {
      cmd.linear.x = 0.2;
      if (std::hypot(p.x, p.y) < 0.1) {
        idx_++;
        RCLCPP_INFO(get_logger(), "[Progress] reached waypoint %zu", idx_);
      }
    }

    cmd_pub_->publish(cmd);
  }

  // 전방 영역에서 장애물 방향 판단(양수: 왼쪽 유리, 음수: 오른쪽 유리)
  int front_min_dir(const std::vector<float> &ranges, size_t center, size_t sector) {
    // 좌우 평균 거리 비교
    float left_avg = 0, right_avg = 0;
    for (size_t i = 1; i <= sector; ++i) {
      left_avg  += ranges[(center + i) % ranges.size()];
      right_avg += ranges[(center + ranges.size() - i) % ranges.size()];
    }
    return (left_avg > right_avg) ? +1 : -1;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TebLocalPlanner>());
  rclcpp::shutdown();
  return 0;
}
