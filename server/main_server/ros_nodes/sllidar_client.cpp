#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include <map>
#include <limits>

#define RAD2DEG(x) ((x)*180.0/M_PI)

// 필터링할 전방 구간
static constexpr float ANGLE_MIN1 = 315.0f;
static constexpr float ANGLE_MAX1 = 360.0f;
static constexpr float ANGLE_MIN2 = 0.0f;
static constexpr float ANGLE_MAX2 = 45.0f;

static void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    int count = static_cast<int>(scan->scan_time / scan->time_increment);

    // 1° 단위로 가장 짧은 거리 저장할 맵
    std::map<int, float> forward_map;

    for (int i = 0; i < count; ++i) {
        // 현재 측정 각도 (°)
        float angle = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        // [0,360) 범위로 정규화
        if (angle < 0.0f)      angle += 360.0f;
        if (angle >= 360.0f)   angle -= 360.0f;

        // 전방 315°~360° 또는 0°~45° 만 처리
        if ((angle >= ANGLE_MIN1 && angle < ANGLE_MAX1) ||
            (angle >= ANGLE_MIN2 && angle <= ANGLE_MAX2)) {

            // 1° 단위로 반올림
            int deg = static_cast<int>(angle + 0.5f) % 360;
            float dist = scan->ranges[i];
            if (dist <= 0.0f || std::isnan(dist)) 
                dist = std::numeric_limits<float>::infinity();

            auto it = forward_map.find(deg);
            if (it == forward_map.end() || dist < it->second) {
                forward_map[deg] = dist;
            }
        }
    }

    // 결과 출력 (315→359도, 0→45도 순서)
    RCLCPP_INFO(rclcpp::get_logger("sllidar_client"),
                "Filtered forward scan (315°→45°):");
    for (int a = 315; a < 360; ++a) {
        float d = forward_map.count(a) ? forward_map[a]
                                       : std::numeric_limits<float>::infinity();
        RCLCPP_INFO(rclcpp::get_logger("sllidar_client"),
                    "  %3d° : %.2f m", a, d);
    }
    for (int a = 0; a <= 45; ++a) {
        float d = forward_map.count(a) ? forward_map[a]
                                       : std::numeric_limits<float>::infinity();
        RCLCPP_INFO(rclcpp::get_logger("sllidar_client"),
                    "  %3d° : %.2f m", a, d);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("sllidar_client");

    auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        rclcpp::SensorDataQoS(),
        scanCb
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}