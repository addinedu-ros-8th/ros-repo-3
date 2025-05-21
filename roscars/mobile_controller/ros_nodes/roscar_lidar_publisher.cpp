#include <memory>
#include <fstream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/lidar_scan.hpp"
#include <sl_lidar.h>
#include <sl_lidar_driver.h>

#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))

using namespace sl;

// SSID 가져오는 함수 정의
std::string get_ap_ssid() {
    std::ifstream hostapd_file("/etc/hostapd/hostapd.conf");
    std::string line;
    while (std::getline(hostapd_file, line)) {
        if (line.find("ssid=") == 0) {
            return line.substr(5);
        }
    }
    return "UNKNOWN_SSID";
}

class SLLidarNode : public rclcpp::Node {
public:
    SLLidarNode()
        : Node("sllidar_node"),
          serial_port_("/dev/ttyAMA0"),
          serial_baudrate_(460800),
          frame_id_("laser"),
          scan_frequency_(10.0),
          inverted_(false),
          angle_compensate_(true),
          need_exit_(false)
    {
        this->declare_parameter<std::string>("serial_port", serial_port_);
        this->declare_parameter<int>("serial_baudrate", serial_baudrate_);
        this->declare_parameter<std::string>("frame_id", frame_id_);
        this->declare_parameter<double>("scan_frequency", scan_frequency_);
        this->declare_parameter<bool>("inverted", inverted_);
        this->declare_parameter<bool>("angle_compensate", angle_compensate_);

        this->get_parameter("serial_port", serial_port_);
        this->get_parameter("serial_baudrate", serial_baudrate_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("scan_frequency", scan_frequency_);
        this->get_parameter("inverted", inverted_);
        this->get_parameter("angle_compensate", angle_compensate_);

        // SSID 기반으로 네임스페이스 설정
        ssid_ = get_ap_ssid();
        std::string topic_name = "/" + ssid_ + "/roscar/sensor/lidar";

        scan_pub_ = this->create_publisher<shared_interfaces::msg::LidarScan>(
            topic_name, rclcpp::QoS(10).reliable().keep_last(10)
        );

        RCLCPP_INFO(this->get_logger(), "Connected AP SSID: %s", ssid_.c_str());
        RCLCPP_INFO(this->get_logger(), "Starting SLLIDAR driver...");
        work_loop();
    }

private:
    std::string serial_port_;
    int serial_baudrate_;
    std::string frame_id_;
    double scan_frequency_;
    bool inverted_;
    bool angle_compensate_;
    bool need_exit_;
    std::string ssid_;

    rclcpp::Publisher<shared_interfaces::msg::LidarScan>::SharedPtr scan_pub_;

    bool getSLLIDARDeviceInfo(sl::ILidarDriver* drv) {
        sl_lidar_response_device_info_t devinfo;
        sl_result op_result = drv->getDeviceInfo(devinfo);

        if (SL_IS_OK(op_result)) {
            RCLCPP_INFO(this->get_logger(), "Device Info:");
            RCLCPP_INFO(this->get_logger(), "Serial Number: %.16s", devinfo.serialnum);
            RCLCPP_INFO(this->get_logger(), "Firmware Version: %d.%02d", devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF);
            RCLCPP_INFO(this->get_logger(), "Hardware Revision: %d", (int)devinfo.hardware_version);
            return true;
        }
        return false;
    }

    bool checkSLLIDARHealth(sl::ILidarDriver* drv) {
        sl_lidar_response_device_health_t healthinfo;
        sl_result op_result = drv->getHealth(healthinfo);

        if (SL_IS_OK(op_result)) {
            if (healthinfo.status == SL_LIDAR_STATUS_OK) {
                return true;
            } else {
                RCLCPP_WARN(this->get_logger(), "Lidar health status is not OK.");
                return false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cannot retrieve health info.");
            return false;
        }
    }

    void publish_scan(
        rclcpp::Publisher<shared_interfaces::msg::LidarScan>::SharedPtr& pub,
        sl_lidar_response_measurement_node_hq_t* nodes,
        size_t node_count,
        rclcpp::Time start,
        double scan_time,
        bool inverted,
        float angle_min,
        float angle_max,
        float max_distance,
        std::string frame_id,
        std::string roscar_name
    ) {
        auto scan_msg = std::make_unique<shared_interfaces::msg::LidarScan>();
        scan_msg->header.stamp = start;
        scan_msg->header.frame_id = frame_id;

        scan_msg->angle_min = angle_min * M_PI / 180.0;
        scan_msg->angle_max = angle_max * M_PI / 180.0;
        scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (double)(node_count - 1);
        scan_msg->scan_time = scan_time;
        scan_msg->time_increment = scan_time / (double)(node_count - 1);
        scan_msg->range_min = 0.15;
        scan_msg->range_max = max_distance;

        scan_msg->ranges.resize(node_count);
        scan_msg->intensities.resize(node_count);

        for (size_t i = 0; i < node_count; ++i) {
            float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
            if (read_value == 0.0f) read_value = std::numeric_limits<float>::infinity();

            scan_msg->ranges[i] = read_value;
            scan_msg->intensities[i] = (float)(nodes[i].quality >> 2);
        }

        scan_msg->roscar_name = roscar_name;

        pub->publish(std::move(scan_msg));
    }

    int work_loop() {
        sl::ILidarDriver* drv = *sl::createLidarDriver();

        if (!drv) {
            RCLCPP_ERROR(this->get_logger(), "Insufficient memory to create driver instance.");
            return -1;
        }

        sl::IChannel* channel = *sl::createSerialPortChannel(serial_port_.c_str(), serial_baudrate_);

        if (SL_IS_FAIL(drv->connect(channel))) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to SLLIDAR on port %s", serial_port_.c_str());
            return -1;
        }

        if (!getSLLIDARDeviceInfo(drv) || !checkSLLIDARHealth(drv)) {
            return -1;
        }

        drv->setMotorSpeed();
        drv->startScan(false, true);

        rclcpp::Rate rate(scan_frequency_);
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);

        while (rclcpp::ok() && !need_exit_) {
            count = _countof(nodes);
            rclcpp::Time scan_start = this->now();
            sl_result result = drv->grabScanDataHq(nodes, count);

            if (SL_IS_OK(result)) {
                drv->ascendScanData(nodes, count);

                publish_scan(scan_pub_, nodes, count, scan_start,
                             1.0 / scan_frequency_, inverted_, 0.0f, 360.0f,
                             8.0f, frame_id_, ssid_);
            }

            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        drv->stop();
        drv->setMotorSpeed(0);
        drv->disconnect();
        delete drv;
        drv = nullptr;

        return 0;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SLLidarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}