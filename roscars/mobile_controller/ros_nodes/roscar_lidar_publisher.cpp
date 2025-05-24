#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/lidar_scan.hpp"
#include <sl_lidar.h>
#include <sl_lidar_driver.h>
#define _countof(_Array) (int)(sizeof(_Array)/sizeof(_Array[0]))
using namespace sl;
class SLLidarNode : public rclcpp::Node {
  rclcpp::Publisher<shared_interfaces::msg::LidarScan>::SharedPtr pub_;
  bool inverted_ = false, angle_compensate_ = true;
public:
  SLLidarNode(): Node("sllidar_node") {
    pub_ = create_publisher<shared_interfaces::msg::LidarScan>("/roscar/sensor/lidar", 10);
    run();
  }
  void run(){
    auto drv = *createLidarDriver();
    auto ch  = *createSerialPortChannel("/dev/ttyAMA0",460800);
    drv->connect(ch);
    drv->startScan(false,true);
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    rclcpp::Rate rate(10);
    while(rclcpp::ok()){
      size_t count=_countof(nodes);
      drv->grabScanDataHq(nodes,count);
      drv->ascendScanData(nodes,count);
      auto msg = shared_interfaces::msg::LidarScan();
      msg.header.stamp = now();
      msg.header.frame_id = "laser";
      msg.angle_min = 0.0f;
      msg.angle_max = 2*M_PI;
      msg.angle_increment = (msg.angle_max-msg.angle_min)/(count-1);
      msg.scan_time = 1.0/10.0;
      msg.time_increment = msg.scan_time/(count-1);
      msg.range_min = 0.15f;
      msg.range_max = 8.0f;
      msg.ranges.resize(count);
      msg.intensities.resize(count);
      for(size_t i=0;i<count;i++){
        float d = nodes[i].dist_mm_q2/4.0f/1000.0f;
        msg.ranges[i] = (d==0.0f?INFINITY:d);
        msg.intensities[i] = (float)(nodes[i].quality>>2);
      }
      pub_->publish(msg);
      rclcpp::spin_some(this->get_node_base_interface());
      rate.sleep();
    }
    drv->stop(); drv->disconnect(); delete drv;
  }
};
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<SLLidarNode>());
  rclcpp::shutdown();
  return 0;
}
