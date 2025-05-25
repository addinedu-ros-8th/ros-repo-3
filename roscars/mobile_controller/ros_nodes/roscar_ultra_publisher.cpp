#include <rclcpp/rclcpp.hpp>
#include "shared_interfaces/msg/ultra_status.hpp"
#include "ultra_sensor.hpp"
using namespace std::chrono_literals;
class UltraPublisher: public rclcpp::Node {
  rclcpp::Publisher<shared_interfaces::msg::UltraStatus>::SharedPtr pub_;
  UltraSensor sensor_;
public:
  UltraPublisher(): Node("ultra_publisher") {
    pub_ = create_publisher<shared_interfaces::msg::UltraStatus>("/roscar/sensor/ultra", 10);
    create_wall_timer(500ms, std::bind(&UltraPublisher::tick,this));
  }
  void tick(){
    float d = sensor_.read_distance();
    auto msg = shared_interfaces::msg::UltraStatus();
    msg.roscar_name = "roscar";
    msg.distance = d;
    pub_->publish(msg);
  }
};
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<UltraPublisher>());
  rclcpp::shutdown();
  return 0;
}
