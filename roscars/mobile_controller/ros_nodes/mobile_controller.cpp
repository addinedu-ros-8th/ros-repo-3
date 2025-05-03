#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <string>
#include <unordered_map>
#include <memory>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

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

class ModeController : public rclcpp::Node {
public:
    ModeController()
    : Node("mode_controller", get_ap_ssid()) {
        namespace_ = get_ap_ssid();
        registered_ = false;
        robot_state_ = "idle";

        run_node("robot_register_publisher");

        reg_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "robot_registered", 10,
            std::bind(&ModeController::registered_callback, this, _1));

        state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "robot_state", 10,
            std::bind(&ModeController::state_callback, this, _1));
    }

private:
    std::string namespace_;
    bool registered_;
    std::string robot_state_;
    std::unordered_map<std::string, pid_t> processes_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reg_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    void registered_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (registered_ != msg->data) {
            registered_ = msg->data;
            RCLCPP_INFO(this->get_logger(), "[등록 상태 변경] registered = %s", registered_ ? "true" : "false");
            update_nodes();
        }
    }

    void state_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (robot_state_ != msg->data) {
            robot_state_ = msg->data;
            RCLCPP_INFO(this->get_logger(), "[로봇 상태 변경] robot_state = %s", robot_state_.c_str());
            update_nodes();
        }
    }

    void update_nodes() {
        terminate_all();

        if (!registered_) {
            run_node("robot_register_publisher");
        }

        if (registered_) {
            run_node("robot_battery_publisher");
        }

        if (robot_state_ == "driving") {
            run_node("robot_imu_publisher");
            run_node("robot_ultra_publisher");
            run_node("robot_lidar_publisher");
        }
    }

    void run_node(const std::string &executable_name) {
        if (processes_.find(executable_name) != processes_.end()) {
            return;
        }
    
        pid_t pid = fork();
        if (pid == 0) {
            // 자식 프로세스
            execlp("ros2", "ros2", "run", "mobile_controller", executable_name.c_str(), nullptr);
            std::exit(1); // 실패 시 종료
        } else if (pid > 0) {
            // 부모 프로세스
            processes_[executable_name] = pid;
            RCLCPP_INFO(this->get_logger(), "[실행] %s", executable_name.c_str());
    
            // 여기에 네임스페이스를 설정하거나, 노드 파라미터로 전달하는 코드 추가
            std::string cmd = "ros2 param set /" + executable_name + " namespace " + namespace_;
            system(cmd.c_str()); // system 명령어로 실행
        } else {
            RCLCPP_ERROR(this->get_logger(), "[실행 실패] %s", executable_name.c_str());
        }
    }

    void terminate_all() {
        for (const auto &pair : processes_) {
            kill(pair.second, SIGTERM);
            waitpid(pair.second, nullptr, 0);
            RCLCPP_INFO(this->get_logger(), "[종료] %s", pair.first.c_str());
        }
        processes_.clear();
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeController>());
    rclcpp::shutdown();
    return 0;
}
