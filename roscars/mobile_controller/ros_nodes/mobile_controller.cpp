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

        reg_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "robot_registered", 10,
            std::bind(&ModeController::registered_callback, this, _1));

        state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "robot_state", 10,
            std::bind(&ModeController::state_callback, this, _1));

        // Start the "robot_register_publisher" node initially
        run_node("robot_register_publisher");
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

    // 모든 노드를 종료하는 메소드
    void stop_node(const std::string &node_name) {
        // 지정된 노드만 종료
        auto it = processes_.find(node_name);
        if (it != processes_.end()) {
            kill(it->second, SIGTERM);
            waitpid(it->second, nullptr, 0);
            RCLCPP_INFO(this->get_logger(), "[종료] %s", node_name.c_str());
            processes_.erase(it); // 노드를 종료한 후 리스트에서 제거
        }
    }

    // 모든 노드를 종료하는 메소드
    void stop_all_nodes() {
        for (const auto &pair : processes_) {
            kill(pair.second, SIGTERM);
            waitpid(pair.second, nullptr, 0);
            RCLCPP_INFO(this->get_logger(), "[종료] %s", pair.first.c_str());
        }
        processes_.clear();
    }

    // 예시: 서버 응답을 처리하는 메소드 (실제 로직에 맞게 수정 가능)
    void handle_server_response(const std::string &response) {
        if (response == "connection_established") {
            // 연결이 완료되면 모든 노드를 종료
            stop_all_nodes();
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeController>());
    rclcpp::shutdown();
    return 0;
}
