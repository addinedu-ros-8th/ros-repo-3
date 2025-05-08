#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"
#include "cartographer_ros_msgs/srv/start_trajectory.hpp"
#include "cartographer_ros_msgs/srv/get_trajectory_states.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

class InitialPoseHandler : public rclcpp::Node
{
public:
  InitialPoseHandler() : Node("initialpose_handler")
  {
    using std::placeholders::_1;
    initialpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10, std::bind(&InitialPoseHandler::initialPoseCallback, this, _1));

    get_states_client_ = this->create_client<cartographer_ros_msgs::srv::GetTrajectoryStates>("/get_trajectory_states");
    finish_client_ = this->create_client<cartographer_ros_msgs::srv::FinishTrajectory>("/finish_trajectory");
    start_client_ = this->create_client<cartographer_ros_msgs::srv::StartTrajectory>("/start_trajectory");

    std::string pkg_path = ament_index_cpp::get_package_share_directory("pinky_cartographer");
    configuration_directory_ = pkg_path + "/params";
    configuration_basename_ = "nav2_cartographer_params.lua";
  }

private:
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    pose_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Received initial pose, querying trajectory states...");
    getLatestTrajectoryId();
  }

  void getLatestTrajectoryId()
  {
    if (!get_states_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "GetTrajectoryStates service not available");
      return;
    }

    auto request = std::make_shared<cartographer_ros_msgs::srv::GetTrajectoryStates::Request>();
    auto future = get_states_client_->async_send_request(request,
      std::bind(&InitialPoseHandler::getLatestTrajectoryCallback, this, std::placeholders::_1));
  }

  void getLatestTrajectoryCallback(rclcpp::Client<cartographer_ros_msgs::srv::GetTrajectoryStates>::SharedFuture future)
  {
    try {
      auto response = future.get();
      const auto & ids = response->trajectory_states.trajectory_id;

      if (!ids.empty()) {
        int latest_id = ids.back();  // 마지막 요소
        RCLCPP_INFO(this->get_logger(), "Latest trajectory_id: %d", latest_id);
        setInitalpose(pose_, latest_id);
        } else {
            RCLCPP_ERROR(this->get_logger(), "No valid trajectory ID found.");
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call GetTrajectoryStates service: %s", e.what());
    }
  }

  void setInitalpose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose, int trajectory_id) 
  {
    if (!finish_trajectory_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "FinishTrajectory service not available");
        return;
    }

    auto request = std::make_shared<cartographer_ros_msgs::srv::FinishTrajectory::Request>();
    request->trajectory_id = trajectory_id;

    finish_trajectory_client_->async_send_request(
        request,
        std::bind(&InitialPoseHandler::finishTrajectoryCallback, this, std::placeholders::_1));
  }

  void finish_trajectory_callback(
    rclcpp::Client<cartographer_ros_msgs::srv::FinishTrajectory>::SharedFuture future) {

    try {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "FinishTrajectory service called successfully");
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call FinishTrajectory service: %s", e.what());
        return;
    }

    if (!start_trajectory_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "StartTrajectory service not available");
        return;
    }

    auto request = std::make_shared<cartographer_ros_msgs::srv::StartTrajectory::Request>();
    request->configuration_directory = configuration_directory_;
    request->configuration_basename = configuration_basename_;
    request->use_initial_pose = true;

    request->initial_pose.position = pose_.pose.pose.position;
    request->initial_pose.orientation = pose_.pose.pose.orientation;
    request->relative_to_trajectory_id = 0;

    start_trajectory_client_->async_send_request(
        request,
        std::bind(&InitialPoseHandler::startTrajectoryCallback, this, std::placeholders::_1));
  }

  void startTrajectoryCallback()
  {
        rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedFuture future) {
    try {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "StartTrajectory service called successfully");
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call StartTrajectory service: %s", e.what());
    }
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped latest_pose_;
  std::string configuration_directory_;
  std::string configuration_basename_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Client<cartographer_ros_msgs::srv::GetTrajectoryStates>::SharedPtr get_states_client_;
  rclcpp::Client<cartographer_ros_msgs::srv::FinishTrajectory>::SharedPtr finish_client_;
  rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedPtr start_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitialPoseHandler>());
  rclcpp::shutdown();
  return 0;
}
