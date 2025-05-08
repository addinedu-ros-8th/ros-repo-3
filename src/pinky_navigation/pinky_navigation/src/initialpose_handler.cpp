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
    latest_pose_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Received initial pose, querying trajectory states...");

    if (!get_states_client_->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_ERROR(this->get_logger(), "GetTrajectoryStates service not available");
      return;
    }

    auto request = std::make_shared<cartographer_ros_msgs::srv::GetTrajectoryStates::Request>();
    auto future = get_states_client_->async_send_request(request,
      std::bind(&InitialPoseHandler::onGetTrajectoryStatesResponse, this, std::placeholders::_1));
  }

  void onGetTrajectoryStatesResponse(rclcpp::Client<cartographer_ros_msgs::srv::GetTrajectoryStates>::SharedFuture future)
  {
    try {
      auto response = future.get();

      if (response->trajectory_states.trajectory_id.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No trajectory IDs available");
        return;
      }

      int latest_id = response->trajectory_states.trajectory_id.back();
      RCLCPP_INFO(this->get_logger(), "Latest trajectory_id: %d", latest_id);
      finishTrajectory(latest_id);

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in GetTrajectoryStates callback: %s", e.what());
    }
  }

  void finishTrajectory(int trajectory_id)
  {
    if (!finish_client_->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_ERROR(this->get_logger(), "FinishTrajectory service not available");
      return;
    }

    auto request = std::make_shared<cartographer_ros_msgs::srv::FinishTrajectory::Request>();
    request->trajectory_id = trajectory_id;
    auto future = finish_client_->async_send_request(request,
      std::bind(&InitialPoseHandler::onFinishTrajectoryResponse, this, std::placeholders::_1));
  }

  void onFinishTrajectoryResponse(rclcpp::Client<cartographer_ros_msgs::srv::FinishTrajectory>::SharedFuture /*future*/)
  {
    RCLCPP_INFO(this->get_logger(), "Finished trajectory. Now starting new one...");
    startTrajectory();
  }

  void startTrajectory()
  {
    if (!start_client_->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_ERROR(this->get_logger(), "StartTrajectory service not available");
      return;
    }

    auto request = std::make_shared<cartographer_ros_msgs::srv::StartTrajectory::Request>();
    request->configuration_directory = configuration_directory_;
    request->configuration_basename = configuration_basename_;
    request->use_initial_pose = true;
    request->initial_pose.position = latest_pose_.pose.pose.position;
    request->initial_pose.orientation = latest_pose_.pose.pose.orientation;
    request->relative_to_trajectory_id = 0;

    auto future = start_client_->async_send_request(request,
      std::bind(&InitialPoseHandler::onStartTrajectoryResponse, this, std::placeholders::_1));
  }

  void onStartTrajectoryResponse(rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedFuture /*future*/)
  {
    RCLCPP_INFO(this->get_logger(), "Started new trajectory successfully.");
  }

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
