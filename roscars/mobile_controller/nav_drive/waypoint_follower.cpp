#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include "astar.hpp"

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("waypoint_follower");

  struct WP { int id; double x, y; };
  std::vector<WP> wps = {
    {0, 0.47, -0.009},
    {1, 0.524, 0.48},
    {2, 1.05,  0.8},
    {3, 0.6,  1.23},
    {4, 0.6,  1.16},
    {5, 1.15,  2.23}
  };

  std::vector<Node> nodes;
  std::vector<Edge> edges;  // 필요 시 환경에 맞게 채우세요
  for (auto &wp : wps) {
    nodes.push_back({wp.id, wp.x, wp.y});
  }
  AStar astar(nodes, edges);
  auto path_ids = astar.search(0, 5);

  auto client = rclcpp_action::create_client<FollowWaypoints>(node, "follow_waypoints");
  client->wait_for_action_server(std::chrono::seconds(5));

  FollowWaypoints::Goal goal_msg;
  for (auto id : path_ids) {
    auto &wp = wps[id];
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = wp.x;
    ps.pose.position.y = wp.y;
    ps.pose.orientation.w = 1.0;
    goal_msg.poses.push_back(ps);
  } 

  rclcpp_action::Client<FollowWaypoints>::SendGoalOptions options;
  options.feedback_callback =
    [node](
      std::shared_ptr<rclcpp_action::ClientGoalHandle<FollowWaypoints>>,
      const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "현재 웨이포인트 인덱스: %u",
        feedback->current_waypoint);
    };

  auto gh_future = client->async_send_goal(goal_msg, options);
  rclcpp::spin_until_future_complete(node, gh_future);
  auto gh = gh_future.get();

  auto res_future = client->async_get_result(gh);
  rclcpp::spin_until_future_complete(node, res_future);

  auto wrapped_result = res_future.get();
  auto result = wrapped_result.result;

  auto logger = rclcpp::get_logger("waypoint_follower");

  if (result->error_code == result->NONE && result->missed_waypoints.empty()) {
    RCLCPP_INFO(logger, "FollowWaypoints succeeded.");
  } else {
    RCLCPP_WARN(logger, "FollowWaypoints failed with error: %s",
                result->error_msg.c_str());

    for (const auto &missed : result->missed_waypoints) {
      RCLCPP_WARN(logger, "Missed waypoint index: %d", missed.index);
    }
  }



  rclcpp::shutdown();
  return 0;
}