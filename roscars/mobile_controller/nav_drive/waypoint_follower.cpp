#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include "astar.hpp"

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;

// helper: ids 순서대로 goal 구성, 마지막 pose만 override_yaw 적용
FollowWaypoints::Goal make_goal(
  const std::vector<int>& ids,
  const std::vector<std::pair<double,double>>& wps,
  double override_yaw = NAN)
{
  FollowWaypoints::Goal g;
  for (size_t i = 0; i < ids.size(); ++i) {
    auto [x, y] = wps[ids[i]];
    double yaw;
    if (!std::isnan(override_yaw) && i == ids.size() - 1) {
      yaw = override_yaw;
    } else if (i + 1 < ids.size()) {
      auto [nx, ny] = wps[ids[i+1]];
      yaw = std::atan2(ny - y, nx - x);
    } else {
      yaw = 0.0;
    }
    tf2::Quaternion q; q.setRPY(0, 0, yaw);
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.orientation = tf2::toMsg(q);
    g.poses.push_back(ps);
  }
  return g;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("waypoint_follower");

  struct WP { int id; double x, y; };
  std::vector<WP> wps = {
    {0, 0.47, -0.009},
    {1, 0.524, 0.48},
    {2, 0.08,  0.7},
    {3, 0.08,  1.15},
    {4, 0.43,  1.27},
    {5, 0.57,  1.92}
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

  // ★ 여기서 수정: get()이 반환하는 WrappedResult에 .result로 접근
  auto wrapped_result = res_future.get();
  if (wrapped_result.result->success) {
    RCLCPP_INFO(node->get_logger(), "웨이포인트 주행 완료");
  }

  rclcpp::shutdown();
  return 0;
}
