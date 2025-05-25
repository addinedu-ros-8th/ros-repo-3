#include <memory>
#include <vector>
#include <atomic>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include "astar.hpp"

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("waypoint_follower");

  struct WP { int id; double x, y; };
  // 전체 웨이포인트 정의
  std::vector<WP> wps = {
    {0, 0.47, -0.009},
    {1, 0.524, 0.48},
    {2, 1.05,  0.8},
    {3, 0.6,  1.23},
    {4, 0.6,  1.16},
    {5, 1.15, 2.23}
  };

  // A* 경로 계산 (필요 시 edges 채우기)
  std::vector<Node> nodes;
  std::vector<Edge> edges;
  for (auto &wp : wps) {
    nodes.push_back({wp.id, wp.x, wp.y});
  }
  AStar astar(nodes, edges);

  // 액션 클라이언트 생성
  auto client = rclcpp_action::create_client<FollowWaypoints>(node, "follow_waypoints");
  client->wait_for_action_server(std::chrono::seconds(5));

  // /cart/is_on 구독을 위한 플래그
  std::atomic<bool> cart_on{false};
  auto sub = node->create_subscription<std_msgs::msg::Int32>(
    "/cart/is_on", 10,
    [&cart_on](const std_msgs::msg::Int32::SharedPtr msg) {
      if (msg->data == 1) {
        cart_on.store(true);
      }
    }
  );

  // 1) 처음 0,1,2번 목표 전송
  std::vector<int> first_ids = {0, 1, 2};
  FollowWaypoints::Goal first_goal;
  for (auto id : first_ids) {
    auto &wp = wps[id];
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = wp.x;
    ps.pose.position.y = wp.y;
    ps.pose.orientation.w = 1.0;
    first_goal.poses.push_back(ps);
  }
  auto send_first = client->async_send_goal(first_goal);
  rclcpp::spin_until_future_complete(node, send_first);

  // 2) /cart/is_on == 1 될 때까지 대기
  RCLCPP_INFO(node->get_logger(), "/cart/is_on이 1이 될 때까지 대기함");
  while (rclcpp::ok() && !cart_on.load()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(node->get_logger(), "/cart/is_on == 1 확인!");

  // 3) 이후 3,4,5번 목표 전송
  std::vector<int> next_ids = {3, 4, 5};
  FollowWaypoints::Goal next_goal;
  for (auto id : next_ids) {
    auto &wp = wps[id];
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = wp.x;
    ps.pose.position.y = wp.y;
    ps.pose.orientation.w = 1.0;
    next_goal.poses.push_back(ps);
  }
  auto send_next = client->async_send_goal(next_goal);
  rclcpp::spin_until_future_complete(node, send_next);

  // 결과 확인
  auto gh = send_next.get();
  auto res = client->async_get_result(gh);
  rclcpp::spin_until_future_complete(node, res);
  auto wrapped = res.get();
  if (wrapped.result->error_code == wrapped.result->NONE) {
    RCLCPP_INFO(node->get_logger(), "후속 웨이포인트 성공");
  } else {
    RCLCPP_WARN(node->get_logger(), "실패: %s", wrapped.result->error_msg.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
