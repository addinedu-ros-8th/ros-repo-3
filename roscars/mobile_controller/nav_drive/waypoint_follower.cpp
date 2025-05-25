#include <memory>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "astar.hpp"

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("waypoint_follower");

  // 1) 웨이포인트 목록 정의
  struct WP { int id; double x, y; };
  std::vector<WP> wps = {
    {0, 0.47, -0.009},
    {1, 0.524, 0.48},
    {2, 0.08,  0.7},
    {3, 0.08,  1.15},
    {4, 0.43,  1.27},
    {5, 0.57,  1.92}
  };

  // 2) A* 경로 탐색
  std::vector<Node> nodes;
  std::vector<Edge> edges;  // 환경에 맞게 채우세요
  for (auto &wp : wps) {
    nodes.push_back({wp.id, wp.x, wp.y});
  }
  AStar astar(nodes, edges);
  auto path_ids = astar.search(0, 5);

  // 3) FollowWaypoints 액션 클라이언트 생성 및 서버 대기
  auto client = rclcpp_action::create_client<FollowWaypoints>(node, "follow_waypoints");
  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "액션 서버 연결 실패");
    rclcpp::shutdown();
    return 1;
  }

  // 4) Goal 메시지 구성
  FollowWaypoints::Goal goal_msg;

  // 4-1) 중간 웨이포인트: 다음 포인트로 향하는 yaw 계산
  for (size_t i = 0; i + 1 < path_ids.size(); ++i) {
    auto &cur = wps[path_ids[i]];
    auto &nxt = wps[path_ids[i+1]];
    double yaw = std::atan2(nxt.y - cur.y, nxt.x - cur.x);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);  // roll=0, pitch=0, yaw 설정:contentReference[oaicite:0]{index=0}
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = cur.x;
    ps.pose.position.y = cur.y;
    ps.pose.orientation = tf2::toMsg(q);
    goal_msg.poses.push_back(ps);
  }

  // 4-2) 마지막 목적지: 동일 위치에서 final_yaw 방향으로 회전
  auto &last_wp = wps[path_ids.back()];
  geometry_msgs::msg::PoseStamped final_ps;
  final_ps.header.frame_id = "map";
  final_ps.pose.position.x = last_wp.x;
  final_ps.pose.position.y = last_wp.y;
  double final_yaw = M_PI / 2;  // 예: 90도(라디안) 방향으로 정차
  tf2::Quaternion q_final;
  q_final.setRPY(0.0, 0.0, final_yaw);
  final_ps.pose.orientation = tf2::toMsg(q_final);
  goal_msg.poses.push_back(final_ps);

  // 5) 피드백 콜백 설정
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

  // 6) 목표 전송 및 완료 대기
  auto gh_future = client->async_send_goal(goal_msg, options);
  rclcpp::spin_until_future_complete(node, gh_future);
  auto gh = gh_future.get();

  auto res_future = client->async_get_result(gh);
  rclcpp::spin_until_future_complete(node, res_future);
  auto wrapped_result = res_future.get();
  if (wrapped_result.result->success) {
    RCLCPP_INFO(node->get_logger(), "최종 위치 및 헤딩으로 주행 및 정차 완료");
  } else {
    RCLCPP_ERROR(node->get_logger(), "주행 실패");
  }

  rclcpp::shutdown();
  return 0;
}
