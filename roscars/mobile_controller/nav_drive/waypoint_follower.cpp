#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include "astar.hpp"

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("waypoint_follower");

  // 1. 그래프 노드와 엣지 정의 (예시)
  std::vector<Node> nodes = {{0,0.0,0.0}, {1,1.0,0.0}, {2,1.0,1.0}, {3,0.0,1.0}};
  std::vector<Edge> edges = {{0,1,1.0}, {1,2,1.0}, {2,3,1.0}, {3,0,1.0}};

  AStar astar(nodes, edges);
  auto path_ids = astar.search(0, 2);

  // 2. 액션 클라이언트 생성 및 서버 대기
  auto client = rclcpp_action::create_client<FollowWaypoints>(node, "follow_waypoints");
  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "액션 서버 연결 실패");
    return 1;
  }

  // 3. Goal 메시지 준비
  FollowWaypoints::Goal goal_msg;
  for (int id : path_ids) {
    auto& n = nodes[id];
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = n.x;
    ps.pose.position.y = n.y;
    ps.pose.orientation.w = 1.0;
    goal_msg.poses.push_back(ps);
  }

  // 4. Goal 전송 및 피드백 처리
  auto options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
  options.feedback_callback =
    [](auto, const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
      RCLCPP_INFO(rclcpp::get_logger("waypoint_follower"),
                  "현재 웨이포인트 인덱스: %zu", feedback->current_waypoint_index);
    };
  auto goal_handle_future = client->async_send_goal(goal_msg, options);

  if (rclcpp::spin_until_future_complete(node, goal_handle_future)
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Goal 전송 실패");
    return 1;
  }
  auto goal_handle = goal_handle_future.get();
  auto result_future = client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future)
      == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "웨이포인트 주행 완료");
  }

  rclcpp::shutdown();
  return 0;
}