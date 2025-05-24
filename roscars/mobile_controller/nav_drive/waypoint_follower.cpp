#include <memory>
#include <vector>
#include <cmath>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

  // 웨이포인트 좌표 (idx 3: 중간, idx 6: 최종)
  std::vector<std::pair<double,double>> wps = {
    {0.55, -0.1},
    {0.48,  0.48},
    {0.05,  0.42},
    {0.05,  0.79},  // 경유지점
    {0.05,  1.27},
    {0.45,  1.26},
    {0.60,  2.00}   // 최종 목적지
  };

  // 액션 클라이언트
  auto client = rclcpp_action::create_client<FollowWaypoints>(node, "follow_waypoints");
  client->wait_for_action_server(std::chrono::seconds(5));

  // 피드백 옵션
  rclcpp_action::Client<FollowWaypoints>::SendGoalOptions opts;
  opts.feedback_callback = [node](auto, const auto& fb) {
    RCLCPP_INFO(node->get_logger(), "현재 웨이포인트: %u", fb->current_waypoint);
  };

  // 1단계: 0→1→2→3 (경유지점)
  {
    auto goal1 = make_goal({0,1,2,3}, wps);
    auto f1 = client->async_send_goal(goal1, opts);
    rclcpp::spin_until_future_complete(node, f1);
    auto gh1 = f1.get();
    auto r1  = client->async_get_result(gh1);
    rclcpp::spin_until_future_complete(node, r1);
    RCLCPP_INFO(node->get_logger(), "경유지점 도착: %s",
      r1.get().result->missed_waypoints.empty() ? "완료" : "실패");
  }

  // 2단계: 경유지점에서 180° 회전 & 5초 대기
  {
    auto rot = make_goal({3}, wps, M_PI);
    auto fR  = client->async_send_goal(rot, opts);
    rclcpp::spin_until_future_complete(node, fR);
    auto ghR = fR.get();
    auto rR  = client->async_get_result(ghR);
    rclcpp::spin_until_future_complete(node, rR);
    RCLCPP_INFO(node->get_logger(), "회전 완료, 5초 대기...");
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }

  // travel yaw 계산 (마지막 세그먼트: 5→6)
  double travel_yaw = std::atan2(
    wps[6].second - wps[5].second,
    wps[6].first  - wps[5].first);

  // 3단계: 경유지점→최종 목적지 (최종 orientation = travel_yaw + 90°)
  {
    double final_yaw = travel_yaw + M_PI_2;
    auto goal2 = make_goal({3,4,5,6}, wps, final_yaw);
    auto f2     = client->async_send_goal(goal2, opts);
    rclcpp::spin_until_future_complete(node, f2);
    auto gh2    = f2.get();
    auto r2     = client->async_get_result(gh2);
    rclcpp::spin_until_future_complete(node, r2);
    RCLCPP_INFO(node->get_logger(), "최종 목적지 도착: %s",
      r2.get().result->missed_waypoints.empty() ? "완료" : "실패");
  }

  rclcpp::shutdown();
  return 0;
}
