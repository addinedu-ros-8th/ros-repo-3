#include <chrono>
#include <memory>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "shared_interfaces/action/move_to_goal.hpp"
#include "shared_interfaces/msg/roscar_pose_update.hpp"

using namespace std::chrono_literals;

class MoveToGoalServer : public rclcpp::Node
{
  using MoveToGoal = shared_interfaces::action::MoveToGoal;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToGoal>;
  using PoseMsg    = shared_interfaces::msg::RoscarPoseUpdate;

public:
  MoveToGoalServer()
  : Node("move_to_goal_action_server")
  {
    // 액션 서버 생성
    server_ = rclcpp_action::create_server<MoveToGoal>(
      this,
      "move_to_goal",
      std::bind(&MoveToGoalServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveToGoalServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveToGoalServer::handle_accepted, this, std::placeholders::_1));

    // 현재 위치 구독
    pose_sub_ = this->create_subscription<PoseMsg>(
      "roscar_pose_update", 10,
      std::bind(&MoveToGoalServer::pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "✅ MoveToGoal Action Server ready");
  }

private:
  // Goal 요청 처리: 항상 ACCEPT
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToGoal::Goal> goal)
  {
    RCLCPP_INFO(get_logger(),
      "Goal received: id=%u, x=%.2f, y=%.2f, θ=%.2f",
      goal->roscar_id,
      goal->goal_x, goal->goal_y, goal->theta);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 취소 요청 처리
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "Goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Goal 수락 시 실행
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{[this, goal_handle]() { this->execute(goal_handle); }}.detach();
  }

  // 실제 동작 수행 및 피드백 전송
  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto goal     = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveToGoal::Feedback>();
    auto result   = std::make_shared<MoveToGoal::Result>();

    const double tol = 0.05;  // 목표 도달 허용 오차[m]
    rclcpp::Rate rate(10);

    while (rclcpp::ok()) {
      // 취소 요청 검사
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Execution canceled");
        return;
      }

      // 현재 위치로부터 목표까지 거리 계산
      double dx   = goal->goal_x - current_x_;
      double dy   = goal->goal_y - current_y_;
      double dist = std::hypot(dx, dy);

      // 진행률 계산 (목표까지 남은 비율)
      double total_dist = std::hypot(goal->goal_x, goal->goal_y);
      double progress = total_dist > 0.0
                        ? (1.0 - dist / total_dist) * 100.0
                        : 100.0;

      // 피드백 전송
      feedback->progress  = static_cast<uint8_t>(std::clamp(progress, 0.0, 100.0));
      feedback->current_x = current_x_;
      feedback->current_y = current_y_;
      goal_handle->publish_feedback(feedback);

      // 목표 도달 조건
      if (dist < tol) {
        result->success = true;
        result->message = "Goal reached";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal reached successfully");
        return;
      }

      // TODO: 실제 모터 제어 로직 호출
      // move_robot_towards(goal->goal_x, goal->goal_y, goal->theta);

      rate.sleep();
    }

    // 노드가 종료될 경우 실패 처리
    result->success = false;
    result->message = "Node shutdown before reaching goal";
    goal_handle->abort(result);
  }

  // 현재 위치 콜백: RoscarPoseUpdate.msg 필드명에 맞게 수정
  void pose_callback(const PoseMsg::SharedPtr msg)
  {
    current_x_     = msg->pos_x;
    current_y_     = msg->pos_y;
    current_theta_ = msg->heading_theta;
  }

  // 액션 서버와 구독 멤버
  rclcpp_action::Server<MoveToGoal>::SharedPtr server_;
  rclcpp::Subscription<PoseMsg>::SharedPtr     pose_sub_;

  // 현재 위치 데이터
  double current_x_{0.0};
  double current_y_{0.0};
  double current_theta_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToGoalServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
