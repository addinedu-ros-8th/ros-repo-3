#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "shared_interfaces/action/move_to_goal.hpp"

class MoveToGoalServer : public rclcpp::Node
{
  using MoveToGoal = shared_interfaces::action::MoveToGoal;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToGoal>;

public:
  MoveToGoalServer()
  : Node("move_to_goal_action_server")
  {
    server_ = rclcpp_action::create_server<MoveToGoal>(
      this,
      "/navigation/move_to_goal",
      std::bind(&MoveToGoalServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveToGoalServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveToGoalServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToGoal::Goal> goal)
  {
    RCLCPP_INFO(get_logger(),
      "이동 요청 수신: id=%u, x=%.2f, y=%.2f, θ=%.2f",
      goal->roscar_id, goal->goal_x, goal->goal_y, goal->theta);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "이동 요청 취소됨");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto feedback = std::make_shared<MoveToGoal::Feedback>();
    auto result   = std::make_shared<MoveToGoal::Result>();

    for (uint8_t i = 0; i <= 100; i += 10) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        return;
      }
      feedback->progress  = i;
      feedback->current_x = 1; //실제값 수정
      feedback->current_y = 1; //실제값 수정
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    result->success = true;
    result->message = "목표 도착";
    goal_handle->succeed(result);
  }

  rclcpp_action::Server<MoveToGoal>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveToGoalServer>());
  rclcpp::shutdown();
  return 0;
}
