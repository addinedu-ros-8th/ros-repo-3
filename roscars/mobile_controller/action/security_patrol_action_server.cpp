#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "shared_interfaces/action/security_patrol.hpp"

class SecurityPatrolServer : public rclcpp::Node
{
  using Patrol    = shared_interfaces::action::SecurityPatrol;
  using GoalHandle= rclcpp_action::ServerGoalHandle<Patrol>;

public:
  SecurityPatrolServer()
  : Node("security_patrol_action_server")
  {
    server_ = rclcpp_action::create_server<Patrol>(
      this,
      "/security/patrol",
      std::bind(&SecurityPatrolServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SecurityPatrolServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&SecurityPatrolServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Patrol::Goal> goal)
  {
    RCLCPP_INFO(get_logger(),
      "순찰 시작 요청: id=%u, zone=%s",
      goal->roscar_id, goal->patrol_zone.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "순찰 요청 취소");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> gh)
  {
    std::thread{[this, gh]() { execute(gh); }}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> gh)
  {
    auto feedback = std::make_shared<Patrol::Feedback>();
    auto result   = std::make_shared<Patrol::Result>();
    const std::vector<std::string> zones = {"A","B","C"};

    for (auto & z : zones) {
      if (gh->is_canceling()) {
        result->success = false;
        gh->canceled(result);
        return;
      }
      feedback->current_zone = z;
      feedback->event        = z; //실제값 수정
      gh->publish_feedback(feedback);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    result->success = true;
    result->message = "순찰 완료";
    gh->succeed(result);
  }

  rclcpp_action::Server<Patrol>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SecurityPatrolServer>());
  rclcpp::shutdown();
  return 0;
}
