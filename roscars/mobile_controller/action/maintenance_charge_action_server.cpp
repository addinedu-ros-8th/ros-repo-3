#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "shared_interfaces/action/maintenance_charge.hpp"

class MaintenanceChargeServer : public rclcpp::Node
{
  using MCharge    = shared_interfaces::action::MaintenanceCharge;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MCharge>;

public:
  MaintenanceChargeServer()
  : Node("maintenance_charge_action_server")
  {
    server_ = rclcpp_action::create_server<MCharge>(
      this,
      "/maintenance/charge",
      std::bind(&MaintenanceChargeServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MaintenanceChargeServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MaintenanceChargeServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MCharge::Goal> goal)
  {
    RCLCPP_INFO(get_logger(),
      "충전 시작 요청: id=%u, zone=%s",
      goal->roscar_id, goal->charge_zone.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "충전 요청 취소");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> gh)
  {
    std::thread{[this, gh]() { execute(gh); }}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> gh)
  {
    auto feedback = std::make_shared<MCharge::Feedback>();
    auto result   = std::make_shared<MCharge::Result>();

    const std::vector<std::pair<std::string, float>> status_list = {
        {u8"이동 중", 20.0f},
        {u8"충전 중", 80.0f},
        {u8"완료", 100.0f}
    };

    for (const auto & s : status_list) {
      if (gh->is_canceling()) {
        result->success = false;
        gh->canceled(result);
        return;
      }

      feedback->phase = s.first;
      feedback->battery_percent = s.second;
      gh->publish_feedback(feedback);
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    result->success = true;
    result->message = "충전 완료";
    gh->succeed(result);
  }

  rclcpp_action::Server<MCharge>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MaintenanceChargeServer>());
  rclcpp::shutdown();
  return 0;
}
