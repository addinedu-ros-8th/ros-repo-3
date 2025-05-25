#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "shared_interfaces/action/scan_inventory.hpp"

class ScanInventoryServer : public rclcpp::Node
{
  using ScanInv   = shared_interfaces::action::ScanInventory;
  using GoalHandle= rclcpp_action::ServerGoalHandle<ScanInv>;

public:
  ScanInventoryServer()
  : Node("scan_inventory_action_server")
  {
    server_ = rclcpp_action::create_server<ScanInv>(
      this,
      "/inventory/scan_items",
      std::bind(&ScanInventoryServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ScanInventoryServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ScanInventoryServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const ScanInv::Goal> goal)
  {
    RCLCPP_INFO(get_logger(),
      "스캔 시작 요청: zone=%s, id=%u",
      goal->scan_zone.c_str(), goal->roscar_id);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "스캔 요청 취소");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> gh)
  {
    std::thread{[this, gh]() { execute(gh); }}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> gh)
  {
    auto feedback = std::make_shared<ScanInv::Feedback>();
    auto result   = std::make_shared<ScanInv::Result>();
    uint32_t total = 0;

    // 예시: 단일 존 처리
    feedback->zone         = gh->get_goal()->scan_zone;
    feedback->scanned_item = 1; // 실제값 수정
    gh->publish_feedback(feedback);
    total += feedback->scanned_item;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    result->total_items = total;
    result->success     = true;
    gh->succeed(result);
  }

  rclcpp_action::Server<ScanInv>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanInventoryServer>());
  rclcpp::shutdown();
  return 0;
}
