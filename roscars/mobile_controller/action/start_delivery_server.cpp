#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shared_interfaces/action/start_delivery.hpp"

class StartDeliveryServer : public rclcpp::Node {
public:
  using StartDelivery = shared_interfaces::action::StartDelivery;
  using GoalHandle = rclcpp_action::ServerGoalHandle<StartDelivery>;

  StartDeliveryServer()
  : Node("start_delivery_server") {
    action_server_ = rclcpp_action::create_server<StartDelivery>(
      this,
      "start_delivery",
      std::bind(&StartDeliveryServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&StartDeliveryServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&StartDeliveryServer::handle_accepted, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "StartDelivery Action Server is ready.");
  }

private:
  rclcpp_action::Server<StartDelivery>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                          std::shared_ptr<const StartDelivery::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received StartDelivery goal for delivery_id: %u", goal->delivery_id);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&StartDeliveryServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    auto result = std::make_shared<StartDelivery::Result>();
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Delivery completed");
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StartDeliveryServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
