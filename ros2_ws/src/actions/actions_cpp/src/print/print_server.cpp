#include <functional>
#include <memory>
#include <thread>

#include "action_interfaces/action/print.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class PrintActionServer : public rclcpp::Node
{
public:
  using Print = action_interfaces::action::Print;
  using GoalHandlePrint = rclcpp_action::ServerGoalHandle<Print>;

  explicit PrintActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("print_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Print>(
      this, "print_action", std::bind(&PrintActionServer::handle_goal, this, _1, _2),
      std::bind(&PrintActionServer::handle_cancel, this, _1),
      std::bind(&PrintActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Print>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Print::Goal> goal)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePrint> goal_handle)
  {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePrint> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&PrintActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePrint> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Print::Result>();

    RCLCPP_INFO(get_logger(), "%s", std::string(goal->text).c_str());
    goal_handle->succeed(result);
  }

};  //

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PrintActionServer>();

  rclcpp::spin(node);

  return 0;
}
