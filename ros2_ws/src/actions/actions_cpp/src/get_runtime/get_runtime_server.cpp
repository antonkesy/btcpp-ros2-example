#include <functional>
#include <memory>
#include <thread>

#include "action_interfaces/action/runtime.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class GetRuntimeServer : public rclcpp::Node
{
public:
  using GetRuntime = action_interfaces::action::Runtime;
  using GoalHandleGetRuntime = rclcpp_action::ServerGoalHandle<GetRuntime>;

  explicit GetRuntimeServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("get_runtime_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GetRuntime>(
      this, "get_runtime_action", std::bind(&GetRuntimeServer::handle_goal, this, _1, _2),
      std::bind(&GetRuntimeServer::handle_cancel, this, _1),
      std::bind(&GetRuntimeServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<GetRuntime>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const GetRuntime::Goal> goal)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGetRuntime> goal_handle)
  {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGetRuntime> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&GetRuntimeServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGetRuntime> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<GetRuntime::Result>();

    const auto get_current_seconds = []() {
      return std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
    };

    static const auto startTime = get_current_seconds();
    const auto now = get_current_seconds();

    std::string seconds = std::to_string(now - startTime);
    result->formatted = seconds;
    goal_handle->succeed(result);
  }

};  //

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GetRuntimeServer>();

  rclcpp::spin(node);

  return 0;
}
