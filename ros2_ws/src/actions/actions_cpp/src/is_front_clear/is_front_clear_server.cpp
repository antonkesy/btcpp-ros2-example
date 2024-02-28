#include <functional>
#include <memory>
#include <sensor_msgs/msg/range.hpp>
#include <thread>

#include "action_interfaces/action/is_front_clear.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class IsFrontClearActionServer : public rclcpp::Node
{
public:
  using IsFrontClear = action_interfaces::action::IsFrontClear;
  using GoalHandleIsFrontClear = rclcpp_action::ServerGoalHandle<IsFrontClear>;

  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr leftDisSub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rightDisSub_;

  sensor_msgs::msg::Range lastLeft_{};
  sensor_msgs::msg::Range lastRight_{};

  explicit IsFrontClearActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("is_front_clear_action_server", options)
  {
    using namespace std::placeholders;

    leftDisSub_ = this->create_subscription<sensor_msgs::msg::Range>(
      "/left_front_distance", 10,
      [this](const sensor_msgs::msg::Range::SharedPtr msg) { lastLeft_ = *msg; });

    rightDisSub_ = this->create_subscription<sensor_msgs::msg::Range>(
      "/right_front_distance", 10,
      [this](const sensor_msgs::msg::Range::SharedPtr msg) { lastRight_ = *msg; });

    this->action_server_ = rclcpp_action::create_server<IsFrontClear>(
      this, "is_front_clear_action",
      std::bind(&IsFrontClearActionServer::handle_goal, this, _1, _2),
      std::bind(&IsFrontClearActionServer::handle_cancel, this, _1),
      std::bind(&IsFrontClearActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<IsFrontClear>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const IsFrontClear::Goal> goal)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleIsFrontClear> goal_handle)
  {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleIsFrontClear> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&IsFrontClearActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleIsFrontClear> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<IsFrontClear::Result>();

    const auto detect_distance = static_cast<float>(goal->distance);
    const auto cur_min_dis = std::min(lastLeft_.range, lastRight_.range);
    if (cur_min_dis > detect_distance)
      goal_handle->succeed(result);
    else
      goal_handle->abort(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IsFrontClearActionServer>();

  rclcpp::spin(node);

  return 0;
}
