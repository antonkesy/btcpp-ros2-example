#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <thread>

#include "action_interfaces/action/move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MoveActionServer : public rclcpp::Node
{
public:
  using Move = action_interfaces::action::Move;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

  explicit MoveActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("move_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Move>(
      this, "move_action", std::bind(&MoveActionServer::handle_goal, this, _1, _2),
      std::bind(&MoveActionServer::handle_cancel, this, _1),
      std::bind(&MoveActionServer::handle_accepted, this, _1));

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  rclcpp_action::Server<Move>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Move::Goal> goal)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&MoveActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    const auto goal = goal_handle->get_goal();

    while (rclcpp::ok()) {
      // std::cerr << "Move: " << goal->rotation << " " << goal->speed
      // << std::endl;

      auto feedback = std::make_shared<Move::Feedback>();
      auto result = std::make_shared<Move::Result>();

      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
      }

      geometry_msgs::msg::Twist msg;
      msg.linear.x = goal->speed;
      msg.angular.x = goal->rotation;
      pub_->publish(msg);

      // TODO: rate 1 too slow for reaction but less is goal missing
      // TODO: "Goal handle is not known to this client."
      // rclcpp::Rate(1).sleep();

      // TODO: no effect
      goal_handle->succeed(result);
      break;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveActionServer>();

  rclcpp::spin(node);

  return 0;
}
