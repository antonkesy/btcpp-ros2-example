#include <behaviortree_cpp_v3/bt_factory.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "rclcpp/rclcpp.hpp"

static void FillNav2Blackboard(BT::Blackboard & blackboard, std::shared_ptr<rclcpp::Node> & node)
{
  blackboard.set("node", node);
  blackboard.set(
    "bt_loop_duration",
    std::chrono::milliseconds(10));  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  blackboard.set(
    "server_timeout",
    std::chrono::milliseconds(20));  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  blackboard.set(
    "wait_for_service_timeout",
    std::chrono::milliseconds(20));  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
}

std::string GetTreePath(rclcpp::Node & node)
{
  const auto * const tree_name = "example_main.xml";

  node.declare_parameter("trees_path", rclcpp::PARAMETER_STRING);

  std::string trees_path{};
  if (!node.get_parameter("trees_path", trees_path)) {
    std::cerr << "parameter `trees_path` is not set" << std::endl;
    exit(-1);
  }
  trees_path += std::string("/") + tree_name;
  std::cerr << "Load " << trees_path << std::endl;
  return trees_path;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("coordinator");

  // XX_action, XY_decorator -> XX = cmake executable/lib name
  nav2_behavior_tree::BehaviorTreeEngine a({
    "select_random_control",
    "is_front_clear_action",
    "move_action",
    "print_action",
    "random_delay_decorator",
    "get_runtime_action",
  });

  auto blackboard = BT::Blackboard::create();
  FillNav2Blackboard(*blackboard, node);

  auto tree = a.createTreeFromFile(GetTreePath(*node), blackboard);

  BT::FileLogger logger_file(tree, "log/latest_tree.fbl");
  BT::PublisherZMQ publisher_zmq(tree, 5555);

  a.run(
    &tree, [] { /* every tick */ }, [] { return false; });

  rclcpp::shutdown();

  return 0;
}
