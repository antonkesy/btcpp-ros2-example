#include "actions_cpp/get_runtime/get_runtime.hpp"

#include <memory>
#include <string>

namespace example
{

GetRuntimeAction::GetRuntimeAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<action_interfaces::action::Runtime>(
    xml_tag_name, action_name, conf)
{
}

BT::NodeStatus GetRuntimeAction::on_success()
{
  const auto feedback = result_.result->formatted;
  setOutput("output", feedback);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace example

#include <fstream>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<example::GetRuntimeAction>(name, "get_runtime_action", config);
  };

  factory.registerBuilder<example::GetRuntimeAction>("GetRuntime", builder);
  auto xml = BT::writeTreeNodesModelXML(factory, false);
  std::ofstream("install/nodes.xml") << xml;
}
