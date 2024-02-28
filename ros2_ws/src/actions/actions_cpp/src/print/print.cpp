#include "actions_cpp/print/print.hpp"

#include <memory>
#include <string>

namespace example
{

PrintAction::PrintAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<action_interfaces::action::Print>(
    xml_tag_name, action_name, conf)
{
}

void PrintAction::on_tick() { getInput<std::string>("text", goal_.text); }
}  // namespace example

#include <fstream>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<example::PrintAction>(name, "print_action", config);
  };

  factory.registerBuilder<example::PrintAction>("Print", builder);
  auto xml = BT::writeTreeNodesModelXML(factory, false);
  std::ofstream("install/nodes.xml") << xml;
}
