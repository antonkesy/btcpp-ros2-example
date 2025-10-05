#include "actions_cpp/is_front_clear/is_front_clear.hpp"

#include <memory>
#include <string>

namespace example
{

IsFrontClearAction::IsFrontClearAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<action_interfaces::action::IsFrontClear>(
    xml_tag_name, action_name, conf)
{
  getInput<int>("distance", goal_.distance);
}

}  // namespace example

#include <fstream>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<example::IsFrontClearAction>(name, "is_front_clear_action", config);
  };

  factory.registerBuilder<example::IsFrontClearAction>("IsFrontClear", builder);
  auto xml = BT::writeTreeNodesModelXML(factory, false);
  std::ofstream("install/nodes.xml") << xml;
}
