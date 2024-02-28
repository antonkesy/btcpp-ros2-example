#include "actions_cpp/move/move.hpp"

#include <memory>
#include <string>

namespace example
{

MoveAction::MoveAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<action_interfaces::action::Move>(xml_tag_name, action_name, conf)
{
  getInput<int>("speed", goal_.speed);
  getInput<int>("rotation", goal_.rotation);
}

}  // namespace example

#include <fstream>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<example::MoveAction>(name, "move_action", config);
  };

  factory.registerBuilder<example::MoveAction>("Move", builder);

  auto xml = BT::writeTreeNodesModelXML(factory, false);
  std::ofstream("install/nodes.xml") << xml;
}
