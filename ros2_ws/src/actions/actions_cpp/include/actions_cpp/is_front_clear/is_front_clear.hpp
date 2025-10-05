#pragma once

#include <string>

#include "action_interfaces/action/is_front_clear.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace example
{

class IsFrontClearAction
: public nav2_behavior_tree::BtActionNode<action_interfaces::action::IsFrontClear>
{
public:
  IsFrontClearAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<int>("distance", 0, "Distance to check")});
  }
};

}  // namespace example
