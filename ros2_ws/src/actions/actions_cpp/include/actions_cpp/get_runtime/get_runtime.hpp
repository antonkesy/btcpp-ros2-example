#pragma once

#include <string>

#include "action_interfaces/action/runtime.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace example
{

class GetRuntimeAction : public nav2_behavior_tree::BtActionNode<action_interfaces::action::Runtime>
{
public:
  GetRuntimeAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::OutputPort<std::string>("output", "Current runtime")});
  }
};

}  // namespace example
