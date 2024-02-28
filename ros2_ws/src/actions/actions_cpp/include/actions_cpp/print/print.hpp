#pragma once

#include <string>

#include "action_interfaces/action/print.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace example
{

class PrintAction : public nav2_behavior_tree::BtActionNode<action_interfaces::action::Print>
{
public:
  PrintAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<std::string>("text", "", "Text to print")});
  }
};

}  // namespace example
