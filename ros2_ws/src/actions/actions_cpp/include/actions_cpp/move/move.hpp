#pragma once

#include <string>

#include "action_interfaces/action/move.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace example
{

class MoveAction : public nav2_behavior_tree::BtActionNode<action_interfaces::action::Move>
{
public:
  MoveAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    using BT::InputPort;
    return providedBasicPorts({
      InputPort<int>("speed"),
      InputPort<int>("sides"),
      InputPort<int>("rotation"),
      InputPort<bool>("keepRunning"),
    });
  }
};

}  // namespace example
