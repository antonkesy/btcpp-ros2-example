#pragma once

#include <sensor_msgs/msg/range.hpp>
#include <string>

#include "behaviortree_cpp_v3/control_node.h"
#include "rclcpp/rclcpp.hpp"

namespace example
{

class SelectRandom : public BT::ControlNode
{
public:
  SelectRandom(const std::string & action_name, const BT::NodeConfiguration & conf)
  : BT::ControlNode(action_name, conf)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

private:
  BT::NodeStatus tick() override;
};

}  // namespace example
