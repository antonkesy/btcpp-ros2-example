#include "control_cpp/select_random/select_random.hpp"

#include <memory>
#include <string>

namespace example
{

BT::NodeStatus SelectRandom::tick()
{
  const size_t children_count = children_nodes_.size();

  if (children_count == 0) {
    throw BT::RuntimeError("SelectRandom node '" + name() + "': has no children");
  }

  setStatus(BT::NodeStatus::RUNNING);

  const auto random_id = std::rand() % children_count;
  TreeNode * current_child_node = children_nodes_[random_id];
  const BT::NodeStatus child_status = current_child_node->executeTick();

  return child_status;
}
}  // namespace example

#include <fstream>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<example::SelectRandom>("SelectRandom");
  auto xml = BT::writeTreeNodesModelXML(factory, false);
  std::ofstream("install/nodes.xml") << xml;
}
