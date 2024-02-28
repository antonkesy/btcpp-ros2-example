#include "control_cpp/select_random/select_random.hpp"

#include <memory>
#include <random>
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

  const auto random_id = [&children_count]() -> unsigned int {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0, children_count - 1);
    return dis(gen);
  }();

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
