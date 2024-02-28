#include "decorator_cpp/random_delay/random_delay.hpp"

#include <memory>
#include <random>
#include <string>

namespace example
{

BT::NodeStatus RandomDelay::tick()
{
  if (!delay_started_) {
    delay_complete_ = false;
    delay_aborted_ = false;
    delay_started_ = true;
    setStatus(BT::NodeStatus::RUNNING);

    const auto msec = [this]() -> unsigned int {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<float> dis(min_ms_, max_ms_);
      return dis(gen);
    }();

    timer_id_ = timer_.add(std::chrono::milliseconds(msec), [this](bool aborted) {
      std::unique_lock<std::mutex> lk(delay_mutex_);
      delay_complete_ = (!aborted);
      if (!aborted) {
        emitStateChanged();
      }
    });
  }

  std::unique_lock<std::mutex> lk(delay_mutex_);

  if (delay_aborted_) {
    delay_aborted_ = false;
    delay_started_ = false;
    return BT::NodeStatus::FAILURE;
  } else if (delay_complete_) {
    auto child_status = child()->executeTick();
    if (child_status != BT::NodeStatus::RUNNING) {
      delay_started_ = false;
      delay_aborted_ = false;
      resetChild();
    }
    return child_status;
  } else {
    return BT::NodeStatus::RUNNING;
  }
}
}  // namespace example

#include <fstream>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<example::RandomDelay>("RandomDelay");
  auto xml = BT::writeTreeNodesModelXML(factory, false);
  std::ofstream("install/nodes.xml") << xml;
}
