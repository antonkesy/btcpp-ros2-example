#pragma once

#include <atomic>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "behaviortree_cpp_v3/decorators/timer_queue.h"

namespace example
{

class RandomDelay : public BT::DecoratorNode
{
public:
  RandomDelay(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::DecoratorNode(name, conf)
  {
    getInput<unsigned int>("min_ms", min_ms_);
    getInput<unsigned int>("max_ms", max_ms_);
  }

  ~RandomDelay() override { halt(); }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<unsigned int>("min_ms", 0, "Minimum delay in milliseconds"),
      BT::InputPort<unsigned int>("max_ms", 0, "Maximum delay in milliseconds")};
  }

  void halt() override
  {
    delay_started_ = false;
    timer_.cancelAll();
    DecoratorNode::halt();
  }

private:
  BT::TimerQueue<> timer_;
  uint64_t timer_id_;

  bool delay_started_ = false;
  std::atomic_bool delay_complete_ = false;
  bool delay_aborted_ = false;
  unsigned int min_ms_{};
  unsigned int max_ms_{};
  std::mutex delay_mutex_{};

  BT::NodeStatus tick() override;
};

}  // namespace example
