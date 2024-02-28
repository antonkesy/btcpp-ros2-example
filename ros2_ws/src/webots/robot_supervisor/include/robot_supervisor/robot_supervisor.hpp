#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Node.hpp>

#include "webots/Supervisor.hpp"

namespace example
{
class RobotSupervisor : public rclcpp::Node
{
public:
  explicit RobotSupervisor(const rclcpp::NodeOptions & options);
  RobotSupervisor(const RobotSupervisor &) = delete;
  RobotSupervisor(RobotSupervisor &&) = delete;
  RobotSupervisor & operator=(const RobotSupervisor &) = delete;
  RobotSupervisor & operator=(RobotSupervisor &&) = delete;

  ~RobotSupervisor() override = default;
  void RunLoop();

private:
  std::unique_ptr<webots::Supervisor> supervisor_;
  webots::Motor * leftMotor_;
  webots::Motor * rightMotor_;
  std::thread webotsThread_{};
  webots::Node * robotNode_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr posePub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr leftFrontDistancePub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr rightFrontDistancePub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  geometry_msgs::msg::Twist cmd_vel_msg_;

  webots::DistanceSensor * leftDistanceSensor_;
  webots::DistanceSensor * rightDistanceSensor_;

  void SetSpeed();
  void PublishPose();
  void PublishDistance();
  void CmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg);
};
}  // namespace example
