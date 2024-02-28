#include "robot_supervisor/robot_supervisor.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <webots/Supervisor.hpp>

#include "webots/Motor.hpp"

namespace example
{
RobotSupervisor::RobotSupervisor(const rclcpp::NodeOptions & options)
: Node("robot_supervisor", options)
{
  supervisor_ = std::make_unique<webots::Supervisor>();

  leftMotor_ = supervisor_->getMotor("left wheel motor");
  leftMotor_->setPosition(INFINITY);
  leftMotor_->setVelocity(0);
  rightMotor_ = supervisor_->getMotor("right wheel motor");
  rightMotor_->setPosition(INFINITY);
  rightMotor_->setVelocity(0);

  leftDistanceSensor_ = supervisor_->getDistanceSensor("ds0");
  leftDistanceSensor_->enable(10);
  rightDistanceSensor_ = supervisor_->getDistanceSensor("ds1");
  rightDistanceSensor_->enable(10);

  posePub_ = create_publisher<geometry_msgs::msg::Pose>("/groundtruth/pose", 10);

  leftFrontDistancePub_ = create_publisher<sensor_msgs::msg::Range>("/left_front_distance", 10);
  rightFrontDistancePub_ = create_publisher<sensor_msgs::msg::Range>("/right_front_distance", 10);

  cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
    std::bind(&RobotSupervisor::CmdVelCallback, this, std::placeholders::_1));

  robotNode_ = supervisor_->getFromDef("robot");

  webotsThread_ = std::thread([this]() { RunLoop(); });
}

void RobotSupervisor::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_msg_.linear = msg->linear;
  cmd_vel_msg_.angular = msg->angular;
}

void RobotSupervisor::RunLoop()
{
  while (rclcpp::ok()) {
    int time_step = static_cast<int>(supervisor_->getBasicTimeStep());
    if (supervisor_->step(time_step) != -1) {
      PublishPose();
      PublishDistance();
      SetSpeed();
    }
  }
}

void RobotSupervisor::PublishDistance()
{
  const auto publish_distance =
    [](webots::DistanceSensor & sensor, auto & publisher, const auto name) {
      sensor_msgs::msg::Range range;
      range.header.stamp = rclcpp::Clock().now();
      range.header.frame_id = name;
      range.min_range = static_cast<float>(sensor.getMinValue());
      range.max_range = static_cast<float>(sensor.getMaxValue());
      range.range = static_cast<float>(sensor.getMaxValue() - sensor.getValue());
      publisher->publish(range);
    };

  publish_distance(*leftDistanceSensor_, leftFrontDistancePub_, "left_front_distance");
  publish_distance(*rightDistanceSensor_, rightFrontDistancePub_, "right_front_distance");
}

void RobotSupervisor::SetSpeed()
{
  auto forward_speed = cmd_vel_msg_.linear.x;
  auto rotation_speed = cmd_vel_msg_.angular.x;
  auto command_motor_left = forward_speed;
  auto command_motor_right = forward_speed;

  if (rotation_speed != 0) {
    if (rotation_speed > 0) {
      // turn right
      command_motor_left += std::abs(rotation_speed);
      command_motor_right -= std::abs(rotation_speed);
    } else {
      // turn left
      command_motor_left -= std::abs(rotation_speed);
      command_motor_right += std::abs(rotation_speed);
    }
  }

  leftMotor_->setVelocity(command_motor_left);
  rightMotor_->setVelocity(command_motor_right);
}

void RobotSupervisor::PublishPose()
{
  const double * position = robotNode_->getField("translation")->getSFVec3f();
  const double * rotation = robotNode_->getField("rotation")->getSFRotation();
  geometry_msgs::msg::Pose pose_msg;
  // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  pose_msg.orientation.x = rotation[0];
  pose_msg.orientation.y = rotation[1];
  pose_msg.orientation.z = rotation[2];
  pose_msg.orientation.w = rotation[3];

  pose_msg.position.x = position[0];
  pose_msg.position.y = position[1];
  pose_msg.position.z = position[2];
  // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  posePub_->publish(pose_msg);
}
}  // namespace example
RCLCPP_COMPONENTS_REGISTER_NODE(example::RobotSupervisor)
