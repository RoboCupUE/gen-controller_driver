#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "controller_interface/controller_interface.hpp"

namespace controller_teleop
{

class TeleopTwistJoyNode : public rclcpp::Node
{
public:
  TeleopTwistJoyNode();

private:
  void onJoy(sensor_msgs::msg::Joy::ConstSharedPtr joy_msg);
  void onWatchdog();

  double linear_max_speed_;
  double angular_max_speed_;
  double angular_speed_multiplier_;
  double linear_speed_multiplier_;

  std::string vel_topic;


  std::unique_ptr<controller_interface::ControllerInterface> controller_if_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr timer_watchdog_;
};

}  // namespace controller_teleop
