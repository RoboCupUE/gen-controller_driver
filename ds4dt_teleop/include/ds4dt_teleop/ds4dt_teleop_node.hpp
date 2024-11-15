#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "ds4dt_interface/ds4dt_interface.hpp"

namespace ds4dt_teleop
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


  std::unique_ptr<ds4dt_interface::PlayStationInterface> ds4dt_if_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr timer_watchdog_;
};

}  // namespace ds4dt_teleop
