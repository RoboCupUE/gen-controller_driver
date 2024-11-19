#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "controller_interface/controller_interface.hpp"

namespace controller_test
{
class ControlTest : public rclcpp::Node
{
public:
    ControlTest();

private:
    void onJoy(sensor_msgs::msg::Joy::ConstSharedPtr joy_msg);

    std::unique_ptr<controller_interface::ControllerInterface> controller_if_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};
} // namespace ds4dt_test
