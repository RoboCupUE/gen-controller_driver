#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "ds4dt_interface/ds4dt_interface.hpp"

namespace ds4dt_test
{
class DS4DTTest : public rclcpp::Node
{
public:
    DS4DTTest();

private:
    void onJoy(sensor_msgs::msg::Joy::ConstSharedPtr joy_msg);

    std::unique_ptr<ds4dt_interface::PlayStationInterface> ds4dt_if_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};
} // namespace ds4dt_test
