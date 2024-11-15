#include "ds4dt_teleop/ds4dt_teleop_node.hpp"

namespace ds4dt_teleop
{
TeleopTwistJoyNode::TeleopTwistJoyNode()
: rclcpp::Node("teleop_twist_joy_node")
{
  this->linear_max_speed_ =
    this->declare_parameter<double>("linear_speed", 0.2);
  this->angular_max_speed_ =
    this->declare_parameter<double>("angular_speed", 0.5);
  this->angular_speed_multiplier_ =
    this->declare_parameter<double>("angular_speed_multiplier", 0.05);
  this->linear_speed_multiplier_ =
    this->declare_parameter<double>("linear_speed_multiplier", 0.02);

  this->ds4dt_if_ = std::make_unique<ds4dt_interface::PlayStationInterface>();

  using namespace std::placeholders;  // NOLINT
  this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&TeleopTwistJoyNode::onJoy, this, _1));

  this->twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10).reliable().durability_volatile());

  using namespace std::chrono_literals; // NOLINT
  this->timer_watchdog_ = this->create_wall_timer(
    1s, std::bind(&TeleopTwistJoyNode::onWatchdog, this));

  if (this->joy_sub_->get_publisher_count() == 0) {
    RCLCPP_WARN(this->get_logger(), "Joy node not launched");
  }
}


void TeleopTwistJoyNode::onJoy(sensor_msgs::msg::Joy::ConstSharedPtr joy_msg)
{
  this->timer_watchdog_->reset();
  this->ds4dt_if_->setJoyMsg(joy_msg);

  static bool stopped = true;
  static bool circle_pressed = false;
  static bool cross_pressed = false;
  static bool triangle_pressed = false;
  static bool square_pressed = false;

  //Change speed
  if (this->ds4dt_if_->pressedCircle()) {
    if (!circle_pressed) {
      linear_max_speed_ += linear_speed_multiplier_;
      RCLCPP_INFO(this->get_logger(), "Linear speed increased to %f", linear_max_speed_);
      circle_pressed = true;
    }
  } else {
    circle_pressed = false;
  }

  if (this->ds4dt_if_->pressedSquare()) {
    if (!triangle_pressed) {
      angular_max_speed_ += angular_speed_multiplier_;
      RCLCPP_INFO(this->get_logger(), "Angular speed increased to %f", angular_max_speed_);
      triangle_pressed = true;
    }
  } else {
    triangle_pressed = false;
  }

  if (this->ds4dt_if_->pressedTriangle() && angular_max_speed_ > 0.0) {
    if (!square_pressed) {
      angular_max_speed_ -= angular_speed_multiplier_;
      RCLCPP_INFO(this->get_logger(), "Angular speed decreased to %f", angular_max_speed_);
      square_pressed = true;
    }
  } else {
    square_pressed = false;
  }

  if (this->ds4dt_if_->pressedCross() && linear_max_speed_ > 0.0) {
    if (!cross_pressed) {
      linear_max_speed_ -= linear_speed_multiplier_;
      RCLCPP_INFO(this->get_logger(), "Linear speed decreased to %f", linear_max_speed_);
      cross_pressed = true;
    }
  } else {
    cross_pressed = false;
  }

  //Spin
  if (this->ds4dt_if_->pressedL1()) {
    RCLCPP_INFO(this->get_logger(), "Spinning left");
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    twist_msg->angular.set__z(this->angular_max_speed_);
    this->twist_pub_->publish(std::move(twist_msg));
    stopped = false;
  } else if (this->ds4dt_if_->pressedR1()) {
    RCLCPP_INFO(this->get_logger(), "Spinning right");
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    twist_msg->angular.set__z(-this->angular_max_speed_);
    this->twist_pub_->publish(std::move(twist_msg));
    stopped = false;
  }

  //If moving
  if (this->ds4dt_if_->isTiltedStickL()) {
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    twist_msg->linear.set__x(this->linear_max_speed_ * this->ds4dt_if_->tiltedStickLY());

    if (this->ds4dt_if_->tiltedStickLY() > sin(M_PI * 0.125)) {
      twist_msg->angular.set__z(this->angular_max_speed_ * this->ds4dt_if_->tiltedStickLX());
    } else if (this->ds4dt_if_->tiltedStickLY() < sin(-M_PI * 0.125)) {
      twist_msg->angular.set__z(-this->angular_max_speed_ * this->ds4dt_if_->tiltedStickLX());
    } else {
      twist_msg->linear.set__x(0.0);
      twist_msg->angular.set__z(this->angular_max_speed_ * this->ds4dt_if_->tiltedStickLX());
    }
    this->twist_pub_->publish(std::move(twist_msg));

    stopped = false;
  } else if (!stopped) {
    // Stop cart
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>(rosidl_runtime_cpp::MessageInitialization::ZERO);
    this->twist_pub_->publish(std::move(twist_msg));

    stopped = true;
  }
}

void TeleopTwistJoyNode::onWatchdog()
{
  RCLCPP_WARN(this->get_logger(), "Couldn't subscribe joy topic before timeout");

  // Publish zero velocity to stop vehicle
  auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>(rosidl_runtime_cpp::MessageInitialization::ZERO);
  this->twist_pub_->publish(std::move(twist_msg));

  this->timer_watchdog_->cancel();
}
}  // namespace ds4dt_node


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ds4dt_teleop::TeleopTwistJoyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}