#include "ds4dt_test/ds4dt_test.hpp"

namespace ds4dt_test
{

DS4DTTest::DS4DTTest()
: rclcpp::Node("ds4dt_test")
{
  // Inicializar PlayStationInterface sin especificar hw_type
  this->ds4dt_if_ = std::make_unique<ds4dt_interface::PlayStationInterface>();

  this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
    std::bind(&DS4DTTest::onJoy, this, std::placeholders::_1));

  if (this->joy_sub_->get_publisher_count() == 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "El nodo joy no está lanzado");
  }
}

void DS4DTTest::onJoy(sensor_msgs::msg::Joy::ConstSharedPtr joy_msg)
{
  this->ds4dt_if_->setJoyMsg(joy_msg);

  if (this->ds4dt_if_->pressedCross()) {
    RCLCPP_INFO(this->get_logger(), " ☓ Pressed!");
  }

  if (this->ds4dt_if_->pressedCircle()) {
    RCLCPP_INFO(this->get_logger(), " ○ Pressed!");
  }

  if (this->ds4dt_if_->pressedTriangle()) {
    RCLCPP_INFO(this->get_logger(), " △ Pressed!");
  }

  if (this->ds4dt_if_->pressedSquare()) {
    RCLCPP_INFO(this->get_logger(), " □ Pressed!");
  }

  if (this->ds4dt_if_->pressedL1()) {
    RCLCPP_INFO(this->get_logger(), " L1 Pressed!");
  }

  if (this->ds4dt_if_->pressedR1()) {
    RCLCPP_INFO(this->get_logger(), " R1 Pressed!");
  }

  if (this->ds4dt_if_->pressedL2Analog() < -0.0) {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      " L2 Pressed! : " <<
      std::right << std::setw(6) <<
      std::fixed << std::setprecision(3) <<
      this->ds4dt_if_->pressedL2Analog());
  }

  if (this->ds4dt_if_->pressedR2Analog() < -0.0) {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      " R2 Pressed! : " <<
      std::right << std::setw(6) <<
      std::fixed << std::setprecision(3) <<
      this->ds4dt_if_->pressedR2Analog());
  }
  

  if (this->ds4dt_if_->pressedSelect()) {
    RCLCPP_INFO(this->get_logger(), " Select Pressed!");
  }

  if (this->ds4dt_if_->pressedStart()) {
    RCLCPP_INFO(this->get_logger(), " Start Pressed!");
  }

  if (this->ds4dt_if_->pressedPS()) {
    RCLCPP_INFO(this->get_logger(), " PS Pressed!");
  }

  if (this->ds4dt_if_->pressedDPadUp()) {
    RCLCPP_INFO(this->get_logger(), " DPad Up Pressed!");
  }

  if (this->ds4dt_if_->pressedDPadDown()) {
    RCLCPP_INFO(this->get_logger(), " DPad Down Pressed!");
  }

  if (this->ds4dt_if_->pressedDPadLeft()) {
    RCLCPP_INFO(this->get_logger(), " DPad Left Pressed!");
  }

  if (this->ds4dt_if_->pressedDPadRight()) {
    RCLCPP_INFO(this->get_logger(), " DPad Right Pressed!");
  }

  if(this->ds4dt_if_->pressedLStick()) {
    RCLCPP_INFO(this->get_logger(), " LStick Pressed!");
  }

  if(this->ds4dt_if_->pressedRStick()) {
    RCLCPP_INFO(this->get_logger(), " RStick Pressed!");
  }

  
    if(this->ds4dt_if_->pressedTouchpad()) {
    RCLCPP_INFO(this->get_logger(), " Touchpad Pressed!");
  }
  


  if (
    std::abs(this->ds4dt_if_->tiltedStickLX()) > 1e-2 ||
    std::abs(this->ds4dt_if_->tiltedStickLY()) > 1e-2)
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      " LStick Tilted!" <<
      " x:" << std::fixed << std::setw(6) << std::setprecision(3) <<
      this->ds4dt_if_->tiltedStickLX() <<
      " y:" << std::fixed << std::setw(6) << std::setprecision(3) <<
      this->ds4dt_if_->tiltedStickLY());
  }

  if (
    std::abs(this->ds4dt_if_->tiltedStickRX()) > 1e-2 ||
    std::abs(this->ds4dt_if_->tiltedStickRY()) > 1e-2)
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      " RStick Tilted!" <<
      " x:" << std::fixed << std::setw(6) << std::setprecision(3) <<
      this->ds4dt_if_->tiltedStickRX() <<
      " y:" << std::fixed << std::setw(6) << std::setprecision(3) <<
      this->ds4dt_if_->tiltedStickRY());
  }

  if (this->ds4dt_if_->pressedAny()) {
    using namespace std::chrono_literals;  // NOLINT
    rclcpp::sleep_for(200ms);
  }
}

}  // namespace ds4dt_test

// Añadir la función main
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ds4dt_test::DS4DTTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
