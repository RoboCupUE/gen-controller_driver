#include "ds4dt_interface/ds4dt_interface.hpp"

namespace ds4dt_interface
{
    ControllerInterface::ControllerInterface()
        : LOGGER_(rclcpp::get_logger("ControllerInterface"))
    {

        this->btn_idx_ = std::make_unique<JoyButtonIdx>();
        this->axes_idx_ = std::make_unique<JoyAxesIdx>();

        this->btn_idx_->cross =
            static_cast<size_t>(CONTROLLER_BUTTONS::CROSS);
        this->btn_idx_->circle =
            static_cast<size_t>(CONTROLLER_BUTTONS::CIRCLE);
        this->btn_idx_->triangle =
            static_cast<size_t>(CONTROLLER_BUTTONS::TRIANGLE);
        this->btn_idx_->square =
            static_cast<size_t>(CONTROLLER_BUTTONS::SQUARE);

        this->btn_idx_->dpad_up =
            static_cast<size_t>(CONTROLLER_BUTTONS::DPADUP);
        this->btn_idx_->dpad_down =
            static_cast<size_t>(CONTROLLER_BUTTONS::DPADDOWN);  
        this->btn_idx_->dpad_right =
            static_cast<size_t>(CONTROLLER_BUTTONS::DPADRIGHT);
        this->btn_idx_->dpad_left =
            static_cast<size_t>(CONTROLLER_BUTTONS::DPADLEFT);

        this->btn_idx_->LStickClick =
            static_cast<size_t>(CONTROLLER_BUTTONS::LSTICKCLICK);
        this->btn_idx_->RStickClick =
            static_cast<size_t>(CONTROLLER_BUTTONS::RSTICKCLICK);

        this->btn_idx_->L1 =
            static_cast<size_t>(CONTROLLER_BUTTONS::L1);
        this->btn_idx_->R1 =
            static_cast<size_t>(CONTROLLER_BUTTONS::R1);

        this->btn_idx_->select =
            static_cast<size_t>(CONTROLLER_BUTTONS::SELECT);
        this->btn_idx_->start =
            static_cast<size_t>(CONTROLLER_BUTTONS::START);
        this->btn_idx_->PS =
            static_cast<size_t>(CONTROLLER_BUTTONS::PS);
        this->btn_idx_->touchpad =
            static_cast<size_t>(CONTROLLER_BUTTONS::TOUCHPAD);

        this->axes_idx_->stick_lx =
            static_cast<size_t>(CONTROLLER_AXES::STICK_LX);
        this->axes_idx_->stick_ly =
            static_cast<size_t>(CONTROLLER_AXES::STICK_LY);
        this->axes_idx_->stick_rx =
            static_cast<size_t>(CONTROLLER_AXES::STICK_RX);
        this->axes_idx_->stick_ry =
            static_cast<size_t>(CONTROLLER_AXES::STICK_RY);
        this->axes_idx_->R2_analog =
            static_cast<size_t>(CONTROLLER_AXES::R2);
        this->axes_idx_->L2_analog =
            static_cast<size_t>(CONTROLLER_AXES::L2);
    }

bool ControllerInterface::isAvailable()
{
  if (this->joy_) {
    return true;
  }
  RCLCPP_ERROR_ONCE(
    this->LOGGER_,
    "Joy Message not set. Please call setJotMsg before use.");
  return false;
}

bool ControllerInterface::isTilted(const size_t idx, const double threshold)
{
  return std::abs(this->joy_->axes.at(idx)) > threshold;
}

void ControllerInterface::setJoyMsg(sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  this->joy_ = msg;
}

bool ControllerInterface::pressedAny()
{
  if (!this->isAvailable()) {
    return false;
  }

  bool pressed = false;
  for (auto btn : this->joy_->buttons) {
    pressed |= btn;
  }
  for (size_t i = 0; i < this->joy_->axes.size(); ++i) {
    if (i == this->axes_idx_->L2_analog || i == this->axes_idx_->R2_analog) {
      pressed |= this->joy_->axes.at(i) < 0.0;
      continue;
    }
    pressed |= this->isTilted(i);
  }
  return pressed;
}

bool ControllerInterface::pressedSquare()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->square);
}

bool ControllerInterface::pressedCircle()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->circle);
}

bool ControllerInterface::pressedTriangle()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->triangle);
}

bool ControllerInterface::pressedCross()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->cross);
}

bool ControllerInterface::pressedL1()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->L1);
}

bool ControllerInterface::pressedR1()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->R1);
}


bool ControllerInterface::pressedSelect()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->select);
}

bool ControllerInterface::pressedStart()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->start);
}

bool ControllerInterface::pressedPS()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->PS);
}

bool ControllerInterface::pressedTouchpad()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->touchpad);
}




bool ControllerInterface::pressedDPadUp()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->dpad_up);
}

bool ControllerInterface::pressedDPadDown()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->dpad_down);
}
bool ControllerInterface::pressedDPadRight()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->dpad_right);
}

bool ControllerInterface::pressedDPadLeft()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->dpad_left);
}


float ControllerInterface::tiltedStickLX()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->stick_lx);
}

float ControllerInterface::tiltedStickLY()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->stick_ly);
}

bool ControllerInterface::isTiltedStickL()
{
  if (!this->isAvailable()) {
    return false;
  }

  return this->isTilted(this->axes_idx_->stick_lx) ||
         this->isTilted(this->axes_idx_->stick_ly);
}

float ControllerInterface::tiltedStickRX()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->stick_rx);
}

float ControllerInterface::tiltedStickRY()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->stick_ry);
}

bool ControllerInterface::isTiltedStickR()
{
  if (!this->isAvailable()) {
    return false;
  }

  return this->isTilted(this->axes_idx_->stick_rx) ||
         this->isTilted(this->axes_idx_->stick_ry);
}

float ControllerInterface::pressedR2Analog()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->R2_analog);
}

float ControllerInterface::pressedL2Analog()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->L2_analog);
}

bool ControllerInterface::pressedLStick()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->LStickClick);
}

bool ControllerInterface::pressedRStick()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->RStickClick);
}


}

