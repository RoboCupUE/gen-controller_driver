#include "ds4dt_interface/ds4dt_interface.hpp"

namespace ds4dt_interface
{
    PlayStationInterface::PlayStationInterface()
        : LOGGER_(rclcpp::get_logger("PlayStationInterface"))
    {

        this->btn_idx_ = std::make_unique<JoyButtonIdx>();
        this->axes_idx_ = std::make_unique<JoyAxesIdx>();

        this->btn_idx_->cross =
            static_cast<size_t>(BUTTONS_DUALSHOCK4::CROSS);
        this->btn_idx_->circle =
            static_cast<size_t>(BUTTONS_DUALSHOCK4::CIRCLE);
        this->btn_idx_->triangle =
            static_cast<size_t>(BUTTONS_DUALSHOCK4::TRIANGLE);
        this->btn_idx_->square =
            static_cast<size_t>(BUTTONS_DUALSHOCK4::SQUARE);

        this->btn_idx_->dpad_up =
            static_cast<size_t>(BUTTONS_DUALSHOCK4::DPADUP);
        this->btn_idx_->dpad_down =
            static_cast<size_t>(BUTTONS_DUALSHOCK4::DPADDOWN);  
        this->btn_idx_->dpad_right =
            static_cast<size_t>(BUTTONS_DUALSHOCK4::DPADRIGHT);
        this->btn_idx_->dpad_left =
            static_cast<size_t>(BUTTONS_DUALSHOCK4::DPADLEFT);

        this->btn_idx_->LStickClick =
            static_cast<size_t>(BUTTONS_DUALSHOCK4::LSTICKCLICK);
        this->btn_idx_->RStickClick =
            static_cast<size_t>(BUTTONS_DUALSHOCK4::RSTICKCLICK);

        this->btn_idx_->L1 =
            static_cast<size_t>(BUTTONS_DUALSHOCK4::L1);
        this->btn_idx_->R1 =
            static_cast<size_t>(BUTTONS_DUALSHOCK4::R1);

        this->btn_idx_->select =
            static_cast<size_t>(BUTTONS_DUALSHOCK4::SELECT);
        this->btn_idx_->start =
            static_cast<size_t>(BUTTONS_DUALSHOCK4::START);
        this->btn_idx_->PS =
            static_cast<size_t>(BUTTONS_DUALSHOCK4::PS);
       // this->btn_idx_->touchpad =
       //     static_cast<size_t>(BUTTONS_DUALSHOCK4::TOUCHPAD);

        this->axes_idx_->stick_lx =
            static_cast<size_t>(AXES_DUALSHOCK4::STICK_LX);
        this->axes_idx_->stick_ly =
            static_cast<size_t>(AXES_DUALSHOCK4::STICK_LY);
        this->axes_idx_->stick_rx =
            static_cast<size_t>(AXES_DUALSHOCK4::STICK_RX);
        this->axes_idx_->stick_ry =
            static_cast<size_t>(AXES_DUALSHOCK4::STICK_RY);
        this->axes_idx_->R2_analog =
            static_cast<size_t>(AXES_DUALSHOCK4::R2);
        this->axes_idx_->L2_analog =
            static_cast<size_t>(AXES_DUALSHOCK4::L2);
    }

bool PlayStationInterface::isAvailable()
{
  if (this->joy_) {
    return true;
  }
  RCLCPP_ERROR_ONCE(
    this->LOGGER_,
    "Joy Message not set. Please call setJotMsg before use.");
  return false;
}

bool PlayStationInterface::isTilted(const size_t idx, const double threshold)
{
  return std::abs(this->joy_->axes.at(idx)) > threshold;
}

void PlayStationInterface::setJoyMsg(sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  this->joy_ = msg;
}

bool PlayStationInterface::pressedAny()
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

bool PlayStationInterface::pressedSquare()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->square);
}

bool PlayStationInterface::pressedCircle()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->circle);
}

bool PlayStationInterface::pressedTriangle()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->triangle);
}

bool PlayStationInterface::pressedCross()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->cross);
}

bool PlayStationInterface::pressedL1()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->L1);
}

bool PlayStationInterface::pressedR1()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->R1);
}


bool PlayStationInterface::pressedSelect()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->select);
}

bool PlayStationInterface::pressedStart()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->start);
}

bool PlayStationInterface::pressedPS()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->PS);
}
/*
bool PlayStationInterface::pressedTouchpad()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->touchpad);
}
*/



bool PlayStationInterface::pressedDPadUp()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->dpad_up);
}

bool PlayStationInterface::pressedDPadDown()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->dpad_down);
}
bool PlayStationInterface::pressedDPadRight()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->dpad_right);
}

bool PlayStationInterface::pressedDPadLeft()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->dpad_left);
}


float PlayStationInterface::tiltedStickLX()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->stick_lx);
}

float PlayStationInterface::tiltedStickLY()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->stick_ly);
}

bool PlayStationInterface::isTiltedStickL()
{
  if (!this->isAvailable()) {
    return false;
  }

  return this->isTilted(this->axes_idx_->stick_lx) ||
         this->isTilted(this->axes_idx_->stick_ly);
}

float PlayStationInterface::tiltedStickRX()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->stick_rx);
}

float PlayStationInterface::tiltedStickRY()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->stick_ry);
}

bool PlayStationInterface::isTiltedStickR()
{
  if (!this->isAvailable()) {
    return false;
  }

  return this->isTilted(this->axes_idx_->stick_rx) ||
         this->isTilted(this->axes_idx_->stick_ry);
}

float PlayStationInterface::pressedR2Analog()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->R2_analog);
}

float PlayStationInterface::pressedL2Analog()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->L2_analog);
}

bool PlayStationInterface::pressedLStick()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->LStickClick);
}

bool PlayStationInterface::pressedRStick()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->RStickClick);
}


}

