
#pragma once

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>


namespace ds4dt_interface
{
enum class BUTTONS_DUALSHOCK4
{
  CROSS = 0,
  CIRCLE,
  SQUARE,
  TRIANGLE,
  SELECT,
  PS,
  START,
  LSTICKCLICK,
  RSTICKCLICK,
  L1,
  R1,
  DPADUP,
  DPADDOWN,
  DPADLEFT,
  DPADRIGHT,
  TOUCHPAD = 20
};

enum class AXES_DUALSHOCK4
{
  STICK_LX = 0,
  STICK_LY,
  STICK_RX,
  STICK_RY,
  L2,
  R2
};
typedef struct
{
  size_t square;
  size_t circle;
  size_t triangle;
  size_t cross;

  size_t L1;
  size_t R1;

  size_t select;
  size_t start;
  size_t PS;
  size_t touchpad;

  size_t LStickClick;
  size_t RStickClick;

  size_t dpad_up;
  size_t dpad_down;
  size_t dpad_right;
  size_t dpad_left;
} JoyButtonIdx;

typedef struct
{
  size_t stick_lx;
  size_t stick_ly;
  size_t stick_rx;
  size_t stick_ry;

  size_t R2_analog;
  size_t L2_analog;
} JoyAxesIdx;

class PlayStationInterface
{
public:
  using UniquePtr = std::unique_ptr<PlayStationInterface>;

private:
  sensor_msgs::msg::Joy::ConstSharedPtr joy_;
  const rclcpp::Logger LOGGER_;
  std::unique_ptr<JoyButtonIdx> btn_idx_;
  std::unique_ptr<JoyAxesIdx> axes_idx_;

private:
  bool isAvailable();
  bool isTilted(const size_t, const double = 1e-1);

public:
  explicit PlayStationInterface();
  void setJoyMsg(sensor_msgs::msg::Joy::ConstSharedPtr);

  bool pressedAny();

  bool pressedSquare();
  bool pressedCircle();
  bool pressedTriangle();
  bool pressedCross();

  bool pressedL1();
  bool pressedR1();

  bool pressedSelect();
  bool pressedStart();
  bool pressedPS();
  bool pressedTouchpad();

  bool pressedDPadUp();
  bool pressedDPadDown();
  bool pressedDPadLeft();
  bool pressedDPadRight();

  bool pressedLStick();
  bool pressedRStick();

  float tiltedStickLX();
  float tiltedStickLY();
  bool isTiltedStickL();

  float tiltedStickRX();
  float tiltedStickRY();
  bool isTiltedStickR();

  float pressedR2Analog();
  float pressedL2Analog();
};
}  // namespace ds4d_interface
