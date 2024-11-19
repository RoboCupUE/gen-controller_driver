# Generic Controller JoyDriver w/Teleop for ROS2 Humble
Simple No-Nonsense Controller JoyDriver With Teleop for ROS2 Humble, using ```controller_joy_node```.

## How to use:
Firstly, initialize the controller interface:
```
c_interface = std::make_unique<controller_interface::ControllerInterface>();
```
And then set a callback joy msg:
```
void Example::onJoyCallback(sensor_msgs::msg::Joy::ConstSharedPtr joy_msg) {
  c_interface->setJoyMsg(joy_msg);  // set joy message

  // Sample 
  if (c_interface->pressedDPadDown()) {
    ...
  }
...
```

## Teleop:
Teleop is mapped to the left joystick by default, also using the DPad to adjust angular and linear speed. These two parameters and more can be altered through the ```params.yaml``` file.

## Test:
The included test node can be used to check if the button mappings are correct.

## Supported controllers:

| Supported?         | Hardware Name |
| ------------------ | ------------- |
| Yes | `Xbox 360 Controller`  |
| Yes | `Xbox One Controller`  |
| Yes | `Dualshock4`   |
| Yes | `Dualshock3`   |
| Yes | `Generic Logitech Controller`   |
| Not tested yet | `Dualsense`   |
| Not tested yet | `Xbox Series X Controller`   |

### Additional notes:
**Originally forked from: [PlayStation-JoyInterface-ROS2](https://github.com/HarvestX/PlayStation-JoyInterface-ROS2)** <br>
Now with correct button mappings for DualShock4, support for Joystick button click, touchpad click and much more.
