# ROS 2 package collection for the SO-ARM100/101 robot

This repository provides a collection of packages to run the SO-ARM100/101 with ROS 2.

### Features

- Moveit 2 integration
- Hardware interface
- ROS 2 Control integration
- TODO: Gazebo integration
- TODO: Teach pendand
- TODO: Docker image

### Packages in the repository:
- sc_servo: package with libraries to communicate with the Feetech Servo motors
- sc_servo_examples: package with demos of the interface with the hardware
- so_arm_bring_up: package with launch files to start the robot
- so_arm_config: package with MoveIt ans ros2_control configurations
- so_arm_description: package with description files of the robot
- so_arm_driver: package with the robot arm driver
- so_arm_hardware_interface: package with the hardware interface implementation

### Requirements

- Ubuntu
- ROS 2 Humble -> [Instalation Guide](https://docs.ros.org/en/humble/Installation.html)
- Moveit 2 -> `sudo apt install ros-humble-moveit`
- ros2_control -> `sudo apt install ros-humble-ros2-control`

### Getting Started

First create a ROS 2 workspace and clone this repository in there:

```bash
mkdir ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Mat198/so-arm-ros
```

Source your ROS 2 installation and compile the repository:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws && source  && colcon build --symlink-install
```

Check the name of the USB port the robot is connected with:

```bash
TODO: Find a good and reliable way to find this
```

Source your ROS 2 workspace and them run the main launch

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch so_arm_bring_up so_arm.launch.py 
```

---

### Examples

Planning a path using Moveit 2
TODO: Add picture

Executing the path on the real robot
TODO: Add picture

---

## Future improvements

Planed features:

- Driver:
    - Execute smoother trajectories

- Moveit Integration:
    - Add mode planners

- Hardware interface: 
    - Configure usb port and servo motors IDs
    - Configure joint limits automatically
    - Configure service to disable motors
    - Configure initial position to be outside colisions

- Gazebo simulation
    - Implement Gazebo simulation
    - Implement pick and place example

