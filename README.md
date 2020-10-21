:gear: roboteq control [![Build Status](https://travis-ci.org/rbonghi/roboteq_control.svg?branch=master)](https://travis-ci.org/rbonghi/roboteq_control)
=======

roboteq_control is a Roboteq motor control **[ros_control][ros_control]** based.
This package use a Roboteq device with a serial port.

All parameters, GPIO, Analogs port are controlled by this driver and from dynamic_reconfigure you can setup as you wish this board.

Device included:

| Brushed DC | Brushless DC | Sepex |
| ---------- | ------------ | ----- |
| HDC24xx, VDC24xx, MDC22xx, LDC22xx, LDC14xx, SDC1130, SDC21xx | HBL16xx, VBL16xx, HBL23xx, VBL23xx, LBL13xx, MBL16xx, SBL13xx | VSX18xx |

Advanced Digital Motor Controllers, as described in [this document][roboteq_manual]. 

# Install

Clone on your catkin workspace this repository, download all dependencies and compile!

```bash
# Make catkin workspace if does not exist and clone this repo
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/rbonghi/roboteq_control.git
# Install all dependecies
cd ..
rosdep install --from-paths src --ignore-src -r -y
# Compile package
catkin_make
```

Don't forget to add in your bash your catkin sources

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# Run

```bash
roslaunch roboteq_control roboteq.launch
```

:rocket: That's it!

This launch file can load different parameters such as:
 * port - Serial port (defaut: /dev/ttyUSB0)
 * config - Configuration file (Example in config file)

This driver include a dynamic_reconfigure topics to dynamically update all parameters of your robot

![roboteq_control](https://github.com/rbonghi/roboteq_control/wiki/images/dynamic_reconfigure.png)

Detailed information are available on [wiki](https://github.com/rbonghi/roboteq_control/wiki)

# Example

This package include a differential drive example to drive a robot.

```bash
roslaunch roboteq_control differential_drive.launch
```

There are different parameters than you can setup:
 * size - default: 25cm
 * radius - default: 8cm
 * wheelbase - default: 0.40cm

![roboteq_control](https://github.com/rbonghi/roboteq_control/wiki/images/roboteq_control.png)

To control this robot are available this topics

**Subscribers:**
 * /velocity_controller/cmd_vel [geometry_msgs/Twist]
 * /roboteq/emergency_stop [std_msgs/Bool]

**Publishers:**
 * /velocity_controller/odom [nav_msgs/Odometry]

![roboteq_control](https://github.com/rbonghi/roboteq_control/wiki/images/rosgraph_simple.png)

[roboteq_manual]: https://www.roboteq.com/docman-list/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v17/file
[ros_control]: http://wiki.ros.org/ros_control
