## TerraFlight

Author: Henry Buron

Terraflight is a ROS2-controlled mobile exploration robot built from the ground up and programmed in Python. The custom-built rover carries a drone that can be deployed from the field during operation.

The robot live streams video from both the rover and drone and also uses a LiDAR module to perform SLAM and map its environment. The entire system is controlled and monitored from a base station. The drone is controlled via teleoperation, but is able to autonomously re-land on the rover. The drone localizes the rover with AprilTags, after which the user can trigger the autonomous landing service. The robot uses SLAM Toolbox to build a map of its environment.

![base_station1](https://github.com/henryburon/terra-flight/assets/141075086/b7691a4d-11f2-496f-b1ad-0ef40f266bb6)

## Contents
1. [Launch](#launch)
2. [Control Architecture](#control-architecture)
3. [Drone](#drone)
4. [Rover](#rover)
4. [Requirements](#requirements)

## Launch

Launch on base station: ```ros2 launch terraflight_control base_station.launch.xml```

Launch on robot: ```ros2 launch terraflight_control robot.launch.xml```

**Additional Launch Information**

To access the GPIO pins on the Raspberry Pi and therefore run the launch file, you must be running as root.
1. Ensure base station is connected to the same WiFi as the robot.
2. SSH into the robot  
If on NUMSR WiFi: ```ssh terra@192.168.18.155```  
If on Terra WiFi: ```ssh terra@192.168.1.155```  
Password: ```robotics!```
3. Build and run the launch file on the robot
   ```
   sudo -i
   cd /home/terra/ws/winter_project
   source /opt/ros/iron/setup.bash
   source install/setup.bash
   export ROS_DOMAIN_ID=34
   colcon build
   ros2 launch terraflight_control robot.launch.xml
   ```
4. Use ```sudo halt``` for a clean shutdown.

## Control Architecture

![terraflight_control_architecture](https://github.com/henryburon/terra-flight/assets/141075086/0db0da96-8dd1-475d-9252-bf165300cd86)

The entire system is programmed with ROS 2 Iron and run on both the base station (a laptop) and the robot (Raspberry Pi 4). Both systems must run a ROS 2 Iron-supported version of Ubuntu, such as 22.04. This project is coded fully in Python.

The project is divided into two packages: ```terraflight_description``` and ```terraflight_control```

```terraflight_description``` provides the robot URDF and RViz config file.

```terraflight_control``` implements the full control of the robot.

## Drone

The robot uses a [DJI Tello](https://store.dji.com/product/tello?vid=38421) drone to increase its sensing and operational capabilities.

To control the drone, the host computer must be connected to the drone's WiFi. For this reason, the base station uses a WiFi adapter to simultaneously connect the drone's network and the network shared with the Raspberry Pi 4.

The drone is controlled through its Python API.

## Rover



## Requirements

Python 

1. OpenCV
2. RPi.GPIO
3. Modern Robotics
4. Scipy
5. Pyserial
6. Pyrealsense 2
7. Numpy
8. CV Bridge

ROS2 (non-standard)

1. apriltags_ros
2. Joy
3. djitello
