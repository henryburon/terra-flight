## TerraFlight

Author: Henry Buron

Terraflight is a ROS2-controlled mobile exploration robot built from the ground up and programmed in Python. The custom-built rover carries a drone that can be deployed from the field during operation.

The robot live streams video from both the rover and drone and also uses a LiDAR module to perform SLAM and map its environment. The entire system is controlled and monitored from a base station. The drone is controlled via teleoperation, but is able to autonomously re-land on the rover. The drone localizes the rover with AprilTags, after which the user can trigger the autonomous landing service. The robot uses SLAM Toolbox to build a map of its environment.

![base_station1](https://github.com/henryburon/terra-flight/assets/141075086/b7691a4d-11f2-496f-b1ad-0ef40f266bb6)

**Launch**

Launch on base station: ```ros2 launch terraflight_control base_station.launch.xml```

Launch on robot: ```ros2 launch terraflight_control robot.launch.xml```

**Additional Launch Information**

To access the GPIO pins on the Raspberry Pi, and run the launch file, you must be running as root.
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

**Control Architecture**

![terraflight_control_architecture](https://github.com/henryburon/terra-flight/assets/141075086/0db0da96-8dd1-475d-9252-bf165300cd86)

**Required Packages**
1. apriltags_ros
2. Joy
3. djitello