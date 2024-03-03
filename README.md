## TerraFlight

Author: Henry Buron

Launch on base station: ```ros2 launch terraflight_control base_station.launch.xml```

Launch on robot: ```ros2 launch terraflight_control robot.launch.xml```

**Additional Information**

To access the GPIO pins on the Raspberry Pi, and run the launch file, you must be running as root.
1. SSH into the robot  
```ssh terra@192.168.18.155```  
```robotics!```
2. Run the launch file
   ```
   sudo -i
   cd /home/terra/ws/winter_project
   source /opt/ros/iron/setup.bash
   source install/setup.bash
   export ROS_DOMAIN_ID=34
   ros2 launch terraflight_control robot.launch.xml
   ```
3. Use ```sudo halt``` for a clean shutdown.
4. Wifi
* The base station (laptop) must be connected to the same network as the Raspberry Pi and the Tello's network.

**Required Packages**
1. Joy
2. apriltags_ros
3. djitello