# Sensor's Guide
## Installing ROS2
For these steps to work you must possess Sudo privileges
1. Install ROS2 following these Instructions:
https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html


## Installing the Velodyne Drivers
1. Locate or create a dev_ws/ file
```
cd dev_ws
```
2. Create a src folder in there using: mkdir src
```
cd src
```
3. Use this command once in the src folder: 
```
git clone -b ros2  --single-branch https://github.com/ros-drivers/velodyne.git
```
This copies the Velodyne Lidar Git Hub into your workspace and allows you to proceed to the next step and interpret data from the lidar. We want to use the ros2 version of this driver as we’re running it in conjunction with the other team on ros2.


## RVIZ Steps to see Velodyne Raw Data after Installation
1. Begin by turning on the battery, and plugging in the laptop and ethernet (blue cable)
2.Run the following in the terminal
```
ros2 launch velodyne_driver velodyne_driver_node_VLP16-launch.py
ros2 launch velodyne_pointcloud velodyne_transform_node-VLP16-launch.py
ros2 run rviz2 rviz2
```
Note run the rviz2 command in a separate terminal
3. Add the velodyne points from the topic list 
4. Change map frame to Velodyne
You may echo as well to see the raw packets:
ros2 echo velodyne_driver velodyne_driver_node_VLP16-launch.py


## Installing the Autoware.Universe
Cd into the dev_ws/src
Follow this Tutorial until you reach step 2:
https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/

Once you reach step 2 do the first command of source /opt/ros/humble/setup.bash

Then do this command instead of the second command: 
```
rosdep install -y --from-paths src --ignore-src --rosdistro humble –os=ubuntu:jammy
```
Next, do this command in your terminal
```
colcon build –packages-ignore ament_cmake_core ament_cmake_export_definitions … –parallel-workers 1
```
This command builds the packages that we want and ignores the ones we don't.  It also only allows one to be built at a time to avoid crashing but this does cause the building to take up to and over an hour. Make sure you’re colcon building in the dev_ws directory and not any of its child directories (Use cd ./dev_ws to move back to the dev_ws directory).
## Source Install Package/Bash File
```
nano ~/.bashrc
```
Paste the following at the bottom of the file:
```
source /opt/ros/humble/setup.bash
source install/setup.bash
```
Save the file and exit the nano
This allows ROS2 and built packages to be run as soon as you open any new terminal

## Dependencies:

### repositories:
 &ensp;core: <br>
    &ensp;&ensp;core/autoware_common: <br>
      &ensp;&ensp;&ensp;type: git <br>
      &ensp;&ensp;&ensp;url: https://github.com/autowarefoundation/autoware_common.git <br>
      &ensp;&ensp;&ensp;version: main <br>
    &ensp;&ensp;core/Velodyne Lidar: <br>
      &ensp;&ensp;&ensp;type: git<br>
      &ensp;&ensp;&ensp;url: https://github.com/ros-drivers/velodyne/tree/ros2 <br>
      &ensp;&ensp;&ensp;version: main<br>
      
