https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/
Sensor Team 
Sprint 1
Walker Todd, Jesse Yao, Jack Posada

Research auto ware suggested localization packages. 
Using this link look at localization packages.  Specifically first the Velodyne Package
https://autowarefoundation.github.io/autoware-documentation/main/reference-hw/lidars/
Look into suggested github localization packages from the link:
What do the packages want?
What do they publish (correct Coordinate frame?)
Implement a Localization Package that we choose
Does this localization package work?
Can we see data and is it usable?
If it doesn’t, why does it not work?
Contact Dr Sprague or another Professor to see if it is something we can buy (IMU Sensor) 
Continue to the next package if one does not work




Sensor Team Write Up 
	Our current progress is going very well with a localization package being installed and used for our


https://github.com/koide3/hdl_localization

https://velodynelidar.com/automated-with-velodyne/kudan/


Driving around and construct the map

Use the map you created
Giant point cloud
Giving the current set of points that it sees


Autoware provides the libraries 
Autoware localization folder:
https://github.com/autowarefoundation/autoware.universe/tree/main/localization
Documentation:
https://autowarefoundation.github.io/autoware.universe/main/localization/ekf_localizer/


Documentation Goals 1

For these steps to work you must possess Sudo privileges
Installing ROS2:
Follow these Instructions:
https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html




Installing the Velodyne Drivers
Locate or create a dev_ws/ file
cd dev_ws
Create a src folder in there using: mkdir src
cd src
Use this command once in the src folder: 
git clone -b ros2  --single-branch https://github.com/ros-drivers/velodyne.git

This copies the Velodyne Lidar Git Hub into your workspace and allows you to proceed to the next step and interpret data from the lidar. We want to use the ros2 version of this driver as we’re running it in conjunction with the other team on ros2.


RVIZ Steps to see Velodyne Raw Data after Installation
Begin by turning on the battery, and plugging in the laptop and ethernet (blue cable)
Run the following in the terminal
ros2 launch velodyne_driver velodyne_driver_node_VLP16-launch.py
Connects to the Lidar
ros2 launch velodyne_pointcloud velodyne_transform_node-VLP16-launch.py
Converts the Raw velodyne packets to pointcloud2
To be used by the localization
ros2 run rviz2 rviz2 (new terminal)
Add Velodyne points
Change map frame to Velodyne
You may echo as well to see the raw packets:
ros2 echo velodyne_driver velodyne_driver_node_VLP16-launch.py
It should look like this after rviz2 is setup



Installing the Autoware.Universe
Cd into the dev_ws/src
Follow this Tutorial until you reach step 2:
https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/

Once you reach step 2 do the first command of source /opt/ros/humble/setup.bash

Then do this command instead of the second command: 
rosdep install -y --from-paths src --ignore-src --rosdistro humble –os=ubuntu:jammy

Next, do this command in your terminal
colcon build –packages-ignore ament_cmake_core ament_cmake_export_definitions … –parallel-workers 1
this command builds the packages that we want and ignores the ones we don't.  It also only allows one to be built at a time to avoid crashing but this does cause the building to take up to and over an hour. Make sure you’re colcon building in the dev_ws directory and not any of its child directories (Use cd ./dev_ws to move back to the dev_ws directory).
Source Install Package/Bash File
Nano ~/.bashrc
Paste the following at the bottom of the file:
source /opt/ros/humble/setup.bash
source install/setup.bash
Save the file and exit the nano
This allows ROS2 and built packages to be run as soon as you open any new terminal

Dependencies:

repositories:\
  core\
  core/autoware_common:
    type: git
    url: https://github.com/autowarefoundation/autoware_common.git
    version: main
  core/Velodyne Lidar:
    type: git
    url: https://github.com/ros-drivers/velodyne/tree/ros2
    version: main
  






