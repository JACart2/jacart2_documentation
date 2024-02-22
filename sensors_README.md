# Sensors Guide

## Prerequisites
For the steps below to work, you must possess Sudo privileges.

## Installation

### Installing ROS2
Follow these instructions to install ROS2:
[ROS2 Ubuntu Development Setup](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

### Installing the Velodyne Drivers
1. Locate or create a `dev_ws/` directory.
2. Create a `src` folder inside it.
3. Clone the Velodyne Lidar GitHub repository into your workspace to use the ROS2 version of this driver.
```
git clone -b ros2  --single-branch https://github.com/ros-drivers/velodyne.git
```
This copies the Velodyne Lidar GitHub into your workspace and allows you to proceed to the next step and interpret data from the lidar. 
We want to use the ros2 version of this driver as we’re running it in conjunction with the other team on ros2.

### RVIZ Steps to See Velodyne Raw Data After Installation
1. Begin by turning on the battery, and plugging in the laptop and ethernet (blue cable).
2. Run the following in the terminal to connect to the Lidar.
```
ros2 launch velodyne_driver velodyne_driver_node_VLP16-launch.py
```
4. Convert the Raw Velodyne packets to pointcloud2, to be used by the localization.
```
ros2 launch velodyne_pointcloud velodyne_transform_node-VLP16-launch.py
```
6. In a new terminal, run RVIZ2.
```
ros2 run rviz2 rviz2
```
<img width="688" alt="Screenshot 2024-02-19 004908" src="https://github.com/JACart2/jacart2_documentation/assets/113935478/733bd588-d0ec-4cfc-8af6-7acf889d865d">

- Click add.  Click by topic. then select the Velodyne points to be added
- Where it says fixed frame change map to the word Velodyne
- You may echo as well to see the raw packets:
  ```
  ros2 echo velodyne_driver velodyne_driver_node_VLP16-launch.py
  ```

### Installing the Autoware.Universe
1. Navigate into the `dev_ws/src` directory.
2. Follow this tutorial until you reach step 2: [Autoware Source Installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/)
3. Once you reach step 2, execute the first command:
```
source /opt/ros/humble/setup.bash
```
4. Then, execute this command instead of the second command mentioned in the tutorial:
```
rosdep install -y --from-paths src --ignore-src --rosdistro humble --os=ubuntu:jammy
```
5. Next, run the following command in your terminal to build the packages, allowing only one to be built at a time to avoid crashes. This process can take over an hour.
```
colcon build --packages-ignore ament_cmake_core ament_cmake_export_definitions … --parallel-workers 1
```
Make sure you're running `colcon build` in the `dev_ws` directory and not any of its child directories.

### Source Install Package/Bash File
1. Open your `.bashrc` file in nano.
```
nano ~/.bashrc
```
2. Paste the following at the bottom of the file:
```
source /opt/ros/humble/setup.bash
source install/setup.bash
```
3. Save the file and exit nano. This allows ROS2 and built packages to be run as soon as you open any new terminal.

## Dependencies
Here are the repositories and their respective dependencies required for this project:

- **Core**
- **autoware_common**
 - Type: git
 - URL: https://github.com/autowarefoundation/autoware_common.git
 - Version: main
- **Velodyne Lidar**
 - Type: git
 - URL: https://github.com/ros-drivers/velodyne/tree/ros2
 - Version: main
