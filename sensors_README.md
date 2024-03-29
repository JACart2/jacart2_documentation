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
3. Install Velodyne Driver from the repositories by running.
```
sudo apt-get install ros-humble-velodyne
```
This installs the Velodyne stack into your workspace and allows you to proceed to the next step and interpret data from the lidar. 
We want to use the ros2 version of this driver as we’re running it in conjunction with the other team on ROS2.
[Veloyne ROS WIKI](https://wiki.ros.org/velodyne)
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
   
# ZED Camera Setup WIP
### Prerequisites
- [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
## Install CUDA 
1. [Download CUDA Toolkit 12.3](https://developer.nvidia.com/cuda-downloads)
   To see what Ubuntu release you are running by doing
   ```
   lsb_release -a
   ```
   Select the following options for lab laptops
   - Operating System: Linux
   - Architecture: x86_64
   - Distribution: Ubuntu
   - Version: 22.04
   - Installer Type: deb (network)
3. Base Installer
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-3
```
3. Driver Installer
```
sudo apt-get install -y cuda-drivers
```
## Install ZED SDK
1. Download [ZED SDK](https://www.stereolabs.com/developers/release) for Ubuntu 22 4.0.8
2. Go to the folder where the installer has been downloaded
```
cd Downloads
```
3. Install zstd to run installer
```
sudo apt install zstd
```
4. Add execution permission to the installer using the `chmod +x` command
```
chmod +x ZED_SDK_Ubuntu22_cuda12.1_v4.0.8.zstd.run
```
5. Run the ZED SDK installer
```
./ZED_SDK_Ubuntu22_cuda11.8_v4.0.0.zstd.run
```
## Install ZED ROS2 Wrapper
### Prerequisites
- Ubuntu 22.04 (Jammy Jellyfish)
- ZED SDK v4.0 or later
- CUDA dependency
- ROS 2 Humble Hawksbill:
  ### Note
  The zed-ros2-wrapper repository contains the repository zed-ros2-interfaces as a sub-module. zed-ros2-    interfaces contains the definitions of the custom topics and custom services.
1. Open a terminal and move to your `src` folder of your workspace
```
cd dev_ws/src
```
2. Clone `zed-ros2-wrapper` github
```
git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
```
3. Move back to dev_ws
```
cd ..
```
4. Update
```
sudo apt update
```
5. Install the required dependencies
```
rosdep install --from-paths src --ignore-src -r -y
```
6. Build the wrapper, you can save time by only building the packages in the `zed-ros2-wrapper` folder by doing `--packages-select`
```
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
``` 
7. Setup the environment variables
```
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
source ~/.bashrc
```
