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

### Installing Rsasaki's Lidar Localizalton.
1. Clone the required repositories in src
```
cd /dev_ws/src
git clone https://github.com/rsasaki0109/lidar_localization_ros2
git clone https://github.com/rsasaki0109/ndt_omp_ros2
```
2. Build the workspace in /dev_ws
```
cd ...
colcon build --symlink-install
```
## Localization DEMO
If you're using the launch.py file
```
ros2 launch localization_launch localization_full_launcher.launch.py
```
Otherwise
```
rviz2 -d src/lidar_localization_ros2/rviz/localization.rviz
ros2 launch lidar_localization_ros2 lidar_localization.launch.py
```
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
   
# ZED Camera Setup
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
./ZED_SDK_Ubuntu22_cuda12.1_v4.0.8.zstd.run
```
## Install ZED Python API
The Python install script is located in /usr/local/zed/.
Run the script:
```
cd "/usr/local/zed/"
python3 get_python_api.py
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
# Object Detection
The zed-ros-examples repository is a collection of colcon packages for various tasks.
```
cd ~/dev_ws/src/
git clone https://github.com/stereolabs/zed-ros2-examples.git
cd ../
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source ~/.bashrc
```
### Enable Object Detection
Object detection can be started automatically by setting the parameter `object_detection.od_enabled` to `true` in the file `common.yaml`
### Launching with RVIZ2
```
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=<camera_model>
```
Replace `<camera_model>` with your camera model: `zedm`, `zed2`, `zed2i`, `zedx`, `zedxm`.
