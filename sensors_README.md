# Sensors Guide

## Prerequisites
For the steps below to work, you must possess Sudo privileges.

## Installation

### Installing ROS2
Follow these instructions to install ROS2:
[ROS2 Ubuntu Development Setup](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

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
### Installing the Localization Package
1. Navigate into the `dev_ws/src` directory.
```
cd dev_ws/src/
```
2. Run these commands 
```
git clone https://github.com/rsasaki0109/lidar_localization_ros2.git
git clone https://github.com/rsasaki0109/ndt_omp_ros2.git
```
3. Go back into the previous directory. `dev_ws` 
```
cd ..
```
4. Build the Packages by running this command in the console
```
colcon build --symlink-install
```
Make sure you're running `colcon build` in the `dev_ws` directory and not any of its child directories as it will not work otherwise!

### Running the Localization Script
Make sure you have 
If the Ai-Navigation Github respository has already been cloned into your `dev_ws/src` and built in `dev_ws` ignore step one, two, and three
1. Navigate into the `dev_ws/src` folder
```
cd dev_ws/src
```
2. Install the JACART Ai-Navigation Package
```
git clone https://github.com/JACart2/ai-navigation.git
```
3. Build the ai navigation package
```
colcon build --symlink-install --packages-select ai-navigation
```
4. Change into the `dev_ws` and source your workspace
```
cd dev_ws
source install/setup.bash
```
5. Run the localization command which will bring up the localization required packages
```
ros2 launch localization_launch localization_full_launcher.launch.py
```
6. Using the 2d pose estimator.  Give the RVIZ once it has loaded the map the initial position of the cart
7. Drive and Localize

## Install LiDAR-SLAM package
1. Navigate into dev_ws/src director
```
cd dev_ws/src
```
2. Clone the git repository of the package
```
git clone https://github.com/rsasaki0109/lidarslam_ros2
```
3. Go in and Delete the third party folder in the package
```
cd lidarslam_ros2
rm /Thirdyparty -r
```
4. Build the package (Make sure you're in dev_ws directory, not any of its child directories)
```
cd ..
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
## Running lidar_slam (Launch file in progress)
1. Run the velodyne driver
```
ros2 launch velodyne_driver velodyne_driver_node-VLP16-launch.py 
```
2. Run the velodyne pointcloud
```
ros2 launch velodyne_pointcloud velodyne_transform_node-VLP16-launch.py
```
3. Create a static transform from base_link to velodyne
```
ros2 run tf2_ros static_transform_publisher 1 1 0 0 0 1 base_link velodyne
```
4. Run the lidar launch file
```
ros2 launch lidarslam lidarslam.launch.py
```
5. Now go around with the cart and when you're ready, save the map!
### Saving a map
1. Run the map save service call
```
ros2 service call /map_save std_srvs/Empty
```
2. The map is saved in the dev_ws folder
3. To view the map install then use pcl_viewer
```
sudo apt-get install pcl-tools # use if not installed
```
```
pcl_viewer -multiview 1 <pcd_filepath> # grab file path from properties then tab complete
```
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
  The [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) repository contains the repository zed-ros2-interfaces as a sub-module. zed-ros2-    interfaces contains the definitions of the custom topics and custom services.
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
## Object Detection
### Enable Object Detection
Object detection can be started automatically by setting the parameter `object_detection.od_enabled` to `true` in the file `common.yaml` from `zed-ros2-wrapper`.
To vizualize results of object detection, the ROS2 plugin for ZED2 is required, provided in next step.
## Multi-Camera Launch
### Download zed-ros2-examples
The [zed-ros-examples](https://github.com/stereolabs/zed-ros2-examples#build-the-package) repository is a collection of colcon packages for various tasks, including launching multiple cameras, and the ROS2 plugin for vizualizing object detection data in RVIZ2.
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
Use the package [`zed_multi_camera`](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_multi_camera) from the path `zed-ros2-examples/tutorials/zed_multi_camera/` as a starting point.
For correct transformations, modify `zed_multi_camera.launch.py` to ensure:
```
publish_tf = 'false'
```
### ZED Multi Camera URDF Congifuration
Inside this file, the position of each camera is in relation to a reference link. More documenation for further configuration can be found [here](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_multi_camera#the-multi-camera-urdf)
### Launch Parameters
- cam_names
- cam_models
- cam_serials
- disable_tf

Further documentation on the parameters found [here](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_multi_camera#launch-parameters).
### Running the Launch File
Camera model and serial number found with command:
```
ZED_Explorer --all
```
Launch Command:
```
ros2 launch zed_multi_camera zed_multi_camera.launch.py cam_names:='[zed_front,zed_rear]' cam_models:='[<front_camera_model>,<rear_camera_model>]' cam_serials:='[<front_camera_serial>,<rear_camera_serial>]'
```
- Replace `<front_camera_model>` and `<rear_camera_model>` with each camera model:
  - `zedm`, `zed2`, `zed2i`, `zedx`, or `zedxm`
- Replace `<front_camera_serial>` and `<rear_camera_serial>` with the serial number for each camera


