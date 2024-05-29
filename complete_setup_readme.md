# Complete JACart2 Installation/Setup/Run Guide

You must follow these steps in order, as they appear.

## Step 1: Install/Setup
The following steps are to install all necessary packages and dependencies for use later on.

**(A)** Ensure [ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) is installed on your machine and navigate to ~/dev_ws/src by either creating the directory or going into a premade one.
```
cd ~
mkdir -p dev_ws/src
cd dev_ws/src
```
**(B)** Clone the ai-navigation repository into dev_ws/src folder. After the clone is complete you should cd back into dev_ws.
```
git clone https://github.com/JACart2/ai-navigation.git
cd ..
```
**(C)** Run the setup script from the dev_ws directory.
```
./src/ai-navigation/motor_control/resource/startup_script.sh
```

## Step 2: Run
The following steps are to actually run the code on the golf cart itself or in the simulator.
The cart should start outside at the origin point.

**(A)** Turn on the cart and connect the laptop via the USB-C cable, HDMI, and ethernet located in the rear of the cart. 

**(B)** Run the one singular launch file **or** all node launch files
```
ros2 launch cart cart.launch.py
```
  **or**
```
ros2 launch motor_control motor.launch.py
ros2 launch navigation navigation.launch.py
(insert other launch files here, like the localization stuff)
```

**(D)** Ensure that autonomous mode is turned **on**, power steering is turned **on**, and the emergency break/manual switch is turned **off**.

**(C)** RVIZ should launch and click on **Publish Point** at the top and click where you want to go. (eventually the GUI will replace this step)
