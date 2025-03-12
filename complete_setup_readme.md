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

View run instruction [here](https://github.com/JACart2/jacart2_documentation/wiki/System-Launch)
