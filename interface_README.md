# Interface Guide
The motor_control package is the ROS2 node that handles processing the target instructions and sending messages to the arduino controller to meet them.
## Motor Control Setup

1. Ensure [ROS2 humble](https://docs.ros.org/en/humble/Installation.html) is installed on your machine and navigate to /dev_ws/src by either creating the directory or going into a premade one.
```
mkdir -p dev_ws/src
cd dev_ws/src
```
2. Copy the [99-usb-serial.rules](https://github.com/JACart2/ai-navigation/blob/main/motor_control/resource/99-usb-serial.rules) into your linux user's rule directory. This may require sudo privilege. This rule binds the arduino board to the same `ttyUSB9` port everytime, otherwise you have to check the usb port after connecting. For help with that, check [Tips](###Tips).


2. Clone the ai-navigation repository into dev_ws/src folder. After the clone is complete you should cd back into dev_ws.
```
git clone https://github.com/JACart2/ai-navigation.git
cd ..
```

4. Build the packages inside of dev_ws and soruce them
```
colcon build --symlink-install
source install/setup.bash
```

5. Install all of the required packages: numpy, pyserial, bitstruct
```
sudo pip install <package>
```

6. Run the launch file or the node
```
ros2 launch motor_control motor.launch.py
```
or
```
ros2 launch motor_control motor.launch.py baudrate:=57600 arduino_port:=dev/ttyUSB9
```
or
```
ros2 run motor_control motor_endpoint
```

7. For keyboard control of the cart, open a new terminal and you can start up the teleop
```
ros2 run teleop teleop_node
```

### Tips

- To find the TTY* port that Arduino uses without the linux rule, run `udevadm monitor -u` before plugging it in, then it should give the information you need.
- If you are having trouble finding the arduino device in /dev/ after it has been plugged in, you may need to remove the Brltty package from your system: `sudo apt remove brltty`.

## Usage
The motor_control package uses the topic `/nav_cmd` to receive a target velocity and steering angle. To check the status of the node and the cart, you can subscribe to the `/heartbeat` topic, which is where the controller publishes the arduino's feedback messages.
