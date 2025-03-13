# ZED Camera Overview
## Launch

In order to launch the ZED camera setup, we created a ros2 launchfile that will run any cameras as specified by parameters that are passed into the function from the command line. These launchfiles can be run individually, and should be bundled into the run.sh command that runs the cart as well.

For example, this command launches the two cameras on James:
```
ros2 launch zed_multi_camera zed_multi_camera.launch.py cam_names:=["cam_front","cam_back"] cam_models:=["zed2i","zed2i"] cam_serials:=[37963597,31061594] disable_tf:=False
```

Different camera setups will require variations on this command.

## Reading Output

The ZED cameras have a lot of different ways that their output can be interpreted. All of them are published to ros2 topics, and the data can then be manipulated from our end.

For example, a simple non-stereo image that can be displayed in Rviz can be found under the topic (under the default James launchfile configuration)
```
zed_front/zed_node_0/left_raw/image_raw_color
```
(To add this to Rviz, click 'Add', then 'By topic', then select this topic. Make sure to select the "Image" display option, not the "Camera" one)

When hooked up to a system with multiple cameras, these should follow the form of:
```
{NAME}/zed_node_{INDEX}/....
```
where NAME is the name of the camera provided to the launchfile, and INDEX is the index of that specific camera in the launchfile's array parameters.

## Past Problems
This section is meant to warn of possible problems with work on the cart based on ones already encountered.
### USB Extension
The cameras have a lot of data coming through at any given time, and all of their circuitry requires power to run as well. All of this must be transferred via USB to the cameras. We have found problems with USB extension cords limiting the thoroughput of data, and likely also limiting power draw. This even applies to USB 3.0. Be cautious when extending the signal out of the cameras, and use one of the official sterolabs cables when possible.