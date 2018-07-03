# iOS ROS Bridge

This will stream pose data from an iOS device to a computer via UDP in order to manipulate the data in ROS. Currently, the iOS-ROS streamer sends pose and camera data between the devices.

Built For OccamLab @Olin College 2018

### Running the iOS ROS Streamer

Steps:

(1) Enter your catkin workspace directory

(2) Run `roscore` in your terminal

(3) Open a new terminal window and run `roslaunch ios_streamer stream.launch`

(4) To view different visualizations of the data being streamed, you may choose to run either `rosrun rqt_gui rqt_gui` or `rviz` in another terminal window

(5) If you have OccamLab's mobility_games repository cloned to your device, you may choose to run `roslaunch mobility_sensing ar_localization.launch` in order to view visualizations of april tag detections in either rviz or the rqt gui. You will need to first change the namespace for the apriltags_ros package from `camera` to `camera_lowres`. You may also want to add tag descriptions for the april tags.

If you find the streamer has significant lag, please try relaunching the iOS app.

#### First time setup

(1) Make sure you have a catkin workspace! (If you don't, here are [instructions for setting up a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)).

(2) Clone this directory into `catkin_ws/src`.

(3) Enter your `catkin_ws` direction and run the following commands:

```bash
$ catkin_make install
$ source devel/setup.bash
```

(4) Follow instructions above from step (2)!

#### Attributions:
The following provides the basis for how data is sent from an iOS device via UDP connection to a computer:
https://github.com/gunterhager/UDPBroadcastConnection

To learn more about OccamLab, please visit our website: http://occam.olin.edu/.
