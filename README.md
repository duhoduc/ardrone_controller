Ardrone Controller
====

ROS Package (and utilities) to run the ardrone controller with a vicon system.

## Installation

Clone the package into your catkin workspace:
```
git clone https://github.com/ewcha/ardrone_controller.git
```

Additionally, you need to have ardrone_autonomy (https://github.com/AutonomyLab/ardrone_autonomy) and vicon_bridge (https://github.com/ethz-asl/vicon_bridge) in your ROS Path.

## Usage

1. Connect the AR.Drone to our network:
 * Download binaries and scripts from https://github.com/daraosn/ardrone-wpa2.git
 * Put the battery into the AR.Drone (make sure it is on a level surface while booting). The Ar.Drone will create its own ad-hoc network.
 * Copy over the wpa_supplicant binaries by running the install script 'script/install'
 * Connect to the drone's network
 * Connect the drone to the ACT Cage network 'script/connect 'ACT Cage' -p "crazydrone" -a dhcp'.
 * Log back onto ACT Cage
2. Launch the ardrone driver 'roslaunch ardrone_autonomy ardrone.launch'
3. Launch the vicon driver 'roslaunch vicon_bridge vicon.launch'
4. Launch the rviz file 'roslaunch rviz rviz -d controller.rviz' from the launch directory of ardrone_controller
5. Run the controller 'rosrun ardrone_controller controller.py'
6. Run the ui for the controller 'rosrun ardrone_controller control_ui.py'
## Notes
* If the firmware for the drone comes up a 0.0.0 on launch of ardrone_autonomy, you have to restart the drone and often the computer
* If the drone is in emergency mode (red LEDs on), you can reset using `rostopic pub /ardrone/reset std_msgs/Empty`
