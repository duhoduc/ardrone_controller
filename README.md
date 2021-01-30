Ardrone Controller
====

ROS Package (and utilities) to run the ardrone controller with a qualisys system.

## Installation

Clone the package into your catkin workspace:
```
git clone https://github.com/duhoduc/ardrone_controller.git
```

Additionally, you need to have ardrone_autonomy (https://github.com/AutonomyLab/ardrone_autonomy) and qualisys (https://github.com/KTH-SML/motion_capture_system) in your ROS Path.

## Usage

1. Connect the AR.Drone to our network:
2. Launch the ardrone driver ```roslaunch ardrone_autonomy ardrone.launch```
3. Launch the vicon driver ```roslaunch vicon_bridge vicon.launch```
4. Launch the rviz file ```roslaunch rviz rviz -d controller.rviz``` from the launch directory of ardrone_controller
5. Run the controller ```rosrun ardrone_controller controller.py```
6. Run the ui for the controller ```rosrun ardrone_controller control_ui.py```
## Notes
* If the firmware for the drone comes up a 0.0.0 on launch of ardrone_autonomy, you have to restart the drone and often the computer
* If the drone is in emergency mode (red LEDs on), you can reset using `rostopic pub /ardrone/reset std_msgs/Empty`
* To run the python files (steps 5 and 6), the files need to be executable. Use chmod +x on each of the .py files.
