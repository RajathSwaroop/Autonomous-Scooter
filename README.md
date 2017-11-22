**Autonomous Mobility scooter for the physically challenged**

This repository contains the files for autonomous navigation of mobility scooter

* Leddar - Initial Commit to Leddar Tech M16 Package for Jetson TX2.
* ST32 - GPS code to read Naza GPS data and print to the serial terminal.
* mybot_ws - consists of files from one of the repositories that can be used for general robot navigation after some modification
* Obstacle_avoid - obstacle avoidance program uing RPLIDAR
* Odom_scooter - consists of odometry files related to scooter odometry (C++ programs and ROS files)
* zed-opencv - code for Obstacle detection under 2-meter range using ZED camera and OpenCV.

All ROS programs can be compiled using catkin_make and run using rosrun or roslaunch commands.

Execution

*Leddar - make it from example directory and run ./example
*ST32 - rosrun ST32 gps.py
*mybot_ws has costmaps that can be changed to specific robotn and run amcl_demo.launch using roslaunch from navigation package of mybot
*Obstacle avoidance and Odom_scooter have executables in python 
*zed-opencv can be build using make in build directory and run the zedopencv executable
