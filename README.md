# stewart_platform
This is a ROS2 Package for visualizing and controlling stewart platforms. Specifically stewart platforms where each arm is made up of two links (not ones with linear actuators)

This package was made for the robot in this [youtube video](https://www.youtube.com/watch?v=4MkSBwoUiGk&t=653s), but has the goal of being easily customizeable and expandable. 

## Need Help /  Want to Discuss
Join the [Discord](https://discord.gg/4GBbg6FE)!

If you have any reccomendations, want to help out with this project, or have any questions, I'll be happy to discuss more in the discord!

## DIRECTORIES

the package is broken up into four main directories: scripts, launch,  arudino, and urdf

### scripts
contains the python files that are used to run the differents nodes of the ROS network. 

### launch 
contains the launch files for the package. These are used to start up several nodes at a time

### urdf
 contains all the files needed for building the model of the robot. The files in this folder are .xacro files. Xacro is needed to build the completed robot model. This model is used to set up the TF2 frames from the urdf.launch.py launch file, and is used when displaying the model in rviz2

### microcontroller
 contains the files that can be uploaded to a microcontroller to control a physical robot. 


## HOW IT WORKS

The main launch file is urdf.launch.py. Launching this runs the other two launch files as well as the remaining nodes needed to visualize and control the virtual model.

This package works by setting up TF2 coordinate frames. These frames make up a tree starting at the base link working up to the short arms.

The basic loop for controlling the top plate and arms is this:

1)  A Pose message with the desired position and rotation of the top plate is published to the "end_effector_pose" topic.

2)  The end_effector_tf.py node gets the pose message, and updates the TF2 frame for the end effector.

3)  TF2 updates the frames for the end effector connection points to keep their fixed positions relative to the end effector frame.

4)  The inverse kinematics node gets the positions of the end effector connection points from TF2 and calculates what angle the arms should be at to account for the location of those. 

5)  The inverse kinematics node publishes the angles it found to the "joint_states" topic 

6)  The robot state publisher node reads the "joint_states" topic and updates The TF2 frames for the arms.

7) If the serial communication node is in use, it will read the "joint_states" topic, get the arm angles, create a string of servo positions, and publish that string on the serial bus as a command for the actual robot to read. 

