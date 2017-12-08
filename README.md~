# Fall-17-106A-Project
UC Berkeley EE 106A - Fall 2017 - Team 37

This project is a ROS framework for manipulating points on a flat surface.

It currently erases or paints over OpenCV detected shapes of a specified color or a whiteboard or other surface.

More information can be found at our project website:
https://tomsawyerbot.weebly.com/

How to use:

With the appropriate packages installed (ar_track_alvar, sawyer_moveit_config), SSH into a configured Sawyer robot, and run the following commands:

1. Set up Sawyer with necessary manipulator (sponge with whiteboard wipes or paint).
1. Attach an AR marker (must be ar_marker_4) to target surface
2. rosrun intera_examples joint_trajectory_action_server.py
3. roslaunch sawyer_moveit_config move_group.launch
4. roslaunch planning_baxter roslaunch tsbot_erase.launch or tsbot_paint.launch depending on the desired task.

Sawyer will then detect green points and move to erase or paint over each one.

Team members:

Ashlee Heuston

Lam Nguyen

Sean Keegan

Zoe Cohen

note: most commits were made by all group members on aheuston's instructional lab account.
