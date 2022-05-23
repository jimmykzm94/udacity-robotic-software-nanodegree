# Udacity robotic software nanodegree

## Spec used for this project
1. Ubuntu Xenial 16.04.7 LTS (VMWare Fusion 11)
2. Gazebo 7
3. ROS Kinetic

## Installation of Gazebo
1. Go to https://classic.gazebosim.org/tutorials?tut=install_ubuntu and follow the instructions from alternative-installation and replace `sudo apt-get install gazebo7` command.

## Installation of ROS
1. Go to http://wiki.ros.org/kinetic/Installation/Ubuntu and follow the instructions.

## Short note
The Gazebo or ROS command may not work properly on VSCode terminal. Please use Terminal window instead.

## Project 1
Note to myself:
1. When creating plugin, insert `<plugin name='<name>' filename='<build-filename>.so'/>` after `<world name='default'>` in world file.

How to run project 1:
1. Run `sh run.sh`, uncomment the script if needed.

## Project 2
Ball chaser
1. Open a terminal, source and run `roslaunch my_robot world.launch` to launch gazebo map, urdf robot.
2. Open another terminal, run `roslaunch ball_chaser drive_robot.launch` to auto-chase the white ball.

## Project 3
### Prerequisite
1. teleop_twist_keyboard (update submodule before using it)
2. sudo apt install ros-kinetic-map-server (first time only)
3. sudo apt install ros-kinetic-amcl (first time only)

### Create map
Run `roslaunch pgm_map_creator request_publisher.launch`

### Run simulation
Open terminal and type:
`roslaunch my_robot world.launch`
Open another terminal and type:
`roslaunch my_robot amcl.launch`
Open another terminal to use keyboard control:
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

## Project 4
1. Download `rtabmap_2.db` (https://drive.google.com/file/d/1SVv8yVBpZxNaDHM_4R6DqleEk7AsJXCw/view?usp=sharing).
2. Open a terminal, source and run `roslaunch my_robot world.launch` to launch gazebo map, urdf robot and teleop.
3. Open another terminal, run `roslaunch my_robot mapping.launch` to save mapping.
4. Run `rtabmap-databaseViewer <DB_FILENAME>` to view mapping.

## Project 5
### Prerequisite to run turtlebot example
    Install `sudo apt install ros-kinetic-openslam-gmapping` (or rosdep -i install PACKAGE)
    Install `sudo apt install ros-kinetic-joy` (or rosdep -i install PACKAGE)
    Install `sudo apt install ros-kinetic-turtlebot-*` (or rosdep -i install PACKAGE)
    Install `sudo apt install ros-kinetic-kobuki-*` (or rosdep -i install PACKAGE)

### Build
Run `catkin_make` to build packages

### Test SLAM
#### Note
1. gmapping is used for SLAM. It is a ROS package that uses Grid-based FastSLAM algorithm to map environment. Using slam_gmapping ROS node, we are able to map 2D occupancy grid map from mobile robot's odometry and laser data.

#### Run
Run `sh test_slam.sh` to test SLAM

### Test navigation
#### Note
1. AMCL is used for localization.
2. Use 2D Nav Goal in RVIZ to navigate robot to desired goal.
#### Run
Run `sh test_navigation.sh` to test robot navigation

### Test pick objects
#### Note
1. AMCL (Adaptive Monte Carlo localization) is used for localization. It is part of ROS package and is an improved algorithm from MCL, which basically uses particle filter to localize mobile robot pose. For AMCL, it dynamically adjusts the number of particles over time through navigating around the map. Thus, the algorithm is less computational expensive than MCL. For this ROS node, it will collect laser data from mobile robot and output pose estimate.
2. Once pick_objects package is run in this project, the robot will pick up an object and drop to a desired goal.
#### Run
Run `sh pick_objects.sh` to simulate robot pick-up and dropping object

### Test adding markers
#### Note
1. In small world, the blue cube will be displayed to be picked-up for 5 seconds.
2. The cube will be hidden, indicating it is already picked-up.
3. After 5 seconds, the green cube will be displayed.
#### Run
Run `sh add_markers.sh` to simulate

### Home service
#### Note
1. The robot will pick up blue cube at desired location.
2. Once picked up, the blue cube is hidden. The robot will navigate to goal.
3. Once reaching goal, the green box will be placed at desired place.
#### Run
Run `sh home_service.sh` to simulate home service
