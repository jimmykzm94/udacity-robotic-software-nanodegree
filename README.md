# Udacity robotic software nanodegree

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
### Create map
Run `roslaunch pgm_map_creator request_publisher.launch`

### Submodule
1. teleop_twist_keyboard (update submodule before using it)

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
1. Prerequisite to run turtlebot example:
    1. Install `sudo apt install ros-kinetic-openslam-gmapping` (or rosdep -i install PACKAGE)
    2. Install `sudo apt install ros-kinetic-joy` (or rosdep -i install PACKAGE)
    3. Install `sudo apt install ros-kinetic-turtlebot-*` (or rosdep -i install PACKAGE)
    4. Install `sudo apt install ros-kinetic-kobuki-*` (or rosdep -i install PACKAGE)
2. Home service:
    1. Run `catkin_make` to build packages, then run `sh home_service.sh`
3. Test SLAM:
    1. Opening my small world, RVIZ and keyboard teleop
    2. <b>gmapping</b> is used for SLAM
    3. Run `sh test_slam.sh` to test SLAM
4. Test navigation:
    1. Opening my small world and RVIZ
    2. AMCL is used for localization
    3. Use 2D Nav Goal in RVIZ to navigate robot to desired goal
    4. Run `sh test_navigation.sh` to test robot navigation
5. Test pick objects
    1. Opening my small world and RVIZ
    2. AMCL is used for localization
    3. Once pick_objects package is run, the robot will pick up an object and drop to a desired goal
    4. Run `sh pick_objects.sh` to simulate robot pick-up and dropping object
