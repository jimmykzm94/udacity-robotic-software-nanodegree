# Udacity robotic software nanodegree

## Project 1
Note to myself:
1. When creating plugin, insert `<plugin name='<name>' filename='<build-filename>.so'/>` after `<world name='default'>` in world file.

How to run project 1:
1. Run `sh run.sh`, uncomment the script if needed.

## Project 2
To be added

## Project 3
To be added

## Project 4
1. Download `rtabmap_2.db` (https://drive.google.com/file/d/1SVv8yVBpZxNaDHM_4R6DqleEk7AsJXCw/view?usp=sharing).
2. Open a terminal, source and run `roslaunch my_robot world.launch` to launch gazebo map, urdf robot and teleop.
3. Open another terminal, run `roslaunch my_robot mapping.launch` to save mapping.
4. Run `rtabmap-databaseViewer <DB_FILENAME>` to view mapping.
