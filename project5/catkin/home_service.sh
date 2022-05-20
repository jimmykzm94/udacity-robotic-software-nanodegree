#!bin/sh
xterm -e "source devel/setup.bash; roslaunch my_robot world.launch" &
sleep 5
xterm -e "source devel/setup.bash; rosrun rviz rviz -d "src/my_robot/rviz/world_marker.rviz"" &
sleep 5
xterm -e "source devel/setup.bash; roslaunch my_robot amcl.launch" &
sleep 5
xterm -e "source devel/setup.bash; rosrun add_markers add_markers_home" &
sleep 5
xterm -e "source devel/setup.bash; rosrun pick_objects pick_objects_home"