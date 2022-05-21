#!bin/sh
xterm -e "source devel/setup.bash; roslaunch my_robot world.launch" &
sleep 5
xterm -e "source devel/setup.bash; roslaunch my_robot amcl.launch" &
sleep 5
xterm -e "source devel/setup.bash; rosrun rviz rviz -d "src/my_robot/rviz/world_marker.rviz"" &
sleep 5
