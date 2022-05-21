#!bin/sh
xterm -e "source devel/setup.bash; roslaunch my_robot world.launch" &
sleep 5
xterm -e "source devel/setup.bash; rosrun gmapping slam_gmapping _base_frame:=robot_footprint" &
sleep 5
xterm -e "source devel/setup.bash; rosrun rviz rviz -d "src/my_robot/rviz/world_marker.rviz"" &
sleep 5
xterm -e "source devel/setup.bash; rosrun teleop_twist_keyboard teleop_twist_keyboard.py" &
sleep 5
