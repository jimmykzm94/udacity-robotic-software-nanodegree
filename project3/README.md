# Create map
roslaunch pgm_map_creator request_publisher.launch

# Submodule
1. teleop_twist_keyboard (update submodule before using it)

# Run simulation
Open terminal and type:
`roslaunch my_robot world.launch`
Open another terminal and type:
`roslaunch my_robot amcl.launch`
Open another terminal to use keyboard control:
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

