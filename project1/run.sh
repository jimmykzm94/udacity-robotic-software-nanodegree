mkdir -p build && cd build
cmake ../ && make && cd ..
# Uncomment to set plugin (to print Welcome message)
# export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/workspace/udacity-robotic-software-nanodegree/project1/build
gazebo world/myworld