#!/bin/bash
echo "Setup unitree ros2 environment";
source /opt/ros/humble/setup.bash;
source $HOME/humanoid_ws/install/setup.bash;
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;
