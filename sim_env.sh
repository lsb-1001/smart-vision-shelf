#!/usr/bin/env bash
# Source in each simulation terminal. Does not modify ~/.bashrc.
source /opt/ros/humble/setup.bash
if [ -f /usr/share/gazebo/setup.sh ]; then
  source /usr/share/gazebo/setup.sh
fi
export ROS_DOMAIN_ID=87
export TURTLEBOT3_MODEL=waffle_pi
export ROS_LOCALHOST_ONLY=1
tb3_gazebo_prefix=$(ros2 pkg prefix turtlebot3_gazebo)
export GAZEBO_MODEL_PATH="$tb3_gazebo_prefix/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH:-}"
unset tb3_gazebo_prefix
