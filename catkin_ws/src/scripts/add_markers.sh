#!/bin/bash
cd "$(dirname "$0")"
source ../../devel/setup.bash
export TURTLEBOT_GAZEBO_WORLD_FILE=`realpath ../map/simple_walls.world`
export TURTLEBOT_GAZEBO_MAP_FILE=`realpath ../map/map.yaml`
export TURTLEBOT_CUSTOM_PARAM_FILE=`realpath ../turtlebot_simulator/turtlebot_gazebo/config/custom_cost_params.yaml`

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "rosrun add_markers add_markers" &

