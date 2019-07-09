#!/bin/bash
cd "$(dirname "$0")"
source ../../devel/setup.bash
MAPFILE=`realpath ../map/simple_walls.world`

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${MAPFILE}" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &

