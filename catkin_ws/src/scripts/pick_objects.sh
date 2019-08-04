#!/bin/bash
cd "$(dirname "$0")"
source ../../devel/setup.bash
TURTLEBOT_GAZEBO_WORLD_FILE=`realpath ../map/simple_walls.world`
TURTLEBOT_GAZEBO_MAP_FILE=`realpath ../map/map.yaml`

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "rosrun pick_objects pick_objects" &

