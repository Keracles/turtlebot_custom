# Turtlebot_package of Group S03


# This is the instruction for launching automatic slam :

## Launch Map
export TURTLEBOT3_MODEL=burger  
source ~/turtlebot_custom/devel/setup.bash  
roslaunch turtlebot3_custom turtlebot3_world.launch

## Slam
export TURTLEBOT3_MODEL=burger  
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

## Move Base
export TURTLEBOT3_MODEL=burger  
roslaunch turtlebot3_navigation move_base.launch

## Explorer.py
export TURTLEBOT3_MODEL=burger  
python3 scripts/explore.py


# This is the instruction for launching "fetching boxe" :




# This is the instruction for yolo detection on slam map :