# turtlebot_package

# Turtlebot_package

## First Execution

cd ~/turtlebot_custom
catkin_make
    ROS_PACKAGE_PATH=/home/kira/turtlebot_custom/src:/opt/ros/noetic/share


## To Launch the map

source ~/turtlebot_custom/devel/setup.bash #Never forget to source the directory
roslaunch turtlebot3_custom turtlebot3_custom.launch


## Launch RViz

roslaunch py_astar_planner turtlebot3_navigation.launch

## Note 

I created in src\turtlebot3_custom a script directory. If you have python scripts, create a new directory there and put them in.
