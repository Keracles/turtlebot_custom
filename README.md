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
Depending on the speed of your pc, it might not work the first time we start it since we need lots of things to load.  
If it's not working the first time, close the program and restart it.  
Commands to do if package name is turtlebot3_box_following :  
export TURTLEBOT3_MODEL=burger  
catkin_make  
chmod +x turtlebot_custom/src/turtlebot3_box_following/box_following.py  
roslaunch turtlebot3_box_following camera_and_robot.launch  


# This is the instruction for yolo detection on slam map :

#tutorials followed: https://blog.paperspace.com/yolov7/         https://github.com/leggedrobotics/darknet_ros    

#setup yolo v7:: the installation procedure is explained here, because the file size is too big
#install yolo, ssh key required for recursive clone
- cd /catkin_ws/src
- git clone --recursive git@github.com:leggedrobotics/darknet_ros.git
- cd ..
- install openCV, use code provided in project folder: install_opencv.sh
- pip install setuptools==59.5.0
- pip install torchvision==0.11.3+cu111 -f https://download.pytorch.org/whl/cu111/torch_stable.html
- install graphic magick: sudo apt install graphicsmagick-imagemagick-compat

#install opencv
- run the install_opencv.sh file found in the workspace

#install pretrained weights
- copy the weight and cfg files into the respective folder
catkin_workspace/src/darknet_ros/darknet_ros/yolo_network_config/weights/
catkin_workspace/src/darknet_ros/darknet_ros/yolo_network_config/cfg/

#train the model, the training is done with one cpu, the files were adopted for it
- download the training data: curl -L "https://app.roboflow.com/ds/xQbIcc2V8c?key=GUhGsjPsyF" &gt; roboflow.zip; unzip roboflow.zip; rm roboflow.zip
- copy the files into catkin_workspace/src/darknet_ros/darknet_ros/
- train the model with this coomand
python3 train.py --workers 1 --device cpu --batch-size 1 --data data/data_goal_detection.yaml --img 640 640 --cfg cfg/training/yolov7.yaml --weights '' --name yolov7-goalfinder --hyp data/hyp.scratch.custom.yaml --epochs 30

#build for performance
- catkin_make -DCMAKE_BUILD_TYPE=Release

#run the model
python3 detect.py --weights run