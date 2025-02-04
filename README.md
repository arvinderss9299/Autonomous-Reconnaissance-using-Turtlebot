# Mobile Robotics Final Project
This project aims to implement Sampling-based Victim Searching using a turtlebot
![exploration-gif](https://github.com/arvinderss9299/Autonomous-Reconnaissance-using-Turtlebot/blob/main/exploration.gif)

## Preparation

install Apriltag Package on turtlebot
```
sudo apt install ros-noetic-apriltag-ros
```

download and make explore_lite package on remote pc
```
cd ~/catkin_ws/src
git clone https://github.com/hrnr/m-explore.git
```
follow the pC, OpenCR, Raspberry Pi setup on 
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup


## To run the turtlebot in Gazebo
On pc

```
roslaunch mr_final_project turtlebot3_explore.launch use_sim:=true
```

## To run the turtlebot in real world

On turtlebot
```
roslaunch mr_final_project hardware.launch
```

On remote pc
```
roslaunch mr_final_project explore.launch
```
