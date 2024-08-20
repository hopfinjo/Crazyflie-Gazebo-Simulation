# ROS - GAZEBO - Crazyflie2 Simulation

Some Background info on this Repo:
This repo was build on top of https://github.com/gsilano/CrazyS repo and it adds a different controller for the Crazyflie2 (A drone model).
The code in this Repo has less options and is more focused on essential code, so that new learners can understand easier the essentials of Controllers/Drones.


## Installation Instructions - Ubuntu 20.04 with ROS Noetic and Gazebo 11

Personally, I ran this project on my Windows machine in an Ubuntu 20.04.6 LTS app which I downloaded from the "Microsoft Store".
This does not work in other environments. Ensure exactly needed Ubuntu is installed. 

### 1. Install and initialize ROS Noetic desktop full, additional ROS packages, catkin-tools, and wstool:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink
sudo apt install ros-noetic-octomap-mapping ros-noetic-control-toolbox
sudo apt install python3-vcstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev
sudo rosdep init
rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python3-wstool ros-noetic-ros libgoogle-glog-dev
