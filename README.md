# ROS - GAZEBO - Crazyflie2 Simulation

Some background info on this Repo:
This repo was build on top of https://github.com/gsilano/CrazyS repo and it adds a different controller for the Crazyflie2(A drone) simulation in Gazebo.
The code in this Repo has less options and is more focused on essential code, so that new learners can understand easier the essentials of Controllers/Drones.
## How to get started:
Start by installing the gsilano repo. Instructions are copied below. Once you installed this and configured it you must download the CrazyS folder from this Repo and replace it with the CrazyS folder you cloned from the gsilano repository.


## Installation Instructions - Ubuntu 20.04 with ROS Noetic and Gazebo 11

Personally, I ran this project on my Windows machine in an Ubuntu 20.04.6 LTS app which I downloaded from the "Microsoft Store".

### 1. Install and initialize ROS Noetic desktop full, additional ROS packages, catkin-tools, and wstool:
The following installations are similar to the one on gsilano CrazyS repository, with the change to download the controller code from this repo instead.

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

2. If you don't have ROS workspace yet you can do so by
```$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ cd ~/catkin_ws/
$ catkin init
$ cd ~/catkin_ws/src
$ git clone -b dev/ros-noetic https://github.com/gsilano/CrazyS.git
$ git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
$ cd ~/catkin_ws


