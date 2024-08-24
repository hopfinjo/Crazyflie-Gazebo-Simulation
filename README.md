# ROS - GAZEBO - Crazyflie2 Simulation

This repo was build as an addition to https://github.com/gsilano/CrazyS repo and it adds a different controller for the Crazyflie2(A drone) simulation in Gazebo.
The code in this Repo has less options and is more focused on essential code so that new learners can understand easier the essentials of Controllers/Drones.

## How to get started:
Start by installing the gsilano repo: https://github.com/gsilano/CrazyS/tree/master. Instructions are copied below.
**Once you installed this and configured it you must download the CrazyS folder from this Repo and replace it with the CrazyS folder you cloned from the gsilano repository.**

## Installation Instructions - Ubuntu 20.04 with ROS Noetic and Gazebo 11


To use the code developed and stored in this repository some preliminary actions are needed. They are listed below.

1. Install and initialize ROS Melodic desktop full, additional ROS packages, catkin-tools, and wstool:
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt install curl # if you haven't already installed curl
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo apt update
$ sudo apt install ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink
$ sudo apt install ros-noetic-octomap-mapping ros-noetic-control-toolbox
$ sudo apt install python3-vcstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt-get install python3-wstool ros-noetic-ros libgoogle-glog-dev
```

2. If you don't have a ROS workspace yet you can do so by
```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ cd ~/catkin_ws/
$ catkin init
$ cd ~/catkin_ws/src
$ git clone -b dev/ros-noetic https://github.com/gsilano/CrazyS.git
$ git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
$ cd ~/catkin_ws
```

3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)
```bash
$ rosdep install --from-paths src -i
$ rosdep update
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
$ catkin build
```

4. Add sourcing to your `.bashrc` file
```bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
In the event the `cmd /opt/ros/melodic/lib/gazebo_ros/gzserver -u -e ode` appears, the solution proposed in #40 temporarily fixes the issue.



5. Exchange CrazyS folder from Gsilano Repo with folder from this Repo
Download CrazyS folder from this REPO https://github.com/hopfinjo/Crazyflie-Gazebo-Simulation/edit/master
and change it with the folder in catkin/src/CrazyS
**Change the whole folder**

I prefer to use github desktop. Clone the whole repo to your machine. Copy and paste the folder into you ubuntu environment.


## Basic Usage: Start simulation

run 
```bash
roslaunch rotors_gazebo crazyflie2_maxi_v2.launch
```
This launches the added controller.

```bash
roslaunch rotors_gazebo crazyflie2_maxi_v2.launch
```

This launches the added controller.

## Some More Info

### The Controller

- **Nodes and Frequencies:** The controller operates on two different nodes, allowing it to run with two custom frequencies. In the original repository, the controller runs within the callback of the drone's position, preventing the use of custom frequencies. While this approach speeds up the simulation, it makes the controller harder to influence and use effectively.

### The Launch File

- **Location:** The launch file is the starting point that triggers further files to be executed. You can find the launch file in:
  ```
  yourmounttoubuntu20.04/catkin_ws/src/CrazyS/rotors_gazebo/launch
  ```

- **Controller File:** The file that publishes the trajectory the drone should follow, or the controller file, in this case, is `hovering_eight_v2.cpp`. It is located in:
  ```
  yourmounttoubuntu20.04/catkin_ws/src/CrazyS/rotors_gazebo/src/nodes
  ```

### The Two Controller Nodes

- **Files:** The two controller node files, `attitude_controller_node_v2` and `position_controller_node_v2`, are located in:
  ```
  \\wsl.localhost\Ubuntu-20.04\home\maxi\Crazyflie2-ros-gazebo-simulation\catkin_ws\src\CrazyS\rotors_control\src\nodes
  ```

- **Update Function:** In both controllers, the `UpdateControllerFunction` is triggered at a specified frequency, using messages sent from another node at a different frequency. This function calls the actual code that triggers the controller to manage the drone.

