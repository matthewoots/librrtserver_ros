# RRT Server Wrapper for ROS

## Installation
`librrtserver_ros` serves a searching server on ROS using the `librrtserver` package and acts as a wrapper to pass data into the module.

### Dependencies
- librrtserver (https://github.com/matthewoots/librrtserver.git) 

### Setup
For starters who do not know how to use ROS and do not have a prior workspace, just run the commands below and all will be fine
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/matthewoots/librrtserver_ros.git --recurse-submodules
cd ..
catkin build
```
