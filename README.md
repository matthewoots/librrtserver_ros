# RRT Server Wrapper for ROS

## Installation
`librrtserver_ros` serves a searching server on ROS using the `librrtserver` package and acts as a wrapper to pass data into the module.

Search time is around `1ms` to `0.08s` including shortening of the searched path.
- Included the reuse of search paths without another random path being generated, this will affect smoothness and time optimality

![Alt Text](rrt_local_planning.gif)

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
