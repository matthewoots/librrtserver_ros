# RRT Server Wrapper for ROS

## Installation
`librrtserver_ros` serves a searching server on ROS using the `librrtserver` package and acts as a wrapper to pass data into the module.

Search time is around `1ms` to `0.08s` including shortening of the searched path.
- Included the reuse of search paths without another random path being generated, this will affect smoothness and time optimality
- Faster search time and stability for optimization backend options

| Original version | Modified version |
| :---: | :---: |
| ![Alt Text](rrt_local_planning.gif) | ![Alt Text](modified_rrt.gif) |

### Dependencies
- librrtserver (https://github.com/matthewoots/librrtserver.git) 
- mockamap (https://github.com/HKUST-Aerial-Robotics/mockamap.git)

### Setup
For starters who do not know how to use ROS and do not have a prior workspace, just run the commands below and all will be fine
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/matthewoots/librrtserver_ros.git --recurse-submodules
cd ..
catkin build
```
### Simple Run
1. Inside the `parameters.yaml` change to the desired `end_position` and `start_position`
2. Launch `roslaunch librrtserver_ros sample.launch` if you want to use the `mockamap` package, however if you have a `pcd` file, use the `sample_pcd.launch`
3. **IMPORTANT** If you want to use your pcd files, run `rosrun pcl_ros pcd_to_pointcloud <pcl_file_name>.pcd 0.1 _frame_id:=/world`