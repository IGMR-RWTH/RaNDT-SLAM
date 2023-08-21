# RaNDT SLAM

This package provides the Rader Normal Distribution Transform (RaNDT) SLAM.

A benchmark data set is available at [Zenodo](https://zenodo.org/record/8199947).

## License

This work is lincensed under the BSD-3 license, copyright 2023 Institute of Mechanism Theory, Machine Dynamics and Robotics - RWTH Aachen University.

## Dependencies

The package has following dependencies:
- Ceres Solver
- Sophus
- Eigen

Additionally, the following ros packages should be installed:
- ros-noetic-perception
- ros-noetic-imu-tools
- ros-noetic-imu-pipeline (these are needed to transform IMU messages into correct frame)

## Installation
Prerequisits: Installed all dependencies

```console
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```
If you need debug symbold, select `-DCMAKE_BUILD_TYPE=RelWithDebInfo`. Ceres and Eigen are barely usable in debug mode.

## Docker
Optionally, you can use the provided Dockerfile. 

Build docker:
```console
cd normal_distributions_slam
docker build -t ndt_slam_docker
```

Start docker:
```console
docker run -it -v ~/<your workspace>:/home/root/catkin_ws --network host --env ROS_MASTER_URI=http://localhost:11311 ndt_slam_docker
```

In docker:
```console
cd catkin_ws
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

## Structure

The package holds two nodes:
- ndt_slam
- ndt_visualization
  
The former is the actual SLAM system, while the latter can be optionally used to visualize the NDT representation of the radar scans and the map.

The ndt_slam part has several subcomponents:
- **ndt_slam** is the main interface to ROS. It handles data input and output. Additionally, it stores most of the data and several mutexes
- **local_fuser** depicts the front end of the slam system. It gets the input from the interface and is responsible to manage scan matching, submap initialization, loop closure detection, and global optimization. Here, we inserted also **ScanContext** (https://github.com/irapkaist/scancontext), which is used for loop closure detection.
- **radar_preprocessing** contains methods to filter and cluster the incoming radar scan data. It is invoked by the local fuser
- **ndt_representation** holds the different map and cell types and provides interfaces to them. **ndt_cell** holds a single cell with its mean and covariants. The **ndt_map** holds several cells to form a map. The **ndt_hierarchical_map** stores both a NDT map and a grid counting occupancy values. This is necessary to generate global OGMs for the usage in ROS. All of the occupancy-counting grids are combined in the **ndt_master_map** to generate the OGM.
- **ndt_registration** provides an interface to Ceres to allow NDT matching and motion estimation 
- **global_fuser** is responsible for pose graph optimization. Again, we use Ceres here.

## Setup and Usage

__Tuning__

We prepared three parameter configurations for different environments: 
- indoor (parameters_indoor.yaml)
- outoor (parameters_outdoor.yaml)
- mixed (parameters_mixed.yaml)

We suggest using them. 

If you want to deploy a different sensor, you have to set the parameters accordingly. The frames and topics are given in ndt_radar_slam_base_parameters.yaml. There, you can additionally choose not to use the IMU, or turn off visualization. The meaning of the parameters is explained in the config files.

__Usage__

Four parameters are considered in roslaunch:
- visualization: if true, start visualizer node. Else, just the SLAM node
- base_frame: frame to which the IMU message should be transformed. Has to be the same as specified in the config file
- raw_imu_topic: topic of the incoming IMU messages
- environment: select one of indoor, outdoor, and mixed to load the corresponding paramters

Launch the node using

```console
roslaunch ndt_radar_slam ndt_slam.launch environment:=outdoor
```