# RaNDT SLAM with Oxford Radar RobotCar Dataset

To evaluate the performance of RaNDT SLAM with the oxford dataset, several additional steps need to be taken.

## Dependencies

Following additional dependencies are needed:
- [CFEAR radarodometry] (https://github.com/dan11003/CFEAR_Radarodometry)
- [Radar KITTI benchmark] (https://github.com/dan11003/radar_kitti_benchmark)

Additionally, to convert the raw data of the dataset into bagfiles of the required format, an additional python tool is necessary:
- [Oxford Radar Converter] (https://github.com/maxhilger/oxford_radar_converter)

Make sure that all of these packages are cloned in your workspace before building.

## Convert raw oxford data into rosbags

As our system considers point cloud inputs, we preprocess the raw oxford data such that it is equivalent to our desired input. This task could also be fulfilled by a different ROS node.

1. Download the [Oxford Radar RobotCar Dataset](https://oxford-robotics-institute.github.io/radar-robotcar-dataset/datasets). Note, that at the time of this being written the bulk download script seems not to work, so you'll probably need to download the sequences individually.

We assume the following file structure (in this example with sequence 2019-01-17-13-26-39): 
<pre>
oxford-eval-sequences
|
└───2019-01-17-13-26-39-radar-oxford-10k
    |
    └───radar
        |
        └───<timestamp>.png
    |
    └───gt
        |
        └───radar_odometry.csv
</pre>

2. In the aforementioned Oxford Radar Converter, open file "python/convert_radar.sh". Adjust the filepath in this file to the location of the raw data and your machine and to the sequence that you wish to transform

3. Run the conversion script: 
```console
chmod +x convert_radar.sh
./convert_radar
```
The resulting rosbag will be stored in the radar directory of the sequence.

4. You may need to reindex the bagfile
```console
rosbag reindex 2019-01-17-13-26-39-radar-oxford-10k.bag
```

## Launch RaNDT SLAM

1. Adjust the paths in RaNDT-SLAMs "config/parameters_oxford.yaml". The most easy way is, again, to use the provided docker container. If you run the container using 
```console
docker run -it -v ~/<your workspace>:/root/catkin_ws -v <your_base_location/oxford_eval_sequences>:/root/catkin_ws/rosbags/oxford_raw/oxford_eval_sequences --network host --env ROS_MASTER_URI=http://localhost:11311 ndt_slam_docker
```
the paths should be already correct.

2. Source the workspace
```console
source ~/catkin_ws/devel/setup.bash
```

3. Launch RaNDT SLAM together with the CFEAR evaluation tool using
```console
roslaunch ndt_radar_slam ndt_slam_oxford.launch
```

4. Wait for the launch file to exit by itself. Otherwise, the evaluation tool may not be able to store all estimated poses.

## Evaluate Results

For this step, use the Radar kitti benchmark. The results of the run are stored as follows:

<pre>
<your_ws>
|
└───randt_eval
    |
    └───odom
        |
        └───est
        |
        └───gt
    |
    └─── slam
        |
        └───est
        |
        └───gt
</pre>

To evaluate results of the SLAM, execute
```console
export KITTI_DIR=`rospack find kitti-odom-eval`
python3 $KITTI_DIR/python/eval_odom.py --dir ~/catkin_ws/randt_eval/slam --align 6dof --force yes
```
Similarly, to evaluate the results without loop clousre, use
```console
export KITTI_DIR=`rospack find kitti-odom-eval`
python3 $KITTI_DIR/python/eval_odom.py --dir ~/catkin_ws/randt_eval/odom --align 6dof --force yes

```

## Raw and evaluated results used in submission

For convenience, we provide the raw and evaluated results of the experiments in this repo in the oxford_results directory.