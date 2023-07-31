cmake_minimum_required(VERSION 3.0.2)
project(ndt_msgs)


find_package(catkin REQUIRED
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Covariance.msg
  Mean.msg
  NormalDistribution.msg
  NormalDistributions.msg
  ClusteredPointCloud.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs  # Or other packages containing msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)


