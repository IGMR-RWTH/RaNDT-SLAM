cmake_minimum_required(VERSION 3.0.2)
project(indurad_navtech_converter)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
  indurad_radar_msgs
)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS cv_bridge pcl_conversions pcl_ros roscpp sensor_msgs std_msgs tf2_ros indurad_radar_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(navtech_converter src/navtech_converter_node.cpp)
target_link_libraries(navtech_converter
  ${catkin_LIBRARIES}
)
add_executable(navtech_oxford_converter src/navtech_oxford_converter_node.cpp)
target_link_libraries(navtech_oxford_converter
  ${catkin_LIBRARIES}
)
add_executable(indurad_converter src/indurad_converter_node.cpp)
target_link_libraries(indurad_converter
  ${catkin_LIBRARIES}
)
