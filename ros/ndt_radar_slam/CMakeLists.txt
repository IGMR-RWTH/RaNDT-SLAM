cmake_minimum_required(VERSION 3.0.2)
project(ndt_radar_slam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED) #3.4 REQUIRED NO_MODULE) # PATHS /usr/local/include/eigen3)
find_package(PCL REQUIRED)
find_package(Python COMPONENTS Interpreter Development)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(catkin REQUIRED
	roscpp
	std_msgs
	tf
	visualization_msgs
	ndt_msgs
  sensor_msgs
  nav_msgs
  rosbag
)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/local/include/eigen3")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/local/include/optim") # -I/usr/local/include/eigen3")

if(OPENMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


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
	INCLUDE_DIRS include
  LIBRARIES ndt_representation local_fuser global_fuser radar_preprocessing ndt_registration
  CATKIN_DEPENDS std_msgs tf visualization_msgs ndt_msgs sensor_msgs message_filters
  DEPENDS Eigen3 PCL CERES Sophus
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Python_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIR}
)

add_library(ndt_representation
  src/ndt_representation/ndt_master_map.cpp
  src/ndt_representation/ndt_hierarchical_map.cpp
  src/ndt_representation/ndt_cell.cpp
  src/ndt_representation/ndt_map.cpp
)
target_link_libraries(ndt_representation
  ${catkin_LIBRARIES}
)
add_dependencies(ndt_representation
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS} 
)

add_library(local_fuser
  src/local_fuser/local_fuser.cpp
  src/local_fuser/Scancontext/Scancontext.cpp
)
target_link_libraries(local_fuser
  radar_preprocessing
  ndt_representation
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(local_fuser
  radar_preprocessing
  ndt_representation
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS} 
)

add_library(global_fuser
  src/global_fuser/global_fuser.cpp
)
target_link_libraries(global_fuser
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)
add_dependencies(global_fuser
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS} 
)

add_library(radar_preprocessing
  src/radar_preprocessing/radar_preprocessor.cpp
  src/radar_preprocessing/grid.cpp
)
target_link_libraries(radar_preprocessing
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(radar_preprocessing
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS} 
)

add_library(ndt_registration 
  src/ndt_registration/ndt_matcher.cpp
  src/ndt_registration/ceres_loss_functions.cpp
)
target_link_libraries(ndt_registration
  ndt_representation
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)
add_dependencies(ndt_registration
  ndt_representation
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_executable(ndt_slam
  src/ndt_slam/ndt_slam.cpp
  src/ndt_slam_node.cpp
)
target_link_libraries(ndt_slam
  Eigen3::Eigen
  Sophus::Sophus
  local_fuser
  global_fuser
  ndt_registration
  ${catkin_LIBRARIES}
	${Eigen3_LIBRARIES}
)
add_dependencies(ndt_slam
  local_fuser
  global_fuser
  ndt_registration
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
)

add_executable(ndt_visualization
  src/ndt_visualization/rviz_visualization.cpp
  src/rviz_visualizer_node.cpp
)
target_link_libraries(ndt_visualization
  ${catkin_LIBRARIES}
	${Eigen3_LIBRARIES}
  Eigen3::Eigen
)
add_dependencies(ndt_visualization
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ndt_representation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

