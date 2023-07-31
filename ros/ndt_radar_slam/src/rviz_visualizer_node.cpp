#include <ros/ros.h>
#include "ndt_visualization/rviz_visualization.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndt_visualizer");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  rc::navigation::ndt::RvizVisualizer rviz_visualizer;
  std::cout << "waiting for shutdown...\n";
  ros::waitForShutdown();
}