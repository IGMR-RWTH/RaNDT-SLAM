#include <ros/ros.h>
#include "ndt_slam/ndt_slam.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndt_slam");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  rc::navigation::ndt::NDTSlam ndt_slam;
  std::cout << "waiting for shutdown...\n";
  ros::waitForShutdown();
}