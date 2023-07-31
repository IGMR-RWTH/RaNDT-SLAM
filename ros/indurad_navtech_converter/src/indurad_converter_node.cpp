#include <ros/ros.h>
#include <math.h>
#include <opencv2/imgcodecs.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <indurad_radar_msgs/SpectrumPng.h>
#include <sensor_msgs/Image.h>

class InduradConverter {
  ros::NodeHandle nh;
  ros::Subscriber indurad_sub;
  ros::Publisher image_pub;
  float radar_range_resolution;
  public:
    InduradConverter() : nh("~") {
      nh.param<float>("radar_distance_resolution", radar_range_resolution, 0.0161018390208);

      indurad_sub = nh.subscribe("/radar_2/isdr_2/spectrum/", 1, &InduradConverter::cb, this);
      image_pub = nh.advertise<sensor_msgs::Image>("/Navtech/Polar", 1);
    }

    void cb(const indurad_radar_msgs::SpectrumPng::ConstPtr& msg) {
      cv::Mat polar_png_image = cv::imdecode(msg->peaks, cv::IMREAD_GRAYSCALE);
      cv::rotate(polar_png_image, polar_png_image, cv::ROTATE_90_CLOCKWISE);
      cv::Mat target_image;
      cv::flip(polar_png_image, target_image, 1);
      std_msgs::Header header;
      header.frame_id = msg->frame_id;
      header.stamp = msg->stamps.at(0);
      sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(header, "mono8", target_image).toImageMsg();
      image_pub.publish(out_msg);
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "indurad_converter");
  InduradConverter ic;
  ros::spin();
  return 0;
}