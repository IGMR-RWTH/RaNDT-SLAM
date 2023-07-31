#include <ros/ros.h>
#include <math.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

class NavtechConverter {
  ros::NodeHandle nh;
  ros::Subscriber image_sub;
  ros::Publisher pc_pub;
  float radar_range_resolution_;
  public:
    NavtechConverter() : nh("~") {
      nh.param<float>("radar_distance_resolution", radar_range_resolution_, 0.0161018390208);

      image_sub = nh.subscribe("/Navtech/Polar", 1, &NavtechConverter::cb, this);
      pc_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/radar_2/isdr_2/spectrum_transformed/", 1);
    }

    void cb(const sensor_msgs::Image::ConstPtr& msg) {
      cv_bridge::CvImagePtr cv_ptr, cv_remapped(new cv_bridge::CvImage);
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
      }
      catch (cv_bridge::Exception &e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
 
      pcl::PointCloud<pcl::PointXYZI>::Ptr out_pc(new pcl::PointCloud<pcl::PointXYZI>);
      pcl_conversions::toPCL(msg->header, out_pc->header);
      if (out_pc->header.frame_id == "") {
        out_pc->header.frame_id = "navtech";
      }
      for (int a_bin = 0; a_bin < cv_ptr->image.rows; a_bin++) {
        const double theta = (double(a_bin + 1) / cv_ptr->image.rows) * 2. * M_PI;
        cv::Mat azimuth = cv_ptr->image.row(a_bin);
        for (int r_bin = 0; r_bin < azimuth.cols; r_bin++) {
          const double range = radar_range_resolution_ * double(r_bin);
          const double intensity = double(azimuth.at<uchar>(r_bin));
          if (intensity > 50) {
            pcl::PointXYZI p;
            p.x = range * std::cos(theta);
            p.y = range * std::sin(theta);
            p.intensity = intensity;
            out_pc->push_back(p);
          }
        }
      }
      pc_pub.publish(*out_pc);
    }    
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navtech_converter");
  NavtechConverter nc;
  ros::spin();
  return 0;
}
