#ifndef FEATURE_EXTRACTOR_HEADER
#define FEATURE_EXTRACTOR_HEADER

#include <map>
#include <chrono>

#include <ros/ros.h>


#include <visualization_msgs/Marker.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <h264_bag_playback/h264_bag_playback.hpp>
//#include <pluginlib/class_list_macros.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/PCLPointCloud2.h>

#include <custom_point_types/point_xyzir.h>
#include <custom_point_types/point_xyzirl.h>

#include <opencv/cv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "draw_shapes.hpp"
#include <lane_points_msg/LanePoints.h>


class FeatureExtractor : public dataset_toolkit::h264_bag_playback
{

public:
  FeatureExtractor()
      : h264_bag_playback(),
        current_message_number(0),
        previous_percentage(-1)
  {}

  void bypass_init() {
    this->onInit();
  }

  void MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message);
  void ImagePublisher(image_transport::Publisher &publisher, const sensor_msgs::ImageConstPtr &message) {}
  void CameraInfoPublisher(ros::Publisher &publisher, const sensor_msgs::CameraInfoConstPtr &message) {}
  void WriteImage();
  bool checkRegionOfInterest(std::pair<int,int>, int, int);
  void publishLanePoints(int, int, int, int, int, int, float, float);


private:

  // initialise ros stuff
  virtual void onInit();

  std::vector<std::string> separateCommas(std::string input_string);

  // call whenever receive a pointcloud - spit out new filtered version
  void SegmentPointCloud_intensity(sensor_msgs::PointCloud2::ConstPtr pointcloud_msg,
                                   std::map<std::pair<int,int>, double> &intensity_topic);

  void SegmentPointCloud_label(sensor_msgs::PointCloud2::ConstPtr pointcloud_msg,
                               std::map<std::pair<int,int>, double> &intensity_topic);

  pcl::PointCloud<pcl::PointXYZIR>::Ptr Selector(pcl::PointCloud<pcl::PointXYZIR>::Ptr input_cloud,
                                                 float maximum_range,
                                                 std::set<int> ring_filter,
                                                 std::set<int> &rings_included);
  pcl::PointCloud<pcl::PointXYZIRL>::Ptr Selector_label(pcl::PointCloud<pcl::PointXYZIRL>::Ptr input_cloud,
                                                        float maximum_range,
                                                        std::set<int> ring_filter,
                                                        std::set<int> &rings_included);


  std::map<std::string, std::map<std::pair<int,int>, double>> intensity_map;
  std::list<std::pair<int,int>> vehicle_odom;

  std::vector<std::shared_ptr<DrawItem>> item_draw_properties;

  std::set<int> rings_included;
  std::set<int> ring_filter;

  std::string projection_frame;

  int cm_resolution;

  // variables to estimate the time remaining to perform the projection
  uint32_t current_message_number;
  int previous_percentage;
  std::chrono::steady_clock::time_point start_time, end_time;
  //std::vector<std::pair<int, int>> allPoints;
  ros::NodeHandle n;
  ros::Publisher lanePcPub;
  ros::Publisher signalShutdown;
  std::map<std::pair<int,int>, std::map<std::pair<int, int>, double>> allPoints;
  std::vector<long int> secWatch;
  std_msgs::Int32MultiArray x_array;
  std_msgs::Int32MultiArray y_array;
  std_msgs::Float32MultiArray i_array;
  int seq_count;
  ros::Publisher pointsPerSecPub;
  std_msgs::Int32MultiArray odom_x_array;
  std_msgs::Int32MultiArray odom_y_array;
  std::vector<int> x_point;
  std::vector<int> y_point;
  std::vector<int> i_point;
  std::vector<int> odom_x_point;
  std::vector<int> odom_y_point;

};


#endif
