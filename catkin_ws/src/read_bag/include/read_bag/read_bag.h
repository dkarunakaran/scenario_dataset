#ifndef READBAG_H
#define READBAG_H

#include <map>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <custom_point_types/point_xyzir.h>
#include <custom_point_types/point_xyzirl.h>
#include <opencv/cv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <lane_points_msg/LanePoints.h>


class ReadBag {
    public:
        ReadBag(ros::NodeHandle* nodehandle);
    
    private:

	//Variables
        ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
	ros::Subscriber lanepcSub;
        std_msgs::Int32MultiArray x_array;
 	std_msgs::Int32MultiArray y_array;
  	std_msgs::Float32MultiArray i_array;
  	int seq_count;
  	ros::Publisher lanePointsPerSecPub;
	int cm_resolution;
	
	//Methods
	void initializeSubscribers(); 
       	void initializePublishers();
	void process(const lane_points_msg::LanePoints::ConstPtr &msg);
	bool checkRegionOfInterest(std::pair<int,int>, int, int, std::list<std::pair<int, int>>);
	std::list<std::pair<int,int>> vehicle_odom;

};

#endif // READBAG_H
