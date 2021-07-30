#ifndef LANE_POINTCLOUD_H
#define LANE_POINTCLOUD_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <custom_point_types/point_xyzir.h>
#include <custom_point_types/point_xyzirl.h>
#include <opencv/cv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class LanePointCloud {
    public:
        LanePointCloud(ros::NodeHandle* nh);
    
    private:
        ros::Subscriber lanepcSub;
        void process(sensor_msgs::PointCloud2& data)

    
};

#endif // LANE_POINTCLOUD_H