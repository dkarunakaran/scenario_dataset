#include "lane_pointcloud.h"

LanePointCloud::LanePointCloud(ros::NodeHandle* nh){
    ROS_INFO("In class constructor of LanePointCloud class");
    lanepcSub = nh->subscribe("/lane_pc", 1, &LanePointCloud::process, this);
}

void LanePointCloud::process(sensor_msgs::PointCloud2& data){
    ROS_INFO_STREAM("HERE")
}

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "lane_pointcloud"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type ExampleRosClass");
    LanePointCloud lanePointCloud(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 