#include "read_bag.h"
#include "Conversions.h"


ReadBag::ReadBag(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    ROS_INFO("In class constructor of LanePointCloud class");
    nh_.param<int>("cm_resolution", cm_resolution, 2);
    initializeSubscribers();
    initializePublishers();
}

void ReadBag::initializeSubscribers(){
    lanepcSub = nh_.subscribe("points_per_sec", 1000, &ReadBag::process, this);
}

void ReadBag::initializePublishers(){
    lanePointsPerSecPub = nh_.advertise<lane_points_msg::LanePoints> ("lane_points", 1);
}

  
void ReadBag::process(const lane_points_msg::LanePoints::ConstPtr &msg){
        
}



int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "read_bag"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    ReadBag readBag(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ros::spin();
    return 0;
} 
