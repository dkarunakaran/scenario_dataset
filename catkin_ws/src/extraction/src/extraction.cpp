#include <h264_bag_playback/h264_bag_playback.hpp>
#include <dataset_tools/run_pipeline.hpp>
#include <dataset_tools/point_cloud_features_pipeline.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <dataset_msgs/DatasetEvent.h>
#include <sensor_msgs/NavSatFix.h>
#include <custom_point_types/point_xyzir.h>
#include <chrono>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "json.hpp"
#include <tf/tf.h>
#include <ibeo_object_msg/IbeoObject.h> 
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <tf2_ros/transform_listener.h>
#include "helper_functions.hpp"

using json = nlohmann::json;

class Extraction : public dataset_toolkit::h264_bag_playback {

  public:
    int previous_percentage;
    // variables to estimate the time remaining to perform the projection
    uint32_t current_message_number;
    std::chrono::steady_clock::time_point start_time, end_time;
    std::string odometry_topic; std::string objects_topic;
    std::string bag_file;
    std::vector<uint32_t> odomTimeStamp;
    std::vector<json> odomPos;
    std::vector<uint32_t> objectsTimeStamp;
    std::vector<json> objectsPos;
    bool resume;
    bool storeDataInitially;
    std::string odom_json_file;std::string objects_json_file;
    std::string lanelet_file; lanelet::LaneletMapPtr map;
    std::vector<uint32_t> egoSec;

    Extraction() : h264_bag_playback() {
      previous_percentage = -1;
      current_message_number = 0;
      private_nh.getParam("objects", objects_topic);
      private_nh.getParam("odometry", odometry_topic);
      private_nh.getParam("bag_file", bag_file);
      private_nh.getParam("resume", resume);
      private_nh.getParam("odom_json_file", odom_json_file);
      private_nh.getParam("objects_json_file", objects_json_file);
      private_nh.getParam("lanelet_file", lanelet_file);
      lanelet::projection::UtmProjector projector(lanelet::Origin({0, 0}));  // we will go into details later
      map = lanelet::load(lanelet_file, projector);
      //Below code is an example and do not delete
      /*try {
        geometry_msgs::PointStamped baseLinkPoint;
        baseLinkPoint.point.x = egoPosX;baseLinkPoint.point.y = egoPosY;baseLinkPoint.point.z = 0;
        baseLinkPoint.header.frame_id = "base_link";baseLinkPoint.header.stamp.sec = sec;
        geometry_msgs::PointStamped odomPoint;
        transformer_->setUsingDedicatedThread(true); 
        transformer_->transform(baseLinkPoint, odomPoint, "odom");
      }catch (const std::exception &e) {
        ROS_ERROR_STREAM(e.what());
      }*/
    }

    void loadData(){
      // read a JSON file
      std::ifstream i1(odom_json_file);
      json j1;i1 >> j1; 
      for (auto& item : j1) {
        odomPos.push_back(item);
        auto sec = item["sec"].get<int>();
        odomTimeStamp.push_back(sec);
      }
      std::ifstream i2(objects_json_file);
      json j2;i2 >> j2;
      for (auto& item : j2) {
        objectsPos.push_back(item);
        auto sec = item["sec"].get<int>();
        objectsTimeStamp.push_back(sec);
      }
      
      ROS_INFO_STREAM("Odometry size: "<<odomPos.size()<<" Objects timestamp: "<<objectsPos.size());
    }
    
    void storeData(){
      //Converting the vector to json
      json odomJson(odomPos);json objJson(objectsPos);
      // write prettified JSON to a file
      std::ofstream o1(odom_json_file);
      o1 << std::setw(4) << odomJson << std::endl;
      std::ofstream o2(objects_json_file);
      o2 << std::setw(4) << objJson << std::endl;
      ROS_INFO_STREAM("Odometry size: "<<odomPos.size()<<" Objects timestamp: "<<objectsPos.size());
    }
         
    void MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message) {
      if(storeDataInitially){
        if(message.getTopic() == odometry_topic){
          nav_msgs::Odometry::ConstPtr odomPtr = message.instantiate<nav_msgs::Odometry>();
          if(odomPtr != nullptr){
            if(std::find(odomTimeStamp.begin(), odomTimeStamp.end(), odomPtr->header.stamp.sec) != odomTimeStamp.end()){
            }else{
              json jData = constructJsonData(odomPtr, transformer_);
              odomPos.push_back(jData);  
              odomTimeStamp.push_back(odomPtr->header.stamp.sec);
              ROS_INFO_STREAM("Storing... Time in sec: "<<odomPtr->header.stamp.sec);
            }
          }
        }//odometry topic if close
        if(message.getTopic() == objects_topic){
          ibeo_object_msg::IbeoObject::ConstPtr objPtr = message.instantiate<ibeo_object_msg::IbeoObject>();
          if(objPtr != nullptr){
            if(std::find(objectsTimeStamp.begin(), objectsTimeStamp.end(), objPtr->header.stamp.sec) != objectsTimeStamp.end()){
            }else{
              json jData = constructJsonData(objPtr, transformer_);
              objectsPos.push_back(jData);  
              objectsTimeStamp.push_back(objPtr->header.stamp.sec);  
            }
          }
        }//object topic if close 
      }else{
        current_message_number++;
        int current_percentage = 100.0 * ((float) current_message_number / (float) total_message_count);
        if (current_percentage != previous_percentage) {
          previous_percentage = current_percentage;
          if (current_percentage == 0) {
            std::cout << "Reading the bag file started" << std::endl;
          } else {
            end_time = std::chrono::steady_clock::now();
            std::chrono::duration<float, std::ratio<60>> elapsed = end_time - start_time;
            float estimated_remaining = (float) (100 - current_percentage) * elapsed.count();
            std::cout << current_percentage << "%, est. remaining " << estimated_remaining << " minutes" << std::endl;
          }
          start_time = std::chrono::steady_clock::now();
        }
        
        laneChangeDetection(message);
      }
    }//MessagePublisher method closes

    void laneChangeDetection(const rosbag::MessageInstance &msg){
      if(msg.getTopic() == odometry_topic){
        nav_msgs::Odometry::ConstPtr odomPtr = msg.instantiate<nav_msgs::Odometry>();
        if(std::find(egoSec.begin(), egoSec.end(), odomPtr->header.stamp.sec) != egoSec.end()){
        }else{
          auto sec = odomPtr->header.stamp.sec; 
          std::vector<uint32_t> projectedSec;
          for(size_t i=sec+1; i<=sec+8; i++){
            if(checkSecExist(odomTimeStamp, i))
              projectedSec.push_back(i);
          }
          
          //1. Overtaking scenario
          overtakingScenario(sec, projectedSec);   
          
          egoSec.push_back(sec);
        }
      }//Odometry topic if closes
    }

    void overtakingScenario(uint32_t sec, std::vector<uint32_t> projectedSec){
      
      //1. Find the number lines where the car is at the current sec
      
      for(auto& sec: projectedSec){
        auto index = findIndex(odomTimeStamp, sec);  
        auto egoJsonData = odomPos[index];
        double egoPosX = egoJsonData["position_x"];double egoPosY = egoJsonData["position_y"]; 
        auto egoPoint = lanelet::BasicPoint2d(egoPosX, egoPosY);
        //1. identify the number of lanes in the projected sec
        //a. identify the lanelet that the search point is in
        auto nearLanelets = findTheActualLanelet(map, egoPoint);
        if(std::get<0>(nearLanelets) && lanelet::geometry::length3d(std::get<1>(nearLanelets)) > 0){
          auto egoLanelet = std::get<1>(nearLanelets);
          auto lanelets = std::get<2>(nearLanelets);
          //b. find the lanelet that is left and right 
        
        
        }//Checking lanelet for the egopoint if closes      
        else{
          ROS_INFO_STREAM("No lanelet found: length of the lanelet is zero or simply couldn't find the lanelet that actually has the point");
        } 

      }
    }
};


int main(int argc, char **argv) {

  ros::init(argc, argv, "Extraction");

  Extraction extract;
  extract.init_playback();
  ROS_INFO_STREAM("Resume: "<<extract.resume);
  if(extract.resume){
    ROS_INFO_STREAM("Data is loading from JSON files...");
    extract.loadData();
  }else{
    ROS_INFO_STREAM("Data is loading from Bag file...");
    extract.storeDataInitially = true;  
    extract.ReadFromBag();
    extract.storeData();
  }
  extract.storeDataInitially = false;
  extract.ReadFromBag();
  
  return 0;
}

