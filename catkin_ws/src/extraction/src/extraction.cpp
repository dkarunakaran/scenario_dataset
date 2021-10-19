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
#include <nlohmann/json.hpp>
#include <tf/tf.h>
#include <ibeo_object_msg/IbeoObject.h> 
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
    std::vector<uint32_t> odomTimeStamp; //std::vector<geometry_msgs::Pose> odomPos; 
    std::vector<json> odomPos;
    std::vector<uint32_t> objectsTimeStamp; //std::vector<geometry_msgs::Pose> objectsPos; 
    std::vector<json> objectsPos;
    bool resume;
    bool storeDataInitially;
    std::string odom_json_file;std::string objects_json_file;

    Extraction() : h264_bag_playback() {
      previous_percentage = -1;
      current_message_number = 0;
      private_nh.getParam("objects", objects_topic);
      private_nh.getParam("odometry", odometry_topic);
      private_nh.getParam("bag_file", bag_file);
      private_nh.getParam("resume", resume);
      private_nh.getParam("odom_json_file", odom_json_file);
      private_nh.getParam("objects_json_file", objects_json_file);
    }

    void storeData(){
      if(!resume){
        //Converting the vector to json
        json odomJson(odomPos);json objJson(objectsPos);
        // write prettified JSON to a file
        std::ofstream o1(odom_json_file);
        o1 << std::setw(4) << odomJson << std::endl;
        std::ofstream o2(objects_json_file);
        o2 << std::setw(4) << objJson << std::endl;
      }
      ROS_INFO_STREAM("Odometry size: "<<odomPos.size()<<" Objects timestamp: "<<objectsPos.size());
    }
         
    void MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message) {
      if(storeDataInitially){
        if(resume){
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
        
        }else{
          if(message.getTopic() == odometry_topic){
            nav_msgs::Odometry::ConstPtr odomPtr = message.instantiate<nav_msgs::Odometry>();
            if(odomPtr != nullptr){
              if(std::find(odomTimeStamp.begin(), odomTimeStamp.end(), odomPtr->header.stamp.sec) != odomTimeStamp.end()){
              }else{
                json jData = constructJsonData(odomPtr);
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
                json jData = constructJsonData(objPtr);
                objectsPos.push_back(jData);  
                objectsTimeStamp.push_back(objPtr->header.stamp.sec);  
              }
            }
          }//object topic if close 
          
         
        }//resume else close
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
        
        /*if(message.getTopic() == objects_topic){
          ROS_INFO_STREAM("Topic: "<<message.getTopic());
        }else if(message.getTopic() == odometry_topic){
          ROS_INFO_STREAM("Topic: "<<message.getTopic());
        }*/
        // Do the lane change detection
      }
    }//MessagePublisher method closes
};


int main(int argc, char **argv) {

  ros::init(argc, argv, "Extraction");

  Extraction extract;
  extract.init_playback();
  extract.storeDataInitially = true;  
  extract.ReadFromBag();
  extract.storeData();
  extract.storeDataInitially = false;
  extract.ReadFromBag();
  
  return 0;
}

