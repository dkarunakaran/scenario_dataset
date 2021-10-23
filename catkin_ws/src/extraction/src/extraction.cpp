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
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
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

namespace bg = boost::geometry;
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
    int prevLaneChangeCount;
    std::vector<int> objectClassVec;
    std::vector<std::pair<uint32_t, std::vector<json>>> groupBySec;
    std::vector<std::pair<int, std::vector<json>>> groupByCar;
    
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
      lanelet::projection::UtmProjector projector(lanelet::Origin({0, 0}));  
      map = lanelet::load(lanelet_file, projector);
      prevLaneChangeCount = 0;
      objectClassVec.push_back(5); //car
      objectClassVec.push_back(6); //truck
      objectClassVec.push_back(15); //motorbike
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
        auto sec = item["sec"].get<uint32_t>();
        odomTimeStamp.push_back(sec);
      }
      std::ifstream i2(objects_json_file);
      json j2;i2 >> j2;
      for (auto& item : j2) {
        objectsPos.push_back(item);
        auto sec = item["sec"].get<uint32_t>();
        objectsTimeStamp.push_back(sec);
      }
      
      ROS_INFO_STREAM("Odometry size: "<<odomPos.size()<<" Objects timestamp: "<<objectsPos.size());

      groupThem();
    }

    void groupThem(){
      groupObjects(objectsPos, groupBySec, groupByCar);
      ROS_INFO_STREAM("groupBySec: "<<groupBySec.size()<<" groupByCar: "<<groupByCar.size());
      for(auto& pair: groupBySec){
        objectsTimeStamp.push_back(pair.first);
      }

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
            if(std::find(objectClassVec.begin(), objectClassVec.end(), objPtr->object_id) != objectClassVec.end()){
              json jData = constructJsonData(objPtr, transformer_);
              objectsPos.push_back(jData);  
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
          //overtakingScenario(sec, projectedSec);  

          //2. Cut-in scenario
          cutInScenario(sec, projectedSec);
          
          egoSec.push_back(sec);
        }
      }//Odometry topic if closes
    }

    void overtakingScenario(uint32_t currentSec, std::vector<uint32_t> projectedSec){
      auto index = findIndex(odomTimeStamp, currentSec);  
      auto egoJsonData = odomPos[index];
      double egoPosX = egoJsonData["position_x"];double egoPosY = egoJsonData["position_y"]; 
      auto egoPoint = lanelet::BasicPoint2d(egoPosX, egoPosY);
      auto numLaneCarLane = findTheNumberOfLanesAndCarLane(map, egoPoint);
      ROS_INFO_STREAM("________________________________");
      ROS_INFO_STREAM("Ego car's current pos: "<<numLaneCarLane.first<<" "<<numLaneCarLane.second);
      int differentLane = 0; 
      int elseCount = 0;
      if(numLaneCarLane.first != 0){
        for(auto& sec: projectedSec){
          auto index = findIndex(odomTimeStamp, sec);  
          auto futureEgoJsonData = odomPos[index];
          double futureEgoPosX = futureEgoJsonData["position_x"];double futureEgoPosY = futureEgoJsonData["position_y"]; 
          auto futureEgoPoint = lanelet::BasicPoint2d(futureEgoPosX, futureEgoPosY);
          auto futureNumLaneCarLane = findTheNumberOfLanesAndCarLane(map, futureEgoPoint);
          ROS_INFO_STREAM("future ego car's pos at "<<sec<<" :"<<futureNumLaneCarLane.first<<" "<<futureNumLaneCarLane.second);
          //If there is a mismatch in the line pos, need tp do further check:
          //1. projected ego car lane is different to the ego's current lane in multiple projected sec.
          //2. the car was towards the lanelet edge in multiple projected sec
          if(futureNumLaneCarLane.first != 0){
            if(numLaneCarLane.first == futureNumLaneCarLane.first && numLaneCarLane.second != futureNumLaneCarLane.second){
              differentLane++;
            }else{
              //In some cases number of lanes would be different in current and
              //projected sec which can contribute to the false detection of
              //lane change. this may be due to the map is constructed wrong.
              //We can check this by looking at how many times the number is
              //going different.
              if(numLaneCarLane.second != futureNumLaneCarLane.second)
                elseCount++;
            }
          }
        }
      }

      if(elseCount > 2)
        differentLane++;
      
      if(prevLaneChangeCount > 2 && differentLane > 2){
        ROS_INFO_STREAM("LANE CHANGE!!!");
      }
      ROS_INFO_STREAM(prevLaneChangeCount<<" "<<differentLane<<" "<<elseCount);
      
      //Previous lane change
      prevLaneChangeCount = differentLane;

    }

    void cutInScenario(uint32_t currentSec, std::vector<uint32_t> projectedSec)
    {
     
      for(auto& sec: projectedSec){
        //1.Find all the cars at this sec
        auto carsVec = getAllTheCarsAtSec(groupBySec, sec);
        //2.Loop through each car
        for(auto& jData: carsVec){
          auto car = jData["object_id"];
          auto dataAtCar = getAllTheDataAtCar(groupByCar, car);
          //  a. Loop through ego_s trajectory till 8 second
           
          //  b. Check euclidean distance with ego and current car
          //  c. Check acceleartion change and increment the acceleration change
          //  count
          //  d. identify the cut-in scenario happended or not based on above
          //  information.
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
    extract.groupThem();
    extract.ReadFromBag();
    extract.storeData();
  }
  extract.storeDataInitially = false;
  extract.ReadFromBag();
  
  return 0;
}

