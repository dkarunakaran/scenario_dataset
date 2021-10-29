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
    std::string centerline_json_file;
    bool resume;
    bool storeDataInitially;
    std::string lanelet_file; lanelet::LaneletMapPtr map;
    std::vector<int> objectClassVec;
    double frenetS;
    std::vector<std::pair<int, std::vector<json>>> frenetJson;
    lanelet::LineString3d roadCenterLine;
    std::vector<json> frenetJsonEgo;

    Extraction() : h264_bag_playback() {
      previous_percentage = -1;
      current_message_number = 0;
      private_nh.getParam("objects", objects_topic);
      private_nh.getParam("odometry", odometry_topic);
      private_nh.getParam("bag_file", bag_file);
      private_nh.getParam("resume", resume);
      private_nh.getParam("centerline_json_file", centerline_json_file);
      private_nh.getParam("lanelet_file", lanelet_file);
      lanelet::projection::UtmProjector projector(lanelet::Origin({0, 0}));  
      map = lanelet::load(lanelet_file, projector);
      objectClassVec.push_back(5); //car
      objectClassVec.push_back(6); //truck
      objectClassVec.push_back(15); //motorbike
      frenetS = 0;
    }

    void loadData(){
      std::ifstream i1(centerline_json_file);
      json j1;i1 >> j1; 
      for (auto& item : j1["road_center"]) {
        lanelet::Point3d p1{lanelet::utils::getId(), item["x"], item["y"], 0};
        roadCenterLine.push_back(p1);
      }
    }
    
    void storeData(){
      //Saving the centerline of the road to a json file
      std::vector<json> centerline;
      for(size_t i =0; i<roadCenterLine.size(); i++){
        auto point = roadCenterLine[i];
        json jData;jData["x"] = point.x();jData["y"] = point.y();
        centerline.push_back(jData);    
      }
      json j1(centerline);
      json centerLineJson = {
        {"road_center", j1}
      }; 
      std::ofstream o1(centerline_json_file);
      o1 << std::setw(4) << centerLineJson << std::endl;
    }
         
    void MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message) {
      if(storeDataInitially){
        if(message.getTopic() == odometry_topic){
          nav_msgs::Odometry::ConstPtr odomPtr = message.instantiate<nav_msgs::Odometry>();
          if(odomPtr != nullptr){
            buildCenterLine(odomPtr); 
            ROS_INFO_STREAM("Storing... Time in sec: "<<odomPtr->header.stamp.sec);
          }
        }//odometry topic if close
        if(message.getTopic() == objects_topic){
          ibeo_object_msg::IbeoObject::ConstPtr objPtr = message.instantiate<ibeo_object_msg::IbeoObject>();
          if(objPtr != nullptr){
            if(std::find(objectClassVec.begin(), objectClassVec.end(), objPtr->object_class) != objectClassVec.end()){
              //json jData = constructJsonData(objPtr, transformer_);
              //objectsPos.push_back(jData);  
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
        //Publish all of the bag topics
        publisher.publish(message);
 
        laneChangeDetection(message);
      }
    }//MessagePublisher method closes

    void buildCenterLine(nav_msgs::Odometry::ConstPtr odomPtr){
      auto egoPoint2d = lanelet::BasicPoint2d(odomPtr->pose.pose.position.x, odomPtr->pose.pose.position.y);
      auto tuple = findTheActualLanelet(map, egoPoint2d);
      lanelet::BasicPoint2d startingPoint;
      if(std::get<0>(tuple)){
        startingPoint = lanelet::geometry::project(lanelet::utils::to2D(std::get<1>(tuple).centerline()), egoPoint2d); 
      }else{
        //Just in case map is slightly wrong, we cannot find the lanelet
        //which the egopoint resides
        auto startingPointPair = findTheClosestLaneletForObject(map, egoPoint2d);
        startingPoint = startingPointPair.second;
      }
      auto centerLineTuple = findTheCentralLinePoint(map, startingPoint);
      if(std::get<0>(centerLineTuple)){
        auto roadCenter = std::get<1>(centerLineTuple).first; 
        lanelet::Point3d p1{lanelet::utils::getId(), roadCenter.x(), roadCenter.y(), 0};
        roadCenterLine.push_back(p1);
      }
      ROS_INFO_STREAM("Road center linestring size: "<<roadCenterLine.size());
    }
   
    
    void laneChangeDetection(const rosbag::MessageInstance &msg){
      if(msg.getTopic() == odometry_topic){
        nav_msgs::Odometry::ConstPtr odomPtr = msg.instantiate<nav_msgs::Odometry>();
        constructFrenetFrameEgo(odomPtr);
      }//Odometry topic if closes
    }

    void savePlotData(){
      ROS_INFO_STREAM("Frenet frame plot data size: "<<frenetJson.size()); 
      std::vector<json> storeJson; 
      for(auto& jData: frenetJsonEgo){
        json mainData = jData;
        storeJson.push_back(mainData);
      } 
      json dataJ2(storeJson);
      std::ofstream o2("/model/ego_frent.json");
      o2 << std::setw(4) << dataJ2 << std::endl;
    }
    
    void constructFrenetFrameEgo(nav_msgs::Odometry::ConstPtr odomPtr){
      json jData; json odomJdata;
      double egoPosX = odomPtr->pose.pose.position.x;double egoPosY = odomPtr->pose.pose.position.y; 
      auto egoPoint = lanelet::BasicPoint2d(egoPosX, egoPosY);
      auto egoPoint3d = lanelet::Point3d{lanelet::utils::getId(), egoPosX, egoPosY, 0};
      auto roadCenter = lanelet::geometry::project(roadCenterLine, egoPoint3d);
      auto roadCenterls = getTheRoadLineString(roadCenterLine, lanelet::Point3d{lanelet::utils::getId(), roadCenter.x(), roadCenter.y(), 0});
      if(roadCenterls.size() > 0){
        frenetS = getFrenetSEgo(frenetJsonEgo, lanelet::BasicPoint2d(roadCenter.x(), roadCenter.y()));
        float d = lanelet::geometry::signedDistance(lanelet::utils::to2D(roadCenterls), egoPoint); 
        jData["s"] = frenetS;
        jData["d"] = d;
        jData["sec"] = odomPtr->header.stamp.sec;
        odomJdata["x"] = egoPosX;
        odomJdata["y"] = egoPosY;
        odomJdata["sec"] = odomPtr->header.stamp.sec;
        frenetJsonVecEgo(frenetJsonEgo,std::make_pair(jData,odomJdata),lanelet::BasicPoint2d(roadCenter.x(), roadCenter.y()), frenetS);
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
  extract.savePlotData();
  
  return 0;
}

