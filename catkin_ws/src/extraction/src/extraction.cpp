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
    std::vector<int> carIds;
    double frenetS;
    std::vector<std::pair<int, std::vector<json>>> frenetJson;
    std::vector<double> prevPosS;
    
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
      frenetS = 0;
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
        if(std::find(carIds.begin(), carIds.end(), item["object_id"].get<int>()) != carIds.end()){
        }else{
          if(item["pos_baselink_x"] >= -10. && item["pos_baselink_x"] <= 100.)
            carIds.push_back(item["object_id"].get<int>());
        }
      }
      ROS_INFO_STREAM("Odometry size: "<<odomPos.size()<<" Objects timestamp: "<<objectsPos.size());
      ROS_INFO_STREAM("Car ids:");
      for(auto& car: carIds)
        std::cout<<car<<" ";
      std::cout<<"\n";
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
            if(std::find(objectClassVec.begin(), objectClassVec.end(), objPtr->object_class) != objectClassVec.end()){
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
        //Publish all of the bag topics
        publisher.publish(message);
 
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
          //cutInScenario(sec, projectedSec);
          
          constructFrenetFrame(sec);
          
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
      auto egoDataTillProj = getEgoDataTillProj(projectedSec, odomPos, odomTimeStamp);
     for(auto& sec: projectedSec){
        //1.Find all the cars at this sec
        auto carsVec = getAllTheCarsAtSec(groupBySec, sec);
        //2.Loop through each car
        for(auto& jData: carsVec){
          auto car = jData["object_id"];
          auto dataAtCar = getAllTheDataAtCar(groupByCar, car);
          auto checkData = dataAtCar[0];
          if(checkData["pos_baselink_x"] >= -10. && checkData["pos_baselink_x"] <= 100.){
            auto oDataPair = getCarDataAtSec(dataAtCar, sec);  
            auto oData = oDataPair.second;
            if(oDataPair.first){
              double x2 = oData["position_x"].get<double>();double y2 = oData["position_y"].get<double>();
              //Checking accleration is greater than the threshold and occured more than once
              double linearY = oData["linear_y"].get<double>();
              //b. Loop through ego_s trajectory till 8 second
              for(auto& eData: egoDataTillProj){
                double x1 = eData["position_x"]; double y1 = eData["position_y"];
                //  c. Check euclidean distance with ego and current car 
                float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
                if(d < 0.50){
                  ROS_INFO_STREAM("Cut-in scenario, car: "<<car);
                }
              }
            }//oDataPair.first if closes
          }
        }//Projected sec loop
      }
    }
    
    void savePlotData(){
      ROS_INFO_STREAM("Frenet frame plot data size: "<<frenetJson.size()); 
      std::vector<json> storeJson; 
      for(auto& dataPair: frenetJson){
        if(dataPair.second.size() > 0){
          json mainData = dataPair.second;
          storeJson.push_back(mainData);
        }
      } 
      json dataJ(storeJson);
      std::ofstream o1("/model/cars_frent.json");
      o1 << std::setw(4) << dataJ << std::endl;
    }

    
    void constructFrenetFrame(uint32_t sec){
      auto carsVec = getAllTheCarsAtSec(groupBySec, sec);
      for(auto& carData: carsVec){
        auto car = carData["object_id"];
        if(!checkOjectSecAdded(frenetJson, car, sec)){
          ROS_INFO_STREAM(car);
          auto dataAtCar = getAllTheDataAtCar(groupByCar, car);
          json jData; json odomJdata;
          auto carJson =  getCarDataAtSec(dataAtCar, sec); 
          auto point2d = lanelet::BasicPoint2d(carJson.second["position_x"].get<double>(), carJson.second["position_y"].get<double>());
          auto startingPointPair = findTheClosestLaneletForObject(map, point2d);
          if(startingPointPair.first){
            auto centerLineTuple = findTheCentralLinePoint(map, startingPointPair.second);
            if(std::get<0>(centerLineTuple)){
              auto roadCenter = std::get<1>(centerLineTuple).first;  
              frenetS = getFrenetS(frenetJson, car, roadCenter);
              auto roadCenterls = std::get<1>(centerLineTuple).second; 
              float d = lanelet::geometry::signedDistance(lanelet::utils::to2D(roadCenterls), point2d);
              //ROS_INFO_STREAM("sec:"<<sec<<" s&d:"<<frenetS<<" "<< d<<" car:"<<car<<" lines: "<< std::get<2>(centerLineTuple).second);  
              jData["s"] = frenetS;
              jData["d"] = d;
              jData["sec"] = sec;
              odomJdata["x"] = carJson.second["position_x"];
              odomJdata["y"] = carJson.second["position_y"];
              odomJdata["sec"] = sec;
              frenetJsonVec(frenetJson,std::make_tuple(car,jData,odomJdata),roadCenter, frenetS);
            }
          }
        }
      } //carVec for loop closes  
         
      /* 
        auto index = findIndex(odomTimeStamp, sec);
        auto futureEgoJsonData = odomPos[index];
        auto futureEgoPoint = lanelet::BasicPoint2d(futureEgoJsonData["position_x"], futureEgoJsonData["position_y"]);
        auto centerLinePoint = findTheCentralLinePoint(map, futureEgoPoint);
        if(centerLinePoint.first){
          auto roadCenter = centerLinePoint.second;  
          if(prevPosS.size() == 0){
            s = 0;
          }else{
            s += std::sqrt(std::pow((prevPosS[0]-roadCenter.x()),2)+std::pow((prevPosS[1]-roadCenter.y()),2));
          }
          
          //We need to find another way of finding d as the below code
          //gives only positive value which doesn't indicate which side of
          //the centerline the car is
          float d = std::sqrt(std::pow((futureEgoPoint.x()-roadCenter.x()),2)+std::pow((futureEgoPoint.y()-roadCenter.y()),2));

          ROS_INFO_STREAM("sec:"<<sec<<" s&d:"<<s<<" "<< d<<" "<<roadCenter.x()<<" "<<roadCenter.y());
          prevPosS.clear();
          prevPosS.push_back(roadCenter.x());
          prevPosS.push_back(roadCenter.y());
        }
      */
      
      /*
      std::vector<json> storeJson; 
      for(auto& car :carIds){
        json mainJData;
        std::vector<json> frenetDataVec; 
        std::vector<json> odomDataVec; 
        ROS_INFO_STREAM("--------------- car: "<<car<<"----------------");
        auto dataAtCar = getAllTheDataAtCar(groupByCar, car);
        std::vector<uint32_t> secs;
        for(auto& jData: dataAtCar){
          secs.push_back(jData["sec"].get<uint32_t>());
        }
        
        double s = 0;
        std::vector<double> prevPosS;
        //Go through the secs
        for(auto& sec: secs){
          json jData; json odomJdata;
          auto carJson =  getCarDataAtSec(dataAtCar, sec); 
          auto point2d = lanelet::BasicPoint2d(carJson.second["position_x"].get<double>(), carJson.second["position_y"].get<double>());
          auto startingPointPair = findTheClosestLaneletForObject(map, point2d);
          if(startingPointPair.first){
            auto centerLineTuple = findTheCentralLinePoint(map, startingPointPair.second);
            if(std::get<0>(centerLineTuple)){
              auto roadCenter = std::get<1>(centerLineTuple).first;  
              if(prevPosS.size() == 0){
                s = 0;
              }else{
                s += std::sqrt(std::pow((prevPosS[0]-roadCenter.x()),2)+std::pow((prevPosS[1]-roadCenter.y()),2));
              }
              auto roadCenterls = std::get<1>(centerLineTuple).second; 
              float d=lanelet::geometry::signedDistance(lanelet::utils::to2D(roadCenterls), point2d);
              ROS_INFO_STREAM("sec:"<<sec<<" s&d:"<<s<<" "<< d<<" "<<roadCenter.x()<<" "<<roadCenter.y()<<" lines: "<< std::get<2>(centerLineTuple).second);  
              jData["s"] = s;
              jData["d"] = d;
              jData["sec"] = sec;
              odomJdata["x"] = carJson.second["position_x"];
              odomJdata["y"] = carJson.second["position_y"];
              odomJdata["sec"] = sec;
              frenetDataVec.push_back(jData);
              odomDataVec.push_back(odomJdata);
              prevPosS.clear();
              prevPosS.push_back(roadCenter.x());
              prevPosS.push_back(roadCenter.y());
            }
          }
        }
        if(frenetDataVec.size() > 0){
          json dataJson(frenetDataVec);
          json odomDataJson(odomDataVec);
          mainJData = {
            {"car_id", car},
            {"frenet_data", dataJson},
            {"odom_pos", odomDataJson}
          };
          storeJson.push_back(mainJData);
        }
      }
      json dataJ(storeJson);
      std::ofstream o1("/model/cars_frent.json");
      o1 << std::setw(4) << dataJ << std::endl;
      */
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
    extract.groupThem();
  }
  extract.storeDataInitially = false;
  extract.ReadFromBag();
  extract.savePlotData();
  
  return 0;
}

