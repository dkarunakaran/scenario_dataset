#include <h264_bag_playback/h264_bag_playback.hpp>

#include <dataset_tools/run_pipeline.hpp>
#include <dataset_tools/point_cloud_features_pipeline.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <dataset_msgs/DatasetEvent.h>

#include <sensor_msgs/NavSatFix.h>

#include <custom_point_types/point_xyzir.h>

#include "helper_functions.hpp"
#include <chrono>

class Extraction : public dataset_toolkit::h264_bag_playback {

  public:
    Extraction() : h264_bag_playback() {
      previous_percentage = -1;
      current_message_number = 0;
      private_nh.getParam("objects", objects_topic);
      private_nh.getParam("odometry", odometry_topic);

    }
    void MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message) {
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
      
      if(message.getTopic() == objects_topic){
        ROS_INFO_STREAM("Topic: "<<message.getTopic());
      }else if(message.getTopic() == odometry_topic){
        ROS_INFO_STREAM("Topic: "<<message.getTopic());
      }
      // Do the lane change detection

    }
    
    int previous_percentage;
    
    // variables to estimate the time remaining to perform the projection
    uint32_t current_message_number;

    std::chrono::steady_clock::time_point start_time, end_time;
    std::string odometry_topic; std::string objects_topic;

};


int main(int argc, char **argv) {

  ros::init(argc, argv, "Extraction");

  Extraction extract;
  extract.init_playback();
  extract.ReadFromBag();

  return 0;
}

