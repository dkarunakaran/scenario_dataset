#include "top_down_projection.hpp"

#include <pcl/search/impl/search.hpp>
#include <pcl/filters/crop_box.h>

#include <iostream>
#include <vector>
#include <sstream>

#include <deque>

#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>

#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <signal.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include "Conversions.h"
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "TopDownProjection");
  ros::NodeHandle n;

  FeatureExtractor pole_detect;
  pole_detect.bypass_init();

  return 0;
}


std::vector <std::string>
FeatureExtractor::separateCommas(std::string input_string) {
  std::vector <std::string> result;
  std::stringstream s_stream(input_string); //create string stream from the string
  while (s_stream.good()) {
    std::string substr;
    std::getline(s_stream, substr, ','); //get first string delimited by comma
    result.push_back(substr);
  }

  return result;
}


void
FeatureExtractor::onInit() {


  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param<int>("cm_resolution", cm_resolution, 2);


  private_nh.param<std::string>("projection_frame", projection_frame, "odom");

  std::vector<float> filter_ring_numbers;
  private_nh.getParam("use_rings", filter_ring_numbers);

  for (auto &ring: filter_ring_numbers) {
    ROS_INFO_STREAM("Incorporating lidar ring in image:" << ring);
    ring_filter.insert(ring);
  }

  std::vector <std::string> topics;

  std::vector <std::string> value_dict;
  private_nh.getParam("point_clouds", value_dict);

  for (auto &iter: value_dict) {
    std::vector <std::string> values = separateCommas(iter);

    if (values.size() != 4)
      ROS_INFO_STREAM("incomplete type to display topic " << values[0]);
    else {
      if (values[1] == "circle") {

        std::shared_ptr <CircleItem> new_item = std::make_shared<CircleItem>(std::stoi(values[2]),
                                                                             std::stoi(values[3]));
        new_item->topic_name = values[0];
        new_item->alpha = std::stoi(values[2]);
        new_item->shape_size = std::stoi(values[3]);
        item_draw_properties.push_back(new_item);
/*
        rosbag::View view(bag, rosbag::TopicQuery(new_item->topic_name));
        sensor_msgs::PointCloud2::ConstPtr s = view.begin()->instantiate<sensor_msgs::PointCloud2>();
        if (s != NULL && s->fields.size() > 5 &&  std::string(s->fields[5].name) == std::string("label")) {
          new_item->field_name = "label";
        }
*/
        ROS_INFO_STREAM("Drawing topic " << item_draw_properties.back()->topic_name
                                         << " as a circle with alpha/size " << item_draw_properties.back()->alpha
                                         << ", "
                                         << item_draw_properties.back()->shape_size << ", using field "
                                         << new_item->field_name);
      } else if (values[1] == "point") {

        std::shared_ptr <PointItem> new_item = std::make_shared<PointItem>(std::stoi(values[2]),
                                                                           std::stoi(values[3]));
        new_item->topic_name = values[0];
        new_item->alpha = std::stoi(values[2]);
        new_item->shape_size = std::stoi(values[3]);
        item_draw_properties.push_back(new_item);
/*
        rosbag::View view(bag, rosbag::TopicQuery(new_item->topic_name));
        sensor_msgs::PointCloud2::ConstPtr s = view.begin()->instantiate<sensor_msgs::PointCloud2>();
        auto tmp =  s->fields.size();
        if (s != NULL && s->fields.size() > 5 &&  std::string(s->fields[5].name) == std::string("label")) {
          new_item->field_name =  "label";
        }
*/
        ROS_INFO_STREAM("Drawing topic " << item_draw_properties.back()->topic_name
                                         << " as a point with alpha/size " << item_draw_properties.back()->alpha
                                         << ", "
                                         << item_draw_properties.back()->shape_size << ", using field "
                                         << new_item->field_name);
      } else if (values[1] == "square") {

      } else {
        ROS_INFO_STREAM("Drawing topic " << values[0] << " has no type defined");
        continue;
      }
      topics.push_back(values[0]);

    }
  }

  for (auto &topic: topics) {
    ROS_INFO_STREAM(topic << " is being used");
    intensity_map[topic] = std::map < std::pair < int, int >, double > ();
  }

  this->horizonInBuffer = true;


  this->init_playback();
  start_time = std::chrono::steady_clock::now();
  lanePcPub = n.advertise<lane_points_msg::LanePoints> ("lane_points", 1);
  signalShutdown = n.advertise<std_msgs::Bool> ("signal_shutdown", 1);
  this->ReadFromBag();
  this->WriteImage();

  //Just dealying the publishing the shutdown signal to get all the data
  int sec = 5;
  ROS_INFO_STREAM("Waiting for "<<sec<<" seconds before sending the shutdown signal");
  ros::Duration(sec).sleep();
  std_msgs::Bool msg;
  msg.data = true;
  signalShutdown.publish(msg);

}

bool FeatureExtractor::checkRegionOfInterest(std::pair<int,int> item, int min_x, int min_y){

  bool _return = false;

  // Draw a circle for each of the odom positions
  for (auto &odom: vehicle_odom) {
    int x1 = odom.first - min_x;
    int y1 = odom.second - min_y;

    int x2 = item.first;
    int y2 = item.second;
    float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
    d = (cm_resolution*d)/100;

    if(d<=3){
      _return = true;
      break;
    }
  }

  return _return;
}

void FeatureExtractor::publishLanePoints(int min_x, int min_y, int max_x, int max_y, int image_max_x, int image_max_y, float min_intensity, float max_intensity){

  ROS_INFO_STREAM("all point size:"<<allPoints.size());
  std::map<long int, std::list<std::map<std::pair<int, int>, double>>> allPointsPerSec;
  std::list<std::map<std::pair<int, int>, double>> pointsPerNsec;
  long int last = 0;

  //std::map<std::pair<int, int>, double> storeLanePointsNsec;
  //std::vector<std::map<std::pair<int, int>, double>> storeLanePointsSec;
  for (auto &point: allPoints) {
        if(allPointsPerSec.count(point.first.first)){
                pointsPerNsec.push_back(point.second);
        }else{
                if(last == 0){
                        last = point.first.first;
                }else{
                        allPointsPerSec[last] = pointsPerNsec;
                        pointsPerNsec.clear();
                        pointsPerNsec.push_back(point.second);
                        last = point.first.first;
                }
        }
  }


  if(!allPointsPerSec.count(last)){
        allPointsPerSec[last] = pointsPerNsec;
        pointsPerNsec.clear();
  }

  int count = 0;
  for(auto &secPoints: allPointsPerSec){
   // secPoints.first: long int - seconds
   // secPoints.second: list -  list of each nsec points
   // secPoints.second[].first - it's a x,y pair of points
   // secPoints.second[].second - intensity of each x, y point
   //List level

   lane_points_msg::LanePoints lanePointsMsg;
   lanePointsMsg.header.seq = count;
   lanePointsMsg.header.stamp.sec = secPoints.first;
   lanePointsMsg.header.stamp.nsec = 0;
   lanePointsMsg.header.frame_id = "lane_points";
   lanePointsMsg.max_x = image_max_x;
   lanePointsMsg.max_y = image_max_y;
   count += 1;
   std_msgs::Int32MultiArray x_array;
   x_array.data.clear();
   std_msgs::Int32MultiArray y_array;
   y_array.data.clear();
   std_msgs::Float32MultiArray i_array;
   i_array.data.clear();
   for(auto &nsecPoints: secPoints.second){

    //Map level - nsecs
    for(auto &points: nsecPoints){

        //Intensity filtering code
        int value_x = points.first.first - min_x;
        int value_y = points.first.second - min_y;
        if (value_x < 0 || value_x >= image_max_x) {
                ROS_ERROR_STREAM("Pixel found out of bounds: " << intensity_map.size());
                continue;
        }

        if (value_y < 0 || value_y >= image_max_y) {
                ROS_ERROR_STREAM("Pixel found out of bounds: " << intensity_map.size());
                continue;
        }

        cv::Vec4b colour;
        // scale the intensity value to be between 0 and 1
        double intensity = (points.second - min_intensity) / (max_intensity - min_intensity);

        if (intensity > 1.)
          intensity = 1.;

        if (intensity < 0.)
          intensity = 0.;
	// set the hue to the intensity value (between 0 and 255) to make a rainbow colour scale
        hsv input_hsv;
        input_hsv.h = intensity * 255.;
        input_hsv.s = 1.;
        input_hsv.v = 1.;

        // convert HSV to RGB
        rgb output_rgb = hsv2rgb(input_hsv);

        colour[0] = (uint8_t)(output_rgb.b * 255.);
        colour[1] = (uint8_t)(output_rgb.g * 255.);
        colour[2] = (uint8_t)(output_rgb.r * 255.);
	
	// Only considering the pixels that are in the below colour range. It enables the detection of lanes in the intensity image clearly.
        if(colour[1] < 200 & colour[2] < 200 & colour[0] > 250){
          bool status = this->checkRegionOfInterest(std::make_pair(value_x, value_y), min_x, min_y);
          if(status){
            // Converting to white pixels to apply some 2d lane detction algorithms
            colour[0] = 255.;
            colour[1] = 255.;
            colour[2] = 255.;
            x_array.data.push_back(value_x);
            y_array.data.push_back(value_y);
            i_array.data.push_back(1.0);

          }else{
            continue;
          }

        }else{
          // Do not consider other pixels that are not in the above colour range
          continue;
        }
        //Store the values of nsec points and intensity
    }
    //Add each nsec points to one sec container
    //storeLanePointsSec.push_back(storeLanePointsNsec);
    //storeLanePointsNsec.clear();
   }
   //Publish the sec container
   //Msg: header|size: max_x, max_y|data: x: [], y: [], i: []
   lanePointsMsg.x = x_array;
   lanePointsMsg.y = y_array;
   lanePointsMsg.i = i_array;
   lanePcPub.publish(lanePointsMsg);
  }
}



void FeatureExtractor::WriteImage() {

  ROS_INFO_STREAM("Number of plotted topics: " << intensity_map.size());

  // set the search values to extreme (to be overwritten by the correct values)
  int max_x = -100000000, max_y = -100000000, min_x = 100000000, min_y = 100000000;
  float min_intensity = 100000000.;
  float max_intensity = -100000000.;

  for (auto &topic_intensity: intensity_map) {
    // determine the min/max values for x/y pixels and intensity
    for (auto &entry: topic_intensity.second) {
      min_x = std::min<int>(min_x, entry.first.first);
      max_x = std::max<int>(max_x, entry.first.first);

      min_y = std::min<int>(min_y, entry.first.second);
      max_y = std::max<int>(max_y, entry.first.second);

      min_intensity = std::min<double>(min_intensity, entry.second);
      max_intensity = std::max<double>(max_intensity, entry.second);
    }
  }

  std::cout << min_x << ", " << min_y << ", " << max_x << ", " << max_y << std::endl;

  // restrict the intensity range as determined by the param settings
  float intensity_scale_min = private_nh.param<float>("min_intensity", 0.);
  float intensity_scale_max = private_nh.param<float>("max_intensity", 80.);

  min_intensity = std::max<float>(min_intensity, intensity_scale_min);
  max_intensity = std::min<float>(max_intensity, intensity_scale_max);

  // Add a small buffer for the image to account for rounding errors
  int image_max_x = max_x - min_x + 2;
  int image_max_y = max_y - min_y + 2;
  
  //My coding started
  publishLanePoints(min_x, min_y, max_x, max_y, image_max_x, image_max_y, min_intensity, max_intensity);
  //My coding finished


  // the lidar datapoints projected into 2d
  cv::Mat output_image(image_max_x, image_max_y, CV_8UC4, cv::Scalar(0, 0, 0, 0));

  for (auto &topic_to_draw: item_draw_properties) {
    auto topic_intensity = intensity_map.find(topic_to_draw->topic_name);
    if (topic_intensity == intensity_map.end()) {
      ROS_INFO_STREAM(topic_to_draw->topic_name << " is not found to draw");
      break;
    }

    ROS_INFO_STREAM(topic_intensity->first << " has " << topic_intensity->second.size() << " values");
    for (auto &entry: topic_intensity->second) {

      int value_x = entry.first.first - min_x;
      int value_y = entry.first.second - min_y;

      if (value_x < 0 || value_x >= image_max_x) {
        ROS_ERROR_STREAM("Pixel found out of bounds: " << intensity_map.size());
        continue;
      }

      if (value_y < 0 || value_y >= image_max_y) {
        ROS_ERROR_STREAM("Pixel found out of bounds: " << intensity_map.size());
        continue;
      }

      cv::Vec4b colour;

      // if sematic label is being plotted, use bonnet color scheme
      if (topic_to_draw->field_name == "label") {

        int label = entry.second;
        colour[3] = (uint8_t)(255);

        if (label == 1) {//building
          colour[0] = (uint8_t)(255);
          colour[1] = (uint8_t)(255);
          colour[2] = (uint8_t)(255);
        } else if (label == 2) {//pole
          colour[2] = (uint8_t)(158);
          colour[1] = (uint8_t)(255);
          colour[0] = (uint8_t)(255);
        } else if (label == 3) {//road
          colour[2] = (uint8_t)(139);
          colour[1] = (uint8_t)(69);
          colour[0] = (uint8_t)(19);
        } else if (label == 4) {//undrivable_road
          colour[2] = (uint8_t)(202);
          colour[1] = (uint8_t)(255);
          colour[0] = (uint8_t)(111);
        } else if (label == 5) {//vegetation
          colour[2] = (uint8_t)(0);
          colour[1] = (uint8_t)(255);
          colour[0] = (uint8_t)(0);
        } else if (label == 6) {//sign
          colour[2] = (uint8_t)(158);
          colour[1] = (uint8_t)(255);
          colour[0] = (uint8_t)(255);
        } else if (label == 7) {//fence
          colour[2] = (uint8_t)(160);
          colour[1] = (uint8_t)(160);
          colour[0] = (uint8_t)(160);
        } else if (label == 8) {//vehicle
          colour[2] = (uint8_t)(255);
          colour[1] = (uint8_t)(0);
          colour[0] = (uint8_t)(0);
        } else {
          colour[3] = (uint8_t)(0);
        }

        // else use rainbow color scheme for plotting intensity
      } else {

        // scale the intensity value to be between 0 and 1
        double intensity = (entry.second - min_intensity) / (max_intensity - min_intensity);

        if (intensity > 1.)
          intensity = 1.;

        if (intensity < 0.)
          intensity = 0.;

        // set the hue to the intensity value (between 0 and 255) to make a rainbow colour scale
        hsv input_hsv;
        input_hsv.h = intensity * 255.;
        input_hsv.s = 1.;
        input_hsv.v = 1.;

        // convert HSV to RGB
        rgb output_rgb = hsv2rgb(input_hsv);

        colour[0] = (uint8_t)(output_rgb.b * 255.);
        colour[1] = (uint8_t)(output_rgb.g * 255.);
        colour[2] = (uint8_t)(output_rgb.r * 255.);
        
	if(colour[1] < 200 & colour[2] < 200 & colour[0] > 250) 
        { 
          bool status = this->checkRegionOfInterest(std::make_pair(value_x, value_y), min_x, min_y);
          if(status){
		colour[0] = 255.;
		colour[1] = 255.;
		colour[2] = 255.;
	  }else{
		continue;
	  }	  
	}else{
		continue;
        }
      }

      cv::Point destination_point(value_y, value_x);
      topic_to_draw->drawItem(output_image, destination_point, colour);
    }
  }

  // Set the colour for the odom plot
  cv::Scalar odom_colour(255., 50., 0., 180);
  float odom_radius = 5.;

  // Draw a circle for each of the odom positions
  for (auto &odom: vehicle_odom) {
    int value_x = odom.first - min_x;
    int value_y = odom.second - min_y;
    cv::circle(output_image, cv::Point(value_y, value_x), odom_radius, odom_colour, CV_FILLED);
  }

  for (auto &ring_number: rings_included) {
    ROS_INFO_STREAM("Ring " << ring_number << " included in image");
  }

  ROS_INFO_STREAM(
      "Drawing image with bounds: [" << min_x / 100. * cm_resolution << ", " << min_y / 100. * cm_resolution
                                     << "],  [" <<
                                     max_x / 100. * cm_resolution << ", " << max_y / 100. * cm_resolution
                                     << "] with intensity range [" <<
                                     min_intensity << ", " << max_intensity << "]");

  // output the white background image
  std::string output_image_name = private_nh.param<std::string>("output_image", "");


  if (output_image_name != "")
    cv::imwrite(output_image_name, output_image);



  double x_datum = 0, y_datum = 0;

  try {

    // transform from the world reference to the base link
    auto datum_transform = transformer_->lookupTransform(std::string("utm"),
                                                         projection_frame,
                                                         ros::Time(0));

    Eigen::Vector3f datum_origin(datum_transform.transform.translation.x,
                                 datum_transform.transform.translation.y,
                                 datum_transform.transform.translation.z);

    ROS_INFO_STREAM("DATUM TRANSFORM (to utm): " << datum_origin[0] << ", " << datum_origin[1]);

    //double x_datum = 332722.272927207, y_datum = 6248431.02677212;
    x_datum = datum_origin[0];
    y_datum = datum_origin[1];

    std::cout << "gdal_translate -of GTiff -co \"COMPRESS=JPEG\" -a_srs EPSG:32756 -a_ullr " <<
              std::setprecision(15) << x_datum + cm_resolution * double(min_y) / 100. << " " <<
              std::setprecision(15) << y_datum + -cm_resolution * double(min_x) / 100. << " " <<
              std::setprecision(15) << x_datum + cm_resolution * double(max_y) / 100. << " " <<
              std::setprecision(15) << y_datum + -cm_resolution * double(max_x) / 100. << " " <<
              output_image_name <<
              " output-georeferenced.tif" << std::endl;

  } catch (const std::exception &e) { // reference to the base of a polymorphic object
    ROS_ERROR_STREAM(e.what()); // information from length_error printed
    ROS_INFO_STREAM("No datum (transform to utm) is available");
  }

}


void FeatureExtractor::MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message) {

  current_message_number++;

  int current_percentage = 100.0 * ((float) current_message_number / (float) total_message_count);

  if (current_percentage != previous_percentage) {
    previous_percentage = current_percentage;

    if (current_percentage == 0) {
      std::cout << "starting projection" << std::endl;
    } else {
      end_time = std::chrono::steady_clock::now();
      std::chrono::duration<float, std::ratio<60>> elapsed = end_time - start_time;
      float estimated_remaining = (float) (100 - current_percentage) * elapsed.count();
      std::cout << current_percentage << "%, est. remaining " << estimated_remaining << " minutes" << std::endl;
    }
    start_time = std::chrono::steady_clock::now();
  }

  sensor_msgs::PointCloud2::ConstPtr s = message.instantiate<sensor_msgs::PointCloud2>();
  if (s != NULL) {
    auto map_reference = intensity_map.find(message.getTopic());
    if (map_reference != intensity_map.end()) {

      if (s->fields.size() > 5 && std::string(s->fields[5].name) == std::string("label")) {
        SegmentPointCloud_label(s, map_reference->second);
      } else
        SegmentPointCloud_intensity(s, map_reference->second);
    }
  }
}


pcl::PointCloud<pcl::PointXYZIR>::Ptr
FeatureExtractor::Selector(pcl::PointCloud<pcl::PointXYZIR>::Ptr input_cloud,
                           float maximum_range,
                           std::set<int> ring_filter,
                           std::set<int> &rings_included) {

  float max_range_squared = pow(maximum_range, 2);

  pcl::PointCloud<pcl::PointXYZIR>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZIR>);

  for (size_t i = 0; i < input_cloud->points.size(); ++i) {
    float range_squared =
        pow(input_cloud->points[i].x, 2) + pow(input_cloud->points[i].y, 2) + pow(input_cloud->points[i].z, 2);

    if (range_squared > max_range_squared)
      continue;


    int ring_number = (int) (input_cloud->points[i].ring);

    if (ring_filter.size() > 0) {
      if (ring_filter.find(ring_number) == ring_filter.end()) {
        ROS_INFO_STREAM_THROTTLE(1., "ignoring data from ring " << ring_number);
        continue;
      }
    }

    rings_included.insert(ring_number);

    downsampled->points.push_back(input_cloud->points[i]);
  }

  return downsampled;
}


pcl::PointCloud<pcl::PointXYZIRL>::Ptr
FeatureExtractor::Selector_label(pcl::PointCloud<pcl::PointXYZIRL>::Ptr input_cloud,
                                 float maximum_range,
                                 std::set<int> ring_filter,
                                 std::set<int> &rings_included) {

  float max_range_squared = pow(maximum_range, 2);

  pcl::PointCloud<pcl::PointXYZIRL>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZIRL>);

  for (size_t i = 0; i < input_cloud->points.size(); ++i) {
    float range_squared =
        pow(input_cloud->points[i].x, 2) + pow(input_cloud->points[i].y, 2) + pow(input_cloud->points[i].z, 2);

    if (range_squared > max_range_squared)
      continue;


    int ring_number = (int) (input_cloud->points[i].ring);

    if (ring_filter.size() > 0) {
      if (ring_filter.find(ring_number) == ring_filter.end()) {
        continue;
      }
    }

    rings_included.insert(ring_number);

    downsampled->points.push_back(input_cloud->points[i]);
  }

  return downsampled;
}


// call whenever receive a pointcloud - spit out new filtered version
void
FeatureExtractor::SegmentPointCloud_intensity(sensor_msgs::PointCloud2::ConstPtr pointcloud_msg,
                                              std::map<std::pair<int, int>, double> &intensity_topic) {

  try {

    // transform from the world reference to the base link
    auto world_transform = transformer_->lookupTransform(projection_frame,
                                                         std::string("base_link"), pointcloud_msg->header.stamp);

    // transform from the base_link_horizon to the lidar reference frame -
    //  this will correct for the pitch and roll of the platform
    //auto platform_transform = transformer_->lookupTransform(std::string("base_link_horizon"),
    auto platform_transform = transformer_->lookupTransform(std::string("base_link"),
                                                            std::string(pointcloud_msg->header.frame_id),
                                                            pointcloud_msg->header.stamp);

    // Transform pointcloud into base_link_horizon frame
    Eigen::Quaternionf platform_rotation(platform_transform.transform.rotation.w,
                                         platform_transform.transform.rotation.x,
                                         platform_transform.transform.rotation.y,
                                         platform_transform.transform.rotation.z);

    Eigen::Vector3f platform_origin(platform_transform.transform.translation.x,
                                    platform_transform.transform.translation.y,
                                    platform_transform.transform.translation.z);

    Eigen::Quaternionf world_rotation(world_transform.transform.rotation.w,
                                      world_transform.transform.rotation.x,
                                      world_transform.transform.rotation.y,
                                      world_transform.transform.rotation.z);

    Eigen::Vector3f world_origin(world_transform.transform.translation.x,
                                 world_transform.transform.translation.y,
                                 world_transform.transform.translation.z);


    pcl::PointCloud<pcl::PointXYZIR>::Ptr input_pointcloud_unrotated(new pcl::PointCloud<pcl::PointXYZIR>);
    pcl::PointCloud<pcl::PointXYZIR>::Ptr input_pointcloud_box_filter(new pcl::PointCloud<pcl::PointXYZIR>);
    pcl::PointCloud<pcl::PointXYZIR>::Ptr input_pointcloud(new pcl::PointCloud<pcl::PointXYZIR>);


    // the original unrotated point cloud
    pcl::fromROSMsg(*pointcloud_msg, *input_pointcloud_unrotated);

    pcl::PointCloud<pcl::PointXYZIR>::Ptr input_downsampled = Selector(input_pointcloud_unrotated,
                                                                       30.,
                                                                       ring_filter,
                                                                       rings_included);

    float range_ = 50;
    // Bounding box filter
    pcl::CropBox<pcl::PointXYZIR> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(-range_, -range_, -range_, range_));
    boxFilter.setMax(Eigen::Vector4f(range_, range_, range_, range_));
    boxFilter.setInputCloud(input_downsampled);
    boxFilter.filter(*input_pointcloud_box_filter);

    pcl::transformPointCloud(*input_pointcloud_box_filter, *input_pointcloud, platform_origin, platform_rotation);
    pcl::transformPointCloud(*input_pointcloud, *input_pointcloud, world_origin, world_rotation);

    int x_index = (world_origin[0] * 100.) / cm_resolution;
    int y_index = (world_origin[1] * 100.) / cm_resolution;

    vehicle_odom.push_back(std::make_pair(-1. * y_index, x_index));

    // put each point into the intensity map
    for (size_t i = 0; i < input_pointcloud->points.size(); ++i) {
      int x_index = (input_pointcloud->points[i].x * 100.) / cm_resolution;
      int y_index = (input_pointcloud->points[i].y * 100.) / cm_resolution;

      if (fabs(x_index) > 1e7 || fabs(y_index) > 1e7) {
        continue;
      }
      //ROS_INFO_STREAM("point " << x_index << ", " << y_index);

      //ROS_INFO_STREAM_THROTTLE(0.5, "point " << x_index << ", " << y_index);
      intensity_topic[std::make_pair(-1. * y_index,
                                     x_index)] = input_pointcloud->points[i].intensity; // std::map<std::pair<int,int>, double>
    }
	
    allPoints[std::make_pair(pointcloud_msg->header.stamp.sec, pointcloud_msg->header.stamp.nsec)] = intensity_topic;


  } catch (const std::exception &e) { // reference to the base of a polymorphic object
    ROS_ERROR_STREAM(e.what()); // information from length_error printed
  }

}

// call whenever receive a pointcloud - spit out new filtered version
void
FeatureExtractor::SegmentPointCloud_label(sensor_msgs::PointCloud2::ConstPtr pointcloud_msg,
                                          std::map<std::pair<int, int>, double> &intensity_topic) {

  /*
  // convert ROS message to PCL object
  pcl::PointCloud<pcl::PointXYZIRL>::Ptr input_pointcloud_unrotated(new pcl::PointCloud<pcl::PointXYZIRL>);
  pcl::PointCloud<pcl::PointXYZIRL>::Ptr input_pointcloud(new pcl::PointCloud<pcl::PointXYZIRL>);

  pcl::PointCloud<pcl::PointXYZIRL>::Ptr intermediate(new pcl::PointCloud<pcl::PointXYZIRL>);

  // the original unrotated point cloud
  pcl::fromROSMsg(*pointcloud_msg, *input_pointcloud_unrotated);


  pcl::PointCloud<pcl::PointXYZIRL>::Ptr input_downsampled = Selector_label(input_pointcloud_unrotated,
                                                                            30.,
                                                                            ring_filter,
                                                                            rings_included);

  // transform the point cloud to compensate for the vehicle pitch and roll
  tf::quaternionTFToEigen(imu_rotation, eigen_q);
  tf::quaternionTFToEigen(odom_rotation, eigen_odom);

  pcl::transformPointCloud(*input_downsampled, *input_pointcloud, odom_vector, eigen_odom);

  // put each point into the intensity map
  for (size_t i = 0; i < input_pointcloud->points.size(); ++i) {
    int x_index = (input_pointcloud->points[i].x * 100.) / cm_resolution;
    int y_index = (input_pointcloud->points[i].y * 100.) / cm_resolution;
    intensity_topic[std::make_pair(-1. * y_index,
                                   x_index)] = input_pointcloud->points[i].label; // std::map<std::pair<int,int>, double>
  }
   */
}

