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
#include <pcl/common/io.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types_conversion.h>
#include <random>
#include <Eigen/QR>

int main(int argc, char **argv) {

  ros::init(argc, argv, "TopDownProjection");
  ros::NodeHandle n;
  FeatureExtractor pole_detect;
  pole_detect.bypass_init();
  
  
  return 0;
}

void FeatureExtractor::initialize(){
	cloud_all_filtered = pcl::PointCloud<pcl::PointXYZIR>::Ptr(new pcl::PointCloud<pcl::PointXYZIR>);

	cloud_all_xyzir = pcl::PointCloud<pcl::PointXYZIR>::Ptr(new pcl::PointCloud<pcl::PointXYZIR>);
	roadPointsPub = n.advertise<sensor_msgs::PointCloud2>("/road_points",10);
	otherPointsPub = n.advertise<sensor_msgs::PointCloud2>("/other_points",10);
	seq_count = 0;
	previousPos = -10000.0;
	nsecCount = 1;
  laneSCount = 0;
  laneSCount1 = 0;
  laneSCount2 = 0;
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
  sphericalR = n.advertise<std_msgs::Float32MultiArray>("radius_points", 1);
  sphericalT = n.advertise<std_msgs::Float32MultiArray>("theta_points", 1);
  sphericalP = n.advertise<std_msgs::Float32MultiArray>("phi_points", 1); 

 
  this->ReadFromBag();
  this->constructLane();
  this->WriteImage();
  
}

void FeatureExtractor::constructLane(){
  ROS_INFO_STREAM("constructLane function called");
  ROS_INFO_STREAM("All line size before: "<<allLines.size());  
  for(size_t i=0; i<activeLaneSeg.size(); i++){
    auto lineSegVec = activeLaneSeg[i].first;
    std::vector<std::pair<std::tuple<Segment,double,double>,std::tuple<Segment,double,double>>> segs;
    for(auto segVec: lineSegVec){
      segs.push_back(segVec);
    }
    allLines.push_back(segs);
    activeLaneSeg.erase(activeLaneSeg.begin()+i);
  }
  ROS_INFO_STREAM("All line size after: "<<allLines.size());  
  
 /* std::vector<Segment> tempSegments;
  if(lineSegVec.size() > 1){ 
    for(size_t i=0; i<lineSegVec.size()-1; i++){
      auto seg1 = lineSegVec[i];
      double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
      double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);
      auto seg3 = lineSegVec[i+1];
      double x5 = bg::get<0, 0>(seg3); double y5 = bg::get<0, 1>(seg3);
      double x6 = bg::get<1, 0>(seg3);double y6 = bg::get<1, 1>(seg3);
      double x3 = x1; double y3 = y1;
      double x4 = x6; double y4 = y6;
      Segment seg2(Point(x3, y3), Point(x4, y4));
      tempSegments.push_back(seg1);
      tempSegments.push_back(seg2);
      tempSegments.push_back(seg3);
    } 
  }else{
    auto seg1 = lineSegVec[0];
    tempSegments.push_back(seg1);
  }  
  allLines.push_back(tempSegments); */

  //Apply polynomial fitting
  /*for(auto& lineSegVec: allLines){
    std::vector<double> x;std::vector<double> y;
    for(auto& seg: lineSegVec){
      double x1 = bg::get<0, 0>(seg); double y1 = bg::get<0, 1>(seg);
      double x2 = bg::get<1, 0>(seg);double y2 = bg::get<1, 1>(seg);
      x.push_back(x1);y.push_back(y1);
      x.push_back(x2);y.push_back(y2);
    } 
    if(lineSegVec.size() > 1){ 
      auto coeff = polynomialRegression(x,y,2);
      ROS_INFO_STREAM("Second order - "<<"a: "<<coeff[0]<<" b: "<<coeff[1]<<" c: "<<coeff[2]);
    }else{
      auto coeff = polynomialRegression(x,y,1);
      ROS_INFO_STREAM("First order - "<<"a: "<<coeff[0]<<" b: "<<coeff[1]);
    } 
  }*/
}

std::vector<double> FeatureExtractor::polynomialRegression(const std::vector<double> &t, const std::vector<double> &v, int order){
	
  //Reference: https://towardsdatascience.com/least-square-polynomial-fitting-using-c-eigen-package-c0673728bd01
 
  std::vector<double> coeff;
  // Create Matrix Placeholder of size n x k, n= number of datapoints, k = order of polynomial, for exame k = 3 for cubic polynomial
	Eigen::MatrixXd T(t.size(), order + 1);
	Eigen::VectorXd V = Eigen::VectorXd::Map(&v.front(), v.size());
	Eigen::VectorXd result;

	// check to make sure inputs are correct
	assert(t.size() == v.size());
	assert(t.size() >= order + 1);
	// Populate the matrix
	for(size_t i = 0 ; i < t.size(); ++i)
	{
		for(size_t j = 0; j < order + 1; ++j)
		{
			T(i, j) = pow(t.at(i), j);
		}
	}
	//std::cout<<T<<std::endl;

	// Solve for linear least square fit
	result  = T.householderQr().solve(V);
	coeff.resize(order+1);
	for (int k = 0; k < order+1; k++)
	{
		coeff[k] = result[k];
	}

  return coeff;

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

        //colour[0] = (uint8_t)(output_rgb.b * 255.);
        //colour[1] = (uint8_t)(output_rgb.g * 255.);
        //colour[2] = (uint8_t)(output_rgb.r * 255.);
	      colour[0] = 0.;
	      colour[1] = 255.;
	      colour[2] = 0.;

        //std::cout << "point " << value_x << ", " << value_y << " intensity " << intensity << std::endl;
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

  cv::Scalar odom_colour1(0., 0., 255., 180);
  float odom_radius1 = 1.;
  for(auto& lane_segment: lane_segments){
    cv::Scalar color(
      (double)std::rand() / RAND_MAX * 255,
      (double)std::rand() / RAND_MAX * 255,
      (double)std::rand() / RAND_MAX * 255,
      255
    );


    cv::Scalar odom_colour1(255., 255., 255., 180);
    
    for(auto& point: lane_segment){
        int x_index = ((point.second*100)/cm_resolution)*-1;
        int y_index = (point.first*100)/cm_resolution;
        int value_x = x_index - min_x;
        int value_y = y_index - min_y;
        cv::circle(output_image, cv::Point(value_y, value_x), odom_radius1, odom_colour1, CV_FILLED);
     }         
  }
  
  int thickness = 1;
  int count =0;
  for(auto& lane: lanes){
    cv::Scalar color(
      (double)std::rand() / RAND_MAX * 255,
      (double)std::rand() / RAND_MAX * 255,
      (double)std::rand() / RAND_MAX * 255,
      255
    );

    cv::Scalar odom_colour1(255., 255., 255., 180);

    auto first_pair = lane.first;
    auto second_pair = lane.second;
    int x_index1 = ((first_pair[1]*100)/cm_resolution)*-1;
    int y_index1 = (first_pair[0]*100)/cm_resolution;
    int value_x1 = x_index1 - min_x;
    int value_y1 = y_index1 - min_y;
     
    int x_index2 = ((second_pair[1]*100)/cm_resolution)*-1;
    int y_index2 = (second_pair[0]*100)/cm_resolution;
    int value_x2 = x_index2 - min_x;
    int value_y2 = y_index2 - min_y;

    cv::Point p1(value_y1, value_x1);
    cv::Point p2(value_y2, value_x2);
    if(first_pair[0] == second_pair[0] && first_pair[1] == second_pair[1])
    {
      //cv::line(output_image, p1, p2, odom_colour1, thickness, cv::LINE_8);
      continue;
    } else 
      cv::line(output_image, p1, p2, color, thickness, cv::LINE_8);
    
    cv::circle(output_image, cv::Point(value_y1, value_x1), 1., odom_colour1, CV_FILLED);
    
    //cv::putText(output_image,std::to_string(count),cv::Point(value_y1, value_x1), cv::FONT_HERSHEY_SIMPLEX,0.5,odom_colour1,1,false);
    //count ++;
  }

  thickness = 1;
  for(auto& lane: checkLanes){
    cv::Scalar odom_colour1(0., 0., 255., 180);
    auto first_pair = lane.first;auto second_pair = lane.second;
    int x_index1 = ((first_pair[1]*100)/cm_resolution)*-1;int y_index1 = (first_pair[0]*100)/cm_resolution;
    int value_x1 = x_index1 - min_x;int value_y1 = y_index1 - min_y;
    int x_index2 = ((second_pair[1]*100)/cm_resolution)*-1;int y_index2 = (second_pair[0]*100)/cm_resolution;
    int value_x2 = x_index2 - min_x;int value_y2 = y_index2 - min_y;
    cv::Point p1(value_y1, value_x1);cv::Point p2(value_y2, value_x2);
    cv::line(output_image, p1, p2, odom_colour1, thickness, cv::LINE_8);
  }

  for(auto& lineSeg: lines){
    cv::Scalar odom_colour1(0., 0., 255., 180);
    auto seg = std::get<0>(lineSeg);
    double x1 = bg::get<0, 0>(seg); double y1 = bg::get<0, 1>(seg);
    double x2 = bg::get<1, 0>(seg);double y2 = bg::get<1, 1>(seg);
    
    int x_index1 = ((y1*100)/cm_resolution)*-1;int y_index1 = (x1*100)/cm_resolution;
    int value_x1 = x_index1 - min_x;int value_y1 = y_index1 - min_y;
    int x_index2 = ((y2*100)/cm_resolution)*-1;int y_index2 = (x2*100)/cm_resolution;
    int value_x2 = x_index2 - min_x;int value_y2 = y_index2 - min_y;
    cv::Point p1(value_y1, value_x1);cv::Point p2(value_y2, value_x2);
    cv::line(output_image, p1, p2, odom_colour1, thickness, cv::LINE_8);
  }

  for(auto& lineSegVec: allLines){
    ROS_INFO_STREAM("size: "<<lineSegVec.size());
    cv::Scalar color(
      (double)std::rand() / RAND_MAX * 255,
      (double)std::rand() / RAND_MAX * 255,
      (double)std::rand() / RAND_MAX * 255,
      255
    );

    cv::Scalar odom_colour1(0., 0., 255., 180);

    for(auto& takeSeg: lineSegVec){
      auto segDetails = takeSeg.first;
      auto seg = std::get<0>(segDetails); 
      cv::Scalar odom_colour1(0., 0., 255., 180);
      double x1 = bg::get<0, 0>(seg); double y1 = bg::get<0, 1>(seg);
      double x2 = bg::get<1, 0>(seg);double y2 = bg::get<1, 1>(seg);
      
      int x_index1 = ((y1*100)/cm_resolution)*-1;int y_index1 = (x1*100)/cm_resolution;
      int value_x1 = x_index1 - min_x;int value_y1 = y_index1 - min_y;
      int x_index2 = ((y2*100)/cm_resolution)*-1;int y_index2 = (x2*100)/cm_resolution;
      int value_x2 = x_index2 - min_x;int value_y2 = y_index2 - min_y;
      cv::Point p1(value_y1, value_x1);cv::Point p2(value_y2, value_x2);
      cv::line(output_image, p1, p2, color, thickness, cv::LINE_8);
    } 
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



void  FeatureExtractor::outputImage(std::map<std::pair<int,int>, double> &intensity_topic, std::string output_image_name){
	  
	  ROS_INFO_STREAM("Creating "<<output_image_name<<" image");
	  int max_x = -100000000, max_y = -100000000, min_x = 100000000, min_y = 100000000;
	  float min_intensity = 100000000.;
	  float max_intensity = -100000000.;
	  for (auto &entry: intensity_topic) {
		      min_x = std::min<int>(min_x, entry.first.first);
		      max_x = std::max<int>(max_x, entry.first.first);

		      min_y = std::min<int>(min_y, entry.first.second);
		      max_y = std::max<int>(max_y, entry.first.second);

		      min_intensity = std::min<double>(min_intensity, entry.second);
		      max_intensity = std::max<double>(max_intensity, entry.second);
  	  }

	  // restrict the intensity range as determined by the param settings
	  float intensity_scale_min = private_nh.param<float>("min_intensity", 0.);
	  float intensity_scale_max = private_nh.param<float>("max_intensity", 80.);

	  min_intensity = std::max<float>(min_intensity, intensity_scale_min);
	  max_intensity = std::min<float>(max_intensity, intensity_scale_max);

          // Add a small buffer for the image to account for rounding errors
          int image_max_x = max_x - min_x + 2;
	  int image_max_y = max_y - min_y + 2;
		
	  std::cout << min_x << ", " << min_y << ", " << max_x << ", " << max_y <<", "<<image_max_x<<", "<<image_max_y<<std::endl;

	  // the lidar datapoints projected into 2d
	  cv::Mat output_image(image_max_x, image_max_y, CV_8UC4, cv::Scalar(0, 0, 0, 0));

	  for (auto &entry: intensity_topic) {

	      int value_x = entry.first.first - min_x;
	      int value_y = entry.first.second - min_y;

	      if (value_x < 0 || value_x >= image_max_x) {
	        continue;
	      }

	      if (value_y < 0 || value_y >= image_max_y) {
	        continue;
	      }

	      cv::Vec4b colour;

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
	      
              cv::Point destination_point(value_y, value_x);
	    
	      // Set the corresponding pixel to the RBG colour
   	      colour[3] = 255;
		
    	      // Set the alpha value for the blurred layer
              output_image.at<cv::Vec4b>(destination_point) = colour;
	  }
	  
	  cv::imwrite(output_image_name, output_image);
}


std::map<std::pair<int,int>, double> FeatureExtractor::createIntensityMap(pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud_filtered1){
	std::map<std::pair<int,int>, double> intensity_topic;
	
	// put each point into the intensity map
        for (size_t i = 0; i < cloud_filtered1->points.size(); ++i) {
                int x_index = (cloud_filtered1->points[i].x * 100.) / cm_resolution;
                int y_index = (cloud_filtered1->points[i].y * 100.) / cm_resolution;
                if (fabs(x_index) > 1e7 || fabs(y_index) > 1e7) {
                	continue;
                }

                intensity_topic[std::make_pair(-1. * y_index,
                                             x_index)] = cloud_filtered1->points[i].intensity;
        }

	return intensity_topic;
}

std::map<std::pair<int,int>, double> FeatureExtractor::createIntensityMap(pcl::PointCloud<pcl::PointXYZI> cloud_filtered1){
        std::map<std::pair<int,int>, double> intensity_topic;
	
        // put each point into the intensity map
        for (size_t i = 0; i < cloud_filtered1.points.size(); ++i) {
                int x_index = (cloud_filtered1.points[i].x * 100.) / cm_resolution;
                int y_index = (cloud_filtered1.points[i].y * 100.) / cm_resolution;
                if (fabs(x_index) > 1e7 || fabs(y_index) > 1e7) {
                	continue;
                }

                intensity_topic[std::make_pair(-1. * y_index,
                                             x_index)] = cloud_filtered1.points[i].intensity;
        }

        return intensity_topic;
}



pcl::PointCloud<pcl::PointXYZI> FeatureExtractor::extractEdges(pcl::PointCloud<pcl::PointXYZIR>::Ptr input_cloud, long int sec, long int nsec){

	/*pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud_filtered = pcl::PointCloud<pcl::PointXYZIR>::Ptr(new pcl::PointCloud<pcl::PointXYZIR>);
	pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud_filtered1 = pcl::PointCloud<pcl::PointXYZIR>::Ptr(new pcl::PointCloud<pcl::PointXYZIR>);
    	pcl::PassThrough<pcl::PointXYZIR> pass (false);
    	pass.setInputCloud(input_cloud);
    	pass.setFilterFieldName ("x");
    	pass.setFilterLimits(2, 100);
   	pass.setFilterLimitsNegative(false);
    	pass.filter (*cloud_filtered1);*/

  	/*auto intensity_topic = createIntensityMap(input_cloud);
  	outputImage(intensity_topic, "/constraint_model/images/new_image.png");*/
	std::vector<std::vector<float>>  ring1;
	std::vector<std::vector<float>>  ring2;
	std::vector<std::vector<float>>  ring3;
	std::vector<std::vector<float>>  ring4;
	//std::vector<std::vector<float>>  ring5;
		
	// Sort the pointclouds
	for (size_t i = 0; i < input_cloud->points.size(); ++i) {
		std::vector<float> row;
      		row.push_back(input_cloud->points[i].x); 
		row.push_back(input_cloud->points[i].y); 
		row.push_back(input_cloud->points[i].z); 
		row.push_back(input_cloud->points[i].intensity);
		if(input_cloud->points[i].ring == 90){
			ring1.push_back(row);
		}else if(input_cloud->points[i].ring == 91){
			ring2.push_back(row);
		}else if(input_cloud->points[i].ring == 92){
			ring3.push_back(row);
		}else if(input_cloud->points[i].ring == 93){
			ring4.push_back(row);
		}
	}


	//Sorting	
	std::sort(ring1.begin(), ring1.end(), [](const std::vector<float>& a, const std::vector<float>& b) {return a[1] < b[1];});
	std::sort(ring2.begin(), ring2.end(), [](const std::vector<float>& a, const std::vector<float>& b) {return a[1] < b[1];});
	std::sort(ring3.begin(), ring3.end(), [](const std::vector<float>& a, const std::vector<float>& b) {return a[1] < b[1];});
	std::sort(ring4.begin(), ring4.end(), [](const std::vector<float>& a, const std::vector<float>& b) {return a[1] < b[1];});


	//Just for visulising
	/*pcl::PointCloud<pcl::PointXYZI> pc1 = mat2PCL(ring1);
	intensity_topic.clear();
	intensity_topic = createIntensityMap(pc1);
        outputImage(intensity_topic, "/constraint_model/images/ring1_sort.png");
	*/	

	int middleR1 = middlePoint(ring1, 0);
    	int middleR2 = middlePoint(ring2, 0);
    	int middleR3 = middlePoint(ring3, 0);
	//int middleR4 = middlePoint(ring4, 0);
    	int middleR4 = 0;
	
	//ROS_INFO_STREAM("Middle Points: "<< middleR1<<" "<<middleR2<<" "<<middleR3<<" "<<middleR4);

	std::vector<std::vector<float> > obs_points;
	// 15 10
	std::vector<std::vector<float> > edge1 = findEdges(ring1, 15, 10, 9, ring1[middleR1][3], middleR1, 15, obs_points);// 12 10
	std::vector<std::vector<float> > edge2 = findEdges(ring2, 15, 10, 9, ring2[middleR2][3], middleR2, 14, obs_points);// 12 10
	std::vector<std::vector<float> > edge3 = findEdges(ring3, 15, 10, 9, ring3[middleR3][3], middleR3, 10, obs_points);// 12 10
	//std::vector<std::vector<float> > edge4 = findEdges(ring4, 15, 10, 9, ring4[middleR4][3], middleR4, 8, obs_points);// 12 10
	//ROS_INFO_STREAM("r1: "<<ring1.size()<<" e1: "<<edge1.size()<<", r2: "<<ring2.size()<<" e2: "<<edge2.size()<<", r3: "<<ring3.size()<<" e3: "<<edge3.size()<<", r4:"<<" e4: "<<", other_points: "<<obs_points.size());	

	pcl::PointCloud<pcl::PointXYZI> edge_pc1 = mat2PCL(edge1);
	pcl::PointCloud<pcl::PointXYZI> edge_pc2 = mat2PCL(edge2);
	pcl::PointCloud<pcl::PointXYZI> edge_pc3 = mat2PCL(edge3);
	//pcl::PointCloud<pcl::PointXYZI> edge_pc4 = mat2PCL(edge4);
	pcl::PointCloud<pcl::PointXYZI> obs = mat2PCL(obs_points);
	pcl::PointCloud<pcl::PointXYZI> total;
	
	total += edge_pc1;
	total += edge_pc2;
        total += edge_pc3;
	//total += edge_pc4;
	//ROS_INFO_STREAM("Total points on roads: "<<total.points.size());
	
	/*intensity_topic.clear();
        intensity_topic = createIntensityMap(edge_pc1);
        outputImage(intensity_topic, "/constraint_model/images/edge1.png");
	
	intensity_topic.clear();
        intensity_topic = createIntensityMap(total);
        outputImage(intensity_topic, "/constraint_model/images/edge_all.png");
	*/	

	pcl::PCLPointCloud2 cloudR1;pcl::PCLPointCloud2 cloudR2;
	pcl::PCLPointCloud2 cloudR3;//pcl::PCLPointCloud2 cloudR4;
	pcl::PCLPointCloud2 cloudFinal;pcl::PCLPointCloud2 obs_pc;
	pcl::toPCLPointCloud2(edge_pc1,cloudR1);pcl::toPCLPointCloud2(edge_pc2,cloudR2);
	pcl::toPCLPointCloud2(edge_pc3,cloudR3);//pcl::toPCLPointCloud2(edge_pc4,cloudR4);
	pcl::toPCLPointCloud2(obs,obs_pc);

	float height2 = 0.37;float height3 = 0.42;float height4 = 0.47;
    	if (ring2[middleR2][2]<height2){
      		pcl::concatenatePointCloud (cloudR2, cloudR1, cloudFinal);
      		if (ring3[middleR3][2]<height3){
        		pcl::concatenatePointCloud (cloudFinal, cloudR3, cloudFinal);
        		/*if (ring4[middleR4][2]<height4){
          			pcl::concatenatePointCloud (cloudFinal, cloudR4, cloudFinal);
        		}*/
      		}
    	}
	
	//sensor_msgs::PointCloud2 roadPC;
	pcl::PointCloud<pcl::PointXYZI> pointCloud;

	if(total.points.size() > 500){
		pcl::PointCloud<pcl::PointXYZI> tempCloud;
		pcl::fromPCLPointCloud2(cloudFinal, pointCloud);
		/*pcl::fromPCLPointCloud2(cloudFinal, tempCloud);
		auto result = processForSphericalCoordinateFrame(tempCloud);
		pcl::copyPointCloud(result, pointCloud);*/
	}else{
		auto cloudFiltered = processForSphericalCoordinateFrame(input_cloud);
		pcl::copyPointCloud(cloudFiltered, pointCloud);
		/*intensity_topic.clear();
        	intensity_topic = createIntensityMap(cloudFiltered);
        	outputImage(intensity_topic, "/constraint_model/images/edge_all.png");
		*/
		//ROS_INFO_STREAM("Total points: "<<input_cloud->points.size()<<", "<<cloudFiltered.points.size());
	}
	
	return pointCloud;
}

pcl::PointCloud<pcl::PointXYZI> FeatureExtractor::mat2PCL(std::vector<std::vector<float> > matrixPC){
     pcl::PointCloud<pcl::PointXYZI> pointCloud;
     for (int i=0;i < matrixPC.size(); i=i+1){
       pcl::PointXYZI point;
       point.x = matrixPC[i][0];
       point.y = matrixPC[i][1];
       point.z = matrixPC[i][2];
       point.intensity = matrixPC[i][3];
       pointCloud.push_back(point);
     }
     return pointCloud;
}


std::vector<std::vector<float>> FeatureExtractor::findEdges(std::vector<std::vector<float> > matrixPC, float AngleThreshold, float Angle_d_Threshold, float IntensityThreshold, float Intensity, int middle_intensity_index, int points, std::vector<std::vector<float>>& obs_points) {
     std::vector<std::vector<float> > edges_points;
     int n=20;
     int inclination_change=0;
     float angle;
     float anglexy;
     
     bool obs_N_det= true;
     
     //Angle on the xy palne
     auto angle_past_xy = atan2((matrixPC[middle_intensity_index+points][0]-matrixPC[middle_intensity_index][0]),((matrixPC[middle_intensity_index+points][1]-matrixPC[middle_intensity_index][1])))*180/3.14159265;
     
     //Angle on the yz plane
     auto angle_past = atan2((matrixPC[middle_intensity_index+points][2]-matrixPC[middle_intensity_index][2]),((matrixPC[middle_intensity_index+points][1]-matrixPC[middle_intensity_index][1])))*180/3.14159265;
     
     //One side of the points
     for (int i=middle_intensity_index; i< (matrixPC.size()-points); i=i+1)
     {
        std::vector<float> edge_h;

	//Angle on the xy palne
        anglexy=atan2((matrixPC[i+points][0]-matrixPC[i][0]),((matrixPC[i+points][1]-matrixPC[i][1])))*180/3.14159265;
        
	//Angle on the yz palne
	angle=atan2((matrixPC[i+points][2]-matrixPC[i][2]),((matrixPC[i+points][1]-matrixPC[i][1])))*180/3.14159265;
       
	//ROS_INFO_STREAM("Angle D " << angle << ".\n");
       
       //comparing angle
       if ((std::abs(angle))<AngleThreshold && (std::abs(angle-angle_past))<Angle_d_Threshold && obs_N_det ){
              edge_h.push_back(matrixPC[i][0]);
	      edge_h.push_back(matrixPC[i][1]);
	      edge_h.push_back(matrixPC[i][2]);
	      edge_h.push_back(matrixPC[i][3]);
              edges_points.push_back(edge_h);
              angle_past = angle;
              angle_past_xy =anglexy;
        } else {
          obs_N_det= false;
	  //std::cout << "Angle I " << angle << ".\n";
          //std::cout << "Angle Diff " << (angle-angle_past) << ".\n";
          edge_h.push_back(matrixPC[i][0]);
	  edge_h.push_back(matrixPC[i][1]);
	  edge_h.push_back(matrixPC[i][2]);
	  edge_h.push_back(matrixPC[i][3]);
          obs_points.push_back(edge_h);
       }
     }

     // OTHER SIDE OF THE POINTS
     obs_N_det= true;
     
      //Angle on the yz palne
     angle_past = atan2((matrixPC[middle_intensity_index][2]-matrixPC[middle_intensity_index-points][2]),((matrixPC[middle_intensity_index][1]-matrixPC[middle_intensity_index-points][1])))*180/3.14159265;
     for (int i=middle_intensity_index; i> points; i=i-1)
     {
        std::vector<float> edge_h;

	 //Angle on the yz palne
        angle=atan2((matrixPC[i][2]-matrixPC[i-points][2]),((matrixPC[i][1]-matrixPC[i-points][1])))*180/3.14159265;
        
	//comparing angle
        //cout << "Angle I " << angle << ".\n";
        if ((std::abs(angle))<AngleThreshold && (std::abs(angle-angle_past))<Angle_d_Threshold && obs_N_det ){
              edge_h.push_back(matrixPC[i][0]);edge_h.push_back(matrixPC[i][1]);edge_h.push_back(matrixPC[i][2]);edge_h.push_back(matrixPC[i][3]);
              edges_points.push_back(edge_h);
              angle_past=angle;
         } else {
             obs_N_det= false;
           //  cout << "Izquierda .\n";
           //  cout << "Angle I " << angle << ".\n";
           //  cout << "Angle Diff " << (angle-angle_past) << ".\n";
             edge_h.push_back(matrixPC[i][0]);edge_h.push_back(matrixPC[i][1]);edge_h.push_back(matrixPC[i][2]);edge_h.push_back(matrixPC[i][3]);
             obs_points.push_back(edge_h);
        // break;
       }
     }
     
     return edges_points;
}


int FeatureExtractor::middlePoint(std::vector<std::vector<float>> matrixPC, float value){
     int middle;
     float middle_f = 100000;
     for (int i=0; i< (matrixPC.size()-1); i=i+1){
       if ((std::abs(matrixPC[i][1]-value))<middle_f){
         middle = i;
         middle_f = std::abs(matrixPC[i][1]-value);
       }

     }
     
     return (middle);
}

pcl::PointCloud<pcl::PointXYZI> FeatureExtractor::processForSphericalCoordinateFrame(pcl::PointCloud<pcl::PointXYZI> pointCloud){
        //ROS_INFO_STREAM("processForSphericalCoordinateFrame");
        float radiusThreshold_max = 7.0;
        float angleThreshold_min = 2.0;
        float angleThreshold_max = 4.25;
        std::vector<std::vector<float>>  matrix;
        pcl::PointCloud<pcl::PointXYZI> temp_cloud;
	for (size_t i = 0; i < pointCloud.points.size(); ++i) {
		auto x = pointCloud.points[i].x;
                auto y = pointCloud.points[i].y;
                auto z = pointCloud.points[i].z;
                auto radius = sqrt(pow(x, 2)+pow(y, 2)+pow(z, 2));
                auto theta = atan2(y, x);
                auto phi = acos((z/radius));
                theta = wrapAngle(theta);
                if(theta < angleThreshold_min || theta > angleThreshold_max)
                        continue;

                if(radius > radiusThreshold_max)
                        continue;

                pcl::PointXYZI point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.intensity = pointCloud.points[i].intensity;
                temp_cloud.push_back(point);
        }


        return temp_cloud;
}


pcl::PointCloud<pcl::PointXYZI> FeatureExtractor::processForSphericalCoordinateFrame(pcl::PointCloud<pcl::PointXYZIR>::Ptr input_cloud){
	//ROS_INFO_STREAM("processForSphericalCoordinateFrame");
	/*Converting cartisian to spherical coordinate frame.
	 *Spherical is 3d for of  2d polar cartician frame. Polar is used in c		ircle. Plar has radis and angle and working on xy plane as it is 2d.
	 *More detailed explnation can be found here: https://blog.demofox.org/2013/10/12/converting-to-and-from-polar-spherical-coordinates-made-easy/
	 *Equation of spherical coordinate frames
	 *radius = sqrt(X * X + Y * Y + Z * Z) //distance
	 *theta = atan2(Y, X) // bearing
	 *phi = acos(Z / radius) //pitch
	*/

        pcl::PointCloud<pcl::PointXYZI> pointCloud;
	pcl::copyPointCloud(*input_cloud, pointCloud);
	
	float radiusThreshold_max = 7.0;
	float angleThreshold_min = 2.0;
	float angleThreshold_max = 4.25;
	std::vector<std::vector<float>>  matrix;
	std_msgs::Float32MultiArray r_array, t_array, p_array;
	pcl::PointCloud<pcl::PointXYZI> temp_cloud; 
	for (size_t i = 0; i < pointCloud.points.size(); ++i) {
      		auto x = pointCloud.points[i].x;
      		auto y = pointCloud.points[i].y;
      		auto z = pointCloud.points[i].z;
		auto radius = sqrt(pow(x, 2)+pow(y, 2)+pow(z, 2));
		auto theta = atan2(y, x);
		auto phi = acos((z/radius));
		theta = wrapAngle(theta);
		
		if(theta < angleThreshold_min || theta > angleThreshold_max)
			continue;
		
		if(radius > radiusThreshold_max)
			continue;
          
		pcl::PointXYZI point;
		point.x = x;
		point.y = y;
		point.z = z;
		point.intensity = pointCloud.points[i].intensity;
		temp_cloud.push_back(point);
		
		r_array.data.push_back(radius);
		t_array.data.push_back(theta);
		p_array.data.push_back(phi);
	}
	sphericalR.publish(r_array);
	sphericalT.publish(t_array);
	sphericalP.publish(p_array);

	return temp_cloud;

}

double FeatureExtractor::wrapAngle( double angle )
{
    double twoPi = 2.0 * 3.141592865358979;
    return angle - twoPi * floor( angle / twoPi );
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

pcl::PointCloud<pcl::PointXYZI>  FeatureExtractor::pointCloudFilter(pcl::PointCloud<pcl::PointXYZI> inputCloud){
	

	//-------------------------Filter based on height-----------------------------
	float min_intensity = 100000000.;
	float max_intensity = -100000000.;
  std::vector<std::vector<float>> matrixPC;
  for (size_t i = 0; i < inputCloud.points.size(); ++i) {
          std::vector<float> row;
          row.push_back(inputCloud.points[i].x);
          row.push_back(inputCloud.points[i].y);
          row.push_back(inputCloud.points[i].z);
          row.push_back(inputCloud.points[i].intensity);
          matrixPC.push_back(row);
    min_intensity = std::min<double>(min_intensity, inputCloud.points[i].intensity);
    max_intensity = std::max<double>(max_intensity, inputCloud.points[i].intensity);

	}

	// Sorting
  std::sort(matrixPC.begin(), matrixPC.end(), [](const std::vector<float>& a, const std::vector<float>& b) {return a[1] < b[1];});


	std::vector<std::vector<float>> heightMatrixPC;
	int middle = middlePoint(matrixPC, 0);
	double heightDeltaThreshold = 0.10;
	double heightThreshold = 0.20;
	double previousHeight = 0.;
	
	//One side of the points
     	for (int i=middle; i<matrixPC.size(); i++)
     	{
		double z = matrixPC[i][2];
		double heightDelta = z-previousHeight;
		if(z < heightThreshold && heightDelta < heightDeltaThreshold){
			std::vector<float> row;
			row.push_back(matrixPC[i][0]);
			row.push_back(matrixPC[i][1]);
			row.push_back(matrixPC[i][2]);
			row.push_back(matrixPC[i][3]);
			heightMatrixPC.push_back(row);
			previousHeight = z;
		}
		
	}

	//Other side of the points
  for (int i=middle; i>0; i--)
  {
      double z = matrixPC[i][2];
      double heightDelta = z-previousHeight;
      if(z < heightThreshold && heightDelta < heightDeltaThreshold){
        std::vector<float> row;
        row.push_back(matrixPC[i][0]);
        row.push_back(matrixPC[i][1]);
        row.push_back(matrixPC[i][2]);
        row.push_back(matrixPC[i][3]);
        heightMatrixPC.push_back(row);
        previousHeight = z;
      }

  }

	//ROS_INFO_STREAM("Size of heightPC: "<<heightMatrixPC.size());

	//--------------------Filter based on intensity-------------------	

	// High intensity  = 70% higher
	auto intensity_min = (max_intensity*70)/100;
	
	// Sorting
  std::sort(heightMatrixPC.begin(), heightMatrixPC.end(), [](const std::vector<float>& a, const std::vector<float>& b) {return a[1] < b[1];});

	
	pcl::PointCloud<pcl::PointXYZI> intensityPC;
	double intThreshold = 0.40;	
	bool deltaFlag = false;
	double previousInt = 0.;
	
	for(int i=0; i<heightMatrixPC.size();i++){
		double x = heightMatrixPC[i][0];
    double y = heightMatrixPC[i][1];
    double z = heightMatrixPC[i][2];
    double intensity = heightMatrixPC[i][3];
                        
    // Finding changes in intensity  
    double intDelta = intensity-previousInt;        
		if(intDelta > intThreshold && intensity >= intensity_min){
			deltaFlag = true;

		}else{
			deltaFlag = false;
		}	
		
		if(deltaFlag){
			pcl::PointXYZI point;
      point.x = x;
      point.y = y;
      point.z = z;
      point.intensity = intensity;
      intensityPC.push_back(point);
		}
		previousInt = intensity;
	}
	
  // -----------Averaging the pointcloud-------------------
	pcl::PointCloud<pcl::PointXYZI> lanePC;
	if(intensityPC.points.size() > 1) {	

		//pcl::copyPointCloud(intensityPC, lanePC);
    
    std::vector<std::vector<float>> avgMatrixPC;
		for (size_t i = 0; i < intensityPC.points.size(); ++i) {
			std::vector<float> row;
			row.push_back(intensityPC.points[i].x);
			row.push_back(intensityPC.points[i].y);
			row.push_back(intensityPC.points[i].z);
			row.push_back(intensityPC.points[i].intensity);
			avgMatrixPC.push_back(row);
		}
		
		// Sorting
		std::sort(avgMatrixPC.begin(), avgMatrixPC.end(), [](const std::vector<float>& a, const std::vector<float>& b) {return a[1] < b[1];});
    pcl::PointCloud<pcl::PointXYZI>::Ptr intensityPCPtr(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::copyPointCloud(intensityPC, *intensityPCPtr);
		pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
		kdtree.setInputCloud (intensityPCPtr); 
		double radius = 2.; //2 works
    std::vector<std::pair<double, double>> searchVec;
		for (int i=0; i<avgMatrixPC.size(); i++)
		{
      auto x1 = avgMatrixPC[i][0];
      auto y1 = avgMatrixPC[i][1];
      if(checkVector(searchVec, std::make_pair(x1,y1)))
        continue; 
      std::vector<std::vector<float>> knnMatrixPC;
      std::vector<float> row;
      row.push_back(avgMatrixPC[i][0]);
      row.push_back(avgMatrixPC[i][1]);
      row.push_back(avgMatrixPC[i][2]);
      row.push_back(avgMatrixPC[i][3]); 
			knnMatrixPC.push_back(row);
      for(int j=0; j<avgMatrixPC.size(); j++){
        auto x2 = avgMatrixPC[j][0]; 
        auto y2 = avgMatrixPC[j][1];
        if(i != j){
          float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
          if(d <= radius){
						row.clear();
						row.push_back(avgMatrixPC[i][0]);
            row.push_back(avgMatrixPC[i][1]);
            row.push_back(avgMatrixPC[i][2]);
            row.push_back(avgMatrixPC[i][3]); 
            knnMatrixPC.push_back(row);
            searchVec.push_back(std::make_pair(x2, y2));
          }
        }
      }
      //searchVec.push_back(std::make_pair(x1, y1));
      std::sort(knnMatrixPC.begin(), knnMatrixPC.end(), [](const std::vector<float>& a, const std::vector<float>& b) {return a[1] < b[1];});
      
      int middlePoint = knnMatrixPC.size()/2;
      pcl::PointXYZI point;
      point.x = knnMatrixPC[middlePoint][0];
      point.y = knnMatrixPC[middlePoint][1];
      point.z = knnMatrixPC[middlePoint][2];
      point.intensity = knnMatrixPC[middlePoint][0];
      lanePC.push_back(point);
		}
	}else{
    pcl::copyPointCloud(intensityPC, lanePC);
	}

	return lanePC;

}

bool FeatureExtractor::checkVector(std::vector<std::pair<double, double>> searchVec, std::pair<double, double> xy){
  bool status = false;  
  for(size_t i=0; i<searchVec.size(); i++){
    auto point = searchVec[i];
    auto x2 = point.first;
    auto y2 = point.second;
    if(xy.first == x2 && xy.second == y2){
      status = true;
      break;
    }
  }
  
  return status;
}


bool FeatureExtractor::checkVector(std::vector<std::pair<std::vector<std::pair<double, double>>, int>> searchVec, std::pair<double, double> xy){
  bool status = false;  
  for(size_t i=0; i<searchVec.size(); i++){
    auto segementActiveOrInactive = active_lane_segments[i];
    auto laneSegment = std::get<0>(segementActiveOrInactive);
    for(size_t j=0; j<searchVec.size(); j++){
      auto point = laneSegment[j];
      auto x2 = point.first;
      auto y2 = point.second;
      if(xy.first == x2 && xy.second == y2){
        status = true;
        break;
      }
    }
  }
  
  return status;
}

std::pair<std::vector<double>,std::vector<double>> FeatureExtractor::findODOMPoints(){
  std::pair<std::vector<double>,std::vector<double>> _return;
  std::vector<double> xs;std::vector<double> ys;
  if(vehicle_odom_double.size() > 1){
    auto x1 = vehicle_odom_double[vehicle_odom_double.size()-1].first;
    auto y1 = vehicle_odom_double[vehicle_odom_double.size()-1].second;
    xs.push_back(x1);
    ys.push_back(y1);
    for(size_t i=vehicle_odom_double.size()-2;i>=0;i--){
        auto x2 = vehicle_odom_double[i].first;
        auto y2 = vehicle_odom_double[i].second;
        xs.push_back(x2);
        ys.push_back(y2);
        float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
        if(d >= 1.){
          break;
        }
    }
    _return = std::make_pair(xs, ys);
  }
  
  return _return;
}

std::pair<std::vector<double>,std::vector<double>> FeatureExtractor::findSlopeLaneSeg(std::vector<std::pair<double, double>> laneSeg, double threshold){
  std::pair<std::vector<double>,std::vector<double>> lanePoints;
  auto x1 = laneSeg[laneSeg.size()-1].first;
  auto y1 = laneSeg[laneSeg.size()-1].second;
  for(size_t i=laneSeg.size()-2;i>=0;i--){
      auto x2 = laneSeg[i].first;
      auto y2 = laneSeg[i].second;
      float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
      if(d >= threshold){
        std::vector<double> row1;
        row1.push_back(x1);
        row1.push_back(y1);
        std::vector<double> row2;
        row2.push_back(x2);
        row2.push_back(y2);
        lanePoints = std::make_pair(row1, row2); 
        break;
      }
  }
  
  return lanePoints;
}

void FeatureExtractor::outliersRemoval(std::vector<std::pair<double, double>> laneSegment, std::vector<double>& xs, std::vector<double>& ys){
	pcl::PointCloud<pcl::PointXYZ>::Ptr laneSegPC(new pcl::PointCloud<pcl::PointXYZ>);
  for(auto& lSeg: laneSegment){
    pcl::PointXYZ point;
    point.x = lSeg.first;
    point.y = lSeg.second;
    point.z = 0.;
    laneSegPC->push_back(point);
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (laneSegPC);
  sor.setMeanK (30);
  sor.setStddevMulThresh (0.75);
  sor.filter (*cloud_filtered);
  for(size_t i = 0; i < cloud_filtered->points.size(); ++i) {
			xs.push_back(cloud_filtered->points[i].x);
			ys.push_back(cloud_filtered->points[i].y);
	}
}


void FeatureExtractor::constructLaneSegments(pcl::PointCloud<pcl::PointXYZI> lanePC){
  //std::vector<std::pair<std::vector<std::pair<double, double>>, int>> active_lane_segments;//[lane po    ints in lane segments, inactive count]
  //std::map<long int, std::vector<std::pair<double, double>>> lane_segment;//[lane points lane segements]
  //ROS_INFO_STREAM("_____________________________________: "<<seq_count);
  //ROS_INFO_STREAM("Active lane segments size: "<<active_lane_segments.size());
  //ROS_INFO_STREAM("Lane segments size: "<<lane_segments.size());
  //ROS_INFO_STREAM("lanPC size: "<<lanePC.points.size());

  /*for(size_t i=0;i<lanePC.points.size();i++){
    auto x1 = lanePC.points[i].x;
    auto y1 = lanePC.points[i].y;
    std::vector<std::pair<double, double>> laneSegment;
    laneSegment.push_back(std::make_pair(x1, y1));  
    lane_segments.push_back(laneSegment);
  }*/
  float RADIUS = 1.; //1. works
  float INACTIVECOUNT = 20; //20 and below only works - tried with 50, and 100, but not getting good result
  if(active_lane_segments.size() == 0){
    std::vector<std::pair<double, double>> searchVec; 
    for(size_t i=0;i<lanePC.points.size();i++){
      auto x1 = lanePC.points[i].x;
      auto y1 = lanePC.points[i].y;
      //ROS_INFO_STREAM("active is zero: "<<x1<<" "<<y1);
      if(checkVector(searchVec, std::make_pair(x1,y1))) 
        continue;
      for(size_t j=0;j<lanePC.points.size();j++){
        auto x2 = lanePC.points[j].x;
        auto y2 = lanePC.points[j].y;
        if(i==j || checkVector(searchVec, std::make_pair(x2,y2)))
          continue;
        float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
        auto y_dist = std::abs(std::abs(y2)-std::abs(y1));
        
        //Avoiding closest lateral points
        if(d<RADIUS){
          searchVec.push_back(std::make_pair(x2, y2));
        }
      }

      if(!checkVector(searchVec, std::make_pair(x1,y1))) {
          std::vector<std::pair<double, double>> laneSegment;
          laneSegment.push_back(std::make_pair(x1, y1));
          std::vector<std::pair<double, double>> odomPoints;
          odomPoints.push_back(std::make_pair(vehicle_odom_double[vehicle_odom_double.size()-1].first, vehicle_odom_double[vehicle_odom_double.size()-1].second));
          active_lane_segments.push_back(std::make_tuple(laneSegment, 0, odomPoints));
      }

    }
   
  }else{
    std::vector<std::pair<double, double>> searchVec;
    std::vector<std::pair<double, double>> newPointsVec;
    std::vector<size_t> activeList;
    for(size_t i=0;i<lanePC.points.size();i++){
      auto x1 = lanePC.points[i].x;
      auto y1 = lanePC.points[i].y;
      double smallRadius = 10000.;
      size_t activeIndex = 100000;
      bool found = false;
      for(size_t j=0;j<active_lane_segments.size(); j++){
        
        if (std::find(activeList.begin(), activeList.end(), j) != activeList.end()) 
          continue;
        
        auto laneSegment = std::get<0>(active_lane_segments[j]);
        auto point = laneSegment.back();
        auto x2 = point.first; auto y2 = point.second;
        float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
        if(d<RADIUS && d<smallRadius){
          smallRadius = d;
          activeIndex = j;
          found = true;
        } 
      }//first active lane segment for loop close
      
      if(found){
        auto laneSegment = std::get<0>(active_lane_segments[activeIndex]);
        auto odomPoints = std::get<2>(active_lane_segments[activeIndex]);
        laneSegment.push_back(std::make_pair(x1, y1));
        odomPoints.push_back(std::make_pair(vehicle_odom_double[vehicle_odom_double.size()-1].first, vehicle_odom_double[vehicle_odom_double.size()-1].second));
        active_lane_segments[activeIndex] = std::make_tuple(laneSegment, 0, odomPoints);
        activeList.push_back(activeIndex);
      }else{
        newPointsVec.push_back(std::make_pair(x1, y1));
      }

    }//lanePC for loop close
    for(size_t j=0;j<active_lane_segments.size(); j++){
        if(std::find(activeList.begin(), activeList.end(), j) != activeList.end()) 
          continue;
        else{
          auto laneSegment = std::get<0>(active_lane_segments[j]);
          auto inactiveCount = std::get<1>(active_lane_segments[j]);
          auto odomPoints = std::get<2>(active_lane_segments[j]);
          inactiveCount += 1;
        
          // Remove from active and store them in lane segement
          if(inactiveCount > INACTIVECOUNT){
            
            //Remove the outliers
            
            bool proceed = true;
            std::vector<double> xs;
            std::vector<double> ys;
            /*for(auto& lSeg: laneSegment){
              xs.push_back(lSeg.first);
              ys.push_back(lSeg.second);
            }*/

            outliersRemoval(laneSegment, xs, ys);

            auto lineDetails = linearRegression(xs, ys);
            std::vector<double> odomX; std::vector<double> odomY;
            for(auto& odom: odomPoints){
              odomX.push_back(odom.first);
              odomY.push_back(odom.second);
            } 
            auto odomLineDetails = linearRegression(odomX, odomY);
            double m; double c;
            if(xs.size() > 2){
              auto lineDetails = linearRegression(xs, ys);
              auto mse = std::get<2>(lineDetails);
              m = std::get<0>(lineDetails);
              c = std::get<1>(lineDetails);
              //ROS_INFO_STREAM("m: "<<m<<" c: "<<c<<" mse: "<<mse);
              if(mse>0.05){ //0.05 works fine
                proceed = false;
              }

              //Also checking the slope of odom and lane odom
              auto x1 = odomPoints[0].first;
              auto y1 = odomPoints[0].second;
              auto x2 = odomPoints[odomPoints.size()-1].first;
              auto y2 = odomPoints[odomPoints.size()-1].second;
              float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
              if(odomPoints.size() > 2 && d > 0.15){
                auto slopeOdom =  std::get<0>(odomLineDetails);
                auto slopeDiff = std::abs(slopeOdom-m);
                //ROS_INFO_STREAM("slopeOdom: "<<slopeOdom<<" lineOdom: "<<m<<" diff: "<<slopeDiff);
                if(slopeDiff > 1.) //1.good for 5 to 8
                  proceed = false;
              }

            }else{
                //discard lanSegments that has only one point.
                proceed = false;
            }
            
            // Push the value to lane_segments
            if(proceed){
                lane_segments.push_back(laneSegment);
                //We have equation of the line in the form y=mx+b. It would be
                //great to calculate the start and end points by taking x value
                //from first and last point in the lane segment. 
                auto x1 = laneSegment[0].first;
                auto y1 = m*x1+c;
                auto x2 = laneSegment[laneSegment.size()-1].first;
                auto y2 = m*x2+c;
                Segment seg(Point(x1, y1), Point(x2, y2));
                //lines.push_back(std::make_tuple(seg, m,c));
                auto ox1 = odomPoints[0].first;
                auto oy1 = odomPoints[0].second;
                auto ox2 = odomPoints[odomPoints.size()-1].first;
                auto oy2 = odomPoints[odomPoints.size()-1].second;
                Segment odomSeg(Point(ox1, oy1), Point(ox2, oy2)); 
                joinLaneSegment(std::make_tuple(seg, m,c), std::make_tuple(odomSeg, std::get<0>(lineDetails), std::get<1>(lineDetails)));
            }
            
            // Delete the corresponding active_lane_segments.
            active_lane_segments.erase(active_lane_segments.begin()+j);
          }else{
            active_lane_segments[j] = std::make_tuple(laneSegment, inactiveCount, odomPoints);
          }
        }//else close

    }//second active lane segment for loop close
    
    for(size_t k=0; k<newPointsVec.size(); k++){
      auto point = newPointsVec[k]; auto x1 = point.first; auto y1 = point.second;
      std::vector<std::pair<double, double>> laneSegment;
      laneSegment.push_back(std::make_pair(x1, y1));
      std::vector<std::pair<double, double>> odomPoints;
      odomPoints.push_back(std::make_pair(vehicle_odom_double[vehicle_odom_double.size()-1].first, vehicle_odom_double[vehicle_odom_double.size()-1].second));
      active_lane_segments.push_back(std::make_tuple(laneSegment, 0, odomPoints));

    }
  }//else close
  
}

std::tuple<double, double, double> FeatureExtractor::linearRegression(std::vector<double> x, std::vector<double> y){
  //Linear regression based on least square method
  //Reference doc: https://www.mathsisfun.com/data/least-squares-regression.html  
  auto N = x.size();
  double sumXY = 0;
  double sumX = 0;
  double sumY = 0;
  double sumXSquared = 0;
  for(size_t i=0; i<N; i++){
    
    auto xy = x[i]*y[i];
    sumXY += xy;
    sumX += x[i];
    sumY += y[i];
    auto xSquared = std::pow(x[i], 2);
    sumXSquared += xSquared;
  }
  double m = 0.;
  double c = 0.; 
  m =  ((N*sumXY) - (sumX*sumY))/((N*sumXSquared)-(std::pow(sumX, 2)));
  c = (sumY-(m*sumX))/N; 
  //Mean squared error
  double mse = 0.;
  for(size_t i=0; i<N; i++){
    auto y_pred = m*x[i]+c; // y = mx+c form
    mse += std::pow((y[i] - y_pred),2);
  }
  mse = mse/N;

  return std::make_tuple(m, c, mse);
}

void FeatureExtractor::removeLineIntersects(){
  std::vector<size_t> eraseVec; 
  for(size_t i=0; i<allLines.size(); i++){
    auto lineSegVec = allLines[i];
    if(lineSegVec.size() == 1){
      auto takeSeg = lineSegVec[0];
      auto segTuple = takeSeg.first;
      auto odomSeg = takeSeg.second;
      auto odomM = std::get<1>(odomSeg);
      auto m = std::get<1>(segTuple);
      auto slopeDiff = std::abs(odomM-m);
      auto seg1 = std::get<0>(segTuple);
      double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
      double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);
      float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
      //ROS_INFO_STREAM("Length of the line segment: "<<d);
      //if(d < 5){
        for(size_t j=0; j<allLines.size(); j++){
          if(i==j)
            continue;
          auto lineSegVec2 = allLines[j];
          if(lineSegVec2.size() > 1){
             for(auto& segPair2: lineSegVec2){
              auto segTuple2 = segPair2.first;
              auto seg2 = std::get<0>(segTuple2);
              auto result = boost::geometry::intersects(seg1, seg2);
              if(result){
                eraseVec.push_back(i);
                break;
              }
            }
          }
        }
      //}
    }
  }
  
  for(size_t i=0; i<eraseVec.size(); i++){
    ROS_INFO_STREAM("Removing line: "<<eraseVec[i]);
    allLines.erase(allLines.begin()+eraseVec[i]);
  }

}

void FeatureExtractor::joinLaneSegment(std::tuple<Segment,double,double> newSeg, std::tuple<Segment,double,double> odomSeg){
  
  //ROS_INFO_STREAM("_______________________");
  //ROS_INFO_STREAM("activeLaneSeg size: "<<activeLaneSeg.size());
  
  if(activeLaneSeg.size() == 0){
    std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>> line;
    line.push_back(std::make_pair(newSeg, odomSeg));
    activeLaneSeg.push_back(std::make_pair(line,0));
  }else{
    bool found = false;
    std::vector<std::pair<double, double>> newLineVec;
    double smallD = 100000.;
    size_t smallIndex = 0;
    for(size_t i=0; i<activeLaneSeg.size(); i++){
      auto lineSegVec = activeLaneSeg[i].first;
      auto takeSeg = lineSegVec.back();
      auto lineSeg = takeSeg.first;
      auto seg1 = std::get<0>(lineSeg);
      auto m1 = std::get<1>(lineSeg);
      auto c1 = std::get<2>(lineSeg);
      double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
      double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);

      
      //Using the first principle, find y of the by inputting the x points from
      //the new segment.
      //Then project the active line seg to new point to create the new
      //extended active segment
      //Then use boost to check the extended active segment is intersect with the
      //new seg
      
      auto seg2 = std::get<0>(newSeg);
      auto m2 = std::get<1>(newSeg);
      double x3 = bg::get<0, 0>(seg2); double y3 = bg::get<0, 1>(seg2);
      double x4 = bg::get<1, 0>(seg2); double y4 = bg::get<1, 1>(seg2);
      auto slopeDiff = std::abs(m2-m1);
      auto y = m1*x4+c1;
      auto y_diff1 = std::abs(std::abs(y)-std::abs(y4));
      y = m2*x1+c1;
      auto y_diff2 = std::abs(std::abs(y)-std::abs(y1));
      float d = std::sqrt(std::pow((x4-x1),2)+std::pow((y4-y1),2));
      //ROS_INFO_STREAM("y_diff1: "<<y_diff1<<"y_diff2: "<<y_diff2<<"slopeDiff: "<<slopeDiff);
      if(slopeDiff < 0.05 && y_diff1 < 0.25){
        found = true;
        smallIndex = i;
        break;
      }
    }
    
    if(found){
      
      //One we found a connection between new line segement and existing active
      //lane segment we can draw the connection line between two roken line segements.
      //Sometime the line are joined incorrectly, in that case, we can say is
      //the difference between connected line and new line segment is greater
      //than a threshold, then the new line segment is not part of the current
      //active lane segment and it should be treated as new active lane
      //segment.
      auto lineSegVec = activeLaneSeg[smallIndex].first;
      auto takeSeg = lineSegVec.back();
      auto lastLineSegTuple = takeSeg.first; 
      auto seg1 = std::get<0>(lastLineSegTuple);
      auto m1  = std::get<1>(lastLineSegTuple);
      double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
      double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);
      auto seg3 = std::get<0>(newSeg);
      double x5 = bg::get<0, 0>(seg3); double y5 = bg::get<0, 1>(seg3);
      double x6 = bg::get<1, 0>(seg3);double y6 = bg::get<1, 1>(seg3);
      double x3 = x1; double y3 = y1;
      double x4 = x6; double y4 = y6;
      auto dx = x4 - x3;auto dy = y4 - y3;
      if(dx != 0 && dy != 0){
        auto m2 = dy / dx;auto c2 = y3 - m2 * x3;
        auto slopeDiff = std::abs(m2-m1);
        if(slopeDiff < 0.1){
          //Slope diff between two lines are less than threshold, it should be
          //added to the current active lane segment.
          Segment seg2(Point(x3, y3), Point(x4, y4));
          lineSegVec.push_back(std::make_pair(std::make_tuple(seg2, m2, c2), odomSeg));
          lineSegVec.push_back(std::make_pair(newSeg, odomSeg));
          
        }else{
          //Slope diff is greater than threshold new line segment should be
          //treated as new actibe lane segment.
          std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>> line;
          line.push_back(std::make_pair(newSeg,odomSeg));
          activeLaneSeg.push_back(std::make_pair(line,0));
        }
      }else{
        lineSegVec.push_back(std::make_pair(newSeg, odomSeg));
      }
      
      activeLaneSeg[smallIndex] = std::make_pair(lineSegVec, 0);
    }else{
      std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>> line;
      line.push_back(std::make_pair(newSeg, odomSeg));
      activeLaneSeg.push_back(std::make_pair(line,0));
    }

    for(size_t i=0;i<activeLaneSeg.size();i++){
      if(i == smallIndex)
        continue;
      else{
        auto lineSegVec = activeLaneSeg[i].first;
        auto inactive = activeLaneSeg[i].second;
        inactive += 1;
        //ROS_INFO_STREAM("Inactive: "<<inactive);
        if(inactive > 5){
          allLines.push_back(lineSegVec);
          activeLaneSeg.erase(activeLaneSeg.begin()+i);
          laneSCount++;
        }else{
          activeLaneSeg[i] = std::make_pair(lineSegVec, inactive);
        }
      }
    }// second active lane seg for loop close
  }//main else close
  
  if(laneSCount !=0 && laneSCount%5 == 0 ){
    ROS_INFO_STREAM("_______________________: "<<laneSCount);
    ROS_INFO_STREAM("allLines size: "<<allLines.size());
    joinLinesFurther(0.05, 0.1, 0.1, 20.);
    //removeNoiseLines();
  }
  
  if(laneSCount !=0 && laneSCount%5 == 0 ){
    ROS_INFO_STREAM("_______________________: "<<laneSCount);
    ROS_INFO_STREAM("allLines size: "<<allLines.size());
    joinLinesFurther(0.05, 0.1, 0.1, 20.);
  }
  
  if(laneSCount !=0 && laneSCount%5 == 0 ){
    ROS_INFO_STREAM("_______________________: "<<laneSCount);
    ROS_INFO_STREAM("allLines size: "<<allLines.size());
    joinLinesFurther(0.05, 0.1, 0.1, 30.);
  }

  if(laneSCount !=0 && laneSCount%5 == 0 ){
    ROS_INFO_STREAM("_______________________: "<<laneSCount);
    ROS_INFO_STREAM("allLines size: "<<allLines.size());
    joinLinesFurther(0.05, 0.1, 0.1, 40.);
  }

  if(laneSCount !=0 && laneSCount%10 == 0 ){
    ROS_INFO_STREAM("****************************: "<<laneSCount);
    ROS_INFO_STREAM("allLines size: "<<allLines.size());
    joinLinesFurther(0.1, 0.5, 0.5, 30);
    //removeLineIntersects();
  }
}

void FeatureExtractor::joinLinesFurther(double slopeDiffT, double yDiff1T, double yDiff2T, double dT){
  std::vector<size_t> activeList;
  for(size_t i=0; i<allLines.size(); i++){
     if(std::find(activeList.begin(), activeList.end(), i) != activeList.end())
      continue;
    auto lineSegVec = allLines[i];
    auto takeSeg = lineSegVec.back();
    auto segTuple = takeSeg.first;
    auto seg1 = std::get<0>(segTuple);auto m1 = std::get<1>(segTuple);
    auto c1 = std::get<2>(segTuple);
    double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
    double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);
    bool found = false;
    size_t index;
    double smallDiff = 10000.;
    for(size_t j=0; j<allLines.size(); j++){
      if(i==j || std::find(activeList.begin(), activeList.end(), j) != activeList.end())
        continue;
      auto anotherLineSegVec = allLines[j];
      auto takeSeg2 = anotherLineSegVec.back();
      auto anotherSegTuple = takeSeg2.first;
      auto seg2 = std::get<0>(anotherSegTuple);
      auto m2 = std::get<1>(anotherSegTuple);
      auto c2 = std::get<2>(anotherSegTuple);
      auto slopeDiff = std::abs(m2-m1);
      if(slopeDiff < slopeDiffT){
        double x3 = bg::get<0, 0>(seg2); double y3 = bg::get<0, 1>(seg2);
        double x4 = bg::get<1, 0>(seg2);double y4 = bg::get<1, 1>(seg2);
        auto y = m1*x4+c1;
        auto y_diff1 = std::abs(std::abs(y)-std::abs(y4));
        y = m2*x1+c2;
        auto y_diff2 = std::abs(std::abs(y)-std::abs(y1));
        float d = std::sqrt(std::pow((x4-x1),2)+std::pow((y4-y1),2));
        //ROS_INFO_STREAM("slopeDiff: "<<slopeDiff<<" y_diff1: "<<y_diff1<<" y_diff2: "<<y_diff2);
        if(y_diff1 < yDiff1T){
          found = true;
          index = j;
          break;
        }else if(y_diff2 < yDiff2T){
          found = true;
          index = j;
          break;
        }
      }
    }
  
    if(found){
      std::vector<Segment> tempSegments;
      auto segVec = allLines[index];
      auto takeSeg3 = segVec.back();
      auto segTuple3 = takeSeg3.first;
      auto odomSeg3 = takeSeg3.second;
      auto seg3 = std::get<0>(segTuple3);
      double x5 = bg::get<0, 0>(seg3); double y5 = bg::get<0, 1>(seg3);
      double x6 = bg::get<1, 0>(seg3);double y6 = bg::get<1, 1>(seg3);
      double x3New = x1; double y3New = y1;
      double x4New = x6; double y4New = y6;
      float d = std::sqrt(std::pow((x4New-x3New),2)+std::pow((y4New-y3New),2));
      if(d<dT){
        auto dx = x4New - x3New;auto dy = y4New - y3New;
        if(dx != 0 && dy != 0){
            auto m2New = dy / dx;auto c2New = y3New - m2New * x3New;
            auto odomM = std::get<1>(odomSeg3);
            auto slopeDiff = std::abs(odomM-m2New);
            ROS_INFO_STREAM("slopeDiff: "<<slopeDiff);
            Segment seg2New(Point(x3New, y3New), Point(x4New, y4New));
            lineSegVec.push_back(std::make_pair(std::make_tuple(seg2New, m2New, c2New), odomSeg3));
            for(size_t k=0; k<segVec.size(); k++){
              lineSegVec.push_back(segVec[k]);
            }
        }
        activeList.push_back(i);
        activeList.push_back(index);
        allLines[i] = lineSegVec;
        allLines.erase(allLines.begin()+index);
      }
    }
  }
}

void FeatureExtractor::removeNoiseLines(){
  for(size_t i=0; i<allLines.size(); i++){
    auto lineSegVec = allLines[i];
    if(lineSegVec.size() == 1){
      auto takeSeg = lineSegVec[0];
      auto segTuple = takeSeg.first;
      auto odomSeg = takeSeg.second;
      auto odomM = std::get<1>(odomSeg);
      auto m = std::get<1>(segTuple);
      auto slopeDiff = std::abs(odomM-m);
      auto seg1 = std::get<0>(segTuple);
      double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
      double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);
      float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
      //ROS_INFO_STREAM("Length of the line segment: "<<d);
      if(d < 1.25){
        allLines.erase(allLines.begin()+i);
      }
    }
  }
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
    
    pcl::PointCloud<pcl::PointXYZIR>::Ptr input_pointcloud_bl(new pcl::PointCloud<pcl::PointXYZIR>);
    pcl::copyPointCloud(*input_pointcloud_box_filter, *input_pointcloud_bl);
    pcl::transformPointCloud(*input_pointcloud, *input_pointcloud, world_origin, world_rotation);

    int x_index = (world_origin[0] * 100.) / cm_resolution;
    int y_index = (world_origin[1] * 100.) / cm_resolution;
    vehicle_odom.push_back(std::make_pair(-1. * y_index, x_index));
    vehicle_odom_double.push_back(std::make_pair(world_origin[0], world_origin[1]));

    double z_min = 100000000.;
    double z_max  = -100000000.;
       
    if(input_pointcloud_bl->points.size() > 0){
      auto extractedPC = extractEdges(input_pointcloud_bl, pointcloud_msg->header.stamp.sec, pointcloud_msg->header.stamp.nsec);
      
      // Filtering on baselink
      auto lanePC = pointCloudFilter(extractedPC);
      
      // Transfroming to odom frame
      pcl::transformPointCloud(lanePC, lanePC, world_origin, world_rotation);
      
      // Construct lane segement
      constructLaneSegments(lanePC); 
      
      for(size_t i=0; i<lanePC.points.size(); i++){		
        auto x = lanePC.points[i].x;
        auto y = lanePC.points[i].y;
        int x_index = (x * 100.) / cm_resolution;
        int y_index = (y * 100.) / cm_resolution;
        lane_points.push_back(std::make_pair(-1. * y_index, x_index));
        lane_odom.push_back(std::make_pair(x, y));
      }
      seq_count += 1;
    }
    
    // put each point into the intensity map
    for (size_t i = 0; i < input_pointcloud->points.size(); ++i) {
      int x_index = (input_pointcloud->points[i].x * 100.) / cm_resolution;
      int y_index = (input_pointcloud->points[i].y * 100.) / cm_resolution;

      if (fabs(x_index) > 1e7 || fabs(y_index) > 1e7) {
        continue;
      }
     
      intensity_topic[std::make_pair(-1. * y_index,
                                     x_index)] = input_pointcloud->points[i].intensity; // std::map<std::pair<int,int>, double>
    }


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

