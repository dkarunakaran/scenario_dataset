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

bool ReadBag::checkRegionOfInterest(std::pair<int,int> item, int min_x, int min_y, std::list<std::pair<int, int>> vehicle_odom){
  
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


void ReadBag::process(const lane_points_msg::LanePoints::ConstPtr &msg){
	
	// set the search values to extreme (to be overwritten by the correct values)
  	int max_x = -100000000, max_y = -100000000, min_x = 100000000, min_y = 100000000;
  	float min_intensity = 100000000.;
  	float max_intensity = -100000000.;
	x_array.data.clear();
	y_array.data.clear();
	i_array.data.clear();
	std::vector<int> x_point;
	std::vector<int> y_point;
	std::vector<int> i_point;
	std::vector<int> odom_x_point;
	std::vector<int> odom_y_point;
	
	//ZVehicle ODOM data
	for(std::vector<int>::const_iterator it = msg->odom_x.data.begin(); it != msg->odom_x.data.end(); ++it)
        {
		odom_x_point.push_back(*it);
	}

	for(std::vector<int>::const_iterator it = msg->odom_y.data.begin(); it != msg->odom_y.data.end(); ++it)
        {
                odom_y_point.push_back(*it);
	}
	
	for(std::size_t i = 0; i <odom_x_point.size(); ++i) {
		vehicle_odom.push_back(std::make_pair(odom_y_point[i], odom_x_point[i]));
		
	}


	//Point x & y and intensity, i	
	for(std::vector<int>::const_iterator it = msg->x.data.begin(); it != msg->x.data.end(); ++it)
        {
		x_point.push_back(*it);
	}
	for(std::vector<int>::const_iterator it = msg->y.data.begin(); it != msg->y.data.end(); ++it)
        {
                y_point.push_back(*it);
		//ROS_INFO_STREAM(*it);
        }
	
	for(std::vector<float>::const_iterator it = msg->i.data.begin(); it != msg->i.data.end(); ++it)
        {
                i_point.push_back(*it);
        }
	
	

	// Using a for loop with index
	for(std::size_t i = 0; i < x_point.size(); ++i) {
		min_x = std::min<int>(min_x, x_point[i]);
     		max_x = std::max<int>(max_x, x_point[i]);
      		min_y = std::min<int>(min_y, y_point[i]);
      		max_y = std::max<int>(max_y, y_point[i]);
      		min_intensity = std::min<double>(min_intensity, i_point[i]);
      		max_intensity = std::max<double>(max_intensity, i_point[i]);
	}

	ROS_INFO_STREAM(max_x<<" "<<max_y);
	ROS_INFO_STREAM(x_point.size()<<" "<<y_point.size()<<" "<<i_point.size());

	int image_max_x = max_x - min_x + 2;
 	int image_max_y = max_y - min_y + 2;
	
	lane_points_msg::LanePoints lanePointsMsg;
  	lanePointsMsg.header.seq = seq_count;
   	lanePointsMsg.header.stamp.sec = msg->header.stamp.sec;
   	lanePointsMsg.header.stamp.nsec = 0;
   	lanePointsMsg.header.frame_id = "lane_points";
   	lanePointsMsg.max_x = image_max_x;
   	lanePointsMsg.max_y = image_max_y;
	for(int i=0; i<x_point.size(); i++){
		//Intensity filtering code
		int value_x = x_point[i] - min_x;
		int value_y = y_point[i] - min_y;
		if (value_x < 0 || value_x >= image_max_x) {
			continue;
		}

		if (value_y < 0 || value_y >= image_max_y) {
			continue;
		}

		cv::Vec4b colour;
		// scale the intensity value to be between 0 and 1
		double intensity = (i_point[i] - min_intensity) / (max_intensity - min_intensity);

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
		  bool status = this->checkRegionOfInterest(std::make_pair(value_x, value_y), min_x, min_y, vehicle_odom);
		  
		  status = true;
		  
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
	lanePointsMsg.x = x_array;
        lanePointsMsg.y = y_array;
	lanePointsMsg.i = i_array;
	lanePointsPerSecPub.publish(lanePointsMsg);
	seq_count++;	
        
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
