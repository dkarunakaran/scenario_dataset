#ifndef APPLICATIONS_HELPER_FUNCTIONS_HPP
#define APPLICATIONS_HELPER_FUNCTIONS_HPP

using json = nlohmann::json;

json constructJsonData(nav_msgs::Odometry::ConstPtr dataPtr, std::shared_ptr<tf2_ros::Buffer> transformer_){
  json jData;
  tf::Quaternion q(
    dataPtr->pose.pose.orientation.x,
    dataPtr->pose.pose.orientation.y,
    dataPtr->pose.pose.orientation.z,
    dataPtr->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  jData["linear_x"] = dataPtr->twist.twist.linear.x;
  jData["linear_y"] = dataPtr->twist.twist.linear.y;
  jData["linear_z"] = dataPtr->twist.twist.linear.z;
  jData["position_x"] = dataPtr->pose.pose.position.x;
  jData["position_y"] = dataPtr->pose.pose.position.y;
  jData["position_z"] = dataPtr->pose.pose.position.z;
  jData["roll"] = roll;
  jData["pitch"] = pitch;
  jData["yaw"] = yaw;
  jData["sec"] = dataPtr->header.stamp.sec;

  return jData;
}

json constructJsonData(ibeo_object_msg::IbeoObject::ConstPtr dataPtr, std::shared_ptr<tf2_ros::Buffer>& transformer_){
  json jData;
  tf::Quaternion q(
    dataPtr->pose.pose.orientation.x,
    dataPtr->pose.pose.orientation.y,
    dataPtr->pose.pose.orientation.z,
    dataPtr->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  jData["object_id"] = dataPtr->object_id;
  jData["linear_x"] = dataPtr->twist.twist.linear.x;
  jData["linear_y"] = dataPtr->twist.twist.linear.y;
  jData["linear_z"] = dataPtr->twist.twist.linear.z;
  jData["position_x"] = dataPtr->pose.pose.position.x;
  jData["position_y"] = dataPtr->pose.pose.position.y;
  jData["position_z"] = dataPtr->pose.pose.position.z;
  jData["roll"] = roll;
  jData["pitch"] = pitch;
  jData["yaw"] = yaw;
  jData["sec"] = dataPtr->header.stamp.sec;

  return jData;
}


bool checkSecExist(std::vector<uint32_t> timeStamp, uint32_t sec){
  bool _return = false;
  if(std::find(timeStamp.begin(), timeStamp.end(), sec) != timeStamp.end())  {
    _return = true;
  }
  
  return _return;
}

int findIndex(std::vector<uint32_t> v, uint32_t sec){
  int _return = -1;  
  auto it = std::find(v.begin(), v.end(), sec);
 
  // If element was found
  if (it != v.end())
    _return = it - v.begin();

  return _return;
}

std::tuple<bool, lanelet::Lanelet, std::vector<lanelet::Lanelet>> findTheActualLanelet(lanelet::LaneletMapPtr map, lanelet::BasicPoint2d egoPoint){
  std::vector<std::pair<double, lanelet::Lanelet>> nearLanelets = lanelet::geometry::findNearest(map->laneletLayer, egoPoint, 10);
  bool llFound = false;
  size_t selected = 0;
  std::vector<lanelet::Lanelet> lanelets;
  for(size_t i=0; i<nearLanelets.size(); i++){
    auto ll = nearLanelets[i];
    if(lanelet::geometry::inside(ll.second, egoPoint)){
      llFound = true;
      selected = i;
      break;
    }
  }

  for(size_t i=0; i<nearLanelets.size(); i++){
    if(llFound && i == selected)
      continue;
    else{
      lanelets.push_back(nearLanelets[i].second);
    }
  }
  
  if(llFound)
    return std::make_tuple(true, nearLanelets[selected].second, lanelets);
  else{
    lanelet::Lanelet tempLL;
    return std::make_tuple(true, tempLL, lanelets);
  }
}

std::pair<bool,lanelet::Lanelet> findTheLeftLanelet(lanelet::Lanelet checkLanelet, std::vector<lanelet::Lanelet> lanelets){
  lanelet::Lanelet lanelet;
  std::pair<bool, lanelet::Lanelet> _return = std::make_pair(false, lanelet);
  for(size_t i=0; i<lanelets.size(); i++){
    if(lanelet::geometry::leftOf(lanelets[i], checkLanelet)){
      _return = std::make_pair(true, lanelets[i]);
      break;
    }
  }
  
  return _return;
}

std::pair<bool,lanelet::Lanelet> findTheRightLanelet(lanelet::Lanelet checkLanelet, std::vector<lanelet::Lanelet> lanelets){
  lanelet::Lanelet lanelet;
  std::pair<bool, lanelet::Lanelet> _return = std::make_pair(false, lanelet);
  for(size_t i=0; i<lanelets.size(); i++){
    if(lanelet::geometry::rightOf(lanelets[i], checkLanelet)){
      _return = std::make_pair(true, lanelets[i]);
      break;
    }
  }
  
  return _return;
}


std::pair<int,int> findTheNumberOfLanesAndCarLane(lanelet::LaneletMapPtr map, lanelet::BasicPoint2d startingPoint){
  std::pair<int, int> _return = std::make_pair(0, -1);// number of lanes, position of the car
  auto nearLanelets = findTheActualLanelet(map, startingPoint);
  if(std::get<0>(nearLanelets) && lanelet::geometry::length3d(std::get<1>(nearLanelets)) > 0){
    auto firstLanelet = std::get<1>(nearLanelets);
    auto currentLanelet = firstLanelet;
    auto lanelets = std::get<2>(nearLanelets);
    std::vector<std::pair<int, lanelet::Lanelet>> selectedLanelets;
    int count = 0;
    //Find all the left lanelets
    while(true){
      auto result = findTheLeftLanelet(currentLanelet, lanelets);
      if(result.first){
        selectedLanelets.push_back(std::make_pair(count, result.second));
        currentLanelet = result.second;
        lanelet::ConstLineString3d centerline = currentLanelet.centerline();
        int midSegNo = centerline.numSegments()/2;
        auto midSeg = centerline.segment(midSegNo);
        auto point1 = midSeg.first;auto point2 = midSeg.second;
        auto midX = (point1.x()+point2.x())/2; auto midY = (point1.y()+point2.y())/2;
        auto point = lanelet::BasicPoint2d(midX, midY);
        auto otherLanelets = findTheActualLanelet(map, point);
        lanelets = std::get<2>(otherLanelets);
        count++;
      }else
        break;
    }
    std::sort(selectedLanelets.begin(), selectedLanelets.end(), [](const std::pair<int, lanelet::Lanelet>& a, std::pair<int, lanelet::Lanelet>& b) {return a.first < b.first;});
    std::vector<lanelet::Lanelet> allLanelets;
    for(auto& pair: selectedLanelets)
      allLanelets.push_back(pair.second);
    
    int carLane = selectedLanelets.size();
    allLanelets.push_back(firstLanelet);

    //Find all the right lanelets
    currentLanelet = firstLanelet;
    lanelets = std::get<2>(nearLanelets);
    count = 0;
    selectedLanelets.clear();
    while(true){
      auto result = findTheRightLanelet(currentLanelet, lanelets);
      if(result.first){
        allLanelets.push_back(result.second);
        currentLanelet = result.second;
        lanelet::ConstLineString3d centerline = currentLanelet.centerline();  
        int midSegNo = centerline.numSegments()/2;
        auto midSeg = centerline.segment(midSegNo);
        auto point1 = midSeg.first;auto point2 = midSeg.second;
        auto midX = (point1.x()+point2.x())/2; auto midY = (point1.y()+point2.y())/2;
        auto point = lanelet::BasicPoint2d(midX, midY);
        auto otherLanelets = findTheActualLanelet(map, point);
        lanelets = std::get<2>(otherLanelets);
        count++;
      }else
        break;
    }

    _return = std::make_pair(allLanelets.size(), carLane);

  
  }//Checking lanelet for the egopoint if closes      
  else{
    //ROS_INFO_STREAM("No lanelet found: length of the lanelet is zero or simply couldn't find the lanelet that actually has the point");
  }

  return _return;
}

void groupObjDataBySec(json jData, std::vector<std::pair<uint32_t, std::vector<json>>>& objectsPerSec){
  bool found = false;
  size_t index = 0;
  for(size_t i=0; i<objectsPerSec.size(); i++){
    auto pair = objectsPerSec[i];
    if(pair.first == jData["sec"].get<uint32_t>()){
      found = true;
      index = i;
    }
  }
  if(found){
    auto pair = objectsPerSec[index]; 
    auto vec = pair.second;
    vec.push_back(jData);
    objectsPerSec[index] = std::make_pair(pair.first, vec);
  }else{
    std::vector<json> tempVec; tempVec.push_back(jData);
    objectsPerSec.push_back(std::make_pair(jData["sec"].get<uint32_t>(), tempVec));
  }
}

void groupObjDataByCar(json jData, std::vector<std::pair<int, std::vector<json>>>& groupByCar){
  bool found = false;
  size_t index = 0;
  for(size_t i=0; i<groupByCar.size(); i++){
    auto pair = groupByCar[i];
    if(pair.first == jData["object_id"].get<int>()){
      found = true;
      index = i;
    }
  }
  if(found){
    auto pair = groupByCar[index]; 
    auto vec = pair.second;
    bool proceed = true;
    for(size_t i=0; i<vec.size(); i++){
      auto j = vec[i];
      if(j["sec"] == jData["sec"]){
        proceed = false;
      }
    }
    if(proceed){
      vec.push_back(jData);
      groupByCar[index] = std::make_pair(pair.first, vec);
    }
  }else{
    std::vector<json> tempVec; tempVec.push_back(jData);
    groupByCar.push_back(std::make_pair(jData["object_id"].get<int>(), tempVec));
  }
}

void groupObjects(std::vector<json> objectsPos, std::vector<std::pair<uint32_t, std::vector<json>>>& groupBySec, std::vector<std::pair<int, std::vector<json>>>& groupByCar){
  for(auto& item: objectsPos){
    groupObjDataBySec(item, groupBySec);
    groupObjDataByCar(item, groupByCar);
  }
}

std::vector<json> getAllTheCarsAtSec(std::vector<std::pair<uint32_t, std::vector<json>>> groupBySec, uint32_t sec){
 
  std::vector<json> carsAtSec;
  for(auto& pair: groupBySec){
    if(pair.first == sec){
      carsAtSec = pair.second;
    }
  }
  
  return carsAtSec;
}

std::vector<json> getAllTheDataAtCar(std::vector<std::pair<int, std::vector<json>>> groupByCar, int car){
 
  std::vector<json> dataAtCar;
  for(auto& pair: groupByCar){
    if(pair.first == car){
      dataAtCar = pair.second;
    }
  }
  
  return dataAtCar;
}



#endif //APPLICATIONS_HELPER_FUNCTIONS_HPP
