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
  //auto world_transform = transformer_->lookupTransform(std::string("base_link_horizon"), std::string("base_link"), dataPtr->header.stamp);
 /* Eigen::Quaternionf world_rotation(world_transform.transform.rotation.w,
                                      world_transform.transform.rotation.x,
                                      world_transform.transform.rotation.y,
                                      world_transform.transform.rotation.z);

  Eigen::Vector3f world_origin(world_transform.transform.translation.x,
                                 world_transform.transform.translation.y,
                                 world_transform.transform.translation.z);
  
  pcl::PointCloud<pcl::PointXYZ> pointCloud;
  pcl::PointXYZ point;
  point.x = dataPtr->pose.pose.position.x;
  point.y = dataPtr->pose.pose.position.y;
  point.z = dataPtr->pose.pose.position.z;
  pointCloud.push_back(point);
  pcl::transformPointCloud(pointCloud, pointCloud, world_origin, world_rotation);
*/

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
  auto it = find(v.begin(), v.end(), sec);
 
  // If element was found
  if (it != v.end())
    _return = it - v.begin();

  return _return;
}

std::tuple<bool, lanelet::Lanelet, std::vector<lanelet::Lanelet>> findTheActualLanelet(lanelet::LaneletMapPtr map, lanelet::BasicPoint2d egoPoint){
  std::vector<std::pair<double, lanelet::Lanelet>> nearLanelets = lanelet::geometry::findNearest(map->laneletLayer, egoPoint, 4);
  bool llFound = false;
  size_t selected = 0;
  std::vector<lanelet::Lanelet> lanelets;
  for(size_t i=0; i<nearLanelets.size(); i++){
    auto ll = nearLanelets[i];
    lanelets.push_back(ll.second);
    if(lanelet::geometry::inside(ll.second, egoPoint)){
      llFound = true;
      selected = i;
      break;
    }
  }
  
  if(llFound)
    return std::make_tuple(true, nearLanelets[selected].second, lanelets);
  else{
    lanelet::Lanelet tempLL;
    return std::make_tuple(true, tempLL, lanelets);
  }
}


#endif //APPLICATIONS_HELPER_FUNCTIONS_HPP
