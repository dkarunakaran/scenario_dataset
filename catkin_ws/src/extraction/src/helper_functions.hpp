#ifndef APPLICATIONS_HELPER_FUNCTIONS_HPP
#define APPLICATIONS_HELPER_FUNCTIONS_HPP

using json = nlohmann::json;

json constructJsonData(nav_msgs::Odometry::ConstPtr dataPtr){
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

json constructJsonData(ibeo_object_msg::IbeoObject::ConstPtr dataPtr){
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

#endif //APPLICATIONS_HELPER_FUNCTIONS_HPP
