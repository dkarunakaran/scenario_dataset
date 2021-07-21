#!/usr/bin/env python

from __future__ import division
import rospy
from nav_msgs.msg import Odometry
import json
from rss import RSS
from ibeo_object_msg.msg import IbeoObject
import tf
import math
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf.transformations import quaternion_matrix
import tf2_geometry_msgs
import rosbag
from std_msgs.msg import String

# For Waymo dataset
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class Extract:
    ObjectClass_Unclassified = 0
    ObjectClass_UnknownSmall = 1
    ObjectClass_UnknownBig = 2
    ObjectClass_Pedestrian = 3
    ObjectClass_Bike = 4
    ObjectClass_Car = 5
    ObjectClass_Truck = 6
    ObjectClass_Underdriveable = 12
    ObjectClass_Motorbike = 15
    ObjectClass_Bicycle = 17
    waymoObjectClass = ['Pedestrian', 'Vehicle', 'Sign', 'Cyclist']
    
    def __init__(self):
        rospy.init_node("Extract class initilization")
        self.path_extracted_result = rospy.get_param('result')
        self.name_of_the_file = None
        self.ego_odom  = []
        self.other_cut_in_objects = []
        self.overtaking = []
        self.previous = None
        self.previous_other = None
        self.count = 0
        self.tracked_objects = {}
        self.closest_cars = {}
        self.extracted_data = {
            'ego_odom': None,
            'other_objects': None
        }
        self.dataset = 'ibeo' #ibeo, waymo
        self.ego_sec_track = []
        self.object_class = []
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        print(self.dataset)
        print(self.path_extracted_result)
        if self.dataset is 'ibeo':
            rospy.Subscriber('/ibeo/odometry', Odometry, self.ego_odometry)    
            rospy.Subscriber('/ibeo/objects', IbeoObject, self.objects)
        elif self.dataset is 'waymo':
            #rospy.Subscriber('/lidar_label', MarkerArray, self.objects) 
            rospy.Subscriber('/lidar_label/markers', MarkerArray, self.objects) 
            #rospy.Subscriber('/tf', TFMessage, self.ego_odometry)
            
        # Detecting the lane coordinates. This will help us to find out where the vehicle is,
        # If we have cordinate of the lanes then, it could be easy to quaery where is the veicle at given point.     
        rospy.on_shutdown(self.shutdown)   
        rospy.spin()
     
    def ego_odometry(self, data):
        if self.dataset is 'ibeo':
            quaternion = (
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w
            )
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
            t = data.header.stamp
            #t = t.to_nsec()
            msg = {
                'linear_x': data.twist.twist.linear.x,
                'linear_y': data.twist.twist.linear.y,
                'linear_z': data.twist.twist.linear.z,
                'position_x': data.pose.pose.position.x,
                'position_y': data.pose.pose.position.y,
                'position_z': data.pose.pose.position.z,
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw,
                'sec': data.header.stamp.secs,
                'nsec': data.header.stamp.nsecs,
                'to_sec': t.to_sec(),
                'to_nsec': t.to_nsec()
                        
            }
            self.ego_odom.append(msg)
        elif self.dataset is 'waymo':
            pass
             
    def objects(self, data):
        if self.dataset is 'ibeo':
            
            # Considering only vehicle classes 
            if data.object_class in [self.ObjectClass_Car, self.ObjectClass_Motorbike, self.ObjectClass_Truck]:
                try:
                    # We need to convert this to tf2_geometry_msgs as ibeo_object is a custom object type.
                    pose_stamped = tf2_geometry_msgs.PoseStamped()
                    pose_stamped.pose = data.pose.pose
                    pose_stamped.header.frame_id = "base_link"
                    pose_stamped.header.stamp = data.header.stamp
                    
                    # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
                    data_odom = self.tf_buffer.transform(pose_stamped, "odom", timeout=rospy.Duration(0.0))
                    #print(data_odom)
                    quaternion = (
                        data_odom.pose.orientation.x,
                        data_odom.pose.orientation.y,
                        data_odom.pose.orientation.z,
                        data_odom.pose.orientation.w
                    )
                    euler = tf.transformations.euler_from_quaternion(quaternion)
                    roll = euler[0]
                    pitch = euler[1]
                    yaw = euler[2]
                    t = data_odom.header.stamp
                    msg = {
                        'object_id': data.object_id,
                        'linear_x': data.twist.twist.linear.x,
                        'linear_y': data.twist.twist.linear.y,
                        'linear_z': data.twist.twist.linear.z,
                        'position_x': data_odom.pose.position.x,
                        'position_y': data_odom.pose.position.y,
                        'position_z': data_odom.pose.position.z,
                        'rel_pos_x': data.pose.pose.position.x,
                        'rel_pos_y': data.pose.pose.position.y,
                        'roll': roll,
                        'pitch': pitch,
                        'yaw': yaw,
                        'sec': data_odom.header.stamp.secs,
                        'nsec': data_odom.header.stamp.nsecs,
                        'to_sec': t.to_sec(),
                        'to_nsec': t.to_nsec()
                    }
                
                    #Considering only vehicle at the front up to 100 meter
                    if data.pose.pose.position.x >= 0 and data.pose.pose.position.x <= 100:
                        self.other_cut_in_objects.append(msg)
                    elif data.pose.pose.position.x < 0 and data.pose.pose.position.x >= -50:
                        self.overtaking.append(msg)
                        
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass
                        
                    
        elif self.dataset is 'waymo':
            
            #Getting velocity: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
            
            #Get Ego odom
            try:
                trans_stamp = self.tf_buffer.lookup_transform('global', 'vehicle', rospy.Time())
                quaternion = (
                    trans_stamp.transform.rotation.x,
                    trans_stamp.transform.rotation.y,
                    trans_stamp.transform.rotation.z,
                    trans_stamp.transform.rotation.w
                )
                euler = tf.transformations.euler_from_quaternion(quaternion)
                roll = euler[0]
                pitch = euler[1]
                yaw = euler[2]
                t = trans_stamp.header.stamp
                msg = {
                    'position_x': trans_stamp.transform.translation.x,
                    'position_y': trans_stamp.transform.translation.y,
                    'position_z': trans_stamp.transform.translation.z,
                    'roll': roll,
                    'pitch': pitch,
                    'yaw': yaw,
                    'sec': trans_stamp.header.stamp.secs,
                    'nsec': trans_stamp.header.stamp.nsecs,
                    'to_sec': t.to_sec(),
                    'to_nsec': t.to_nsec()
                    
                }
                self.ego_odom.append(msg)
            
                
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        pass
    
            # For storing the other objects
            for marker in data.markers:
                #Considering only vehicle at the front up to 100 meter
                if (marker.pose.position.x > 0 and marker.pose.position.x <= 100):
                    self.object_class.append(marker.ns)
                    # We need to convert this to tf2_geometry_msgs as ibeo_object is a custom object type.
                    pose_stamped = tf2_geometry_msgs.PoseStamped()
                    pose_stamped.pose = marker.pose
                    pose_stamped.header.frame_id = "vehicle"
                    pose_stamped.header.stamp = marker.header.stamp
                    try:
                        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
                        data_odom = self.tf_buffer.transform(pose_stamped, "global")
                        #print(data_odom)
                        quaternion = (
                            data_odom.pose.orientation.x,
                            data_odom.pose.orientation.y,
                            data_odom.pose.orientation.z,
                            data_odom.pose.orientation.w
                        )
                        euler = tf.transformations.euler_from_quaternion(quaternion)
                        roll = euler[0]
                        pitch = euler[1]
                        yaw = euler[2]
                        t = data_odom.header.stamp
                        msg = {
                            'object_id': marker.id,
                            'ns': marker.ns,
                            'position_x': data_odom.pose.position.x,
                            'position_y': data_odom.pose.position.y,
                            'position_z': data_odom.pose.position.z,
                            'rel_pos_x': marker.pose.position.x,
                            'rel_pos_y': marker.pose.position.y,
                            'roll': roll,
                            'pitch': pitch,
                            'yaw': yaw,
                            'sec': data_odom.header.stamp.secs,
                            'nsec': data_odom.header.stamp.nsecs,
                            'to_sec': t.to_sec(),
                            'to_nsec': t.to_nsec()

                        }
                        self.other_cut_in_objects.append(msg)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        pass
            
    def shutdown(self):
        print("Extract node is shutting down!!!")
        
        self.extracted_data = {
            'ego_odom': self.ego_odom,
            'other_objects': {
                'cut-in': self.other_cut_in_objects,
                'overtaking': self.overtaking
            }
        }
        
        #print(self.extracted_data['other_objects'])
        
        print(list(set(self.object_class)))
        
        # Save the json file
        with open(self.path_extracted_result+".txt", 'w') as outfile:
            json.dump(self.extracted_data, outfile)
    

if __name__ == '__main__':
    try:
        Extract()
    except rospy.ROSInterruptException:
	    rospy.logerr('Could not start Extract node.')
