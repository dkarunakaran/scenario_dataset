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
    def __init__(self):
        rospy.init_node("Extract class initilization")
        #self.bag = rosbag.Bag(rospy.get_param("/bag_file"))
        self.ego_odom  = []
        self.ibeo_object = []
        self.previous = None
        self.previous_other = None
        self.count = 0
        self.tracked_objects = {}#TrackObjects()
        self.closest_cars = {}
        self.rss = RSS()
        self.extracted_data = {
            'ego_odom': None,
            'ibeo_objects': None
        }
        
        self.f_g = open('/constraint_model/data/stamped_groundtruth.txt', 'w')
        self.f_g.write('# timestamp tx ty tz qx qy qz qw\n')
        self.f_e = open('/constraint_model/data/stamped_traj_estimate.txt', 'w')
        self.f_e.write('# timestamp tx ty tz qx qy qz qw\n')
        
        rospy.Subscriber('/ibeo/odometry', Odometry, self.ego_odometry)    
        rospy.Subscriber('/ibeo/objects', IbeoObject, self.ibeo_objects)  
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        
        # Sepearte the other_object data based on each car_id
        # plot the trajectory of each car.
        
        rospy.on_shutdown(self.shutdown)   
        rospy.spin()
        
    def ego_odometry(self, data):
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
            'pitch': pitch,
            'yaw': yaw,
            'sec': data.header.stamp.secs,
            'nsec': data.header.stamp.nsecs,
            'to_sec': t.to_sec(),
            'to_nsec': t.to_nsec()
                        
        }
        self.ego_odom.append(msg)
        
        self.f_g.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' % 
                       (data.header.stamp.to_sec(),
                        data.pose.pose.position.x, data.pose.pose.position.y,
                        data.pose.pose.position.z,
                        data.pose.pose.orientation.x,
                        data.pose.pose.orientation.y,
                        data.pose.pose.orientation.z,
                        data.pose.pose.orientation.w))
    
    def ibeo_objects(self, data):
        
        # Considering only vehicle classes 
        if data.object_class in [self.ObjectClass_Car, self.ObjectClass_Motorbike, self.ObjectClass_Truck]:
            
            #Considering only vehicle at the front up to 100 meter
            if data.pose.pose.position.x > 0 and data.pose.pose.position.x <= 100:
                # We need to convert this to tf2_geometry_msgs as ibeo_object is a custom object type.
                pose_stamped = tf2_geometry_msgs.PoseStamped()
                pose_stamped.pose = data.pose.pose
                pose_stamped.header.frame_id = "base_link"
                pose_stamped.header.stamp = data.header.stamp
                try:
                    # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
                    data_odom = self.tf_buffer.transform(pose_stamped, "odom")
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
                    print(data_odom.header.stamp.secs)
                    #t = t.to_nsec()
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
                    self.ibeo_object.append(msg)
                    
                    self.f_e.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' % 
                       (data_odom.header.stamp.to_sec(),
                        data_odom.pose.position.x, data_odom.pose.position.y,
                        data_odom.pose.position.z,
                        data_odom.pose.orientation.x,
                        data_odom.pose.orientation.y,
                        data_odom.pose.orientation.z,
                        data_odom.pose.orientation.w))
                    
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass
        
                
    def compute_the_ego_trajectory(self):
  
        cumulative_x = 0
        cumulative_y = 0
        cumulative_theta = 0
        
        # It is important to remember that yaw rate is the rate of change of yaw. That it is a rate of change of z-axis on the Euler. 
        for key in self.ego_odom.group_item:
            data = self.ego_odom.group_item[key][0]
            v = data.long_vel
            if v > 0.:
                dt = 1 
                cumulative_theta += dt * data.yaw
                cumulative_x += math.cos(cumulative_theta) * v * dt
                cumulative_y += math.sin(cumulative_theta) * v * dt
                self.extracted_data['ego_odom']['x'].append(cumulative_x)
                self.extracted_data['ego_odom']['y'].append(cumulative_y)
                self.extracted_data['ego_odom']['theta'].append(cumulative_theta)
                
    def compute_the_other_trajectory(self, cars = None):
        for car in cars:
            item = self.ibeo_object.group_objects[car]
            self.extracted_data['other_odom'][car] = {
                'x': [],
                'y': [],
                'theta': []
            }
            cumulative_x = 0
            cumulative_y = 0
            cumulative_theta = 0
            
            # It is important to remember that yaw rate is the rate of change of yaw. That it is a rate of change of z-axis on the Euler. 
            for data in item:
                #data = self.ego_odom.group_item[key][0]
                v = data.long_vel
                if v > 0.:
                    dt = 1 
                    cumulative_theta += dt * data.yaw
                    cumulative_x += math.cos(cumulative_theta) * v * dt
                    cumulative_y += math.sin(cumulative_theta) * v * dt
                    self.extracted_data['other_odom'][car]['x'].append(cumulative_x)
                    self.extracted_data['other_odom'][car]['y'].append(cumulative_y)
                    self.extracted_data['other_odom'][car]['theta'].append(cumulative_theta)
                
    def extract(self):
        print("Extracting now...")
        time_index = 0
        offset = rospy.Time.now().to_sec() - self.bag.get_start_time()
        for topic, msg, t in self.bag.read_messages():
            
            '''playback_time = rospy.Time.now().to_sec() - offset
            wait_time = t.to_sec() - playback_time

            if wait_time > 0:
                rospy.sleep(wait_time)'''

            
            if topic == 'ibeo/objects':
                # We are not getting ibeo_objects in every topics and he is just adding the topic content to a list. Do not think we are getting continous objects
                # I need to check with stew to findout this is the behaviour in the dataset.
                if msg.object_class in [self.ObjectClass_Car, self.ObjectClass_Motorbike, self.ObjectClass_Truck]:
                    name = t.to_sec()
                    v_long_other = msg.twist.twist.linear.x
                    v_lat_other = msg.twist.twist.linear.y
                    quaternion = (
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w
                    )
                    euler = tf.transformations.euler_from_quaternion(quaternion)
                    yaw = euler[2]
                    if name not in self.tracked_objects.keys():
                        self.tracked_objects[name] = [] 
                    self.tracked_objects[name].append(
                        {
                            'id': msg.object_id,
                            'rel_pos_x': msg.pose.pose.position.x,
                            'rel_pos_y': msg.pose.pose.position.y,
                            'long_vel':v_long_other,
                            'lat_vel': v_lat_other,
                            'yaw': yaw
                        }
                    )
            if topic == 'ibeo/odometry':
                v_long_ego = msg.twist.twist.linear.x
                v_lat_ego = msg.twist.twist.linear.y
                quaternion = (
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                )
                euler = tf.transformations.euler_from_quaternion(quaternion)
                yaw = euler[2]
                name = t.to_sec()
                
                # Logic to capture the data at each second
                diff = 1  
                if self.previous is not None:
                    diff = name-self.previous   
                if name in self.tracked_objects:# and diff >= 1:
                    objects = self.tracked_objects[name]
                    front_car = None
                    front_side_car = []
                    front_side_each_car = {}
                    #print(objects)
                    for _object in objects:
                        key = _object['id']

                        # Detecting the front vehicle from dataset
                        # If this condition satisfied for one of the object then, we found the car infront of it. 
                        if _object is not None and (_object['rel_pos_y'] >= -1.5 and _object['rel_pos_y'] <= 1.5) and _object['rel_pos_x'] > 0:
                            front_car = {'key': key, 'rel_pos_y': _object['rel_pos_y'] , 'rel_pos_x': _object['rel_pos_x'], 'long_vel': _object['long_vel']}

                        # Detect the lateral vehicle at the front_side
                        if _object is not None  and (_object['rel_pos_y'] <= -1.5 or _object['rel_pos_y'] >= 1.5) and _object['rel_pos_x'] > 0:
                            front_side_each_car[key] = {'timestamp':t.to_sec(), 'id': key, 'rel_pos_y': _object['rel_pos_y'] , 'rel_pos_x': _object['rel_pos_x'], 'long_vel': _object['long_vel'], 'lat_vel': _object['lat_vel'], 'yaw': _object['yaw']}
                            
                    
                    # We need to worry about the vehicle at the front only for deteting the cut-in scenario. 
                    # The below logic finds the cars that is more relavant in cut in scenario
                    # We have put 100m as threshold to detect the vehicle near the ego-vehicles.
                    threshold_dist = 100 #meter
                    for id in front_side_each_car.keys():
                        _object = front_side_each_car[id]
                        if _object['rel_pos_x'] <= threshold_dist:            
                            if id in self.closest_cars.keys():
                                self.closest_cars[id].append(_object)
                            else:
                                self.closest_cars[id] = []
                                
                    #print(self.closest_cars)
                    
                    
                # Paper: Scenario Identification for Validation of Automated Driving Functions mentioned that duration of the lane change activity is 8 second.
                # For now finds what happens after 3 seconds.
                # How do we detect the one of the other vehicle moved to ego's lane. When it comes to curve rel_pos_y of vehicle at the right become zero.
                future_sec = time_index+3
                future_data = self.get_future_data(time_index=future_sec)
                if future_data[0] is not None:
                    for car_id in self.closest_cars.keys():
                       _object = self.closest_cars[car_id]
                       
                time_index += 1
                
                
    # This function get the data in the future second. 
    # INPUT: time_index is the future index
    # OUTPUT: (ego_odomenty, ibeo_objects) In the specified time index.
    def get_future_data(self, time_index = 0):
        if time_index < len(self.ego_odom.item) and time_index < len(self.ibeo_object.item):
            return (self.ego_odom.item[time_index], self.ibeo_object.item[time_index])
        else:
            return (None, None)
        
            
    def shutdown(self):
        print("Extract node is shutting down!!!")
        
        self.extracted_data = {
            'ego_odom': self.ego_odom,
            'ibeo_objects': self.ibeo_object
        }
        
        print(self.extracted_data['ibeo_objects'])
        
        # Save the json file
        with open('/constraint_model/extracted_data.txt', 'w') as outfile:
            json.dump(self.extracted_data, outfile)
    

if __name__ == '__main__':
    try:
        Extract()
    except rospy.ROSInterruptException:
	    rospy.logerr('Could not start Extract node.')