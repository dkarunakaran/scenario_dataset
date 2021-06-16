#!/usr/bin/env python

from __future__ import division
import rospy
from nav_msgs.msg import Odometry
import json
import os
from rss import RSS
from ibeo_object_msg.msg import IbeoObject
import message_filters
import tf

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
        self.previous = None
        self.previous_other = None
        self.count = 0
        self.tracked_objects = {}#TrackObjects()
        self.closest_cars = {}
        self.all_objects_timesteps = []
        self.odom = {}
        self.rss = RSS()
        self.extracted_data = {
            'ego':{
                'long_vel': [],
                'rss_long_no_front_vehicle': [],
                'accel': [],
                'rss_long_with_vehicle': []
            },
            'other_car': {
                'long_vel': [],
                'lat_vel': [],
                'long_accel': [],
                'rel_pos_x': [],
                'rel_pos_y': [],
                'yaw': []
            },
            'tracked_objects': None,
            'all_objects_timesteps': None,
            'closest_cars': None
        }
        rospy.Subscriber('/ibeo/odometry', Odometry, self.ego_odometry)    
        rospy.Subscriber('/ibeo/objects', IbeoObject, self.ibeo_objects)  
        
        #Combined topic subscription 
        #odometry_sub = message_filters.Subscriber('/ibeo/odometry', Odometry)
        #objects_sub = message_filters.Subscriber('/ibeo/objects', IbeoObject)
        #ts = message_filters.ApproximateTimeSynchronizer([odometry_sub, objects_sub], 10, 0.1)
        #ts.registerCallback(self.process_odom_objects_topics)
        rospy.on_shutdown(self.shutdown)   
        rospy.spin()
        
    def process_odom_objects_topics(self, odom, obj):
        v_long_ego = odom.twist.twist.linear.x
        if v_long_ego > 0 and (obj.object_class in [self.ObjectClass_Car, self.ObjectClass_Motorbike, self.ObjectClass_Truck]):
            if obj.pose.pose.position.y >= -1.5 and obj.pose.pose.position.y <= 1.5:
                print(obj.object_id)
        
    def ibeo_objects(self, data):
        
        # We are not getting ibeo_objects in every topics and he is just adding the topic content to a list. Do not think we are getting continous objects
        # I need to check with stew to findout this is the behaviour in the dataset.
        if data.object_class in [self.ObjectClass_Car, self.ObjectClass_Motorbike, self.ObjectClass_Truck]:
            name = str(data.header.stamp.secs)
            name = int(name)
            v_long_other = data.twist.twist.linear.x
            v_lat_other = data.twist.twist.linear.y
            
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
                        
            self.extracted_data['other_car']['yaw'].append(yaw)
            if name in self.tracked_objects.keys():
                self.tracked_objects[name].append(
                    {
                        'id': data.object_id,
                        'rel_pos_x': data.pose.pose.position.x,
                        'rel_pos_y': data.pose.pose.position.y,
                        'long_vel':v_long_other,
                        'lat_vel': v_lat_other,
                        'yaw': yaw
                    }
                )
            else:
                self.tracked_objects[name] = [] 
            
        
        '''long_accel = 0
        if data.object_class in [self.ObjectClass_Car, self.ObjectClass_Motorbike, self.ObjectClass_Truck]:
            v_long_other = data.twist.twist.linear.x
            v_lat_other = data.twist.twist.linear.y
            self.extracted_data['other_car']['rel_pos_x'].append(data.pose.pose.position.x)
            self.extracted_data['other_car']['rel_pos_y'].append(data.pose.pose.position.y)                
            self.extracted_data['other_car']['long_vel'].append(v_long_other)
            self.extracted_data['other_car']['lat_vel'].append(v_lat_other)
            if self.previous_other is not None:
                long_accel = self.average_acceleration(v_long_other, self.previous_other)
            self.extracted_data['other_car']['long_accel'].append(long_accel)
            self.previous_other = v_long_other
            if data.object_id in self.tracked_objects.keys():
                self.tracked_objects[data.object_id].append(
                    {
                        'rel_pos_x': data.pose.pose.position.x,
                        'rel_pos_y': data.pose.pose.position.y,
                        'long_vel':v_long_other,
                        'lat_vel': v_lat_other,
                        'long_accel': long_accel
                    }
                )
            else:
                self.tracked_objects[data.object_id] = []   '''
        
        
    def ego_odometry(self, data):
        v_long_ego = data.twist.twist.linear.x
        self.extracted_data['ego']['long_vel'].append(v_long_ego)
        name = str(data.header.stamp.secs)
        name = int(name)     
        if name in self.tracked_objects and self.previous is not name:
            objects = self.tracked_objects[name]
            temp_tracked_objects = {}
            for index in range(len(objects)):
                if objects[index]['id'] in temp_tracked_objects.keys():
                    '''temp_tracked_objects[objects[index]['id']].append(
                        {
                            'rel_pos_x': objects[index]['rel_pos_x'],
                            'rel_pos_y': objects[index]['rel_pos_y'],
                            'long_vel': objects[index]['long_vel'],
                            'lat_vel': objects[index]['lat_vel'],
                            'yaw': objects[index]['yaw']
                        }
                    )'''
                    
                    temp_tracked_objects[objects[index]['id']] = {
                        'rel_pos_x': objects[index]['rel_pos_x'],
                        'rel_pos_y': objects[index]['rel_pos_y'],
                        'long_vel': objects[index]['long_vel'],
                        'lat_vel': objects[index]['lat_vel'],
                        'yaw': objects[index]['yaw']
                    }
                else:
                    temp_tracked_objects[objects[index]['id']] = None #[]
            front_car = None
            front_side_car = []
            front_side_each_car = {}
            for key in temp_tracked_objects.keys():
                _object = temp_tracked_objects[key]
                for index in range(len(_object)):
                   
                    # Detecting the front vehicle from dataset
                    # If this condition satisfied for one of the object then, we found the car infront of it. 
                    if (_object[index]['rel_pos_y'] >= -1.5 and _object[index]['rel_pos_y'] <= 1.5) and _object[index]['rel_pos_x'] > 0:
                        front_car = {'key': key, 'rel_pos_y': _object[index]['rel_pos_y'] , 'rel_pos_x': _object[index]['rel_pos_x'], 'long_vel': _object[index]['long_vel']}

                    # Detect the lateral vehicle at the front_side
                    if (_object[index]['rel_pos_y'] <= -1.5 or _object[index]['rel_pos_y'] >= 1.5) and _object[index]['rel_pos_x'] > 0:
                        if key in front_side_each_car.keys():
                            front_side_each_car[key].append({'key': key, 'rel_pos_y': _object[index]['rel_pos_y'] , 'rel_pos_x': _object[index]['rel_pos_x'], 'long_vel': _object[index]['long_vel'], 'lat_vel': _object[index]['lat_vel'], 'yaw': _object[index]['yaw']})
                        else:
                            front_side_each_car[key] = []
                            
                        #front_side_car.append({'key': key, 'rel_pos_y': _object[index]['rel_pos_y'] , 'rel_pos_x': _object[index]['rel_pos_x'], 'long_vel': _object[index]['long_vel'], 'lat_vel': _object[index]['lat_vel']})
            
            #print(front_side_each_car)
            
            # We need to worry about the vehicle at the front only for deteting the cut-in scenario. 
            # The below logic finds the cars that is more relavant in cut in scenario
            # We have put 100m as threshold to detect the vehicle near the ego-vehicles.
            threshold_dist = 100 #meter
            closest_cars = []
            for id in front_side_each_car.keys():
                _object = front_side_each_car[id]
                for index in range(len(_object)):
                    if _object[index]['rel_pos_x'] <= threshold_dist:
                        #if id not in closest_cars:
                        #    closest_cars.append(id)                
                        if id in self.closest_cars.keys():
                            self.closest_cars[id].append(_object[index])
                        else:
                            self.closest_cars[id] = []
                            
            
            self.all_objects_timesteps.append(name)
                        
            #print(smallest_dist_object)
                
            
            # Longitudinal RSS from the perspective of ego-vehicle        
            if front_car is not None:   
                self.extracted_data['ego']['rss_long_with_vehicle'].append(self.rss.calculate_rss_safe_dist(v_long_ego, front_car['long_vel']))
            elif v_long_ego > 0:
                self.extracted_data['ego']['rss_long_with_vehicle'].append(self.rss.calculate_rss_safe_dist(v_long_ego, 0))  
            else:
                self.extracted_data['ego']['rss_long_with_vehicle'].append(0)              
        else:
            # Longitudinal RSS from the perspective of ego-vehicle
            if v_long_ego > 0:
                self.extracted_data['ego']['rss_long_with_vehicle'].append(self.rss.calculate_rss_safe_dist(v_long_ego, 0))  
            else:
                self.extracted_data['ego']['rss_long_with_vehicle'].append(0)       
                
        self.previous = name    
                        
                
        # 11/06/21
        # Next step is to group the message under object_id with in that second - Done
        # Find the objects in the front: rel_pos_y will be helful - Done
        # Calculate longitudinal RSS - Done
        # ----------------------------------------------------------------------------
        # 15/06/21 - week
        # Plot lane change activity of each car
        # Plot 3d plot with timesteps, rel_lateral_distance, rel_long_distance
        # Automatic detection of the events such as lane_change and cut-in
        # Find the objects in the side:rel_pos_x and rel_pos_y will be helpful
        # Calculate lateral RSS
        # Find the timestamps where list of events happening and that will be useful for start and stop.

    
        '''
        accel = 0
        self.extracted_data['ego']['rss_long_no_front_vehicle'].append(self.rss.calculate_rss_safe_dist(v_long_ego, 0))
        self.extracted_data['ego']['long_vel'].append(v_long_ego)
        if self.previous is not None:
            accel = self.average_acceleration(v_long_ego, self.previous)
        self.extracted_data['ego']['accel'].append(accel)
        self.previous = v_long_ego
        '''
        
        #Detect the front vehicle and calculate longitudinal RSS.
        
    def average_acceleration(self, v_current, v_previous):
        # We are recieving the rostopic '/ibeo/odometry' in 25hz rate.
        # 25hz to second is 0.04 second. It means that every 0.04 second we receive the topic.  
        # So the difference of time is 0.04 second
        accel =  (v_current-v_previous)/0.04
        
        return accel
        

    def shutdown(self):
        print("Extract node is shutting down!!!")
        self.extracted_data['tracked_objects'] = self.tracked_objects
        self.extracted_data['closest_cars'] = self.closest_cars
        self.extracted_data['all_objects_timesteps'] = self.all_objects_timesteps
        
        # Save the json file
        with open('/constraint_model/extracted_data.txt', 'w') as outfile:
            json.dump(self.extracted_data, outfile)
    
    

if __name__ == '__main__':
    try:
        Extract()
    except rospy.ROSInterruptException:
	    rospy.logerr('Could not start Extract node.')