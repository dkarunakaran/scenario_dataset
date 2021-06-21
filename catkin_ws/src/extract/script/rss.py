#!/usr/bin/env python
#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
import math

class RSS:

    # Below ones are default parameter from the RSS library: https://intel.github.io/ad-rss-lib/documentation/Main.html#Section::ParameterDiscussion
    # RESPONSE TIME: 
    # common sense value for human drivers is about 2 seconds, but this can be lower for AV.
    # LONGITUDINAL ACCELERATION: 
    # Finding acceleration values for RSS is more complicated. These values may vary from ODD to ODD.
    # The minimum deceleration values must not exceed normal human driving behaviour.
    # Also big difference between the minimum and maximum acceleration will lead to defencisve driving. That leads to not participating in the dense traffic.
    # maximum acceleration a low speeds a standard car is in the range of 3.4 m/s4 to 7 m/s2
    # Required safety distance for the car at 50 km/h is brake_min = 4 m/s2 and brake_max = 8 m/s2
    # Rule of thump in german driving schools are brake_min = 4 m/s2 and brake_max = 8 m/s2
    # We are only considering one way and longitudinal safe distance. 

    def __init__(self):
        # V_ego in m/s
        # V_other in m/s

        self.res_ego = 1 #s
        self.res_other = 2 #s
        self.accel_max = 3.5 #m/s2
        self.brake_min = 4 #m/s2
        self.break_max = 8 #m/s2
        self.lat_break_min = 0.8 #m/s2
        self.lat_accel_max = 0.2 #m/s2
        self.mu = 0.1 #m
        #self.margin = 10 #cm
        #self.vb_vel_margin = 0.25 #m/s


    def calculate_rss_safe_dist(self, V_ego, V_other, _type="long"):
        dist = None
        if _type is "long":
            dist = self.long_safe_dist(V_ego, V_other)
        else:
            #V_ego and V_other are lateral velocities 
            dist = self.lat_safe_dist(V_ego, V_other)

        return dist

    def long_safe_dist(self, V_ego, V_other):
        first_term = V_ego*self.res_ego
        second_term = self.accel_max*(math.pow(self.res_ego,2))/2
        third_term = (math.pow(V_ego+(self.res_ego*self.accel_max), 2))/(2*self.brake_min)
        fourth_term = (math.pow(V_other,2))/(2*self.break_max)
        d_min = first_term+second_term+third_term-fourth_term

        return d_min    

    def lat_safe_dist(self, V_lat_ego, V_lat_other):
        
        # v1 is the velocity of other vehicle
        # v2 is the velocity of ego-vehicle
        v_1_p = V_lat_other - (self.res_other*self.lat_accel_max) 
        v_2_p = V_lat_ego + (self.res_ego*self.lat_accel_max) 
        first_term = ((V_lat_other+v_1_p)/2)*self.res_other
        second_term = (v_1_p*v_1_p)/(2*self.lat_break_min)
        third_term = ((V_lat_ego+v_2_p)/2)*self.res_ego
        fourth_term = (v_2_p*v_2_p)/(2*self.lat_break_min)
        d_min = self.mu+first_term+second_term-(third_term-fourth_term)

        return d_min

