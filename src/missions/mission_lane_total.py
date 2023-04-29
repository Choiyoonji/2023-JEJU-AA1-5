#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy
import sys, os
import numpy as np
import time

from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import LaserScan

class mission_lane_total:
    def __init__(self):
        self.steer_sub = rospy.Subscriber("lane_steer", Int16, self.lane_callback, queue_size=10)
        self.lane_steer = 0.0
        
        self.returning = False
    
    def lane_callback(self, data):
        self.lane_steer = data
    
    def is_obs(self):
        pass
    
    def avoid(self):
        pass
    
    def stop(self):
        pass
    
    def get_steer(self, obs_xy):
        if not self.returning and len(obs_xy) == 0:
            return self.lane_steer