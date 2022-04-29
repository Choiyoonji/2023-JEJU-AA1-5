#!/usr/bin/env python
#-*-coding:utf-8-*-

# Python packages
import rospy
import sys, os
import math
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")
from path_planning_ing import Path_Tracking

class mission_cruising:

    def __init__(self, filename, file = 0):
        self.PT = Path_Tracking(filename, file)
        self.done = False

    def path_tracking(self, pose, heading):
        steer = self.PT.gps_tracking(pose, heading)
        return steer

    def static_obstacle(self, pose, heading, obs):
        steer = self.PT.gps_tracking(pose, heading, obs, path_num = 5)
        return steer