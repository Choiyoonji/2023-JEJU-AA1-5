#!/usr/bin/env python
#-*-coding:utf-8-*-

# Python packages
import rospy
import sys, os
import math
import numpy as np
from path_planning_tracking import Path_Tracking
from path_planning_tracking_dwa_PP import Path_Tracking_DWA

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")
# mode = 0
mode = 1

# noinspection PyPep8Naming
class mission_cruising:
    def __init__(self, filename, speed, steer, file=0):
        if mode == 0:
            self.PT = Path_Tracking(filename, file)
        elif mode == 1:
            self.PT = Path_Tracking_DWA(filename, speed, steer, file)
        else:
            pass

    def path_tracking(self, pose, heading):
        steer = self.PT.gps_tracking(pose, heading)
        return steer

    def static_obstacle(self, pose, heading, obs):
        steer = 0.0
        if mode == 0:
            steer = self.PT.gps_tracking(pose, heading, obs, path_num=9)
        elif mode == 1:
            steer = self.PT.gps_tracking(pose, heading, obs)
        else:
            pass
        return steer
