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
MODE = 1

# noinspection PyPep8Naming
class mission_cruising:
    def __init__(self, filename, file=0):
        self.PT_tra = Path_Tracking(filename, file)
        self.PT_dwa = Path_Tracking_DWA(filename, file)

    def path_tracking(self, pose, heading, mode=MODE):
        if mode == 0:
            steer = self.PT_tra.gps_tracking(pose, heading)
        elif mode == 1:
            steer = self.PT_dwa.gps_tracking(pose, heading)
        else:
            steer = 0.0
        return steer

    def static_obstacle(self, pose, heading, speed, steer, obs, mode=MODE):
        if mode == 0:
            steer = self.PT_tra.gps_tracking(pose, heading, obs, path_num=9)
        elif mode == 1:
            steer = self.PT_dwa.gps_tracking(pose, heading, speed, steer, obs)
        else:
            pass
        return steer
