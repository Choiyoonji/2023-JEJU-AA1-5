#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy
import sys, os
import numpy as np
import time

from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import LaserScan
from path_planning_tracking_dwa_PP import Path_Tracking_DWA

class mission_lane_total:
    def __init__(self, pose, heading, speed, steer, obs):
        self.steer_sub = rospy.Subscriber("lane_steer", Int16, self.lane_callback, queue_size=1)
        self.lane_steer = 0.0
        
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.sub_scan = []
        self.exist_obs = None
        self.max_dis = 2.0
        
        self.search_range = 30  # 중심 기준으로 양쪽 30도 범위 탐색
        
        self.stop = False
        self.avoid = False
        self.returning = False
        self.last_time_stop = 0
        self.last_time_avoid = 0
        self.dwa_PP = Path_Tracking_DWA()

        self.pose = pose
        self.heading = heading
        self.speed = speed
        self.steer = steer
        self.obs = obs
    
    def lane_callback(self, data):
        self.lane_steer = data
        
    def scan_callback(self, scan):
        sub_scan = np.array(scan.ranges[0:810 + 1:3])  # 0° ~ 270° 범위, 0.333° 간격의 811개 data 를 1° 간격의 361개 data 로 필터링
        sub_scan = np.where(sub_scan >= self.max_dis, self.max_dis, sub_scan)  # max_dis 를 넘는 값 or inf -> max_dis
        self.sub_scan = np.where(sub_scan <= 0.003, self.max_dis, sub_scan)  # 0.002 로 뜨는 noise -> max_dis

    def scan_ROI(self):
        center = int(len(self.sub_scan) * 0.5)
        search_data = np.array(self.sub_scan[center - self.search_range:center + self.search_range + 1])
        self.exist_obs = np.where(search_data < self.max_dis, True, False)

    def avoid_judgement(self):
        if not self.avoid:
            if not self.stop:
                self.scan_ROI()
                self.stop = True if any(self.exist_obs) else False
                if self.stop:
                    self.last_time_stop = time.time()
            else:
                if time.time() - self.last_time_stop > 10:
                    self.stop = False
                    self.avoid = True
        else:
            self.scan_ROI()
            if any(self.exist_obs):
                self.last_time_avoid = time.time()
            else:
                if self.last_time_avoid - time.time() > 5:
                    self.avoid = False

    def get_steer(self):
        self.avoid_judgement()
        if not self.stop and self.avoid:
            return self.dwa_PP.gps_tracking(self.pose, self.heading, self.speed, self.steer, self.obs)
        elif not self.stop and not self.avoid:
            return self.lane_steer  # 여기에 lane detection steer 리턴
        else:
            pass