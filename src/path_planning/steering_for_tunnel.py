#!/usr/bin/env python3
import rospy
import numpy as np
from math import *
from sensor_msgs.msg import LaserScan


class SteeringInTunnel:
    def __init__(self):
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.sub_scan = []  # np.zeros(810)

        # self.wheel_base = 0.75  # 차축 간 거리 [m]
        self.width = 0.77  # 차 폭 [m]
        # self.length = 1.35  # 차 길이 [m]
        self.tunnel_width = 1.5  # 터널 폭 [m]

        self.w1 = 0.5
        self.w2 = 0.5

    def scan_callback(self, scan):
        self.sub_scan = scan.ranges     # ranges 하면 모든 값들 다 받아 오는 건가? 그리고 값 들의 해상도 는 얼마나 되지??

    def get_steer(self):
        self.sub_scan[self.sub_scan == 0.0] = 10.0
        longest_angle = np.argmax(self.sub_scan[30:150 + 1:5])
        steer1 = max(min(longest_angle - 60, 22), -22)

        r_data, l_data = self.sub_scan[80:100 + 1:3], self.sub_scan[260:280 + 1:3]
        r_avg, l_avg = sum(r_data) / 21, sum(l_data) / 21
        steer2 = ((l_avg - r_avg) / (self.tunnel_width - self.width)) * 22

        return self.w1 * steer1 + self.w2 * steer2
