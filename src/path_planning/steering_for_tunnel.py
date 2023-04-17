#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan


class SteeringInTunnel:
    def __init__(self):
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.sub_scan = []  # np.zeros(360)

        # self.wheel_base = 0.75  # 차축 간 거리 [m]
        self.width = 0.77  # 차 폭 [m]
        # self.length = 1.35  # 차 길이 [m]
        self.tunnel_width = 1.5  # 터널 폭 [m]
        self.max_dis = 10.0  # lidar 센서 최대 측정 범위 [m]

        self.w1 = 0.5
        self.w2 = 0.5

    def scan_callback(self, scan):
        self.sub_scan = scan.ranges     # ranges 하면 모든 값들 다 받아 오는 건가? 그리고 값 들의 해상도 는 얼마나 되지??

    def get_steer(self):    # scan_data 의 데이터 타입 확인 후 0.0의 데이터 타입 수정 필요
        self.sub_scan[self.sub_scan == 0.0] = self.max_dis  # 최대 측정 거리를 넘어가 값이 0.0 으로 들어올 때 max_dis 로 변환
        longest_angle = np.argmax(self.sub_scan[300:360:5] + self.sub_scan[0:60 + 1:5])  # 좌우 60° 중 거리가 가장 먼 angle 탐색
        steer1 = max(min(longest_angle - 60, 22), -22)  # 가장 먼 angle 을 정면 기준 으로 바꿔 steer 값으로 변환
                                                        # -22°를 넘으면 -22°로, 22°를 넘으면 22°

        r_data, l_data = self.sub_scan[260:280 + 1:3], self.sub_scan[80:100 + 1:3]  # 좌우 20° 범위 거리 데이터
        r_avg, l_avg = sum(r_data) / len(self.sub_scan), sum(l_data) / len(self.sub_scan)  # 20° 범위의 값들의 평균
        steer2 = ((l_avg - r_avg) / (self.tunnel_width - self.width)) * 22  # normalization 후 22를 곱하여 steer 값으로 변환

        return self.w1 * steer1 + self.w2 * steer2
