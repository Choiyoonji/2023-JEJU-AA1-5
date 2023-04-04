#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
import time
from math import cos, sin, pi
import numpy as np

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point, Point32

class lidar:
    def __init__(self):
        self.obs_xy = []

    def tf(self, scan) :
        self.obs_xy = []
        resolution = 3
        last_scan_data = scan
        scan_data_for_search = []
        for i in range(0, 540):
            scan_data_for_search.append(last_scan_data[i+135])
            if np.isinf(scan_data_for_search[i]) or scan_data_for_search[i] < 0.1 or scan_data_for_search[i] > 13: # -90~90
                pass
            elif scan_data_for_search[i] <= 13:
                obs_x = scan_data_for_search[i]*sin(np.deg2rad(float(i)/resolution)) + 1.35 # 라이다 위치 offset
                obs_y = -scan_data_for_search[i]*cos(np.deg2rad(float(i)/resolution))
                self.obs_xy.append([obs_x, obs_y])
        print(len(self.obs_xy))
        return  self.obs_xy
        
    def clean(self):
        self.code = np.array(self.obs_xy)
        try:
            f_arr = []
            f_arr.append([self.code[0][0],self.code[0][1],0.0])

            for i in range(len(self.code)-1):
                m = abs(f_arr[-1][0] - self.code[i][0])
                n = abs(f_arr[-1][1] - self.code[i][1])
                if m >= 0.1 or n >= 0.1:
                    f_arr.append([self.code[i][0],self.code[i][1],0.0]) 
                else:
                    pass
        except:
            f_arr = []

        return f_arr  

class SensorDataHub:
    def __init__(self):
        #구독자 선언
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)

        #발행자 선언
        self.localization_pub=rospy.Publisher('current_pose', Point, queue_size=1) # 형식상
        self.obs_pub = rospy.Publisher('/object', PointCloud, queue_size=1) # PointCloud.points[i].x 이런식으로 사용
        
        #사용하는 객체 선언
        self.Lidar = lidar()
        
        #Sub 받은 데이터 저장 공간
        self.sub_scan = []
        for i in range(810):
            self.sub_scan.append(0.0)
        
        #obs_pub에 사용되는 msg 객체 선언
        self.pos = Point()
        self.obs = PointCloud()
        
    ##########callback 함수 모음##########
    # 각 센서에서 데이터가 들어오면 객체 내부의 데이터 저장공간에 저장
    def scan_callback(self, scan):
        self.sub_scan = scan.ranges
        self.lidar_flag = True
    ######################################

    ##########update 함수 모음############
    def object_update(self):
        self.obs = PointCloud()
        obs_clean = self.Lidar.tf(self.sub_scan) # 들어온 점을 tm좌표로 변환
        # obs_clean = self.Lidar.clean() #격자화
        
        for i in obs_clean:
            p = Point32()
            p.x = i[0]
            p.y = i[1]
            p.z = 0
            self.obs.points.append(p)
    ######################################

    ##########publish 함수 모음############
    def pub_pose(self):
        self.pos.x = 0
        self.pos.y = 0
        self.pos.z = 0
        self.localization_pub.publish(self.pos)

    def pub_obs(self):
        self.obs_pub.publish(self.obs)
    ######################################

def main():
    #기본 설정
    rospy.init_node('data_hub', anonymous=True)

    Data = SensorDataHub() # 객체 생성
    start_rate = time.time() # 시간 스템프

    while not rospy.is_shutdown():
        if time.time() - start_rate > 0.1: # 0.1초 간격으로 실행.
            print("rate : "),
            print(int((time.time() - start_rate)*1000)),
            print("(ms), "),

            start_rate = time.time() # 시작 시간 스템프 찍고
            # 각 정보 업데이트 후 발행
            Data.object_update()
            Data.pub_pose()
            Data.pub_obs()

            print("Processing time : "),
            print(int((time.time() - start_rate)*1000)),
            print("(ms)")

if __name__ == '__main__':
    main()