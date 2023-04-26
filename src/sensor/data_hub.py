#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
import time

#from gigi_park.msg import erp42_read
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan, NavSatFix, PointCloud
from geometry_msgs.msg import Point, Point32, Quaternion

from ublox_msgs.msg import NavPVT

from location import only_gps
from lidar import lidar

class SensorDataHub:
    def __init__(self):
        # 구독자 선언
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
        self.sub_gps = rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.pose_callback, queue_size=1)
        self.sub_heading = rospy.Subscriber('/ublox_gps/navpvt', NavPVT, self.heading_callback, queue_size=1)

        # 발행자 선언
        self.localization_pub=rospy.Publisher('current_pose', Point, queue_size=1) # x,y는 tm좌표, z에 들어가 있는 값이 heading
        self.obs_pub=rospy.Publisher('object', PointCloud, queue_size=1) # PointCloud.points[i].x 이런식으로 사용
        
        # 사용 하는 객체 선언
        self.loc_gps = only_gps()
        self.lidar = lidar()

        # flag 선언
        self.lidar_flag = False
        self.gps_flag = False
        
        # Sub 받은 데이터 저장 공간
        self.sub_coord = [0.0, 0.0]
        self.sub_gps_heading = 0.0
        self.sub_scan = []
        for i in range(810):
            self.sub_scan.append(0.0)
        
        # obs_pub 에 사용 되는 msg 객체 선언
        self.pos = Point()
        self.obs = PointCloud()
        
    ##########callback 함수 모음##########
    # 각 sensor 에서 data 가 들어 오면 객체 내부의 데이터 저장 공간에 저장
    def pose_callback(self, fix):
        self.sub_coord = [fix.longitude, fix.latitude]
        self.gps_flag = True

    def heading_callback(self, head):
        self.sub_gps_heading = float(head.heading)
    
    def scan_callback(self, scan):
        self.sub_scan = scan.ranges
        self.lidar_flag = True
    ######################################

    # gps, lidar 가 모두 작동 해서 data 가 들어 왔는지 확인
    def sensor_check(self):
        return self.gps_flag and self.lidar_flag

    ##########update 함수 모음############
    def localization_update(self):
        x, y = self.loc_gps.tf_to_tm(self.sub_coord[0], self.sub_coord[1])
        heading = self.loc_gps.get_heading(x, y, self.sub_gps_heading)
        self.pos.x = x  # 955920.9088
        self.pos.y = y  # 1950958.212
        self.pos.z = heading

    def object_update(self):
        self.obs = PointCloud()
        self.lidar.tf_tm(self.sub_scan, self.pos.x, self.pos.y, self.pos.z)  # 들어온 점을 tm 좌표로 변환
        obs_clean = self.lidar.clean()  # 격자화
        # obs_clean = [[955920.0, 1950958.0], [955921.0, 1950958.0], [955919.0, 1950958.0], [955921.0, 1950959.0],
        # [955922.0, 1950960.0], [955918.0, 1950958.0], [955920.0, 1950960.0]]
        
        for i in obs_clean:
            p = Point32()
            p.x = i[0]
            p.y = i[1]
            p.z = 0
            self.obs.points.append(p)
    ######################################

    ##########publish 함수 모음############
    def pub_pose(self):
        self.localization_pub.publish(self.pos)

    def pub_obs(self):
        self.obs_pub.publish(self.obs)
    ######################################

def main():
    # 기본 설정
    rospy.init_node('data_hub', anonymous=True)

    Data = SensorDataHub()  # 객체 생성
    start_rate = time.time()  # 시간 스템프

    while not rospy.is_shutdown():
        if time.time() - start_rate > 0.1:  # 0.2초 간격으로 실행. 데이터 처리가 0.3초보다 빨라도 여유롭게 0.3초간격으로 실행하고, 더 늦으면 업데이트 되는대로 실행.

            start_rate = time.time()  # 시작 시간 스템프 찍고
            
            # 각 정보 update 후 발행
            Data.localization_update()
            Data.object_update()
            Data.pub_pose()
            Data.pub_obs()
            
            print('pos: ', Data.pos)


if __name__ == '__main__':
    main()
