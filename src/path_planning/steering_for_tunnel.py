#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

Version = 2

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
        self.sub_scan = scan.ranges[0:810:3]     # ranges 하면 모든 값들 다 받아 오는 건가? 그리고 값 들의 해상도 는 얼마나 되지??

    def get_steer(self):    # scan_data 의 데이터 타입 확인 후 0.0의 데이터 타입 수정 필요
        self.sub_scan[self.sub_scan == 0.0] = self.max_dis  # 최대 측정 거리를 넘어가 값이 0.0 으로 들어올 때 max_dis 로 변환
        longest_angle = np.argmax(self.sub_scan[75:195 + 1:5])  # 좌우 60° 중 거리가 가장 먼 angle 탐색
        steer1 = max(min(longest_angle - 60, 22), -22)  # 가장 먼 angle 을 정면 기준 으로 바꿔 steer 값으로 변환
                                                        # -22°를 넘으면 -22°로, 22°를 넘으면 22°

        r_data, l_data = self.sub_scan[35:55 + 1:3], self.sub_scan[215:235 + 1:3]  # 좌우 20° 범위 거리 데이터
        r_avg, l_avg = sum(r_data) / len(self.sub_scan), sum(l_data) / len(self.sub_scan)  # 20° 범위의 값들의 평균
        steer2 = ((l_avg - r_avg) / (self.tunnel_width - self.width)) * 22  # normalization 후 22를 곱하여 steer 값으로 변환

        if Version == 1:
            return self.w1 * steer1 + self.w2 * steer2
        elif Version == 2:
            return max(min(steer1 + steer2, 22), -22)
        else:
            pass
    '''
    Version 1 : 두 steer 의 평균을 반환
    Version 2 : steer1 과 steer2 를 단순 합산
    
    Case 1 ①가장 먼 지점이 정면 기준 좌측 10°이고 ②현재 차의 위치가 터널 '중앙' 에 놓여 있을 때
        steer1 = 10, steer2 = 0
            ▷ Version 1   =>   return = 5
            ▷ Version 2   =>   return = 10 
    
    Case 2 ①가장 먼 지점이 정면 기준 좌측 10°이고 ②현재 차의 위치가 터널 '왼쪽' 에 붙어 있을 때
        steer1 = 10, steer2 = -22
            ▷ Version 1   =>   return = -6      
            ▷ Version 2   =>   return = -12
            
    Case 3 ①가장 먼 지점이 정면 기준 좌측 10°이고 ②현재 차의 위치가 터널 '오른쪽' 에 붙어 있을 때
        steer1 = 10, steer2 = 22
            ▷ Version 1   =>   return = 16      
            ▷ Version 2   =>   return = 22
            
    Version 1 은 두 값을 평균 내기 때문에 최대 steer 인 22 or -22 에 도달 하는 경우가 극히 드물다.
    따라서 두 steer 를 더해 빠르게 반응할 수 있는 Version 2 를 생각해 보았다.
    
    급격한 코너가 있는 터널의 경우는 Version 2가, 완만한 곡률의 터널인 경우는 Version 1 이 더 성능이 좋을 것이라 생각 한다.
    둘 다 안좋을 수도...?
    '''


class PublishToErp:
    def __init__(self):
        self.erp_pub = rospy.Publisher("erp_write", Twist, queue_size=30)
        self.erp = Twist()

    def pub_erp(self, speed, steer):
        self.erp.linear.x = speed
        self.erp.angular.z = steer
        self.erp_pub.publish(self.erp)


def main():
    rospy.init_node('tunnel_node', anonymous=True)

    Tunnel = SteeringInTunnel()
    pub = PublishToErp()

    while not rospy.is_shutdown():
        speed = 30
        steer = Tunnel.get_steer()
        pub.pub_erp(speed, steer)


if __name__ == '__main__':
    main()
