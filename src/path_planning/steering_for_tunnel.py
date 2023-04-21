#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SteeringInTunnel:
    def __init__(self):
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.sub_scan = []
        # self.wheel_base = 0.75  # 차축 간 거리 [m]
        self.width = 0.77  # 차 폭 [m]
        # self.length = 1.35  # 차 길이 [m]
        self.tunnel_width = 1.5  # 터널 폭 [m]
        self.max_dis = 15.0  # lidar 센서 최대 측정 범위 [m]

    def scan_callback(self, scan):
        self.sub_scan = list(scan.ranges[0:811:3])

    def get_steer(self):
        self.sub_scan[self.sub_scan == 'inf'] = self.max_dis  # 최대 측정 거리를 넘어가 값이 'inf' 로 들어올 때 max_dis 로 변환
        r_data, l_data = self.sub_scan[55:75 + 1:3], self.sub_scan[195:215 + 1:3]  # 정면 기준 좌우 50° ~ 80° 범위 거리 데이터
        r_avg, l_avg = sum(r_data) / len(r_data), sum(l_data) / len(l_data)  # 50° ~ 80° 범위의 거리 값들의 평균
        steer = ((r_avg - l_avg) / (self.tunnel_width - self.width)) * 22 * 1.1  # normalization 후 steer 로 변환, 1.1은 가중치
        return max(min(steer, 22), -22)


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
        time.sleep(0.1)
        speed = 50
        steer = Tunnel.get_steer()
        pub.pub_erp(speed, steer)


if __name__ == '__main__':
    main()
