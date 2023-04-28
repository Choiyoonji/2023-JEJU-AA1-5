#!/usr/bin/env python3
# noinspection PyUnresolvedReferences
import rospy
import time
import numpy as np
# noinspection PyUnresolvedReferences
from sensor_msgs.msg import LaserScan
# noinspection PyUnresolvedReferences
from geometry_msgs.msg import Twist


class mission_tunnel:
    def __init__(self):
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.sub_scan = []
        self.vehicle_width = 0.77  # 차 폭 [m]
        # self.wheel_base = 0.75  # 차축 간 거리 [m]
        # self.length = 1.35  # 차 길이 [m]
        self.tunnel_width = 1.4  # 터널 폭 [m]
        self.max_dis = 2.0  # lidar 센서 최대 측정 범위 [m]

        self.tunnel_flag = False
        self.step = 0

    def scan_callback(self, scan):
        sub_scan = np.array(scan.ranges[0:810 + 1:3])  # 0° ~ 270° 범위, 0.333° 간격의 811개 data 를 1° 간격의 361개 data 로 필터링
        # self.sub_scan = np.where(sub_scan >= self.max_dis, self.max_dis, sub_scan)  # max_dis 를 넘는 값 or inf -> max_dis
        for i in range(len(sub_scan)):
            if sub_scan[i] >= self.max_dis:
                sub_scan[i] = self.max_dis
            elif np.isinf(sub_scan[i]):
                sub_scan[i] = self.max_dis
            else:
                pass
        self.sub_scan = sub_scan
        # print('65도 dis : ', sub_scan[65])
    # noinspection PyMethodMayBeStatic
    # def find_largest_second_largest(self, list_data):
    #     max_val, second_max_val = float('-inf'), float('-inf')
    #     max_index, second_max_index = -1, -1
    #
    #     for i, val in enumerate(list_data):
    #         if val > max_val:
    #             second_max_val = max_val
    #             second_max_index = max_index
    #             max_val = val
    #             max_index = i
    #         elif val > second_max_val:
    #             second_max_val = val
    #             second_max_index = i
    #     return [max_index, second_max_index]

    def find_largest_second_largest(self, list_data):
        r_index = 0
        for i in range(len(list_data)):
            if list_data[i] >= self.tunnel_width:
                r_index = i
        # r_index = np.argmax(list_data >= self.tunnel_width)
        # if np.isnan(r_index):
        #     r_index = 0
        l_index = np.argmax(np.flip(list_data) >= self.tunnel_width)
        if np.isnan(l_index):
            l_index = 0
        print('r_index : {0} , l_index : {1}'.format(r_index, l_index))
        return r_index, 180 - l_index

    def search_tunnel_entrance(self):
        # diff_list = np.abs(np.diff(self.sub_scan[45:225 + 1]))
        diff_list = []
        scan = self.sub_scan[45:225 + 1]
        for i in range(len(scan) - 1):
            diff_list.append(scan[i] - scan[i + 1])
        r_angle, l_angle = self.find_largest_second_largest(np.array(diff_list))
        goal_angle = 0.5 * (r_angle + l_angle)
        center_angle = int(len(diff_list) * 0.5)
        # first_angle, second_angle = r_index - center_angle, l_index - center_angle
        # if first_angle >= center_angle and second_angle <= -center_angle:
        #     self.tunnel_flag = True
        if r_angle <= 10 and l_angle >= 170:
            self.tunnel_flag = True
            # print('################################################################################ \
            #       터널 진입 ####################################################################')
        return np.clip(center_angle - goal_angle, -22, 22)

    def get_steer_in_tunnel(self):
        r_data, l_data = self.sub_scan[55:75+1:3], self.sub_scan[195:215+1:3]  # 정면 0° 기준 좌우 60° ~ 80° 범위 거리 데이터
        r_avg, l_avg = np.sum(r_data) / len(r_data), np.sum(l_data) / len(l_data)  # 60° ~ 80° 범위의 거리 값들의 평균
        steer = ((r_avg - l_avg) / (self.tunnel_width - self.vehicle_width)) * 1.1 * 22  # (-1 ~ 1 로 정규화) * 22(steer)
        if r_avg >= self.tunnel_width and l_avg >= self.tunnel_width:
            self.tunnel_flag = False
            # print('#------------------------------------------------------------------------------- \
            #       터널 탈출 ------------------------------------------------------------------------')
        return np.clip(steer, -22, 22)



    def get_steer(self):
        if self.tunnel_flag is False:
            return self.search_tunnel_entrance()
        elif self.tunnel_flag is True:
            return self.get_steer_in_tunnel()


class PublishToErp:
    def __init__(self):
        self.erp_pub = rospy.Publisher("erp_write", Twist, queue_size=30)
        self.erp = Twist()

    def pub_erp(self, speed, steer):
        self.erp.linear.x = speed
        self.erp.angular.z = steer
        self.erp_pub.publish(self.erp)


# def main():
#     rospy.init_node('tunnel_node', anonymous=True)
#     Tunnel = mission_tunnel()
#     pub = PublishToErp()

#     while not rospy.is_shutdown():
#         time.sleep(0.1)
#         speed = 50
#         steer = Tunnel.get_steer()
#         pub.pub_erp(speed, steer)


# if __name__ == '__main__':
#     main()
