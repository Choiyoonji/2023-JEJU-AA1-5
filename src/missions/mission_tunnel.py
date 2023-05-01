#!/usr/bin/env python3
import rospy
import os
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class mission_tunnel:
    def __init__(self, where):
        PATH =  (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"
        if where == 1:
            self.mission_np = np.load(file = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/yaeseon_xy.npy")
        else:
            self.mission_np = np.load(file = PATH + where)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.sub_scan = []
        self.vehicle_width = 0.77  # 차 폭 [m]
        # self.wheel_base = 0.75  # 차축 간 거리 [m]
        # self.length = 1.35  # 차 길이 [m]
        self.tunnel_width = 1.4  # 터널 폭 [m]
        self.max_dis = 3.0  # lidar 센서 최대 측정 범위 [m]
        self.tunnel_flag = False

    def scan_callback(self, scan):
        sub_scan = np.array(scan.ranges[0:810 + 1:3])  # 0° ~ 270° 범위, 0.333° 간격의 811개 data 를 1° 간격의 361개 data 로 필터링
        sub_scan = np.where(sub_scan >= self.max_dis, self.max_dis, sub_scan)  # max_dis 를 넘는 값 or inf -> max_dis
        self.sub_scan = np.where(sub_scan <= 0.003, self.max_dis, sub_scan)  # 0.002 로 뜨는 noise -> max_dis
    
    # def two_dis(p1, p2):
    # a = p1[0] - p2[0]
    # b = p1[1] - p2[1]
    # c = np.hypot(a, b)
    # return c
    
    # def find_ind(pose, path):
    # d_min = 1000.0
    # d_ind = -1
    # d_save = 0
    # for p in path:
    #     d_ind += 1
    #     d = two_dis(p, pose)
    #     if d < d_min:
    #         d_save = d_ind
    #         d_min = d
    # return d_save

    # def is_obs(self, pose, obs):
    #     # print('pose', pose)
    #     global car_w_offset, stop_dis, restart_time
    #     for p in obs:
    #         n = find_ind(p, self.mission_np) #원하는 P index  - - -  - - - mission  /  pose(erp) - - - mission
    #         # print('n: ',n)
    #         # print('dis', two_dis(p, self.mission_np[n]))
    #         if (two_dis(p, self.mission_np[n]) < car_w_offset) and (two_dis(pose, self.mission_np[n]) < stop_dis):
    #             return True
    #     return False
    
    # noinspection PyMethodMayBeStatic
    def find_largest_second_largest(self, list_data):
        r_index = np.argmax(list_data >= 0.1)
        l_index = np.argmax(np.flip(list_data) >= 0.1)
        return r_index, 180 - l_index

    def search_tunnel_entrance(self):
        diff_list = np.abs(np.diff(self.sub_scan[45:225 + 1]))
        r_angle, l_angle = self.find_largest_second_largest(diff_list)
        # goal_angle = 0.5 * (r_angle + l_angle)
        # center = int(0.5 * len(diff_list))
        # print('r_index : {0} , l_index : {1}'.format(r_angle, l_angle))
        if 0 < r_angle <= 10 or 170 <= l_angle < 180:
            self.tunnel_flag = True
            print('################################################################################ \
                  터널 진입 ########################################################################')
        # return np.clip(center - goal_angle, -22, 22)

    def get_steer_in_tunnel(self):
        r_data, l_data = self.sub_scan[55:75+1:3], self.sub_scan[195:215+1:3]  # 정면 0° 기준 좌우 60° ~ 80° 범위 거리 데이터
        r_avg, l_avg = np.sum(r_data) / len(r_data), np.sum(l_data) / len(l_data)  # 60° ~ 80° 범위의 거리 값들의 평균
        steer = ((r_avg - l_avg) / (self.tunnel_width - self.vehicle_width)) * 1.1 * 22  # (-1 ~ 1 로 정규화) * 22(steer)
        if r_avg >= self.tunnel_width and l_avg >= self.tunnel_width:
            self.tunnel_flag = False
            print('#------------------------------------------------------------------------------- \
                  터널 탈출 ------------------------------------------------------------------------')
        return np.clip(steer, -22, 22)

    def get_steer(self):
        if self.tunnel_flag is False:
            pass
            # return self.search_tunnel_entrance()
        elif self.tunnel_flag is True:
            print("터널 안에 있습니당")
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
