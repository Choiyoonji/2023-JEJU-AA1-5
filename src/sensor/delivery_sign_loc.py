#!/usr/bin/env python
# -- coding: utf-8 --

### 사용할 윤택이에게 
# 객체 선언
# --- = delivery_sign_loc(1~3) # 1이면 PJ, 3이면 본선이니 테스트일땐 1로 고정하던가하고
# delivery_sign_loc.update_PJ(obs) # 이걸로 라이다에서 가공된 장애물 정보 계속 넣어주셈
# 그러다가
# delivery_sign_loc.run() 한번 해주고 (지금까지 누적해온 장애물 좌표를 바탕으로 표지판 좌표를 계산하는 함수)
# delivery_sign_loc.PJ[0] 을 읽어오면 표지판 위치가 [x, y] 로 저장되어 있을 꺼임 

import rospy
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/missions")
import numpy as np

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sub_erp_state import sub_erp_state

from dbscan import DBSCAN
from sub_erp_state import sub_erp_state
from mission_parking import two_dis, find_ind

zone_width = 2.5 # 미터 단위 # 대회장 = 2.5 # 만해광장 = ???
static_loc = True # True면 라이다에 관계없이 고정좌표
static_loc_update = True

x1_raw = [[955816.731, 1951113.137], [955816.731 - 0.2, 1951113.137 - 0.1], [955816.731 - 0.1, 1951113.137 + 0.15], [955816.731, 1951113.137 + 0.2], [955816.731 + 1.0, 1951113.137 + 1.0]]
x2_raw = [[955814.092, 1951105.850], [955814.092 - 0.2, 1951105.850 - 0.1], [955814.092 - 0.1, 1951105.850 + 0.15], [955814.092, 1951105.850 + 0.2], [955814.092 + 1.0, 1951105.850 + 1.0]]
x3_raw = [[955811.454, 1951098.563], [955811.454 - 0.2, 1951098.563 - 0.1], [955811.454 - 0.1, 1951098.563 + 0.15], [955811.454, 1951098.563 + 0.2], [955811.454 + 1.0, 1951098.563 + 1.0], [955811.454 + 1.5, 1951098.563 + 5.0]]
x1 = np.array(x1_raw)
x2 = np.array(x2_raw)
x3 = np.array(x2_raw)

class delivery_sign_loc():
    def __init__(self, where):
        global static_loc
        self.sign_pub = rospy.Publisher('/sign', PointCloud, queue_size = 5)
        self.DB_pickup = DBSCAN(1.5, 4)
        self.DB_delivery = DBSCAN(1.5, 4)
        self.obs_pickup = np.empty((0, 2), float)
        self.obs_delivery = np.empty((0, 2), float)
        self.where = where

        if where == 1:
            self.sign_line = np.load(file = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/obstacle/mission_sign_path.npy")
            self.sign_line_range = [[5, 48], [100, 150]] #0~66
            self.pickup_loc = [[0.0, 0.0]]
            self.delivery_loc = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
            if static_loc == True:
                self.pickup_loc = [[self.sign_line[52][0], self.sign_line[52][1]]]
                self.delivery_loc = [[self.sign_line[105][0], self.sign_line[105][1]], [self.sign_line[120][0], self.sign_line[120][1]], [self.sign_line[135][0], self.sign_line[135][1]]]
            else:
                self.pickup_loc = [[0.0, 0.0]]
                self.delivery_loc = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
            self.pickup_group = 1
            self.delivery_group = 3

        else:
            self.sign_line = np.load(file = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/obstacle/mission_sign_path.npy") #mission_sign_path
            self.sign_line_range = [[5, 48], [100, 150]] # 0.5 미터 단위
            if static_loc == True:
                # 고정좌표여도 뭔가 뜨면 들어가는식? 이라면 이 부분을 else와 통일 해도 됨.
                self.pickup_loc = [[self.sign_line[52][0], self.sign_line[52][1]]]
                self.delivery_loc = [[self.sign_line[105][0], self.sign_line[105][1]], [self.sign_line[120][0], self.sign_line[120][1]], [self.sign_line[135][0], self.sign_line[135][1]]]
            else:
                self.pickup_loc = [[0.0, 0.0]]
                self.delivery_loc = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
            self.pickup_group = 1
            self.delivery_group = 3

    # 건드릴 필요 없음
    def update(self, obs_n):
        for p in obs_n:
            n = find_ind(p, self.sign_line)
            if two_dis(p, self.sign_line[n]) < zone_width:
                if self.sign_line_range[0][0] <= n <= self.sign_line_range[0][1]:
                    self.obs_pickup = np.append(self.obs_pickup, np.array([p]), axis=0)
                elif self.sign_line_range[1][0] <= n <= self.sign_line_range[1][1]:
                    self.obs_delivery = np.append(self.obs_delivery, np.array([p]), axis=0)
            else:
                pass

    def grouping_pickup(self):
        try:
            o = 0
            idx = self.DB_pickup.run(self.obs_pickup)
            while idx != self.pickup_group:
                if (idx > self.pickup_group):
                    self.DB_pickup.minpts += 1
                    o += 1
                elif (idx < self.pickup_group):
                    self.DB_pickup.minpts = 4
                    if o != 0:
                        return 2
                    else:
                        return 3
                if (self.DB_pickup.minpts > (self.obs.size - 1)) :
                    self.DB_pickup.minpts = 4
                    return 0
                idx = self.DB_pickup.run(self.obs_pickup)
            return 1
        except:
            return 0

    def grouping_delivery(self):
        try:
            o = 0
            idx = self.DB_delivery.run(self.obs_delivery)
            while idx != self.delivery_group:
                if (idx > self.delivery_group):
                    self.DB_delivery.minpts += 1
                elif (idx < self.delivery_group):
                    self.DB_delivery.minpts = 3
                    if o != 0:
                        return 2
                    else:
                        return 3
                if (self.DB_delivery.minpts > (self.obs.size - 1)) :
                    self.DB_delivery.minpts = 3
                    return 0
                idx = self.DB_delivery.run(self.obs_delivery)
            return 1
        except:
            return 0

    def sign_specific_pickup(self):
        global static_loc_update
        i = 0
        sign_loc = np.array(self.DB_pickup.cluster)
        for x in sign_loc:
            self.pickup_loc[i] = x[0].mean(axis = 0).tolist()
            i += 1
        if static_loc_update == False and self.where == 2:
            self.pickup_loc = [[self.sign_line[52][0], self.sign_line[52][1]]]
        if static_loc_update == False and self.where == 1:
            self.pickup_loc = [[self.sign_line[52][0], self.sign_line[52][1]]]

# 순서대로 좌표 넣는거 구현
    def sign_specific_delivery(self):
        global static_loc_update
        i = 0
        B = [0, 1, 2]
        s = []
        index_s = [0, 0, 0]
        delivery_loc_1 = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        sign_loc = np.array(self.DB_delivery.cluster)
        for x in sign_loc:
            delivery_loc_1[i] = x[0].mean(axis = 0).tolist()
            i += 1

        for x in range(3):
            if delivery_loc_1[x][0] == 0.0:
                index_s[x] = 99999
            else:
                index_s[x] = find_ind(delivery_loc_1[x], delivery_loc_1)
    
        loc_copy = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        for s in range(3):
            loc_copy[s] = delivery_loc_1[s]

        B[0] = index_s.index(min(index_s))
        B[2] = index_s.index(max(index_s))
        if (B[0] + B[2]) == 1:
            B[1] = 2
        elif (B[0] + B[2]) == 2:
            B[1] = 1
        elif (B[0] + B[2]) == 3:
            B[1] = 0

        for x in range(3):
            delivery_loc_1[x] = loc_copy[B[x]] #

        for x in range(3):
            if delivery_loc_1[x][0] == 0.0:
                pass
            else:
                if x == 0:
                    self.delivery_loc[x] = delivery_loc_1[x]
                else:
                    d1 = find_ind(delivery_loc_1[x - 1], self.sign_line)
                    d2 = find_ind(delivery_loc_1[x], self.sign_line)
                    if (d2 - d1) <= 6:
                        pass
                    else:
                        self.delivery_loc[x] = delivery_loc_1[x]

        if static_loc_update == False and self.where == 2:
            self.delivery_loc = [[self.sign_line[105][0], self.sign_line[105][1]], [self.sign_line[120][0], self.sign_line[120][1]], [self.sign_line[135][0], self.sign_line[135][1]]]
        if static_loc_update == False and self.where == 1:
            self.delivery_loc = [[self.sign_line[105][0], self.sign_line[105][1]], [self.sign_line[120][0], self.sign_line[120][1]], [self.sign_line[135][0], self.sign_line[135][1]]]
    
    def run(self):
        a = self.grouping_pickup()
        if a == 0:
            pass
        elif a == 1:
            self.sign_specific_pickup()
        else:
            self.sign_specific_pickup()

        b = self.grouping_delivery()
        if b == 0:
            pass
        elif b == 1:
            self.sign_specific_delivery()
        elif b == 2 or b == 3:
            self.sign_specific_delivery()
        else:
            self.sign_specific_delivery()
        # print("data_pickup" , self.pickup_loc)
        # print("data_delivery" , self.delivery_loc)

        self.pub_vis_sign()

    def pub_vis_sign(self):
        sign_msg = PointCloud()
        for i in self.delivery_loc:
            p = Point32()
            p.x = i[0]
            p.y = i[1]
            p.z = 0
            sign_msg.points.append(p)
        for i in self.pickup_loc:
            p = Point32()
            p.x = i[0]
            p.y = i[1]
            p.z = 0
            sign_msg.points.append(p)
        self.sign_pub.publish(sign_msg)
        
def main():
    rospy.init_node('delivery_sign_loc', anonymous=True)

    d = delivery_sign_loc(2)
    data = sub_erp_state()

    while not rospy.is_shutdown():

        d.update(data.obs)
        d.run()
        # d.pub_vis_sign()
        print(d.pickup_loc, d.delivery_loc)
        rospy.sleep(0.1)

if __name__ == '__main__':
    main()