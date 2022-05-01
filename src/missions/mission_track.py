#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/sensor")
from math import cos, sin, pi, atan2, sqrt, asin, isnan
import numpy as np
import copy
from sensor.dbscan_track import DBSCAN
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import PointCloud
import scipy.interpolate as interp
from path_planning_tracking import Path_Tracking
from global_path import GlobalPaths

# Parameters
WB = 1.03  # [m] wheel base of vehicle
MAX_STEER = 2000
MIN_STEER = -2000
n_path_points=15

# Parameters in matching class
offset_dis = 1.0 # offset 계산시 얼마나 떨어져 있는 점까지 찾을 것인지
matching_dis = 1.0 # 제드 라이다 매칭할때 얼마나 떨어져 있는 점까지 매칭 할 것인지
lidar_cal_dis = 1.0 # 라이다 점 추적할 때 얼마나 떨어져 있는 점까지 추적 할 것인지
w_dis = 3.0 # 감지 안된 점 보정시 계산 할 도로 폭 

class mission_track:

    def divide(self, obs):
        right = []
        left = []
        for h in range(len(obs)):
            if obs[h][1] > 0 and float(obs[h][1]/obs[h][0]) > 1 :
                left.append([obs[h][0], obs[h][1],0.0])
            elif obs[h][1] < 0 and float(obs[h][1]/obs[h][0]) < -1 :
                right.append([obs[h][0], obs[h][1],0.0])
        
        return left, right

    def cluster(self, obs):
        try:
            dbscan = DBSCAN(obs,0.2,3)
            idx,noise = dbscan.run()
            global g_cluster
            g_cluster,n_cluster = dbscan.sort()
            # if len(g_cluster) == 1:
            #     append.g_cluster([int(g_cluster[0]),int(g_cluster[1])])
            #     return g_cluster
            # means = np.array(g_cluster).mean(axis=0)
            # a = list(means)
            # for k in range(len(g_cluster)):
            #     for j in range(1, len(g_cluster)):
            #         if abs(g_cluster[k] - g_cluster[j]) > 2:
            #             del g_cluster[j]
        except np.AxisError:
            pass
        except UnboundLocalError:
            pass

        return g_cluster

    def filter(self, obs):
        # cl = track.cluster(np.array(obs))
        cll = np.array(obs)
        # print(cll)
        clll = []
        for i in cll:
            clll_x_t = []
            clll_y_t = []
            for j in i[0]:
                clll_x_t.append(j[0])
                clll_y_t.append(j[1])
            clll.append([np.mean(clll_x_t), np.mean(clll_y_t)])

        # if len(clll) == 0:
        #     del clll[:]
        return clll

    def length(self, obs):
        while len(obs) < 1:
            if len(obs) == 0:
                obs.append([0,0])
                obs.append([1,1])

        return obs


class path_planning:
    def spline(self, cone):
        x=[]
        y=[]
        for i in range(len(cone)):
            x.append(cone[i][0])
            y.append(cone[i][1])
        t = np.linspace(0.0, len(x) - 1, len(x))
        try:
            spl_x = interp.interp1d(t, x)
            spl_y = interp.interp1d(t, y)
            xnew = np.linspace(0.0, len(x) - 1, n_path_points)
            return spl_x(xnew), spl_y(xnew)
        except:
            return x, y

    def combine(self, rx, ry, lx, ly):
        answer = []
        for (ix, iy) in zip(rx,ry):
            d = ((ix - lx)**2 + (iy - ly)**2)**0.5
            min = np.min(d)
            ind = np.where(d == min)
            answer.append([(ix+lx[ind[0][0]])/2, (iy+ly[ind[0][0]])/2])
                
        return answer
    
    def tracking(self, speed, path, rubber, Left_x, Left_y, Right_x, Right_y):
        x=[]
        y=[]
        for i in range(len(path)):
            x.append(path[i][0])
            y.append(path[i][1])
        point = [x,y]
        PT = Path_Tracking(point, file = 1)
        
        rubber_obs = []
        # for i in range(len(Left_x)):
        #     rubber_obs.append([Left_x[i], Left_y[i]])
        # for i in range(len(Right_x)):
        #     rubber_obs.append([Right_x[i], Right_y[i]])
        rubber_obs.extend(rubber)
        target_steer = PT.gps_tracking(pose = [0,0], heading = 0, obs_xy = rubber_obs, path_len = 3, ld = 6, path_num = 5, speed = speed)
        
        return target_steer

class visual:
    def __init__(self):
        self.goal_pub = rospy.Publisher("/goal_point", Point, queue_size=1)
        self.lidar_pub = rospy.Publisher('lidar_rubber', PointCloud, queue_size=1)
        self.left_final_pub = rospy.Publisher('left_rubber_final', PointCloud, queue_size=1)
        self.right_final_pub = rospy.Publisher('right_rubber_final', PointCloud, queue_size=1)
        self.track_gb_pub = rospy.Publisher('/track_gbpath', PointCloud, queue_size = 1)
        self.left_path_pub = rospy.Publisher('/left_path', PointCloud, queue_size = 1)
        self.right_path_pub = rospy.Publisher('/right_path', PointCloud, queue_size = 1)

    def array_msg(self, array):
        obs = PointCloud()
        # obs_clean = [[955920.0, 1950958.0],[955921.0, 1950958.0],[955919.0, 1950958.0],[955921.0, 1950959.0],[955922.0, 1950960.0],[955918.0, 1950958.0],[955920.0, 1950960.0]]
        
        for i in array:
            p = Point32()
            p.x = i[0]
            p.y = i[1]
            p.z = 0
            obs.points.append(p)

        return obs

    def pub_goal(self, x, y):
        gp = Point()
        gp.x = x
        gp.y = y
        gp.z = 0
        self.goal_pub.publish(gp)

    def pub_lidar_vis(self, rubber):
        lidar_msg = self.array_msg(rubber)
        self.lidar_pub.publish(lidar_msg)

    def pub_left_final_vis(self, rubber):
        left_final_msg = self.array_msg(rubber)
        self.left_final_pub.publish(left_final_msg)
    
    def pub_right_final_vis(self, rubber):
        right_final_msg = self.array_msg(rubber)
        self.right_final_pub.publish(right_final_msg)

    def pub_track_gb_vis(self, rubber):
        track_gb_msg = self.array_msg(rubber)
        self.track_gb_pub.publish(track_gb_msg)
    
    def pub_left_path_vis(self, x, y):
        rubber = []
        for i in range(len(x)):
            rubber.append([x[i], y[i]])
        pub_msg = self.array_msg(rubber)
        self.left_path_pub.publish(pub_msg)
    
    def pub_right_path_vis(self, x, y):
        rubber = []
        for i in range(len(x)):
            rubber.append([x[i], y[i]])
        pub_msg = self.array_msg(rubber)
        self.right_path_pub.publish(pub_msg)
    