#!/usr/bin/env python
#-*-coding:utf-8-*-

# Python packages
import rospy
import time
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float64
import numpy as np
from math import sin, cos, tan, pi, isnan
#from cv2 import getGaussianKernel
import os, sys
#sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from global_path import GlobalPath
import polynomial as polynomial
import frenet_path as frenet_path

# Cost Weight
W_OFFSET = 1 #safety cost 가중치
W_CONSISTENCY = 0.3 #smoothness cost 가중치
MACARON_TREAD = 3 # 충돌 지름
ROAD_WIDTH = 3.4

#parameter
sl_d = 0.5      # sl 경로 사이의 거리 (m)

# mode 1은 곡률, mode 2 는 yaw값 비교
mode = 2

class TrajectoryPlanner: # path planner

    def __init__(self, glob_path):
        self.last_selected_path = frenet_path.Frenet_path() # for consistency cost
        self.glob_path = glob_path

        #중앙차선
        PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/" #/home/gigi/catkin_ws/src/macaron_3/
        self.center = []
        #self.center.extend(np.load(file=PATH_ROOT+"obstacle/" + "curb_PJ1.npy")) # 팔정도
        self.center.extend(np.load(file=PATH_ROOT+"obstacle/" + "kcity_tryout_solidline.npy")) #kcity
        self.center.extend(np.load(file=PATH_ROOT+"obstacle/" + "kcity_final_busline.npy")) #kcity
        # self.center.extend(np.load(file=PATH_ROOT+"obstacle/" + "PG_solidline.npy")) #대운동장
       
        self.candidate_pub = rospy.Publisher('/CDpath', PointCloud, queue_size = 3)
        self.selected_pub = rospy.Publisher('/SLpath', PointCloud, queue_size = 3)
        self.curvature_pub = rospy.Publisher("curvature", Float64, queue_size=1)

        self.obstacle_time = 0
        self.visual = True

        self.current_s = 0
        self.current_q = 0

        self.S_MARGIN = 5    # 생성한 경로 끝 추가로 경로 따라서 생성할 길이

        
    def visual_candidate_5(self, candidate_paths):
        self.cd_path = PointCloud()

        # 첫번째 경로
        # cd1_x = candidate_paths[0].x
        # cd1_y = candidate_paths[0].y
        for i in range(len(candidate_paths)):
            # print(candidate_paths[i].x[0])
            for j in range(len(candidate_paths[i].x)):
                p = Point32()
                p.x = candidate_paths[i].x[j]
                p.y = candidate_paths[i].y[j]
                p.z = 0
                self.cd_path.points.append(p)

        self.candidate_pub.publish(self.cd_path)
    
    def visual_selected(self, selected_path):
        self.sl_path = PointCloud()

        # sl_x = selected_path[0].x
        # sl_y = selected_path[0].y

        for i in range(len(selected_path.x)):
            p = Point32()
            p.x = selected_path.x[i]
            p.y = selected_path.y[i]
            p.z = 0
            self.sl_path.points.append(p)

        self.selected_pub.publish(self.sl_path)

    def max_curvature_pub(self, selected_path, collision_count, path_len, heading):
        
        if mode == 1:
            max_max = max(selected_path.k[((path_len*2) + 1) :])
            min_min = abs(min(selected_path.k[((path_len*2) + 1) :]))
            # print((selected_path.yaw[-1]))
            # print(selected_path.k[((path_len*2) + 1) :])

            if max_max <= min_min:
                max_max = min_min
            
            # 경로를 펼치면 무조건 최대 감속
            if collision_count == True:
                max_max = 1.4

        ##############헤딩과 마지막 yaw 값 비교방식##############
        elif mode == 2:
            path_yaw = selected_path.yaw[-1]
            max_max = abs(heading - path_yaw)
            
            # yaw 값이 360도를 넘어갈때
            if max_max >= pi:
                max_max = 2*pi - max_max
            
            # 경로를 펼치면 무조건 최대 감속
            if collision_count == True:
                self.obstacle_time = time.time() + 3
                max_max = 90 * pi/180
            if self.obstacle_time > time.time():
                max_max = 90 * pi/180
        max_cur = Float64()
        max_cur.data = max_max

        self.curvature_pub.publish(max_cur)
    
    def set_global_path(self, glob_path):
        self.glob_path = glob_path


    def generate_path(self, si, qi, dtheta, ds = 5, qf = ROAD_WIDTH/2, path_num = 5): 
        # (si, qi): 시작상태, dtheta: heading - ryaw, ds: polynomial의 길이, qf: 종료상태 q
        candidate_paths = [] # 후보경로들은 frenet_path class가 리스트 안에 담긴다. 
        sf_final = 0 # 최종 s값 중간 경로만 길게 뻗기 위해서 만듦
        sf = si + ds + self.S_MARGIN # 종료상태 s
        sf_side = sf - 5.0
        
        if path_num == 5:
            path_num = 3

        # generate path to each offset goal
        for qf_ in np.linspace(qf, -qf, path_num): # 양수부터 차례대로 생성
        # for qf_ in [ROAD_WIDTH, 1, 0, -ROAD_WIDTH/2, -ROAD_WIDTH]: # 양수부터 차례대로 생성
            # 가운데 경로만 길게 뻗기
            if abs(qf_) <= 0.1:
                sf_final = sf
            elif 1.0 < abs(qf_) < 2.0:
                sf_final = sf - 1.0
            else: # 나머지 경로는 짧게 뻗기
                sf_final = sf_side
            fp = frenet_path.Frenet_path() # 경로. 이 안에 모든 정보가 담긴다.
            qs = polynomial.cubic_polynomial(si, qi, dtheta, ds, qf_)  
            fp.s = [s for s in np.arange(si, sf_final, sl_d)]
            fp.q = [qs.calc_point(s) for s in fp.s]
            # 각 경로의 x, y, yaw, kappa계산
            for i in range(len(fp.s)): 
                x, y = self.glob_path.sl2xy(fp.s[i], fp.q[i])

                yaw = self.glob_path.get_current_reference_yaw()
                rkappa = self.glob_path.get_current_reference_kappa()
                fp.x.append(x)
                fp.y.append(y)
                path_yaw = yaw
                if path_yaw <= 0:
                    # print(path_yaw, 'pi')
                    path_yaw = 2 * pi + path_yaw
                fp.yaw.append(path_yaw)
                fp.k.append(qs.calc_kappa(fp.s[i], rkappa))
            
            # calculate path cost
            fp.offset_cost = abs(qf_)
            fp.consistency_cost = self.calc_consistency_cost(fp.q, self.last_selected_path.q)
            fp.total_cost = W_CONSISTENCY * fp.consistency_cost + W_OFFSET * fp.offset_cost
            
            candidate_paths.append(fp)

        return candidate_paths


    def calc_consistency_cost(self, target_q, last_selected_q):

        consistency_cost = 0
        select_q_len = len(last_selected_q)
        if select_q_len <= 0:
            return 0
        for i in range(0, select_q_len):
            if i >= len(target_q):
                break
            consistency_cost += abs(target_q[i] - last_selected_q[i])
        consistency_cost /= select_q_len
            
        return consistency_cost


    def __select_optimal_trajectory(self, candidate_paths, obs_xy):
        mincost = candidate_paths[0].total_cost
        select_path = None
        collision = False
        center_collision = False
        self.non_center = []
        num = 0
        
        for fp in candidate_paths:
            num += 1
            for xy in self.center:
                if self.check_center(xy[0], xy[1], fp.x[6], fp.y[6]):
                    center_collision = True
                    break
            if center_collision:
                self.non_center.append(num-1)
                print(self.non_center)
                center_collision = False
                continue

            for xy in obs_xy:
                if self.check_collision(xy[0], xy[1], fp.x, fp.y):
                    collision = True
                    break
            if collision :
                collision = False
                continue
            
            if mincost >= fp.total_cost:
                mincost = fp.total_cost
                select_path = fp

        return select_path


    def check_collision(self, obs_x, obs_y, target_xs, target_ys):
        d = [((ix - obs_x)**2 + (iy - obs_y)**2)**0.5
                for (ix, iy) in zip(target_xs, target_ys)]

        collision = any([di <= (MACARON_TREAD/2) for di in d])
        if collision:
                print('장애물 감지!')
                self.current_q = 0
                return True

        return False

    def check_center(self, obs_x, obs_y, target_xs, target_ys):
        d = ((target_xs - obs_x)**2 + (target_ys - obs_y)**2)**0.5

        collision = (d <= (MACARON_TREAD/2))
        if collision:
                print('중앙선 침범!')
                return True

        return False

    def calc_collision_distance(self, obs_x, obs_y, target_xs, target_ys):
        d = [((ix - obs_x)**2 + (iy - obs_y)**2)
                for (ix, iy) in zip(target_xs, target_ys)]
    
        return sum(d)


    def __select_longest_trajectory(self, candidate_paths, obs_xy):
        max_distance = 0
        original_candidate = candidate_paths
        #print(candidate_paths)
        candidate_paths = np.delete(candidate_paths,self.non_center)
        #print(candidate_paths)
        
        #try:
        select_path = candidate_paths[0]
        for fp in candidate_paths:
            cur_distance = 0
            for xy in obs_xy:
                cur_distance += self.calc_collision_distance(xy[0], xy[1], fp.x, fp.y)
            if max_distance < cur_distance:
                max_distance = cur_distance
                select_path = fp

        select_path = candidate_paths[-1]
        '''
        except:
            select_path = original_candidate[0]
            for fp in original_candidate:
                cur_distance = 0
                for xy in obs_xy:
                    cur_distance += self.calc_collision_distance(xy[0], xy[1], fp.x, fp.y)
                if max_distance < cur_distance:
                    max_distance = cur_distance
                    select_path = fp
'''
        return select_path


    def optimal_trajectory(self, x, y, heading, obs_xy, qf = ROAD_WIDTH, path_num = 5, path_len = 4):
        collision_count = False
        if path_num == 5:
            self.S_MARGIN = 6 # 3차 사전주행 값 3
        else:
            self.S_MARGIN = 12 # 3차 사전주행 값 5

        si, qi = self.glob_path.xy2sl(x, y)
        self.current_s = si
        self.current_q = qi
        ryaw = self.glob_path.get_current_reference_yaw()
        dtheta = heading - ryaw
        safe_candidate_paths = self.generate_path(si, qi, dtheta, path_len, 0, 1)
        
        if path_num == 1:
            if self.visual == True:
                self.visual_selected(safe_candidate_paths[0])
                self.max_curvature_pub(safe_candidate_paths[0], collision_count, path_len, heading)
            return safe_candidate_paths[0]

        selected_path = self.__select_optimal_trajectory(safe_candidate_paths, obs_xy)
        if selected_path is None:
            collision_count = True
            safe_candidate_paths = self.generate_path(si, qi, dtheta, path_len, qf, path_num)
            ############### RVIZ 비쥬얼 코드 ##############
            if self.visual == True:
                self.visual_candidate_5(safe_candidate_paths)
            ##############################################
            selected_path = self.__select_optimal_trajectory(safe_candidate_paths, obs_xy)

            if selected_path is None:
                print("nothing is selected!!!!!!!!!!!!!!!!!")
                selected_path = self.__select_longest_trajectory(safe_candidate_paths,obs_xy)
        
        self.last_selected_path = selected_path
        ############### RVIZ 비쥬얼 코드 ##############
        if self.visual == True:
            self.visual_selected(selected_path)
            self.max_curvature_pub(selected_path, collision_count, path_len, heading)

        ##############################################

        return selected_path