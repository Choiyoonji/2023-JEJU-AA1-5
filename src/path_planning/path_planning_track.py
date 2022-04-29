#!/usr/bin/env python
#-*-coding:utf-8-*-

# Python packages
import rospy
import os, sys
import numpy as np

# 필요한 Library
from pure_pursuit_clean import pid_control, pure_pursuit
from trajectory_planner_track import TrajectoryPlanner
from global_path import GlobalPath 

# msg 파일
from macaron_4.msg import erp_read

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))

# 속도에 따른 LD값 적용할건지 (트랙주행에서는 끄기)
Variable_LD = True

class Path_Tracking():

    def __init__(self, path, file = 0):
        # 파일 경로 설정 (경로 파일이 아닌 직접 x,y 좌표 경로를 넣어준다면 file에 0이 아닌 수를 넣어주면 된다)
        self.PP = pure_pursuit()
        self.PID = pid_control(0.1) # 0.1초에 한 번씩 함수가 실행되므로

        if file == 0:
            GLOBAL_NPY = path  #"8jung_test2.npy"
            PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"
            gp_name = PATH_ROOT + GLOBAL_NPY
            glob_path = GlobalPath(gp_name)
        else:
            glob_path = GlobalPath(x = path[0], y = path[1])


        self.path_planner = TrajectoryPlanner(glob_path = glob_path)
    
    
    def det_LD(self, speed):
        if speed <= 80:
            ld = np.round(((speed - 100 + 6)/10) + 10) 
            # ld = 10 
        else:
            ld = np.round(((speed - 100 + 6)/10) + 10) 
            # ld = 10 
        
        if ld >= 20:
            ld = 20
        elif ld <= 5:
            ld = 5

        print(ld, 'ld')
        return ld

    def det_Kd(self):
        Kd = 0

        return Kd

    # 함수 사용법 path_len은 경로를 몇m 앞까지 생성할지.
    # ld는 Tracking 할 때 몇 인덱스 앞의 점을 추적할지. 
    # (전역경로 상의 거리가 0.5m 이므로 ld가 7 이면 3.5m 앞의 점을 추적)
    def gps_tracking(self, pose, heading, obs_xy , path_len, ld, path_num, speed):
        x, y = pose[0], pose[1]

        # 경로 생성 및 선택   
        selected_path = self.path_planner.optimal_trajectory(x, y, heading, obs_xy, path_num = path_num, path_len = path_len)

        #speed, steer, goal 
        goal = [selected_path.x, selected_path.y]

        # 가변 LD 적용 및 ld값이 기본 크루징 일때
        if Variable_LD == True:
            ld = self.det_LD(speed)

        # PID 제어 추가        
        if self.path_planner.current_s == 0:
            D_steer = 0
        else:
            D_steer = self.PID.D_control(self.path_planner.current_q)

        Kd = self.det_Kd()

        P_steer = self.PP.get_steer_state(x, y, heading, ld, goal)
        PID_steer = P_steer + Kd * D_steer + 0
        
        if PID_steer >= 2000:
            PID_steer = 2000
        elif PID_steer <= -2000:
            PID_steer = -2000

        return PID_steer