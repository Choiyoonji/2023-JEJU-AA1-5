#!/usr/bin/env python
#-*-coding:utf-8-*-

# Python packages
import rospy
import os, sys
import numpy as np
#import matplotlib.pyplot as plt #pid 제어값 튜닝을 위해 사용 

# 필요한 Library
from pure_pursuit_PID import pid_control, pure_pursuit
from trajectory_planner import TrajectoryPlanner
from global_path import GlobalPath 

# msg 파일
from std_msgs.msg import Int16, Int32

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))

# 속도에 따른 LD값 적용할건지 (트랙주행에서는 끄기)
Variable_LD = True

class Path_Tracking():

    def __init__(self, path, file = 0):
        # 파일 경로 설정 (경로 파일이 아닌 직접 x,y 좌표 경로를 넣어준다면 file에 0이 아닌 수를 넣어주면 된다)
        self.PP = pure_pursuit()
        self.PID = pid_control(0.1) # 0.1초에 한 번씩 함수가 실행되므로

        if Variable_LD == True:
            self.erp_speed = 0.0
            self.erp_steer = 0.0
            self.erp_ENC = 0.0
            self.erp_sub_speed= rospy.Subscriber('speed_read', Int16, self.erp_callback_speed, queue_size=1)
            self.erp_sub_steer= rospy.Subscriber('steer_read', Int32, self.erp_callback_steer, queue_size=1)

        if file == 0:
            GLOBAL_NPY = path  #"8jung_test2.npy"
            PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"
            gp_name = PATH_ROOT + GLOBAL_NPY
            glob_path = GlobalPath(gp_name)
        else:
            glob_path = GlobalPath(x = path[0], y = path[1])

        self.path_planner = TrajectoryPlanner(glob_path = glob_path)
    
    def erp_callback_speed(self, data):
        self.erp_speed = data.data
        
    def erp_callback_steer(self, data):
        self.erp_steer = data.data
    
    def det_LD(self):
        ld = int(((self.erp_speed - 100 + 6)/10) + 10) + 3
        
        if ld >= 23:
            ld = 23
        elif ld <= 8:
            ld = 8
        
        return ld

    def det_Kd(self):
        # 속도 150 에서 Kd 270 이후 속도 10 증가당 Kd 10 감소
        # Kd = (-10 / 10) * (self.erp_speed - 150) + 270


        if self.erp_speed >= 50:
            Kd = 0.5
        elif self.erp_speed >= 30:
            Kd = 1.5
        else:
            Kd = 4

        return Kd

    def det_Ki(self):
        Ki = 1
        return Ki

    # 함수 사용법 path_len은 경로를 몇m 앞까지 생성할지.
    # ld는 Tracking 할 때 몇 인덱스 앞의 점을 추적할지. 
    # (전역경로 상의 거리가 0.5m 이므로 ld가 7 이면 3.5m 앞의 점을 추적)
    def gps_tracking(self, pose, heading, obs_xy = [[0.0, 0.0]], path_len = 4, ld = 8, path_num = 1):
        x, y = pose[0], pose[1]
        # 경로 생성 및 선택   
        selected_path = self.path_planner.optimal_trajectory(x, y, heading, obs_xy, path_num = path_num, path_len = path_len, MACARON_TREAD=1.5)#예선 : 1.5, 본선 : 3.0

        # print("&&&&&&&&&&&&&&&&&&pose: ",pose)
        #speed, steer, goal 
        goal = [selected_path.x, selected_path.y]

        # print('========== goal: ', goal)
        
        # 가변 LD 적용 및 ld값이 기본 크루징 일때
        if Variable_LD == True and ld == 8 and path_len == 4:
            ld = self.det_LD()
        if path_num == 5:
            ld = ld - 3
            if ld >= 23:
                ld = 23
            elif ld <= 8:
                ld = 8

        # PID 제어 추가        

        if self.path_planner.current_s == 0 :
            D_steer = 0
            I_steer = 0
        else:
            D_steer = self.PID.D_control(self.path_planner.current_q)
            if self.erp_speed == 0 :
                I_steer = self.PID.I_control(0)
            else:
                I_steer = self.PID.I_control(self.path_planner.current_q)

        Kp = 1.0
        Kd = self.det_Kd()
        Ki = self.det_Ki()
###########################################################
	#pid테스트부분(기존엔 Ki=400,Kd=300사용)
        # print('Kp:',Kp ,'Ki:',Ki ,'Kd', Kd)
###########################################################

        P_steer = self.PP.get_steer_state(x, y, heading, ld, goal)
        PID_steer = Kp*P_steer + Kd * D_steer + Ki * I_steer + 0 # 5는 얼라이먼트 보정값

        
        if PID_steer >= 22:
            PID_steer = 22
        elif PID_steer <= -22:
            PID_steer = -22

        print('***********P_steer',P_steer)
        return PID_steer


    def gps_tracking_parking(self, pose, heading, obs_xy = [[0.0, 0.0]], path_len = 4, ld = 12, path_num = 1):
        x, y = pose[0], pose[1]
        selected_path, _ = self.path_planner.optimal_trajectory_parking(x, y, heading, obs_xy, path_num = path_num, path_len = path_len, MACARON_TREAD=1.5)
        goal = [selected_path.x, selected_path.y]

        if Variable_LD == True and ld == 8 and path_len == 4:
            ld = self.det_LD()
        if path_num == 5:
            ld = ld - 3
            if ld >= 23:
                ld = 23
            elif ld <= 8:
                ld = 8

        if self.path_planner.current_s == 0 :
            D_steer = 100
            I_steer = 100
        else:
            D_steer = self.PID.D_control(self.path_planner.current_q)
            if self.erp_speed == 0 :
                D_steer = 0
                I_steer = 0    
            else:
                D_steer = 0
                I_steer = 0
        Kp = 1.5
        Kd = self.det_Kd()
        Ki = self.det_Ki()

        P_steer = self.PP.get_steer_state(x, y, heading, ld, goal)
        PID_steer = Kp*P_steer + Kd * D_steer + Ki * I_steer + 0
        
        if PID_steer >= 2000:
            PID_steer = 2000
        elif PID_steer <= -2000:
            PID_steer = -2000


        return PID_steer