#!/usr/bin/env python
#-*-coding:utf-8-*-

"""

경로 계획과 경로 추종을 담은 클래스이다.
차의 현재 상태를 받아와서 TrajectoryPlanner와 pure_pursuit_PID으로 보내고 결과값을 받는다.
pid 제어의 파라미터값은 지글러-니콜스 방법을 통해 구한 뒤 실험적으로 조절하였다.

"""

# Python packages
import rospy
import os, sys
import numpy as np

from stanley_controller_PID import stanley_control, pid_control
from trajectory_planner import TrajectoryPlanner
from global_path import GlobalPath
from dwa import DWA

from macaron_5.msg import erp_read

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))

class Path_Tracking():

    def __init__(self, path, file = 0):
        # 파일 경로 설정 (경로 파일이 아닌 직접 x,y 좌표 경로를 넣어준다면 file에 0이 아닌 수를 넣어주면 된다)
        self.stanley = stanley_control()
        self.PID = pid_control(0.1) # I 제어값 계산을 위해. 0.1초에 한 번씩 함수가 실행되므로.

        self.erp_speed = 0.0
        self.erp_steer = 0.0
        self.erp_ENC = 0.0
        self.erp_sub= rospy.Subscriber('/erp_read', erp_read, self.erp_callback, queue_size=1)

        self.selected_path = []

   
        self.planningIntervalCount = 100
        self.selected_path = [0,0]


        if file == 0:
            GLOBAL_NPY = path  #"8jung_test2.npy"
            PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"
            gp_name = PATH_ROOT + GLOBAL_NPY
            glob_path = GlobalPath(gp_name)
        else:
            glob_path = GlobalPath(x = path[0], y = path[1])

        # 경로 생성, 선택
        # self.path_planner = TrajectoryPlanner(glob_path = glob_path)
        self.path_planner = DWA(glob_path = glob_path)
    
    def erp_callback(self, data):
        self.erp_speed = data.read_speed
        self.erp_steer = data.read_steer
        self.erp_ENC = data.read_ENC
        if data.read_gear == 2 and self.erp_speed > 0:
            self.erp_speed *= -1

    def det_Kd(self):
        # 속도 150 에서 Kd 270 이후 속도 10 증가당 Kd 10 감소
        Kd = (-10 / 10) * (self.erp_speed - 150) + 270

        if self.erp_speed >= 180:
            Kd = 50
        elif self.erp_speed >= 110:
            Kd = 150
        else:
            Kd = 400

        return Kd

    def det_Ki(self):
        # Ki = 100
        Ki = 50

        # Ki = 300
        Ki = 100
        return Ki


    def gps_tracking(self, pose, heading, obs_xy = [[0.0, 0.0]], path_len = 4, ld = 8, path_num = 1):
        """경로 생성, 선택, 추종 모든 과정

        Args:
            pose (list): 현재 위치
            heading (double): 현재 헤딩
            obs_xy (list, optional): 장애물 xy 좌표. Defaults to [[0.0, 0.0]].
            path_len (int, optional): 경로를 몇m 앞까지 생성할지. Defaults to 4.
            ld (int, optional): Tracking 할 때 몇 인덱스 앞의 점을 추적할지. Defaults to 8.
            path_num (int, optional): 경로 생성 개수. Defaults to 1.
        """
        x, y = pose[0], pose[1]

        
        # 경로 생성 및 선택;
        self.selected_path = self.path_planner.DWA(x, y, heading, obs_xy)
        # self.selected_path = self.path_planner.optimal_trajectory(x, y, heading, obs_xy, path_num = path_num, path_len = path_len, MACARON_TREAD=1.5)
            

        goal = [self.selected_path.x, self.selected_path.y]

        # PID 제어값 받아오기. 현재 위치가 출발점보다 뒤에 있을 때는 pid 제어 안함.
        if self.path_planner.current_s == 0 :
            D_steer = 0
            I_steer = 0
        else:
            D_steer = self.PID.D_control(self.path_planner.current_q)
            if self.erp_speed == 0 :
                I_steer = self.PID.I_control(0)
            else:
                I_steer = self.PID.I_control(self.path_planner.current_q)

        # pid 제어 파라미터 설정.
        Kp = 1.0
        Kd = self.det_Kd()
        Ki = self.det_Ki()

        # pure pursuit + pid
        P_steer = self.stanley.stanley_control(x, y, heading, self.erp_speed, self.selected_path.x, self.selected_path.y, self.selected_path.yaw)
        # PID_steer = Kp*P_steer + Kd * D_steer + Ki * I_steer

        PID_steer = Kp*P_steer + Kd * D_steer + Ki * I_steer
        # PID_steer = Kp*P_steer

        
        # steer 최대최소 설정
        if PID_steer >= 2000:
            PID_steer = 2000
        elif PID_steer <= -2000:
            PID_steer = -2000

        print("targer_steer: ", PID_steer)

        return PID_steer


    def gps_tracking_parking(self, pose, heading, obs_xy = [[0.0, 0.0]], path_len = 4, ld = 12, path_num = 1):
        """주차 미션용 코드

            :세부 내용 위와 동일
        """
        x, y = pose[0], pose[1]
        selected_path, _ = self.path_planner.optimal_trajectory_parking(x, y, heading, obs_xy, path_num = path_num, path_len = path_len, MACARON_TREAD=1.5)
        goal = [selected_path.x, selected_path.y]

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

        P_steer = self.stanley.stanley_control(x, y, heading, self.speed, selected_path.x, selected_path.y, selected_path.yaw)
        PID_steer = Kp*P_steer + Kd * D_steer + Ki * I_steer + 0
        
        if PID_steer >= 2000:
            PID_steer = 2000
        elif PID_steer <= -2000:
            PID_steer = -2000


        return PID_steer