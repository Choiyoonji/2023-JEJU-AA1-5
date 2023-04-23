#!/usr/bin/env python
# -*-coding:utf-8-*-

import os
import sys

import numpy as np
import rospy
from math import sqrt

# msg 파일
from macaron_5.msg import erp_read

# 필요한 Library
from dwa import DWA
from global_path import GlobalPath
from pure_pursuit_PID_for_dwa import PidControl, PurePursuit

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))

# noinspection PyPep8Naming
class Path_Tracking_DWA:
    def __init__(self, path, file=0):
        # 파일 경로 설정 (경로 파일이 아닌 직접 x,y 좌표 경로를 넣어 준다면 file 에 0이 아닌 수를 넣어 주면 된다)
        self.PP = PurePursuit()
        self.PID = PidControl(0.1)  # 0.1초에 한 번씩 함수가 실행 되므로
        self.Ki = 1.0

        if file == 0:
            GLOBAL_NPY = path
            PATH_ROOT = (os.path.dirname(
                os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))) + "/path/npy_file/path/"
            gp_name = PATH_ROOT + GLOBAL_NPY
            glob_path = GlobalPath(gp_name)
        else:
            glob_path = GlobalPath(x=path[0], y=path[1])

        self.path_planner = DWA(glob_path=glob_path)

    # noinspection PyMethodMayBeStatic
    def det_Kd(self, speed):
        if speed >= 50:
            Kd = 0.5
        elif speed >= 30:
            Kd = 1.5
        else:
            Kd = 4
        return Kd

    def gps_tracking(self, pose, heading, speed, steer, obs_xy=None):
        if obs_xy is None:
            obs_xy = [[0.0, 0.0]]

        x, y = pose[0], pose[1]
        selected_path = self.path_planner.DWA(x, y, heading, speed, steer, obs_xy)   # 경로 생성 및 선택
        goal = [list(x) for x in zip(*selected_path)][0:2]

        # PID 제어 추가
        if self.path_planner.current_s == 0:
            D_steer = 0
            I_steer = 0
        else:
            D_steer = self.PID.D_control(self.path_planner.current_q)
            if speed == 0:
                I_steer = self.PID.I_control(0)
            else:
                I_steer = self.PID.I_control(self.path_planner.current_q)

        Kp = 1.0
        Kd = self.det_Kd(speed)
        Ki = self.Ki

        P_steer = self.PP.get_steer_state(x, y, heading, goal)
        PID_steer = Kp * P_steer + Kd * D_steer + Ki * I_steer

        return np.clip(PID_steer, -22, 22)
