#!/usr/bin/env python
# -- coding: utf-8 --
"""
Path tracking simulation with pure pursuit steering and PID speed control.
author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)
"""
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
from std_msgs.msg import Int64
import time

from geometry_msgs.msg import Point

# Parameters
k = 0.15  # look forward gain
Lfc = 2.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 0.75  # [m] wheel base of vehicle
MAX_STEER = 22
MIN_STEER = -22

class visual:
    def __init__(self):
        self.goal_pub = rospy.Publisher("/goal_point", Point, queue_size=1)

    def pub_goal(self, x, y):
        gp = Point()
        gp.x = x
        gp.y = y
        gp.z = 0
        self.goal_pub.publish(gp)

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
    
    def update(self,x,y,yaw):                         
        self.x = x
        self.y = y
        self.yaw = yaw
        print("yaw: ")
        print(yaw)  

def proportional_control(target, current):     # 급발진 방지
    a = Kp * (target - current)

    return a

def pure_pursuit_steer_control(state, goal, ind, g_dis):
    try:
        tx = goal[0][ind]
        ty = goal[1][ind]
    except:
        tx = goal[0][-1]
        ty = goal[1][-1]
    v = visual()
    v.pub_goal(tx , ty)

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
    delta = math.atan2(2.0 * WB * math.sin(alpha) / g_dis, 1.0)   #이게 pure_pursuit 공식
    
    return delta
class pure_pursuit:                  

    def __init__(self):
        self.target_speed = 10.0 / 3.6  # [m/s]
        self.state = State()
        self.ind = 0

    # 함수를 사용할 때 velocity 위치에 ld를 넣는데 이유가 뭐지..?
    def get_steer_state(self, x, y, heading, ind, goal):
        self.state.update(x,y, heading)
        try:
            g_dis = np.hypot(x-goal[0][ind],y-goal[1][ind])
        except:
            g_dis = np.hypot(x-goal[0][-1],y-goal[1][-1])
        steer = pure_pursuit_steer_control(self.state, goal, ind, g_dis)
        
        target_steer = np.rad2deg(steer)
        if target_steer > MAX_STEER:
            target_steer = MAX_STEER
        if target_steer < MIN_STEER:
            target_steer = MIN_STEER

        return -target_steer

class pid_control:
    def __init__(self, time):
        self.last_q = 0
        self.I_value = 0
        self.time = time
        
    def D_control(self, q):
        D_value = (q - self.last_q) / self.time
               
        self.last_q = q
        return D_value
    
    def I_control(self, q):
        if self.I_value * q <= 0 or abs(q) <= 0.3:
            self.I_value = 0
        self.I_value += q * self.time

        # I value 만땅 2로 제한
        if self.I_value >= 2.0:
            self.I_value = 2.0
        elif self.I_value <= -2.0:
            self.I_value = -2.0

        return self.I_value



    '''
    def get_ld(self,velocity):
        Lf = k * velocity/3.6 + Lfc
        if Lf > 8 :
            Lf = 8
        return int(Lf)
    '''

def main():
    #  target course
    cx,cy = get_manhae_course(1.0)
    
    target_speed = 10.0 / 3.6  # [m/s]

    # initial state
    state = State(x=cx[0], y=cy[0], yaw=0.0, v=0.0)

    lastIndex = len(cx) - 1
    target_course = TargetCourse(cx, cy)
    target_ind, LD = target_course.search_target_index(state)

    while lastIndex > target_ind:

        # Calc control input
        speed = proportional_control(target_speed, state.v)   #속도
        steer, target_ind = pure_pursuit_steer_control(       #조향 (rad), 현재ind 
            state, target_course, target_ind)
        
        #print("angle",np.rad2deg(steer))
        state.update(speed, steer)  # Control vehicle


    # 오류
    assert lastIndex >= target_ind, "Cannot goal"

if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
