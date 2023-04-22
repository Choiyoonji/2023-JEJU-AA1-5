#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
import numpy as np
from math import *

from geometry_msgs.msg import Point

# Parameters
Kp = 1.0  # speed proportional gain
WB = 1.03  # [m] wheelbase of vehicle
MAX_STEER = 22
MIN_STEER = -22


class Visual:
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

    def update(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw


def pure_pursuit_steer_control(state, goal, g_dis):
    tx = goal[0][-1]
    ty = goal[1][-1]
    v = Visual()
    v.pub_goal(tx, ty)

    alpha = atan2(ty - state.y, tx - state.x) - state.yaw
    delta = atan2(2.0 * WB * sin(alpha) / g_dis, 1.0)  # pure_pursuit 공식
    return delta


class PurePursuit:
    def __init__(self):
        self.state = State()

    def get_steer_state(self, x, y, heading, goal):
        self.state.update(x, y, heading)
        g_dis = sqrt((x - goal[0][-1]) ** 2 + (y - goal[1][-1]) ** 2)
        steer = pure_pursuit_steer_control(self.state, goal, g_dis)
        return np.clip(-np.rad2deg(steer), MIN_STEER, MAX_STEER)

class PidControl:
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
        return np.clip(self.I_value, -2.0, 2.0)
