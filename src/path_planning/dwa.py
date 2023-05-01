#!/usr/bin/env python3
from math import *

import numpy as np
import rospy
from geometry_msgs.msg import Point32
from macaron_5.msg import erp_read
from sensor_msgs.msg import PointCloud
from sub_erp_state import sub_erp_state


class DWA:
    def __init__(self, glob_path):
        self.glob_path = glob_path
        self.candidate_pub = rospy.Publisher('/CDpath', PointCloud, queue_size=3)
        self.selected_pub = rospy.Publisher('/SLpath', PointCloud, queue_size=3)

        self.visual = True
        self.cd_path = None
        self.sel_path = None

        # 로봇의 운동학적 모델 상수 설정
        self.max_speed = 3  # 최고 속도 [m/s]
        self.max_steer = np.deg2rad(22)  # 22도 [deg]
        self.max_a = 0.5  # 내가 정하면 됨 [m/s^2]
        self.max_steer_a = np.deg2rad(22)

        self.length = 1.35  # 차 길이 [m]
        self.width = 0.77  # 차 폭 [m]
        self.tread = 0.67  # 같은 축의 바퀴 중심 간 거리 [m]
        self.wheel_base = 0.75  # 차축 간 거리 [m]

        self.current_s = 0.0
        self.current_q = 0.0

        self.predict_time = 0.35  # 미래의 위치를 위한 예측 시간
        self.search_frame = 5  # 정수로 입력 (range 에 사용)
        self.DWA_search_size = [3, 9]  # Dynamic Window 에서 [vel, steer] -> (vel) x (steer) 로 분할 , 정수로 넣어 줘야 함. 무조건 홀수.
        self.obstacle_force = 2.0  # 2m

        # cost function 에서 사용 되는 가중치
        self.w1 = 1  # global path 와의 이격
        self.w2 = 2  # obs 와의 거리
        self.w3 = 0  # global path 와의 heading 차이

    # ↓↓ 비주얼 코드 ↓↓
    def visual_candidate_paths(self, candidate_paths):
        self.cd_path = PointCloud()
        for i in range(len(candidate_paths)):
            for j in range(len(candidate_paths[i])):
                p = Point32()
                p.x = candidate_paths[i][j][0]
                p.y = candidate_paths[i][j][1]
                p.z = 0
                self.cd_path.points.append(p)
        self.candidate_pub.publish(self.cd_path)

    def visual_selected_path(self, selected_path):
        self.sel_path = PointCloud()
        for i in range(len(selected_path)):
            p = Point32()
            p.x = selected_path[i][0]
            p.y = selected_path[i][1]
            p.z = 0
            self.sel_path.points.append(p)
        self.selected_pub.publish(self.sel_path)
    # ↑↑ 비주얼 코드 ↑↑

    # noinspection PyMethodMayBeStatic
    def convert_coordinate_l2g(self, d_x, d_y, d_theta):  # local -> global 좌표 변환 함수
        d_theta = -pi / 2 + d_theta
        trans_matrix = np.array([[cos(d_theta), -sin(d_theta), 0],  # 변환 행렬
                                 [sin(d_theta), cos(d_theta), 0],
                                 [0, 0, 1]])
        d_theta = pi / 2 + d_theta
        return np.dot(trans_matrix, np.transpose([d_x, d_y, d_theta]))
        # return : local coordinate 에서의 [d_x, d_y, d_theta] 를 global coordinate 에서의 [d_x', d_y', d_theta'] 로 반환

    def generate_predict_point(self, x, y, velocity, steer, heading):  # local 좌표계 에서 예측 점 좌표를 구해서 global 좌표로 return
        tan_dis = velocity * self.predict_time  # 접선 이동 거리 (= 호의 길이로 사용할 예정)
        if steer != 0:  # steer 가 0일 때의 ZeroDivisionError 를 해결하기 위한 if-else 문
            R = self.wheel_base / tan(-steer)  # 자전거 모델로 가정, (곡률 반경) = (차축 간 거리) / tan(조향각)
        else:
            R = float('inf')

        theta, future_pos = 0.0, []
        for i in range(self.search_frame):
            if R == float('inf'):
                predict_point = [0, tan_dis * (i + 1), theta]
            else:
                theta += tan_dis / R
                predict_point = [R * (1 - cos(theta)), R * sin(theta), theta]  # [d_x, d_y, d_theta] at local coordinate
            pos = np.transpose(self.convert_coordinate_l2g(predict_point[0], predict_point[1], theta + heading))
            future_pos.append([round(x + pos[0], 4), round(y + pos[1], 4), round(pos[2], 4)])
        return future_pos  # return 값은 global coordinate 의 예측 점 x, y 좌표  -> [[x1, y1, theta1], [x2, y2, theta2], .....]

    def calc_dynamic_window(self, velocity, steer=0.0):
        DWA_velocity = velocity + self.max_a
        DWA_step_rot = 2 * self.max_steer_a / (self.DWA_search_size[1] - 1)
        DWA_steer = [steer - self.max_steer_a + DWA_step_rot * i for i in range(self.DWA_search_size[1]) if
                     abs(steer - self.max_steer_a + DWA_step_rot * i) <= self.max_steer]
        dw = [DWA_velocity, DWA_steer]
        return dw

    def DWA(self, x, y, heading, speed, steer, obs_xy=None):
        speed = 1.5
        if len(obs_xy) == 0:
            obs_xy = [[0.0, 0.0]]
        self.current_s, self.current_q = self.glob_path.xy2sl(x, y)

        def cost_function(pos):
            gp_separation = abs(self.glob_path.xy2sl(pos[0], pos[1])[1])
            cost1 = gp_separation / 1.5 if gp_separation <= 1.5 else gp_separation * 10
            obs_d = min([sqrt((pos[0] - obstacle[0]) ** 2 + (pos[1] - obstacle[1]) ** 2) for obstacle in obs_xy])
            cost2 = (self.obstacle_force - obs_d) / self.obstacle_force if obs_d < self.obstacle_force else 0
            heading_difference = abs(self.glob_path.get_current_reference_yaw() - heading)
            cost_heading = heading_difference if heading_difference <= pi else heading_difference - pi
            cost3 = abs(cost_heading) / pi
            return self.w1 * cost1 + self.w2 * cost2 + self.w3 * cost3

        """
        목적 함수 (pos = [x, y, theta], velocity)
            cost1 : pos 가 global path 와 가까울 수록 낮은 cost 부과
                    (but, 차선 <global path 로 부터 수직 거리 1.5m> 을 넘어 가면 매우 높은 cost 부과
            cost2 : obstacles 중 pos 와 가장 가까운 obs 까지의 거리가 멀수록 낮은 cost 부과
                    (but, obstacle_force 범위 안에 들어 왔을 때)
            cost4 : global path 의 yaw 와, 차량의 heading 과의 차이가 작을 수록 낮은 cost 부과
            return : 각각의 weight 를 __init__ 에서 조정
        """

        best_cost = float('inf')
        best_actual = [speed, steer]
        candidate_paths, selected_path = [], []

        dw = self.calc_dynamic_window(best_actual[0])
        # dw = self.calc_dynamic_window(best_actual[0], best_actual[1])    <-- steer 에 따라 경로 생성을 다르게 하고 싶을 때
        velocity = dw[0]
        for steer in dw[1]:
            future_pos = self.generate_predict_point(x, y, velocity, steer, heading)
            candidate_paths.append(future_pos)
            cost = cost_function(future_pos[-1])
            if cost < best_cost:
                best_cost = cost
                selected_path = future_pos

        if self.visual:
            self.visual_candidate_paths(candidate_paths)
            self.visual_selected_path(selected_path)
        return selected_path
