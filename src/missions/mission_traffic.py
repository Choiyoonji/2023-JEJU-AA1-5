#!/usr/bin/env python
#-*-coding:utf-8-*-
# 직진 only 없이 직좌만 뜨면 247번째 줄 or 조건 추가
# 정지후 빨간불 다음에 바로 직좌가 뜰 때 273번째 줄 주석 해제 274번째 줄 주석 248주석
# 정지 > 직진 > 직좌가 뜨는 경우 직진신호 몇 초 연속 들어오면 출발
FIND_GREEN_ONLY_TIME = 10000000000000000000000000

# Python Package
import rospy
import sys, os
import time
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")

# 알고리즘
# 기본적으로 정지 가능구간 들어갈 때 까지 크루징                   flag 0
# 정지 가능구간 들어간 뒤 노or빨이면 정지                          flag 1
# 파란불 뜨면 출발 or 정지 가능구간 지나면 2초간 크루징 후          flag 2
# done 뱉기                                                      flag 3

# Parameter
CAN_STOP_START = 15      # 감속 시작구간
CAN_STOP_LAST = 3.0       # 정지 위치 (정지선으로부터)
DONE_TIME = 5           # 파란불 인식 후 초 동안 직진

BEFORE_STOP_SPEED = 100  # 정지선 전까지 속도
AFTER_STOP_SPEED = 200  # 정지선 지나고 속도

RED_TO_GREEN_TIME = 4 # 파란불 인식 못했을 때 안전장치

def stop_distance(erp_speed):
    erp_speed = np.round((erp_speed - 1)/10) * 10
    
    stop_dist = (0.4/10) * (erp_speed - 50) + 1.5

    # return 6.5 # 고정으로 쓰고 싶다면
    return CAN_STOP_LAST + stop_dist

class mission_traffic_straight:
    def __init__(self, WHERE):
        # 정지선 좌표
        if WHERE == 1:
            self.STRAIGHT_STOP_LINE = [148.9, 148.9] # 직진 신호등 정지선 s좌표
            self.LEFT_STOP_LINE = [9999] # 좌회전 신호등 정지선 s좌표

        elif WHERE == 2:
            self.STRAIGHT_STOP_LINE = [110.3, 248.6] # 직진 신호등 정지선 s좌표
            self.LEFT_STOP_LINE = [9999] # 좌회전 신호등 정지선 s좌표
            
        elif WHERE == 3:
            self.STRAIGHT_STOP_LINE = [178.2, 233.0, 366.5, 763.9, 901.6, 1246.5, 1380.5, 1426.4] # 직진 신호등 정지선 s좌표
            self.LEFT_STOP_LINE = [1079.5, 1079.5] # 좌회전 신호등 정지선 s좌표 1043.2
        
        elif WHERE == 4:
            self.STRAIGHT_STOP_LINE = [106.8, 106.8] # 직진 신호등 정지선 s좌표
            self.LEFT_STOP_LINE = [1042.5, 1042.5] # 좌회전 신호등 정지선 s좌표 1043.2
        
        elif WHERE == 5:
            self.STRAIGHT_STOP_LINE = [36.5, 36.5] # 직진 신호등 정지선 s좌표
            self.LEFT_STOP_LINE = [1042.5, 1042.5] # 좌회전 신호등 정지선 s좌표 1043.2
        
        self.traffic_flag = 0
        self.time_rec = 0
        self.done = False
        self.stop = True
        self.start_count = 0
        self.green_count = 0
        self.red_count = 0
        self.start_red_time = 99999999999999999999
        self.start_red_count = 0

    def reset(self):
        self.traffic_flag = 0
        self.done = False
        self.stop = True
        self.time_rec = 0
        self.start_count = 0
        self.green_count = 0
        self.red_count = 0
        self.start_red_time = 99999999999999999999
        self.start_red_count = 0

    def run(self, s, traffic, erp_speed):
        for i in self.STRAIGHT_STOP_LINE:
            if i - 16 <= s and s <= i + 10:
                self.traffic_s = i
        # 초기값
        speed = 0

        # 신호등 인식 시작
        if self.traffic_flag == 0:
            speed = 200

            if traffic[3] == 1: # 파란불이면
                self.green_count += 1
                self.red_count = 0
            elif traffic[0] == 1 or traffic[1] == 1: # 노란불 or 빨간불이면
                self.red_count += 1
                self.green_count = 0

            if self.green_count >= 2:
                self.stop = False
            elif self.red_count >= 2:
                self.stop = True

            if (self.traffic_s - s <= CAN_STOP_START) and (self.traffic_s - 3 > s):
                self.traffic_flag = 1
                print('정지에 대비하기 위해 감속 합니다!')

        # 속도 줄이기 (신호등 인식중)
        elif self.traffic_flag == 1:
            speed = BEFORE_STOP_SPEED

            if traffic[3] == 1: # 파란불이면
                self.green_count += 1
                self.red_count = 0
            elif traffic[0] == 1 or traffic[1] == 1: # 노란불 or 빨간불이면
                self.red_count += 1
                self.green_count = 0

            if self.green_count >= 2:
                self.stop = False
            elif self.red_count >= 2:
                self.stop = True 


            VARIABLE_CAN_STOP_LAST = stop_distance(erp_speed)
            # print(VARIABLE_CAN_STOP_LAST)
            if self.traffic_s - s <= VARIABLE_CAN_STOP_LAST: # 정지 가능 구간을 지났다면
                self.traffic_flag = 2
                if self.stop == True:
                    print('빨간불 인식 정지!')
                else:
                    print('파란불 인식 가속!')


        # 정지 유무 판단해서 정지
        elif self.traffic_flag == 2:
            if self.stop == True:
                speed = 0
                self.start_red_count = 0
                if traffic[3] == 1:
                    self.start_count += 1
                else:
                    self.start_count = 0
                # 빨간불 안전장치
                if traffic[0] == 1:
                    self.start_red_count = 1
                    self.start_red_time = time.time() + RED_TO_GREEN_TIME
                
                if self.start_red_count == 0 and (time.time() >= self.start_red_time):
                    print('초록불 인식 못했는데 너무 오래 서있어서 그냥 출발함')
                    self.traffic_flag = 3
                if self.start_count >= 3:
                    self.traffic_flag = 3
            else:
                speed = AFTER_STOP_SPEED
                self.traffic_flag = 3
            
            self.time_rec = time.time()

        # 파란불 뜨고 or 파란불에 지나가는 상황이라면
        elif self.traffic_flag == 3:
            speed = AFTER_STOP_SPEED
            if (time.time() - self.time_rec) >= DONE_TIME:
                self.done = True
                print('신호등 미션 끝!')
        
        if abs(self.traffic_s - s) <= 0.2:
            print('정지선 위치 지남')

        # print(self.green_count, self.red_count, 'green, red')
                
        return speed


class mission_traffic_left:
    def __init__(self, WHERE):
        # 정지선 좌표
        if WHERE == 1:
            self.STRAIGHT_STOP_LINE = [148.9, 148.9] # 직진 신호등 정지선 s좌표
            self.LEFT_STOP_LINE = [9999] # 좌회전 신호등 정지선 s좌표

        elif WHERE == 2:
            self.STRAIGHT_STOP_LINE = [110.3, 248.6] # 직진 신호등 정지선 s좌표
            self.LEFT_STOP_LINE = [9999] # 좌회전 신호등 정지선 s좌표
            
        elif WHERE == 3:
            self.STRAIGHT_STOP_LINE = [178.2, 233.0, 365.8, 763.9, 901.6, 1247.5, 1380.5, 1426.4] # 직진 신호등 정지선 s좌표
            self.LEFT_STOP_LINE = [1079.5, 1079.5] # 좌회전 신호등 정지선 s좌표 1043.2
        
        self.traffic_flag = 0
        self.time_rec = 0
        self.done = False
        self.stop = True
        self.start_count = 0
        self.green_count = 0
        self.red_count = 0
        self.start_red_time = 99999999999999999999
        self.start_green_time = 99999999999999999999
        self.start_red_count = 0
        self.start_green_count = 0

    def reset(self):
        self.traffic_flag = 0
        self.done = False
        self.stop = True
        self.time_rec = 0
        self.start_count = 0
        self.green_count = 0
        self.red_count = 0
        self.start_red_time = 99999999999999999999
        self.start_red_count = 0

    def run(self, s, traffic, erp_speed):
        for i in self.LEFT_STOP_LINE:
            if i - 16 <= s and s <= i + 10:
                self.traffic_s = i
        # 초기값
        speed = 0

        # 신호등 인식 시작
        if self.traffic_flag == 0:
            speed = 200

            if traffic[2] == 1: # 파란불이면
                self.green_count += 1
                self.red_count = 0
            elif traffic[0] == 1 or traffic[1] == 1: # 노란불 or 빨간불이면
                self.red_count += 1
                self.green_count = 0

            if self.green_count >= 2:
                self.stop = False
            elif self.red_count >= 2:
                self.stop = True

            if (self.traffic_s - s <= CAN_STOP_START) and (self.traffic_s - 3 > s):
                self.traffic_flag = 1
                print('정지에 대비하기 위해 감속 합니다!')

        # 속도 줄이기 (신호등 인식중)
        elif self.traffic_flag == 1:
            speed = BEFORE_STOP_SPEED
            
            if traffic[2] == 1: # 파란불이면
            # if traffic[2] == 1 or traffic[3] == 1: # 파란불이면
                self.green_count += 1
                self.red_count = 0
            elif traffic[0] == 1 or traffic[1] == 1: # 노란불 or 빨간불이면
                self.red_count += 1
                self.green_count = 0

            if self.green_count >= 2:
                self.stop = False
            elif self.red_count >= 2:
                self.stop = True
            
            VARIABLE_CAN_STOP_LAST = stop_distance(erp_speed)
            if self.traffic_s - s <= VARIABLE_CAN_STOP_LAST: # 정지 가능 구간을 지났다면
                self.traffic_flag = 2
                if self.stop == True:
                    print('빨간불 인식 정지!')
                else:
                    print('파란불 인식 가속!')


        # 정지 유무 판단해서 정지
        elif self.traffic_flag == 2:
            if self.stop == True:
                speed = 0
                self.start_red_count = 0
                # if traffic[2] == 1 or traffic[3] == 1: # 정지후 빨간불 다음에 바로 직좌가 뜰 때 사용
                if traffic[2] == 1:
                    self.start_count += 1
                # else:
                #     self.start_count = 0
                # 빨간불 안전장치
                if traffic[0] == 1:
                    self.start_red_count = 1
                    self.start_red_time = time.time() + RED_TO_GREEN_TIME
                
                if self.start_red_count == 0 and (time.time() >= self.start_red_time):
                    print('초록불 인식 못했는데 너무 오래 서있어서 그냥 출발함')
                    self.traffic_flag = 3

                # 직진 only 안전장치
                if self.start_green_count == 0 and (traffic[0] == 0 and traffic[1] == 0 and traffic[2] == 0 and traffic[3] == 1):
                    self.start_green_count = 1
                    self.start_green_time = time.time() + FIND_GREEN_ONLY_TIME
                
                if self.start_green_count == 1 and (time.time() >= self.start_green_time) and (traffic[0] == 0 and traffic[1] == 0 and traffic[2] == 0 and traffic[3] == 1):
                    print('Green only만 오래 인식해서 그냥 출발함')
                    self.traffic_flag = 3

                if self.start_count >= 2:
                    self.traffic_flag = 3
            else:
                speed = AFTER_STOP_SPEED
                self.traffic_flag = 3
            
            self.time_rec = time.time()

        # 파란불 뜨고 or 파란불에 지나가는 상황이라면
        elif self.traffic_flag == 3:
            speed = AFTER_STOP_SPEED
            if (time.time() - self.time_rec) >= DONE_TIME:
                self.done = True
                print('좌회전 신호등 미션 끝!')
                
        if abs(self.traffic_s - s) <= 0.2:
            print('정지선 위치 지남')

        return speed