from glob import glob
from re import T
# import rospy
import time
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud
import math

class mission_uturn:
    def __init__(self):
        self.distance = 0 # 라이다 값으로 얻은 현재 거리
        self.interval = [950.0, 1050.0] # 유지해야 하는 거리의 기준값 (임의 구간 설정)
        self.global_path = 1 # 전역경로에서 미션 구간 진입 했을 때 1, 벗어났을 때 0
        self.point_cloud = [0, 0, 0, 0, 0, 0, 0, 0] # 라이디 depth값 (list)
        # self.point_cloud = 900 # 라이다에서 한점으로 군집화했다고 가정
        
    def oscilliate(self):
        for i in range(len(self.point_cloud)):
            self.point_cloud[i] = np.random.randint(1030, 1100)
    
    def curve(self):
        self.point_cloud += 30
        
    def two_dis(self, p1, p2):
        a = p1[0] - p2[0]
        b = p1[1] - p2[1]
        c = np.hypot(a, b)
        return c
    
    def run(self):
        # steer, speed 초기화(테스트 용)
        steer = 0 # 직진이라고 가정
        speed = 100
        current_time = time.time()
        
        if self.global_path == 1: # 미션구간에 진입했을 때

            # self.curve()
            self.oscilliate()

            # length = len(self.point_cloud) # 라이다 값이 리스트라면
            
            # if length < 4: continue # 데이터 값이 너무 적으면 다음 루프로
            
            # if not(self.interval[0] < self.point_cloud[length // 2] < self.interval[1]):
            if np.mean(self.point_cloud, axis=0) > self.interval[1] or np.mean(self.point_cloud, axis=0) < self.interval[0]:
                steer += 2 # +를 오른쪽 방향이라고 가정했을 때
                speed -= 1 # 점진적으로 속도 감소  
                print("steer :", steer)
                print("speed :", speed)
                print("distance :", np.mean(self.point_cloud, axis=0))
                print("distance after steering:", np.mean(self.point_cloud, axis=0) - 50)
                print()

            # time.sleep(1) # 1초 마다 한번씩 반복 (n초로 변경 가능) 
            
            # if time.time() - current_time > 15: 
            # self.global_path = 0
            return steer
        
        elif self.global_path == 0: 
            speed = 100
            steer = 0
            print("steer :", steer)
            print("speed :", speed)
            
            return
                
                
    
    def quadratic(self, x):
        return (math.sqrt(x) - 1000*x + 10000) / 5000
    
    def uturnTrack(self, x, y, t):
        x += 10
        y += self.quadratic(x)
        
        return x, y
                
    def testing(self):
        w, h = 1024, 1024
        x0 = w // 2 - 250
        y0 = h // 2
        x1 = w // 2 + 50
        y1 = h // 2 + 50
        center = [w // 2, h // 2]
        current = [x0, y0]
        black = np.zeros((h, w , 3), np.int8)
        
        # steer, speed 초기화(테스트 용)
        steer = 0 # 직진이라고 가정
        speed = 100
        current_time = time.time()
        
        if self.global_path == 1: # 미션구간에 진입했을 때
            speed -= 20 # 회전해야하는 구간이기 때문에 속도 감소 
            cv2.circle(black, tuple(current), 5, (0, 0, 255), -1)
            cv2.imshow("X", black)
            cv2.waitKey(0)
            while True:
                # self.curve()
                self.oscilliate()
                
                # length = len(self.point_cloud) # 라이다 값이 리스트라면
                
                # if length < 4: continue # 데이터 값이 너무 적으면 다음 루프로
            
                # if not(self.interval[0] < self.point_cloud[length // 2] < self.interval[1]):
                cv2.circle(black, tuple(center), 20, (100, 255, 0), -1)
                print("current :", current)
                print("distance :", self.two_dis(center, current))
                if self.two_dis(center, current) > self.interval[1] or self.two_dis(center, current) < self.interval[0]:
                    pass
                # if np.mean(self.point_cloud, axis=0) > self.interval[1] or np.mean(self.point_cloud, axis=0) < self.interval[0]:
                #     x0, y0 = self.uturnTrack(x0, y0, time.time() - current_time)
                #     print("(x0, y0) :", x0, y0)
                #     print("distance :", np.mean(self.point_cloud, axis=0))
                    
                    
                cv2.circle(black, (int(x0), int(y0)), 5, (0, 0, 255), -1)
                cv2.imshow("X", black)

                time.sleep(0.1) # 1초 마다 한번씩 반복 (n초로 변경 가능) 
                print()
                print()
                cv2.waitKey(0)
                # if time.time() - current_time > 15: break
                if self.global_path == 0: break
                
    
                    
test = mission_uturn()

# test.run()
test.testing()