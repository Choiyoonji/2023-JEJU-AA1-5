#!/usr/bin/env python
#-*-coding:utf-8-*-

# Python packages
import rospy
import os, sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/missions")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/sensor")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/path_planning")

import math
import numpy as np

# message 파일
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import LaserScan
# from sensor_msgs import PointCloud

# 모듈 import
from global_path import GlobalPath
from sub_erp_state import sub_erp_state
from mission_cruising import mission_cruising
from mission_dynamic_obstacle import mission_dynamic_obstacle
from mission_tunnel import mission_tunnel
from mission_lane_total import mission_lane_total

WHERE = 1

CRUISING_SPEED = 40
TUNNEL_SPEED = 50
DYNAMIC_SPEED = 40
LANE_SPEED = 50

# 미션별 SL 좌표
if WHERE == 1: # 동국대 직선
    GLOBAL_PATH_NAME = "won_c_0419.npy" 
    mission_coord = {"Static_Obstacle" : [999990.0,9930],
                    "Dynamic_Obstacle" : [0.0, 99999.2], "lane" : [0.0,0.0],
                    "Tunnel" : [9999, 9999]}

elif WHERE == 2: # jeju track -> traffic none, ccw-cw
    GLOBAL_PATH_NAME = "jeju_island1.npy" #end is 125.2 start is 0.8
    mission_coord = {"Static_Obstacle" : [9999, 9999],
                    "Dynamic_Obstacle" : [9999, 9999], "lane" : [999,9999],
                    "Tunnel" : [9999, 9999]}

elif WHERE == 3: # jeju track -> traffic available, cw-ccw
    GLOBAL_PATH_NAME = "jeju_island3.npy" # end is 133.2 start is 0.0
    mission_coord = {"Static_Obstacle" : [99999, 99999],
                    "Dynamic_Obstacle" : [9999, 9999], "lane" : [999,9999],
                    "Tunnel" : [9999, 9999]}

elif WHERE == 4: # jeju track -> traffic none, ccw-cw  ##now test jeju_island0
    GLOBAL_PATH_NAME = "jeju_island0.npy" #end is 125.2 start is 0.8 #start is 0.0 end is 26.5
    mission_coord = {"Static_Obstacle" : [9999, 9999],
                    "Dynamic_Obstacle" : [99999.5, 99999.2], "lane" : [999,9999],
                    "Tunnel" : [9999, 9999]}


def distance(mission_pose, s):
    determine = False
    mission_start = mission_pose[0]
    mission_final = mission_pose[1]
    if (s >= mission_start) and (s <= mission_final):
        determine = True
    return determine

class publish_erp():
    def __init__(self):
        self.erp_pub = rospy.Publisher("erp_write", Twist, queue_size=5)
        self.erp = Twist()

    def pub_erp(self, speed, steer):
        self.erp.linear.x = speed
        self.erp.angular.z = steer
        self.erp_pub.publish(self.erp)
class Mission_State():
    def __init__(self):
        self.mission_state = 0
        self.mission_zone = 0
        self.mission_ing = 0
        
        self.Lane_done = False
        
        self.max_dis = 10.0
        self.Tunnel_width = 1.4
        
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.sub_scan = []
        
        self.steer_sub = rospy.Subscriber("lane_steer", Int16, self.lane_callback, queue_size=10)
        self.lane_steer = 0.0
        
    def scan_callback(self, scan):
        sub_scan = np.array(scan.ranges[0:810 + 1:3]) 
        self.sub_scan = np.where(sub_scan >= self.max_dis, self.max_dis, sub_scan)
        
    def lane_callback(self, data):
        self.lane_steer = data
    
    def is_Tunnel(self):
        r_data, l_data = self.sub_scan[55:75+1:3], self.sub_scan[195:215+1:3]
        if len(r_data) > 0 and len(l_data) > 0:
            r_avg, l_avg = np.sum(r_data) / len(r_data), np.sum(l_data) / len(l_data) 
        else:
            r_avg = 999999
            l_avg = 999999
        if r_avg + l_avg < self.Tunnel_width:
            return True
        else:
            return False
        
    def is_Lane(self, q):
        return q > 3
    
    def mission_loc(self, s, q):
        global mission_coord

        self.mission_zone = 0
        
        if (distance(mission_coord["Tunnel"], s)):
            self.mission_zone = 4
        elif (distance(mission_coord["lane"], s)) and self.is_Lane(q):
            self.mission_zone = 3
        elif (distance(mission_coord["Static_Obstacle"], s)):
            self.mission_zone = 1
        elif (distance(mission_coord["Dynamic_Obstacle"], s)):
            self.mission_zone = 2
            

    def mission_update(self, s, q):
        self.mission_loc(s, q)

        if (self.mission_zone == 0): #mission_zone if not in mission zone -> cruising!
            self.mission_state = self.mission_zone
            self.mission_ing = 0

        else: # 미션존 안에 있는 경우
            if self.mission_ing == 0:
                print("미션 시작")
                self.mission_state = self.mission_zone
                self.mission_ing = 1
            elif self.mission_ing == 1:
                self.mission_state = self.mission_zone
            elif self.mission_ing == 2:
                self.mission_state = 0

    def mission_done(self):
        print("미션 완료")
        self.mission_ing = 2
    
def main():
    global WHERE
    rate = rospy.Rate(10)
    speed, steer = 0.0, 0.0

    # vis_reset = publish_Visreset()
    pub = publish_erp()
    erp = sub_erp_state()
    MS = Mission_State()

    # Global Path 선언
    PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))+"/path/npy_file/path/"
    gp_name = PATH_ROOT + GLOBAL_PATH_NAME
    GB = GlobalPath(gp_name)

    # 미션 선언
    Mission_cruising = mission_cruising(GLOBAL_PATH_NAME)    # path_tracking 할 경로 넣어주기 (macaron_3.path 폴더 안에 있어야함)
    Mission_dynamic_obstacle = mission_dynamic_obstacle(GLOBAL_PATH_NAME)
    Mission_tunnel = mission_tunnel()
    Mission_lane = mission_lane_total()
    
    print("제주도 한라봉맛 마카롱")
    rospy.sleep(1)
    
    # vis_reset.pub_reset()

    while not rospy.is_shutdown():
        print('====================================')
        s, q = GB.xy2sl(erp.pose[0], erp.pose[1])
        print('current s', s)
        print('current q', q)
        print(erp.states)
        state = MS.mission_update(s)

        if MS.mission_state == 0: # 크루징(디폴트) 모드
            print("떼굴떼굴---")
            steer = Mission_cruising.path_tracking(erp.pose, erp.heading)
            speed = CRUISING_SPEED
        
        elif MS.mission_state == 1: # 정적 장애물
            print("라봉아 피해!")
            steer = Mission_cruising.static_obstacle(erp.pose, erp.heading, erp.erp_speed, erp.erp_steer, erp.obs)
            speed = CRUISING_SPEED
            
        elif MS.mission_state == 2:  # 동적 장애물
            print("고라니 악,,,,")
            steer = Mission_cruising.path_tracking(erp.pose, erp.heading)
            speed = DYNAMIC_SPEED
            # if Mission_dynamic_obstacle.scan(erp.pose, erp.obs) == "stop":
            Mission_dynamic_obstacle.scan(erp.pose, erp.obs)
            if Mission_dynamic_obstacle.stop:
                speed = 0
                MS.stop = True
                print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++고라니발견++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
            
            if Mission_dynamic_obstacle.done:
                MS.mission_done()
                
        elif MS.mission_state == 3:  # 차선
            print("누끼 장인 두둥등장 !!")
            steer = Mission_lane.get_steer(erp.pose, erp.heading, erp.erp_speed, erp.erp_steer, erp.obs)
            speed = LANE_SPEED if not Mission_lane.stop else 0

        elif MS.mission_state == 4:  # 터널
            print("길을 잃었다... 자랑이다~!!")
            steer = Mission_tunnel.get_steer()
            speed = TUNNEL_SPEED
                
        # rospy.sleep(3)
        print("steer: %d speed: %d"%(steer, speed))
        pub.pub_erp(speed, steer)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("state_node", anonymous=True)
    main()