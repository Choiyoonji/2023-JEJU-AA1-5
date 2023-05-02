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

CRUISING_SPEED = 80
TUNNEL_SPEED = 50
DYNAMIC_SPEED = 80
LANE_SPEED = 50

# 미션별 SL 좌표
if WHERE == 1: # 동국대 직선
    GLOBAL_PATH_NAME = "jeju_island_gp.npy" 
    mission_coord = {"Static_Obstacle" : [999990.0,9930],
                    "Dynamic_Obstacle" : [0.0, 99999.2], "lane" : [0.0,0.0],
                    "Tunnel" : [92.0, 110]}

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
        self.erp_pub = rospy.Publisher("erp_writes", Twist, queue_size=1)
        self.erp = Twist()

    def pub_erp(self, speed, steer):
        self.erp.linear.x = speed
        self.erp.angular.z = steer
        self.erp_pub.publish(self.erp)
        
class Mission_State():
    def __init__(self):
        self.mission_state = 0
        self.mission_zone = 0
        self.static_cnt = 0
        
        self.avoid = False

    def mission_update(self, s, is_obs):
        global mission_coord

        self.mission_zone = 0

        if (distance(mission_coord["Tunnel"], s)):
            self.mission_zone = 3
        else:
            if self.avoid:
                self.mission_zone = 1
            else:
                self.mission_zone = 2
            
        self.mission_state = self.mission_zone
        
        if is_obs:
            self.static_cnt += 1
            if self.static_cnt > 50:
                self.avoid = 0
                return self.avoid
        
        return True

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
    Mission_tunnel = mission_tunnel(GLOBAL_PATH_NAME)
    
    print("제주도 한라봉맛 마카롱")
    rospy.sleep(1)
    
    # vis_reset.pub_reset()

    while not rospy.is_shutdown():
        print('====================================')
        s, q = GB.xy2sl(erp.pose[0], erp.pose[1])
        print('current s', s)
        print('current q', q)
        print(erp.states)
        state = MS.mission_update(s, Mission_dynamic_obstacle.is_obs(s, erp.obs))
        
        if not state:
            Mission_dynamic_obstacle.avoid = MS.avoid
            
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
            MS.avoid = Mission_dynamic_obstacle.avoid
            if Mission_dynamic_obstacle.stop:
                speed = 0
                MS.stop = True
                print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++고라니발견++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
                
        elif MS.mission_state == 3:  # 터널
            print("길을 잃었다... 자랑이다~!!")
            if not Mission_tunnel.tunnel_flag:
                Mission_tunnel.search_tunnel_entrance()
                steer = Mission_cruising.path_tracking(erp.pose, erp.heading)
            else:
                steer = Mission_tunnel.get_steer()
            
            speed = TUNNEL_SPEED   
                
        # rospy.sleep(3)
        print("steer: %d speed: %d"%(steer, speed))
        pub.pub_erp(speed, steer)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("state_node", anonymous=True)
    main()