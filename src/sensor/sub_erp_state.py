#!/usr/bin/env python
# -- coding: utf-8 --
import rospy

from jeju.msg import Traffic
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
from std_msgs.msg import Int16, Int32
from geometry_msgs.msg import Twist

class sub_erp_state:
    def __init__(self):
        #구독자 선언
        self.pose_sub = rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size = 10)
        self.obs_sub = rospy.Subscriber('/object', PointCloud, self.obs_callback, queue_size=1)
        # self.erp_sub= rospy.Subscriber('/erp_read', erp_read, self.erp_callback, queue_size=10)
        self.erp_sub_speed= rospy.Subscriber('speed_read', Int16, self.erp_callback_speed, queue_size=1)
        self.erp_sub_steer= rospy.Subscriber('steer_read', Int32, self.erp_callback_steer, queue_size=1)
        self.lane_sub = rospy.Subscriber('/lane_dist', Int16MultiArray, self.lane_callback, queue_size=1) # 카메라쪽 정보 받아야 됨
        self.obj_sub = rospy.Subscriber('/traffic_obj', Traffic, self.obj_callback, queue_size=1)

        #Sub 받은 데이터 저장 공간
        self.pose = [955828.025, 1951142.677] # 팔정런할때 켜두면 좋음
        # self.pose = [935508.503, 1915801.339] # kcity에서 켜두면 좋음
        self.heading = 0.0
        self.obs = []
        self.erp_speed = 0.0
        self.erp_steer = 0.0
        self.erp_ENC = 0.0
        self.lane_dis = [0.0, 0.0]

        self.trffic = [0, 0, 0, 0] # [빨, 노, 좌, 초]
        self.traffic_y = 416
        self.delivery = [0, 0, 0, 0, 0, 0]
        self.delivery_zone1 = [[0.0, 0.0],[0.0, 0.0]]
        self.delivery_zone2 = [[0.0, 0.0],[0.0, 0.0]]
        self.delivery_zone3 = [[0.0, 0.0],[0.0, 0.0]]
        self.delivery_zone4 = [[0.0, 0.0],[0.0, 0.0]]
        # self.delivery = []
        # self.sign = []

    ##########callback 함수 모음##########
    # 각 센서에서 데이터가 들어오면 객체 내부의 데이터 저장공간에 저장
    def pose_callback(self, data):
        self.pose = [data.x, data.y]
        self.heading = data.z

    def obs_callback(self, data): # PointCloud.points[i].x
        self.obs = []
        for i in data.points:
            self.obs.append([i.x, i.y])
    
    # def erp_callback(self, data):
    #     self.erp_speed = data.read_speed
    #     self.erp_steer = data.read_steer
        # self.erp_ENC = data.read_ENC
        # if data.read_gear == 2 and self.erp_speed > 0:
        #     self.erp_speed *= -1

    def erp_callback_speed(self, data):
        # self.current_speed = data.read_speed
        self.erp_sub_speed = data
        self.erp_speed = self.erp_sub_speed
                # self.erp_ENC = data.read_ENC
        # if data.read_gear == 2 and self.current_speed > 0:
        #     self.current_speed *= -1

    def erp_callback_steer(self, data):
        self.erp_sub_steer = data
        self.erp_steer = self.erp_sub_steer

    def lane_callback(self, data):
        self.lane_dis = data.data
        
    def obj_callback(self, data):
        self.trffic = [0, 0, 0, 0]
        self.delivery = [0, 0, 0, 0, 0, 0]
        self.traffic_y = 416       
        for cl in data.obj:
            if cl.ns[0:3] == "del":
                
                if cl.ns[9:11] == "a1":
                    self.delivery[0] = 1
                else:
                    self.delivery[0] = 0
                if cl.ns[9:11] == "a2":
                    self.delivery[1] = 1
                else:
                    self.delivery[1] = 0
                if cl.ns[9:11] == "a3":
                    self.delivery[2] = 1
                else:
                    self.delivery[2] = 0

                if cl.ns[9:11] == "b1":
                    self.delivery[3] = 1
                else:
                    self.delivery[3] = 0
                if cl.ns[9:11] == "b2":
                    self.delivery[4] = 1
                else:
                    self.delivery[4] = 0
                if cl.ns[9:11] == "b3":
                    self.delivery[5] = 1
                else:
                    self.delivery[5] = 0

            else:
                if (cl.ymin + cl.ymax) / 2.0 <= self.traffic_y: # 가장 위의 신호등을 잡는 if문
                    if cl.ns == "green_3":
                        self.trffic = [0, 0, 0, 1]
                    elif cl.ns == "red_3":
                        self.trffic = [1, 0, 0, 0]
                    elif cl.ns == "orange_3":
                        self.trffic = [0, 1, 0, 0]
                    elif cl.ns == "left_green_4":
                        self.trffic = [0, 0, 1, 0]
                    elif cl.ns == "all_green_4":
                        self.trffic = [0, 0, 1, 1]
                    elif cl.ns == "orange_4":
                        self.trffic = [0, 1, 0, 0]
                    elif cl.ns == "red_4":
                        self.trffic = [1, 0, 0, 0]
                    elif cl.ns == "straight_green_4":
                        self.trffic = [0, 0, 0, 1]
                    else:
                        self.trffic = [0, 0, 0, 0]
                    self.traffic_y = (cl.ymin + cl.ymax) / 2.0                        

            

        # self.traffic_count += 1
        # if self.traffic_count > 3:
        #     self.trffic = [0, 0, 0, 0]

    def camera_test(self, lane = 0, obj = 0):
        if lane == 1:
            print("// lane dis :"),
            print(self.lane_dis)
        if obj == 1 and self.trffic != [0, 0, 0, 0]:
            print("// traffic :"),
            print(self.trffic)
        
######################## test ###############################
def main():
    Data = sub_erp_state() # 객체 생성

    while not rospy.is_shutdown():
        print("test : "),
        print(Data.trffic)

        rospy.sleep(0.1)
       
if __name__ == '__main__':
    rospy.init_node('sub_erp_state', anonymous=True)
    main()