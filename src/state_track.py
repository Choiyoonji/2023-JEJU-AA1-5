#!/usr/bin/env python
#-*-coding:utf-8-*-

# Python packages
import rospy
import sys, os
import numpy as np
import math

# message 파일
from jeju.msg import erp_write, erp_read
from sensor_msgs.msg import PointCloud

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/sensor")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/missions")

# 모듈 import
from missions.mission_track import mission_track, path_planning, visual, lidar_zed_matching


# Parameter
SPEED = 120


class sub_erp_state:
    def __init__(self):
        #구독자 선언
        self.obs_sub = rospy.Subscriber('/object', PointCloud, self.obs_callback, queue_size=1)
        self.erp_sub= rospy.Subscriber('/erp_read', erp_read, self.erp_callback, queue_size=1)

        #Sub 받은 데이터 저장 공간
        self.obs = [[]]
        self.erp_speed = 0.0
        self.erp_steer = 0.0
        self.erp_ENC = 0.0

    def erp_callback(self, data):
        self.erp_speed = data.read_speed
        self.erp_steer = data.read_steer
        self.erp_ENC = data.read_ENC
        if data.read_gear == 2 and self.erp_speed > 0:
            self.erp_speed *= -1
    
    def obs_callback(self, data): # PointCloud.points[i].x
        self.obs = []
        for i in data.points:
            self.obs.append([i.x, i.y])

    def left_rubber_callback(self, GB):
        self.zed_left_rubber = []
        for b in GB.points:
            self.zed_left_rubber.append([b.x, b.y])

    def right_rubber_callback(self, GB):
        self.zed_right_rubber = []
        for b in GB.points:
            self.zed_right_rubber.append([b.x, b.y])

class publish_erp:
    def __init__(self):
        self.erp_pub = rospy.Publisher("speed_planner", erp_write, queue_size=1)
        self.erp = erp_write()

    def pub_erp(self, speed, steer):
        self.erp.write_speed = speed
        self.erp.write_steer = steer

        self.erp_pub.publish(self.erp)

def main():
    rate = rospy.Rate(10)
    pub = publish_erp()
    Data = sub_erp_state() 
    track = mission_track()
    Path = path_planning()
    Match = lidar_zed_matching()
    v = visual()

    steer = 0.0

    print("track_drive start!!")

    rospy.sleep(1)

    while not rospy.is_shutdown():

        # 라이다 라바콘 받아오기
        right, left = track.divide(Data.obs)
        L = track.cluster(np.array(left))
        R = track.cluster(np.array(right))
        L_f = track.filter(L)
        R_f = track.filter(R)
        # clus = track.cluster(np.array(Data.obs))
        # fin_clus = track.filter(clus)
        # print(fin_clus)
        
        
        # Combine = []
        # Combine.extend(R_f)
        # Combine.extend(L_f)

        # print(fin_clus)
        # v.pub_lidar_vis(fin_clus)
        # 라이다 제드 매칭
        ### Left_final, Right_final = Match.match(fin_clus, Data.zed_left_rubber, Data.zed_right_rubber)
        # Left_final = L_f
        # Right_final = R_f
        # # Spline
        # Left_x, Left_y = Path.spline(Left_final)
        # Right_x, Right_y = Path.spline(Right_final)

        # # 가운데 경로
        # Center_Path = Path.combine(Right_x, Right_y, Left_x, Left_y)
        # steer = Path.tracking(Center_Path)

        # # pub.pub_erp(SPEED, steer)
        # v.pub_left_final_vis(Left_final)
        # v.pub_right_final_vis(Right_final)
        # Spline
        Left_x, Left_y = Path.spline(L_f)
        Right_x, Right_y = Path.spline(R_f)
        # v.pub_left_path_vis(Left_x, Left_y)
        # v.pub_right_path_vis(Right_x, Right_y)

        # 가운데 경로
        Center_Path = Path.combine(Right_x, Right_y, Left_x, Left_y)
        Center_Path.insert(0, [0,0])
        v.pub_track_gb_vis(Center_Path)
        steer = Path.tracking(Data.erp_speed, Center_Path, fin_clus, Left_x, Left_y, Right_x, Right_y)

        pub.pub_erp(SPEED, steer)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node("track_drive", anonymous=True)
    main()