#!/usr/bin/env python
#-*-coding:utf-8-*-

# Python packages
import rospy
import os, sys
import numpy as np
#sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

# 필요한 Library
from pure_pursuit_clean import pure_pursuit
from trajectory_planner_test import TrajectoryPlanner
from global_path import GlobalPath 
#from ?? import ?? -> velplanner

# message 파일
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))
from macaron_4.msg import erp_read
from macaron_4.msg import erp_write

#parameter
ld = 4

class get_state:
    def __init__(self):  
        self.erp_sub = rospy.Subscriber("erp_read", erp_read, self.erp_sub_callback, queue_size=1)
        self.obs_sub = rospy.Subscriber('object', PointCloud, self.obs_callback, queue_size = 1)
        self.subtm = rospy.Subscriber('current_pose', Point, self.pose_callback, queue_size=1)
        self.current_pose = [0.0, 0.0]
        self.heading = 0.0
        self.obs = []

    def pose_callback(self, point):
        self.x = point.x
        self.y = point.y
        self.heading = point.z
        self.current_pose = [self.x, self.y]

    def obs_callback(self, pointcloud):
        self.obs = pointcloud.points  # 사용할 때 : PointCloud.points[i].x
        
    def erp_sub_callback(self, data):
        self.current_erp = data

    def get_steer(self):
        return self.current_erp.read_steer

    def get_speed(self):
        return self.current_erp.read_speed

class publish_erp:

    def __init__(self):
        self.erp_pub = rospy.Publisher("erp_write", erp_write, queue_size=1)
        self.erp = erp_write()

    def pub_erp(self, speed, steer):

        self.erp.write_gear = 0
        self.erp.write_brake = 1
        self.erp.write_speed = speed
        self.erp.write_steer = steer

        self.erp_pub.publish(self.erp)


def main():
    rospy.init_node("planning_node_test", anonymous=True)
    rate = rospy.Rate(5)
    get = get_state()
    pub = publish_erp()

    # 파일 경로 설정
    GLOBAL_NPY = "PJ1.npy"
    PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"
    gp_name = PATH_ROOT + GLOBAL_NPY
    glob_path = GlobalPath(gp_name)

    path_planner = TrajectoryPlanner(ld, glob_path = glob_path)

    print('obstacle avoidance start!!')
    rospy.sleep(1)

    while not rospy.is_shutdown():    
        x, y, heading = get.current_pose[0], get.current_pose[1], get.heading
        PP = pure_pursuit(ld)

        target_vel = 50 # 고정속도 50

        # 장애물 받아오기
        obs_xy = get.obs

        # 경로 생성 및 선택   
        path_planner.set_ld(ld)
        selected_path = path_planner.optimal_trajectory(x, y, heading, obs_xy)

        #speed, steer, goal 
        goal = [selected_path.x, selected_path.y]
        
        gx = goal[0]
        gy = goal[1]
            
        v,target_steer = PP.get_steer_state(x, y, heading, ld, goal, np.hypot(x-gx[ld],y-gy[ld]))
    
        # publish
        pub_speed = target_vel
        pub_steer = target_steer

        pub.pub_erp(pub_speed, pub_steer)

        rate.sleep()

if __name__ == '__main__':
    main()