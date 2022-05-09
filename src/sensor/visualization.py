#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from math import cos, sin, pi
import numpy as np
import os, sys
from sub_erp_state import sub_erp_state
# sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))+"/sensor")

from geometry_msgs.msg import Vector3, Pose, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan, NavSatFix, PointCloud, Imu
from std_msgs.msg import Header, Float64, ColorRGBA

WHERE = 7 # 1 팔정도 2 예선 3 본선 4 만해광장 5 대운동장 직선 6 대운동장 찐 직선

#지도 정보 경로 설정
PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/" #/home/gigi/catkin_ws/src/macaron_3/

#전역경로파일
if WHERE == 1:
    Tracking_path="PG.npy"
elif WHERE == 2:
    Tracking_path="kcity_tryout.npy"
elif WHERE == 3:
    Tracking_path="kcity_final.npy"
elif WHERE == 4:
    Tracking_path="MH_D.npy"
elif WHERE == 5:
    Tracking_path="MH_straight.npy"
elif WHERE == 6:
    Tracking_path="DP_straight.npy"
elif WHERE == 7:
    Tracking_path="jeju_island0.npy"

#지도의 파일 개수. HD 맵의 차선을 추가하거나하면 수정해야함
DGU_line=16
line=27
center=21
bus=8

#헤딩을 그려주기위해 값을 변환해주는 메서드
def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

class Visualization():
    def __init__(self, where):
        self.erp = sub_erp_state()
        # global path file set
        # 실행될 때 동국대지도인지 kcity 지도인지에따라 읽어도는 지도파일을 if 문으로 선택
        self.PATH=[]
        if where==1:
            print("draw DGU map")
            for i in range(DGU_line):
                # file_name="DGU_%d.npy"%(i+1)
                file_name="jeju_island0.npy"
                self.PATH.append(file_name)

        elif where==2:
            print("draw k-city map")
            for i in range(line):
                file_name="kcity_line_%d.npy"%(i+1)
                self.PATH.append(file_name)
            for i in range(center):
                file_name="kcity_center_%d.npy"%(i+line+1)
                self.PATH.append(file_name)
            for i in range(bus):
                file_name="kcity_bus_%d.npy"%(i+line+center+1)
                self.PATH.append(file_name)
        
        #publisher 설정
        self.global_pub = rospy.Publisher('/rviz_global_path', Marker, queue_size = len(self.PATH)+1)
        self.cdpath_pub = rospy.Publisher('/rviz_CDpath', Marker, queue_size = 5)
        self.slpath_pub = rospy.Publisher('/rviz_SLpath', Marker, queue_size = 1)
        self.goalpoint_pub = rospy.Publisher('/rviz_goalpoint', Marker, queue_size = 1)
        self.pose_pub = rospy.Publisher('pose', Marker, queue_size = 1)
        self.line_pub = rospy.Publisher('line', Marker, queue_size = 1)
        self.obs_pub = rospy.Publisher('obs', Marker, queue_size = 5)
        self.map_pub = rospy.Publisher('map', Marker, queue_size = 1)

        self.cdpath_sub = rospy.Subscriber('/CDpath', PointCloud, self.CDpath_callback, queue_size = 1)
        self.slpath_sub = rospy.Subscriber('/SLpath', PointCloud, self.SLpath_callback, queue_size = 1)
        self.goalpoint_sub = rospy.Subscriber('/goal_point', Point, self.goalpoint_callback, queue_size = 1)
        self.track_gb_sub = rospy.Subscriber('/track_gbpath', PointCloud, self.track_GBpath_callback, queue_size = 1)
        self.sign_sub = rospy.Subscriber('/sign', PointCloud, self.sign_loc_callback, queue_size = 1)

        self.offset = [0.0, 0.0]
        self.obs = []
        self.past_path = []
        self.track_gb_path = [[0.0, 0.0], [0.0, 0.0]]
        self.cd_path = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        self.sl_path = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]

    def CDpath_callback(self, cd):
        self.cd_path = []
        for b in cd.points:
            self.cd_path.append([b.x, b.y])
        self.CDpath()

    def SLpath_callback(self, sl):
        self.sl_path = []
        for b in sl.points:
            self.sl_path.append([b.x, b.y])
        self.SLpath()

    def track_GBpath_callback(self, GB):
        self.track_gb_path = []
        for b in GB.points:
            self.track_gb_path.append([b.x, b.y])

    def sign_loc_callback(self, data): # PointCloud.points[i].x
        self.obs = []
        for i in data.points:
            self.obs.append([i.x, i.y])
            
    def goalpoint_callback(self, g):
        self.goal_pos = [g.x, g.y, g.z]
        self.goalpoint()

    def goalpoint(self): #목표점
        rviz_msg_goalpoint=Marker(
            header=Header(frame_id='map', stamp=rospy.get_rostime()),
            ns="goal_point",
            id = 300,
            type=Marker.CYLINDER,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(x=0.4,y=0.4,z=1.0),
            color=ColorRGBA(r=0.0,g=0.0,b=1.0,a=1.0),
            pose=Pose(position=Point(x = self.goal_pos[0]-self.offset[0], y = self.goal_pos[1]-self.offset[1], z = 0.1)
            )
        )
        self.goalpoint_pub.publish(rviz_msg_goalpoint)

    def CDpath(self): #후보경로
        d = len(self.cd_path)//5
        a = c = 0
        for i in [0, 1, 2, 3, 4]:
            rviz_msg_cdpath=Marker(
                header=Header(frame_id='map', stamp=rospy.get_rostime()),
                ns="cd_path",
                id=105 + i,
                type=Marker.LINE_STRIP,
                lifetime=rospy.Duration(0.5),
                action=Marker.ADD,
                scale=Vector3(0.15,0.0,0.0),
                color=ColorRGBA(r=1.0,g=0.7,b=0.0,a=0.8)
            )
            while True:
                if a//d >= 1.0:
                    self.cdpath_pub.publish(rviz_msg_cdpath)
                    a = 0
                    break
                p = Point()
                p.x = self.cd_path[c][0]-self.offset[0]
                p.y = self.cd_path[c][1]-self.offset[1]
                p.z = 0
                rviz_msg_cdpath.points.append(p)
                c += 1
                a += 1

    def SLpath(self): #선택경로
        rviz_msg_slpath=Marker(
            header=Header(frame_id='map', stamp=rospy.get_rostime()),
            ns="sl_path",
            id=104,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.2,0.0,0.0),
            color=ColorRGBA(r=0.0,g=1.0,b=1.0,a=0.8)
        )
        for a in self.sl_path:
            p = Point()
            p.x = a[0]-self.offset[0]
            p.y = a[1]-self.offset[1]
            p.z = 0.1
            rviz_msg_slpath.points.append(p)

        self.slpath_pub.publish(rviz_msg_slpath)

        #전역경로를 pub 해주는 메서드
    def global_path(self):  # 지도
        i=0
        for st in self.PATH:
            if st[6:8] == "ce":
                cl=ColorRGBA(1.0,1.0,0.0,1.0)
            elif st[6:8] == "li":
                cl=ColorRGBA(1.0,1.0,1.0,1.0)
            elif st[6:8] == "st":
                cl=ColorRGBA(1.0,0.0,0.0,1.0)
            elif st[0:2] == "DG":
                cl=ColorRGBA(1.0,1.0,1.0,1.0)
            elif st[6:8] == "bu":
                cl=ColorRGBA(0.0,0.0,1.0,1.0)

            rviz_msg_global=Marker(
                header=Header(frame_id='map', stamp=rospy.get_rostime()),
                ns="global_path",
                type=Marker.LINE_STRIP,
                action=Marker.ADD,
                id=i,
                scale=Vector3(0.1,0.1,0),
                color=(1.0,1.0,1.0,1.0))

            path_arr=np.load(file=PATH_ROOT+"path/"+self.PATH[i])
            s=range(len(path_arr))
            for a in s:
                p=Point()
                p.x=float(path_arr[a,0])-self.offset[0]
                p.y=float(path_arr[a,1])-self.offset[1]
                p.z=0
                rviz_msg_global.points.append(p)
            self.global_pub.publish(rviz_msg_global)
            i+=1

    #오프셋(지도상 0,0점이 되는 좌표) 를 업데이트 해주는 메서드
    def offset_update(self):
        self.offset=[self.erp.pose[0], self.erp.pose[1]]
        # self.offset = [955926.9659, 1950891.243]

    def present_OBJECT(self,ID,TYPE,X,Y,Z,R,G,B,A):
        pose = Pose()

        q = euler_to_quaternion(0, 0, 1) #좌표변환
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        pose.position.x = self.erp.pose[0] - self.offset[0]
        pose.position.y = self.erp.pose[1] - self.offset[1]
        pose.position.z = 0

        rviz_msg_pose=Marker(
            header=Header(frame_id='map', stamp=rospy.get_rostime()), # fraeme_id -> fixed frame 이랑 같게 맞추어 주어야함.
            ns="object", #이건 뭐지 -> test 해봐 
            id=ID,#무조건 다 달라야함
            type=TYPE, # 튜토리얼에 있는것 보고 바꾸면 돼..
            lifetime=rospy.Duration(), #얼마동안 보여줄건지
            action=Marker.ADD,
            pose=pose, #lins_strip은point 사용 ->  def tracking_line(self): 참고하기
            scale=Vector3(x=X,y=Y,z=Z), 
            color=ColorRGBA(r=R,g=G,b=B,a=A), #색,a는 투명도 1이 최대
            )

        self.pose_pub.publish(rviz_msg_pose)

    def present_LINE(self,ID,R,G,B,A,PATH_ARR,log=False):
        if log == True :
            self.past_path.append([self.erp.pose[0], self.erp.pose[1]])
        
        rviz_msg_line=Marker(
            header=Header(frame_id='map', stamp=rospy.get_rostime()),
            ns="track_Line",
            id=ID,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(),
            action=Marker.ADD,
            scale=Vector3(0.1,0.0,0.0),
            color=ColorRGBA(r=R,g=G,b=B,a=A)
        )
        if PATH_ARR == 1:
            path_arr = self.track_gb_path
        elif PATH_ARR == 2:
            path_arr = self.past_path

        for a in path_arr:
            p=Point()
            p.x=a[0]-self.offset[0]
            p.y=a[1]-self.offset[1]
            p.z=0.0
            rviz_msg_line.points.append(p)

        self.line_pub.publish(rviz_msg_line)

    def present_OBS(self,TYPE,X,Y,Z,R,G,B,A,p):
        i=200
        for a in self.erp.obs:
            rviz_msg_obs=Marker(
                header=Header(frame_id='map', stamp=rospy.get_rostime()),
                ns="obs",
                id = i,
                type=TYPE,
                lifetime=rospy.Duration(0.5),
                action=Marker.ADD,
                scale=Vector3(x=X,y=Y,z=Z),
                color=ColorRGBA(r=R,g=G,b=B,a=A),
                pose=Pose(position=Point(x = a[0]-self.offset[0], y = a[1]-self.offset[1], z = p))
            )
            self.pose_pub.publish(rviz_msg_obs)
            i += 1

    def present_MAP(self,ID,R,G,B,A):
        rviz_msg_map=Marker(
            header=Header(frame_id='map', stamp=rospy.get_rostime()),
            ns="track_Map",
            id=ID,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(),
            action=Marker.ADD,
            scale=Vector3(0.1,0.0,0.0),
            color=ColorRGBA(r=R,g=G,b=B,a=A)
        )

        path_arr=np.load(file=PATH_ROOT+"path/"+Tracking_path)
        s=range(len(path_arr))
        for a in s:
            p=Point()
            p.x=float(path_arr[a,0])-self.offset[0]
            p.y=float(path_arr[a,1])-self.offset[1]
            p.z=0
            rviz_msg_map.points.append(p)

        self.map_pub.publish(rviz_msg_map)

def main():
    rospy.init_node('rviz_test',anonymous=True)
    rate=rospy.Rate(10) #0.5초마다 그림

    if WHERE == 2 or WHERE == 3:
        where = 2
    else:
        where = 1

    Vis = Visualization(where)
    
    count = 1
    while not rospy.is_shutdown():
        if count == 1:
            Vis.offset_update()
            Vis.global_path()
            Vis.present_MAP(102,0.0,1.0,0.0,0.7) #전역경로
            count = 0

        Vis.present_LINE(101,1.0,0.0,0.0,1.0,2,True) #pathLOG #내가 지나온 길을 pub해주는 메서드
        Vis.present_OBJECT(100,Marker.ARROW,2.0,0.5,0.5,0.0,1.0,0.0,1.0) #presentPOSE() #현재 내 위치랑 헤딩방향을 pub 해주는 메서드
        Vis.present_OBS(Marker.SPHERE,0.2,0.2,0.2,1.0,0.0,0.0,1.0,0.5) #vis_obs
        Vis.present_OBS(Marker.CYLINDER,0.5,0.5,2.0,1.0,0.7,0.0,1.0,0.0) #vis_obs

        Vis.present_LINE(301,0.0,1.0,0.0,0.8,1) #track_GBpath
        count += 1
        rate.sleep()
            
if __name__ == '__main__':
    main()