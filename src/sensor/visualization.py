#!/usr/bin/env python
# -- coding: utf-8 --

# <<<0 is tracking car, 1 is st>>>
#   0을 누르면 6초마다 오프셋(지도 중심) 이 내 위치로 초기화됨.
#   1을 누르면 내 위치가 초기화되지 않고, 내가 지나온 길이 계속 그려짐
#       이때 ready? 라는 글이 뜰텐데 이때 rviz에서 세팅을 다시 하고 아무 숫자를 입력하면 됨.

# rviz error 임시방편 -> 아래 코드를 창 하나 더 열어서 실행시키면 오류 안남
# rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map macaron 100

# << 실행 방법 >>
# 1. state.py와 bag파일 실행(bag파일 관련 내용은 마카롱 카페에 자세하게 나와 있음)
# 2. visualization.lauch 실행 또는 visualization.py와 rviz 실행(rviz>file>open config 에서 macaron_5>rviz에 있는 macaron_5.rviz 실행)

from matplotlib import offsetbox
import rospy
from math import cos, sin, pi
import numpy as np
import os, sys
from sub_erp_state import sub_erp_state
# sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))+"/sensor")

from geometry_msgs.msg import Vector3, Pose, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan, NavSatFix, PointCloud
from std_msgs.msg import Header, Float64, ColorRGBA

WHERE = 2
where = 4 # 1 DGU 2 kcity 3 서울대 시흥캠퍼스 4 제주도

#지도 정보 경로 설정
PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/" #/home/gigi/catkin_ws/src/macaron_3/

#지도의 파일 개수. HD 맵의 차선을 추가하거나하면 수정해야함
DGU_line=16
line=27
center=21
bus=8
snu_parking = 4

#전역경로파일
Tracking_path=[]
if WHERE == 1:
    Tracking_path="PG.npy" #대운동장 전체 경로
elif WHERE == 2:
    Tracking_path="jeju_island_gp.npy"
elif WHERE == 3:
    Tracking_path="jeju_island_gp_ccw.npy"
elif WHERE == 4:
    Tracking_path="mh_lidar.npy" #만해 한 바퀴
elif WHERE == 5:
    Tracking_path="mh_stair_in.npy" #만해 배달 바퀴
elif WHERE == 6:
    # Tracking_path="kcity_trial1.npy" #대운동장 직선만 
    # Tracking_path="PG.npy" #대운동장 전체 경로
    Tracking_path="snu_del_path1.npy"
elif WHERE == 7:
    # Tracking_path="k_city_bonseon1.npy"
    # Tracking_path="kcity_trial2.npy"
    # Tracking_path="snu_main_del_main.npy" #주차공간 코스
    # Tracking_path="snu_delivery.npy"
    # Tracking_path="20220820_kcity_map.npy"
    # Tracking_path = "snu_main_parking.npy" #곡선 코스
    # Tracking_path = "snu_tracking_test1.npy" #곡선 코스 + 직선코드
    # Tracking_path = "snu_go1.npy" # 직선코스
    # Tracking_path = "snu_camera_test_curve.npy" #차선보정용 직선 흔들 코스
    # Tracking_path = "snu_testmap1.npy"
    # Tracking_path = "track_map3.npy"
    Tracking_path = "mhgp_0501.npy"
    # Tracking_path = "won230111.npy"
elif WHERE == 8:
    Tracking_path="kcity_uturn.npy"
elif WHERE == 9:
    for i in range(snu_parking):
        file_name="snu_add_parking%d.npy"%(i+1)
        Tracking_path.append(file_name)
#snu_go : 직선코스 
    

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
                file_name="DGU_%d.npy"%(i+1)
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
            self.PATH.append("kcity_bus_static.npy")

        elif where==3:
            print("draw snu map")
            #file_name="snu_line.npy"
            #self.PATH.append(file_name)
            file_name="snu_bus_line.npy"
            self.PATH.append(file_name)
        
        #publisher 설정
        self.global_pub = rospy.Publisher('/rviz_global_path', Marker, queue_size = len(self.PATH)+1)
        # self.pose_pub = rospy.Publisher('/rviz_pose', Marker, queue_size = 1)
        # self.log_pub = rospy.Publisher('/rviz_log', Marker, queue_size = 1)
        # self.tracking_pub = rospy.Publisher('/rviz_tracking_path', Marker, queue_size = 1)
        # self.obs_pub = rospy.Publisher('/rviz_obs', Marker, queue_size = 50)
        self.cdpath_pub = rospy.Publisher('/rviz_CDpath', Marker, queue_size = 5)
        self.slpath_pub = rospy.Publisher('/rviz_SLpath', Marker, queue_size = 1)
        # self.track_gbpath_pub = rospy.Publisher('/rviz_trackGBpath', Marker, queue_size = 1)
        self.goalpoint_pub = rospy.Publisher('/rviz_goalpoint', Marker, queue_size = 1)
        self.point_pub = rospy.Publisher('/rviz_point', Marker, queue_size = 1)
        # self.obs_sign_pub = rospy.Publisher('/rviz_obs_sign', Marker, queue_size = 5)
        self.pose_pub = rospy.Publisher('pose', Marker, queue_size = 1)
        self.line_pub = rospy.Publisher('line', Marker, queue_size = 1)
        self.obs_pub = rospy.Publisher('obs', Marker, queue_size = 5)
        self.map_pub = rospy.Publisher('map', Marker, queue_size = 1)

        self.cdpath_sub = rospy.Subscriber('/CDpath', PointCloud, self.CDpath_callback, queue_size = 1)
        self.slpath_sub = rospy.Subscriber('/SLpath', PointCloud, self.SLpath_callback, queue_size = 1)
        self.goalpoint_sub = rospy.Subscriber('/goal_point', Point, self.goalpoint_callback, queue_size = 1)
        self.track_gb_sub = rospy.Subscriber('/track_gbpath', PointCloud, self.track_GBpath_callback, queue_size = 1)
        self.sign_sub = rospy.Subscriber('/sign', PointCloud, self.sign_loc_callback, queue_size = 1)

        if where == 1: self.offset = [955926.9659, 1950891.243]
        elif where == 2 : self.offset = [935482.4315, 1915791.089]
        elif where == 3 :self.offset = [931326.1071073, 1929913.8061744] 
        elif where == 4: self.offset = [260065.2578, 3681161.771]
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
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
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
                header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
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
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
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
                z=0
            elif st[6:8] == "li":
                cl=ColorRGBA(1.0,1.0,1.0,1.0)
                z=0
            elif st[6:8] == "st":
                cl=ColorRGBA(1.0,0.0,0.0,1.0)
                z=0
            elif st[0:2] == "DG":
                cl=ColorRGBA(1.0,1.0,1.0,1.0)
                z=0
            elif st[6:8] == "bu":
                cl=ColorRGBA(0.0,0.0,1.0,1.0)
                z=0.2
            elif st[0:2] == "sn":
                cl=ColorRGBA(1.0,1.0,1.0,1.0)
                z=0

            rviz_msg_global=Marker(
                header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
                ns="global_path",
                type=Marker.LINE_STRIP,
                action=Marker.ADD,
                id=i,
                scale=Vector3(0.1,0.1,0),
                color=cl)

            path_arr=np.load(file=PATH_ROOT+"global_map/"+self.PATH[i])
            s=range(len(path_arr))
            for a in s:
                p=Point()
                p.x=float(path_arr[a,0])-self.offset[0]
                p.y=float(path_arr[a,1])-self.offset[1]
                p.z=z
                rviz_msg_global.points.append(p)
            self.global_pub.publish(rviz_msg_global)
            i+=1

    def present_OBJECT(self,ID,TYPE,X,Y,Z,R,G,B,A): #점 # 사용 방법 : id, rviz에 띄울 도형(ros rviz 튜토리얼 참고), 도형 크기(x,y,z), 색(r,g,b), 투명도(1이 최대)
        pose = Pose()

        q = euler_to_quaternion(0, 0, self.erp.heading) #좌표변환
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        pose.position.x = self.erp.pose[0] - self.offset[0]
        pose.position.y = self.erp.pose[1] - self.offset[1]
        pose.position.z = 0

        rviz_msg_pose=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()), # fraeme_id -> fixed frame 이랑 같게 맞추어 주어야함.
            ns="object", 
            id=ID,#무조건 다 달라야함
            type=TYPE, # 튜토리얼에 있는것 보고 바꾸면 돼..
            lifetime=rospy.Duration(), #얼마동안 보여줄건지
            action=Marker.ADD,
            pose=pose, #lins_strip은point 사용
            scale=Vector3(x=X,y=Y,z=Z), 
            color=ColorRGBA(r=R,g=G,b=B,a=A), #색,a는 투명도 1이 최대
            )

        self.pose_pub.publish(rviz_msg_pose)

    def present_LINE(self,ID,R,G,B,A,PATH_ARR,log=False): #선 # 사용 방법 : id, 색(r,g,b), 투명도(1이 최대) , path 아래 추가하고 사용할 path 번호, path_log 사용할 때만 true
        if log == True :
            self.past_path.append([self.erp.pose[0], self.erp.pose[1]])
        
        rviz_msg_line=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
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

    def present_OBS(self,TYPE,X,Y,Z,R,G,B,A,p,sign=False): #장애물
        if sign == False :
            i=200
            for a in self.erp.obs:
                rviz_msg_obs=Marker(
                    header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
                    ns="obs",
                    id = i,
                    type=TYPE,
                    lifetime=rospy.Duration(0.5),
                    action=Marker.ADD,
                    scale=Vector3(x=X,y=Y,z=Z),
                    color=ColorRGBA(r=R,g=G,b=B,a=A),
                    pose=Pose(position=Point(x = a[0]-self.offset[0], y = a[1]-self.offset[1], z = p))
                )
                self.obs_pub.publish(rviz_msg_obs)
                i += 1
        else : 
            i=250
            for a in self.obs:
                rviz_msg_obs=Marker(
                    header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
                    ns="obs",
                    id = i,
                    type=TYPE,
                    lifetime=rospy.Duration(0.5),
                    action=Marker.ADD,
                    scale=Vector3(x=X,y=Y,z=Z),
                    color=ColorRGBA(r=R,g=G,b=B,a=A),
                    pose=Pose(position=Point(x = a[0]-self.offset[0], y = a[1]-self.offset[1], z = p))
                )
                self.obs_pub.publish(rviz_msg_obs)
                i += 1

    def present_OBS1(self): #캡스톤용 코드
        i=200
        for b in self.erp.obs:
            d = ((self.erp.pose[0] - b[0])**2 + (self.erp.pose[1] - b[1])**2)**0.5
            print(d)
            if d <= 0 : #### 이것만 괜찮은 거리로 고치면 돼
                rviz_msg_obs=Marker(
                    header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
                    ns="obs",
                    id = i,
                    type=Marker.CYLINDER,
                    lifetime=rospy.Duration(0.5),
                    action=Marker.ADD,
                    scale=Vector3(0.4,0.4,0.8),
                    color=ColorRGBA(1.0,0.85,0.8,1.0),
                    pose=Pose(position=Point(x = b[0]-self.offset[0], y = b[1]-self.offset[1], z = 0.0))
                )
                self.obs_pub.publish(rviz_msg_obs)
                i += 1
            else : 
                rviz_msg_obs=Marker(
                    header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
                    ns="obs",
                    id = i,
                    type=Marker.SPHERE,
                    lifetime=rospy.Duration(0.5),
                    action=Marker.ADD,
                    scale=Vector3(0.4,0.4,0.2),
                    color=ColorRGBA(1.0,0.0,0.0,1.0),
                    pose=Pose(position=Point(x = b[0]-self.offset[0], y = b[1]-self.offset[1], z = 0.5))
                )
                self.obs_pub.publish(rviz_msg_obs)
                i += 1

    def present_MAP(self,ID,R,G,B,A): #전역경로
        i=ID
        if WHERE == 9:
            for j in range(len(Tracking_path)):
                rviz_msg_map=Marker(
                header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
                ns="track_Map",
                id=i,
                type=Marker.LINE_STRIP,
                lifetime=rospy.Duration(),
                action=Marker.ADD,
                scale=Vector3(0.1,0.0,0.0),
                color=ColorRGBA(r=R,g=G,b=B,a=A)
                )
                path_arr=np.load(file=PATH_ROOT+"path/"+Tracking_path[j])
                s=range(len(path_arr))
                for a in s:
                    p=Point()
                    p.x=float(path_arr[a,0])-self.offset[0]
                    p.y=float(path_arr[a,1])-self.offset[1]
                    p.z=0
                    rviz_msg_map.points.append(p)
                self.map_pub.publish(rviz_msg_map)
                i+=1
        else : 
            rviz_msg_map=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
            ns="track_Map",
            id=i,
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

        #오프셋(지도상 0,0점이 되는 좌표) 를 업데이트 해주는 메서드
    def offset_update(self):
        self.offset=[self.erp.pose[0], self.erp.pose[1]]

        # if where == 1: self.offset = [955926.9659, 1950891.243]
        # elif where == 2 :  self.offset = [935482.4315,	1915791.089]
        # elif where == 3 :self.offset = [931326.1071073, 1929913.8061744] 


def main():
    rospy.init_node('visualization',anonymous=True)
    rate=rospy.Rate(10) #0.5초마다 그림

    mode = 0

    Vis = Visualization(where)

    if mode == 0:
        # raw_input('ready?')
        count = 1
        while not rospy.is_shutdown():
            if count == 1:
                Vis.offset_update()

                Vis.present_MAP(102,0.0,1.0,0.0,0.7) #전역경로
                # Vis.present_OBS(Marker.CYLINDER,0.2,0.2,0.2,0.2,0.2,0.2,1.0,0.5) #미션 시작과 끝
                count = 0

            # Vis.present_LINE(101,1.0,0.0,0.0,1.0,2,True) #pathLOG #내가 지나온 길을 pub해주는 메서드
            Vis.present_OBJECT(100,Marker.ARROW,1.0,0.3,0.3,0.0,1.0,0.0,1.0) #presentPOSE() #현재 내 위치랑 헤딩방향을 pub 해주는 메서드
            # Vis.present_OBS(Marker.SPHERE,0.2,0.2,0.2,1.0,0.0,0.0,1.0,0.5) #vis_obs
            Vis.present_OBS1() #캡스톤
            # Vis.present_OBS(Marker.CYLINDER,0.5,0.5,2.0,1.0,0.7,0.0,1.0,0.0,True) #vis_obs_sign

            Vis.present_LINE(301,0.0,1.0,0.0,0.8,1) #track_GBpath
            count += 1
            rate.sleep()
    else:
        # raw_input('ready?')
        Vis.offset_update()
        Vis.global_path()# 지도
        Vis.present_MAP(102,0.0,1.0,0.0,0.7) #전역경로

        while not rospy.is_shutdown():
            # Vis.present_LINE(101,1.0,0.0,0.0,1.0,2,True) #pathLOG #내가 지나온 길을 pub해주는 메서드
            Vis.present_OBJECT(100,Marker.ARROW,2.0,0.5,0.5,0.0,1.0,0.0,1.0) #presentPOSE() #현재 내 위치랑 헤딩방향을 pub 해주는 메서드
            Vis.present_OBS(Marker.SPHERE,0.2,0.2,0.2,1.0,0.0,0.0,1.0,0.5) #vis_obs
            Vis.present_OBS(Marker.CYLINDER,0.5,0.5,2.0,1.0,0.7,0.0,1.0,0.0,True) #vis_obs_sign
            Vis.present_LINE(301,0.0,1.0,0.0,0.8,1) #track_GBpath
            rate.sleep()

            
if __name__ == '__main__':
    main()