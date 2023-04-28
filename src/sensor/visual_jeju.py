#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from math import cos, sin, pi
import numpy as np
import os, sys

from geometry_msgs.msg import Vector3, Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, Float64, ColorRGBA


#지도 정보 경로 설정
PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/" #/home/gigi/catkin_ws/src/macaron_3/


class Visualization():
    def __init__(self):
        #publisher 설정
        self.map_pub = rospy.Publisher('map', Marker, queue_size = 1)


    def present_MAP(self,ID,R,G,B,A): #전역경로
        i = ID
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
        rviz_msg_map.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        path_arr=np.load(file=PATH_ROOT+"path/jeju_island_gp.npy")
        s=range(len(path_arr))
        for a in s:
            p=Point()
            p.x=float(path_arr[a,0])-260065.2578
            p.y=float(path_arr[a,1])-3681161.771
            p.z=0
            print(p)
            rviz_msg_map.points.append(p)
        # for a in range(10):
        #     p = Point()
        #     p.x=float(a)
        #     p.y=float(a)
        #     p.z=0
        #     print(p)
        #     rviz_msg_map.points.append(p)
        self.map_pub.publish(rviz_msg_map)

        #오프셋(지도상 0,0점이 되는 좌표) 를 업데이트 해주는 메서드
    


def main():
    rospy.init_node('visualization',anonymous=True)
    rate=rospy.Rate(10) #0.5초마다 그림

    Vis = Visualization()
    while not rospy.is_shutdown():
        Vis.present_MAP(102,0.0,1.0,0.0,0.7)
        rate.sleep()

    
if __name__ == '__main__':
    main()