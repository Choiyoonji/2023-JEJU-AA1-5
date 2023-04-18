#!/usr/bin/env python
# -- coding: utf-8 --
# import rospy
from pyproj import Proj, transform
import numpy as np
from math import pi

heading_offset = 0   #오른쪽 +, 왼쪽 -
DIST = 0.3           # 헤딩 기록 구간
RECORD_NUMBER = 2   # 헤딩 기록 개수
STRAIGHT_ANGLE = 5  # 직진 판정 각도
VAL_WEIGHT = 3

class only_gps:
    def __init__(self):
        #Projection definition
        #UTM-K
        self.proj_UTMK = Proj(init='epsg:5179')
        #WGS1984
        self.proj_WGS84 = Proj(init='epsg:4326')
        self.last_x = 0
        self.last_y = 0
        self.last_heading = 0
        
    def tf_to_tm(self,lon,lat):
        x,y=transform(self.proj_WGS84,self.proj_UTMK,lon,lat)
        return x,y

    def tf_heading_to_rad(self, head):
        heading = 5*pi/2 - np.deg2rad(float(head / 100000))
        if heading > 2*pi:
            heading -= 2*pi
        return heading
    
    def rec_heading(self,x,y,heading):
        if self.last_x == 0 and self.last_y == 0 and self.last_heading == 0:
            self.last_x = x
            self.last_y = y
            self.last_heading = heading
        elif np.hypot(self.last_x - x, self.last_y - y) >= DIST:
            self.last_x = x
            self.last_y = y
            self.last_heading = heading
            
    def get_heading(self,x,y,heading):
        gps_heading = self.tf_heading_to_rad(heading)
        
        self.rec_heading(x,y,gps_heading)
        
        return self.last_heading