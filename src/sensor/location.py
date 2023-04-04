#!/usr/bin/env python
# -- coding: utf-8 --
# import rospy
from pyproj import Proj, transform
# from tf.transformations import euler_from_quaternion
# from geometry_msgs.msg import Quaternion
import numpy as np
from math import pi

heading_offset = 0   #오른쪽 +, 왼쪽 -
DIST = 0.3           # 헤딩 기록 구간
RECORD_NUMBER = 2   # 헤딩 기록 개수
STRAIGHT_ANGLE = 5  # 직진 판정 각도
VAL_WEIGHT = 3

class gps_imu_fusion:
    def __init__(self):
        #Projection definition
        #UTM-K
        self.proj_UTMK = Proj(init='epsg:5179')
        #WGS1984
        self.proj_WGS84 = Proj(init='epsg:4326')

        self.b = np.zeros((RECORD_NUMBER, 4))
        self.c = np.zeros((10,1))

    def tf_to_tm(self,lon,lat):
        x,y=transform(self.proj_WGS84,self.proj_UTMK,lon,lat)
        return x,y

    def tf_heading_to_rad(self, head):
        heading = 5*pi/2 - np.deg2rad(float(head / 100000))
        if heading > 2*pi:
            heading -= 2*pi
        return heading

    def q_to_yaw(self, imu):
        # orientation_list = [imu.x, imu.y, imu.z, imu.w]
        # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        # print(yaw) 
        yaw = (-1) * imu.x * pi / 180

        if yaw < 0:
            yaw = pi + (pi + yaw)

        # print(yaw)

        return yaw

    def heading_correction(self, x, y, imu_heading, gps_heading):
        global heading_offset

        if self.b[0][0] == 0 and self.b[0][1]==0:
                self.b[0][0] = x
                self.b[0][1] = y
                self.b[0][2] = imu_heading
                self.b[0][3] = gps_heading

        else:
            distance = np.hypot(self.b[0][0] - x, self.b[0][1] - y)

            if distance >= DIST:
                
                for i in range(RECORD_NUMBER - 1, -1, -1) :
                    self.b[i][0] = self.b[i-1][0]
                    self.b[i][1] = self.b[i-1][1]
                    self.b[i][2] = self.b[i-1][2]
                    self.b[i][3] = self.b[i-1][3]

                self.b[0][0] = x
                self.b[0][1] = y
                self.b[0][2] = imu_heading
                self.b[0][3] = gps_heading

                if self.b[RECORD_NUMBER - 1][0] != 0 or self.b[RECORD_NUMBER - 1][1] != 0:

                    max_heading = np.max(self.b, axis=0)
                    min_heading = np.min(self.b, axis=0)

                    print(max_heading, min_heading)


                    if (max_heading[3] - min_heading[3] < STRAIGHT_ANGLE*pi/180) and (max_heading[2] - min_heading[2] < STRAIGHT_ANGLE*pi/180) :

                        # avg_heading = np.mean(self.b, axis=0)
                        # heading_offset = avg_heading[3] - avg_heading[2]
                        
                        var_heading = np.var(self.b, axis=0)
                        avg_heading = np.mean(self.b, axis=0)

                        # x = Symbol('x')
                        # f = exp(-(x-avg_heading[3])**2/(2*var_heading[3]**2))/(var_heading[3]*sqrt(2*pi))

                        if (avg_heading[2] < avg_heading[3] - VAL_WEIGHT*var_heading[3]) :
                            heading_offset = (avg_heading[3] - VAL_WEIGHT*var_heading[3]) - avg_heading[2]  

                        elif (avg_heading[2] > avg_heading[3] + VAL_WEIGHT*var_heading[3]) :
                            heading_offset = (avg_heading[3] + VAL_WEIGHT*var_heading[3]) - avg_heading[2] 

                        else :
                            heading_offset = 0

                        #     print(var_heading[3])



                        # Integral(f, (x, 3, 7)).doit().evalf()
                        # heading_offset = 0

                        # self.b = np.zeros((RECORD_NUMBER, 4))
                        


        # print(self.b)
        print(heading_offset)
        # heading_offset = 0        
        return heading_offset

    def get_heading(self, x, y, imu_orientation, gps, i):
        global heading_offset
        gps_heading = self.tf_heading_to_rad(gps)
        imu_heading = self.q_to_yaw(imu_orientation) #imu heading update


        heading = imu_heading
        print(heading), 'imu heading'    

        # heading = heading + (heading_offset * pi / 180)
        if heading > 2 * pi:
            heading = heading - (2 * pi)
        elif heading <= 0:
            heading = heading + (2 *pi)


        off_temp = heading_offset
        heading_offset = self.heading_correction(x, y, heading, gps_heading)


        if self.c[0][0] == 0 and abs(heading_offset) < 30*pi/180:
            self.c[0][0] = heading_offset

        else:
            
            if abs(heading_offset) < 30*pi/180:
                for i in range(10 - 1, -1, -1) :
                    self.c[i][0] = self.c[i-1][0]

                self.c[0][0] = heading_offset

                if self.c[9][0] != 0:
                    avg_heading_offset = np.mean(self.c, axis=0)

                    heading_offset = avg_heading_offset[0]


        print(self.c)




        if abs(heading_offset) > 30*pi/180:
            heading_offset = off_temp

        heading = heading + heading_offset
        # heading = heading
                        
        # # print(heading)gps_imu_fusion
        return heading