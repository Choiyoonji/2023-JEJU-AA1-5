#!/usr/bin/env python
import rospy

from math import cos, sin, pi
import numpy as np

class lidar:
    def __init__(self):
        self.obs_xy = np.empty((1, 3))

    def tf_tm(self, scan, x, y, heading) :
        self.obs_xy = np.empty((1, 3))
        resolution = 3
        T = [[cos(heading), -sin(heading), x], \
             [sin(heading),  cos(heading), y], \
             [      0     ,      0       , 1]]       # transform matrix, which is tranform lidar_xy coordinate to tm coordinate.
        
        last_scan_data = scan
        scan_data_for_search = []
        for i in range(0, 540):
            scan_data_for_search.append(last_scan_data[i+135])
            if scan_data_for_search[i] < 0.1 or scan_data_for_search[i] > 15: # -90~90
                # scan_data_for_search[i] = 0
                pass
            elif scan_data_for_search[i] <= 15:
                obs_x = scan_data_for_search[i]*sin(np.deg2rad(float(i)/resolution)) + 1.35
                obs_y = -scan_data_for_search[i]*cos(np.deg2rad(float(i)/resolution))
                self.obs_xy = np.append(self.obs_xy, [np.dot(T, np.transpose([obs_x, obs_y, 1]))], axis=0)
                
        self.obs_xy[:,2] = 0
    
    def clean(self):
        self.code = np.array(self.obs_xy)
        f_arr = []
        f_arr.append([self.code[0][0],self.code[0][1],0.0])

        for i in range(len(self.code)-1):
            m = abs(f_arr[-1][0] - self.code[i][0])
            n = abs(f_arr[-1][1] - self.code[i][1])
            if m >= 0.5 or n >= 0.5:
                f_arr.append([self.code[i][0],self.code[i][1],0.0]) 
            else:
                pass

        return f_arr  

def main():
    rate = rospy.Rate(0.1)
    Lidar = lidar()
    
    a = np.array([[1.232, 0.123, 0],[1.390, 0.232, 0],[1.682, 0.133, 0],[1.891, 0.143, 0],[1.893, 0.179, 0],[3,1,0],[4,2,0]])
    Lidar.obs_xy = a
    print(Lidar.obs_xy)
    b = Lidar.clean()
    print(b)

if __name__ == '__main__':
    rospy.init_node('pcl_tutorial',anonymous=True)
    main()