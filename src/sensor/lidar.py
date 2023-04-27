#!/usr/bin/env python
import rospy
import numpy as np

from math import cos, sin

class lidar:
    def __init__(self):
        self.obs_xy = None

    def tf_tm(self, scan, x, y, heading):
        self.obs_xy = np.empty((1, 2))
        resolution = 3
        T = [[cos(heading), -sin(heading), x],
             [sin(heading),  cos(heading), y],
             [0, 0, 1]]       # transform matrix, which is transform lidar_xy coordinate to tm coordinate.

        scan_data_for_search = scan[135:675]  # 정면 0° 기준 -90° ~ 90° 범위, 분해능 0.333°, 540개 데이터 (180 x 3)
        for i in range(0, 540):
            if 0.1 <= scan_data_for_search[i] <= 15:
                obs_x = scan_data_for_search[i] * sin(np.deg2rad(float(i) / resolution)) + 1.1  # 1.1m: gps-lidar offset
                obs_y = -scan_data_for_search[i] * cos(np.deg2rad(float(i) / resolution))
                self.obs_xy = np.append(self.obs_xy, [np.dot(T, np.transpose([obs_x, obs_y, 1]))[0:2]], axis=0)
            else:
                pass

    def clean(self):
        f_arr = [[self.obs_xy[0][0], self.obs_xy[0][1]]]
        for i in range(len(self.obs_xy) - 1):
            m = abs(f_arr[-1][0] - self.obs_xy[i][0])
            n = abs(f_arr[-1][1] - self.obs_xy[i][1])
            if m >= 0.5 or n >= 0.5:
                f_arr.append([self.obs_xy[i][0], self.obs_xy[i][1]])
            else:
                pass
        return f_arr

def main():
    lidar = Lidar()
    
    a = np.array([[1.232, 0.123, 0], [1.390, 0.232, 0], [1.682, 0.133, 0], [1.891, 0.143, 0], [1.893, 0.179, 0],
                  [3, 1, 0], [4, 2, 0]])
    lidar.obs_xy = a
    print(lidar.obs_xy)
    b = lidar.clean()
    print(b)


if __name__ == '__main__':
    rospy.init_node('pcl_tutorial', anonymous=True)
    main()
