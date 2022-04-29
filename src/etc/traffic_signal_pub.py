#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy
import time

from macaron_4.msg import Traffic, obj_info


TRAFFIC_TIME = [1, 1, 1] # 초 노 빨 시간

class pub_signal:
    def __init__(self):
        self.traffic_pub = rospy.Publisher("/traffic_obj", Traffic, queue_size=1)
        
    def pub_traffic(self, signal):
        t = obj_info()
        t.ns = signal
        t.ymin = 100
        t.ymax = 102

        traffic = Traffic()
        traffic.obj.append(t)
       
        self.traffic_pub.publish(traffic)


def main():
    rospy.init_node("traffic_signal_pub", anonymous=True)

    rate = rospy.Rate(5)

    p = pub_signal()
    time_rec = time.time()
    while not rospy.is_shutdown():
        if time.time() - time_rec <= TRAFFIC_TIME[0]:
            # p.pub_traffic('straight_green_4')
            p.pub_traffic('all_green_4')
            print("파란불")
        
        elif time.time() - time_rec <= TRAFFIC_TIME[0] + TRAFFIC_TIME[1]:
            p.pub_traffic("orange_4")
            print("노란불")

        elif time.time() - time_rec <= TRAFFIC_TIME[0] + TRAFFIC_TIME[1] + TRAFFIC_TIME[2]:
            p.pub_traffic("red_4")
            print("빨간불")
        
        else:
            time_rec = time.time()

        rate.sleep()

if __name__ == '__main__':
    main()