#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
from std_msgs.msg import Int16, Int32, String
from geometry_msgs.msg import Twist

class erpreadTest:
    def __init__(self):
        self.state_sub = rospy.Subscriber('/state_read', String, self.state_callback, queue_size=100)
        self.speed_sub= rospy.Subscriber('/speed_read', Int16, self.speed_callback, queue_size=1)
        self.steer_sub= rospy.Subscriber('/steer_read', Int32, self.steer_callback, queue_size=1)
        self.state = ''
        self.erp_speed = 9999999
        self.erp_steer = 999999
        
    def state_callback(self, data):
        self.state = data.data    
        
    def speed_callback(self, data):
        self.erp_speed = data.data
        
    def steer_callback(self, data):
        self.erp_steer = data.data

def main():
    #기본 설정
    rospy.init_node('read', anonymous=True)
    Read = erpreadTest()

    while not rospy.is_shutdown():
        print("=============================")
        print(Read.state)
        print("speed: ",Read.erp_speed)
        print("steer: ",Read.erp_steer)
        rospy.sleep(0.1)

if __name__ == '__main__':
    main()