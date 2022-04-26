#!/usr/bin/env python
# -- coding: utf-8 --

# basic package
from datetime import date
import rospy
import time
from math import pi

# message file
from macaron_4.msg import erp_read
from macaron_4.msg import erp_write
from std_msgs.msg import Float64

# declaration
MAX__SPEED = int(60)
MIN___SPEED = int(30) # 평상시

class keyboard_in():
    def __init__(self):
        
        self.steer = int(0)
        self.speed = int(0)

        self.max_speed = int(200)
        self.min_speed = int(0)

        #pub sub
        self.erp_sub= rospy.Subscriber('erp_read', erp_read, self.erp_callback, queue_size=1)        
        self.erp_pub = rospy.Publisher("erp_write", erp_write, queue_size=1)
        self.erp = erp_write()


    def erp_callback(self, data):
        self.speed = data.read_speed
        self.steer = data.read_steer



    def serial(self,key):
        
        if(key==1):
            self.speed = self.speed + int(30)
        elif(key==2):
            self.speed = self.speed - int(30)
        elif(key==3):
            self.steer = self.steer + int(60)
        elif(key==4):
           self.steer = self.steer - int(60)
        elif(key==0):
            self.speed = int(0)

        if(self.speed > 200):
            self.speed = 200
        elif(self.speed < 0):
            self.speed = 0
        if(self.steer > 2000):
            self.steer = 2000
        elif(self.steer < -2000):
            self.steer = -2000

        speed = self.speed
        steer = self.steer
        
        return speed, steer
    
    def pub_serial(self, speed, steer):
        speed, steer = speed, steer
        self.erp.write_speed = speed
        self.erp.write_steer = steer

        self.erp_pub.publish(self.erp)


    def speed_planner(self):
        self.speed = 0


def main():
    rospy.init_node("speed_planner", anonymous=True)

    rate = rospy.Rate(10)

    sp = keyboard_in()
    while not rospy.is_shutdown():
        speed = sp.speed_planner()
        print("speed : 1+ / 2- , steer : 3+ , 4- ,stop : 0")
        key = input("key : ")

        speed, steer = sp.serial(key)
        sp.pub_serial(speed, steer)



        rate.sleep()


if __name__ == '__main__':
    main()