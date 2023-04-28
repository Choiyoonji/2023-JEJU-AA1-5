#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

class publish_erp():
    def __init__(self):
        self.erp_pub = rospy.Publisher("erp_write", Twist, queue_size=1)
        self.erp = Twist()

    def pub_erp(self, speed, steer):
        self.erp.linear.x = speed
        self.erp.angular.z = steer
        self.erp_pub.publish(self.erp)

class publish_cmd():
    def __init__(self):
        self.cmd_pub = rospy.Publisher("cmd_write", Twist, queue_size=1)
        self.cmd = Twist()

    def pub_cmd(self, speed, steer):
        self.cmd.linear.x = speed
        self.cmd.angular.z = steer
        self.cmd_pub.publish(self.cmd)

class publish_state():
    def __init__(self):
        self.state_pub = rospy.Publisher("state_read", String, queue_size=1)
        self.state = String()
    
    def pub_state(self, E_STOP, MANUAL, AUTO):
        self.state = "state: "
        
        if E_STOP: self.state = self.state + "E-STOP "
        if MANUAL: self.state = self.state + "MANUAL "
        if AUTO: self.state = self.state + "AUTO"
        
        self.state_pub.publish(self.state)

class Sub_Twists:
    def __init__(self):
        self.teleop_sub = rospy.Subscriber('/cmd_vel', Twist, self.teleop_callback, queue_size=1)
        self.state_sub = rospy.Subscriber('/erp_write', Twist, self.state_callback, queue_size=1)
        
        self.manual_speed = 0
        self.manual_steer = 0
        self.auto_speed = 0
        self.auto_steer = 0
        
        self.E_STOP = True
        self.MANUAL = True
        self.AUTO = False
        
    def teleop_callback(self, data):
        self.manual_speed = data.linear.x
        self.manual_steer = data.angular.z
        
        if self.manual_speed == 49.5 and self.manual_steer == 99:
            self.E_STOP = True
        elif self.manual_speed == -49.5 and self.manual_steer == -99:
            self.E_STOP = False
        elif self.manual_speed == 500 and self.manual_steer == 1000:
            self.AUTO = False
            self.MANUAL = True
        elif self.manual_speed == -500 and self.manual_steer == -1000:
            self.AUTO = True
            self.MANUAL = False
        
    def state_callback(self, data):
        self.auto_speed = data.linear.x
        self.auto_steer = data.angular.z
        
def main():
    rospy.init_node('total_node', anonymous=True)
    
    rate = rospy.Rate(10)
    sub = Sub_Twists()
    pub_erp = publish_erp()
    pub_cmd = publish_cmd()
    pub_state = publish_state()
    
    while not rospy.is_shutdown():
        if sub.E_STOP:
            pub_cmd.pub_cmd(0,0)
        elif sub.MANUAL:
            pub_cmd.pub_cmd(sub.manual_speed,sub.manual_steer)
        elif sub.AUTO:
            pub_erp.pub_erp(sub.auto_speed,sub.auto_steer)
            
        pub_state.pub_state(sub.E_STOP, sub.MANUAL, sub.AUTO)
        
        rate.sleep()
        
if __name__ == '__main__':
    main()