#!/usr/bin/python
import sys
# Python libs
import sys, time
# numpy and scipy
import numpy as np
# Ros libraries
import rospy
import tf
 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
 
from sys import *
from std_msgs.msg import String
import sys, select, termios, tty
 
## Message
## Topic: sim
## "start" begins motion detection

 
class ctr:
    def __init__(self):
        print "attaching to ROS"
        self.pub = rospy.Publisher('node0/cmd_vel', Twist, queue_size=10)
        self.subscriber = rospy.Subscriber("node0/odom", Odometry, self.callback)
        self.speed = 0.2
        self.keysin = ['i','o','p','l']
 
    def callback(self,data):
        self.loop(data) 

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
 
    def loop(self,data):
        # get dt
        key = self.getKey()
        twist = Twist()
        rospy.loginfo("position %f,%f yaw %f, dt %f, u %f",data.pose.pose.position.x, data.pose.pose.position.y, 3.0, 10, self.speed)
        self.pub.publish(twist)
        if key in self.keysin:
            if key == 'i':
                twist.linear.x = -1*self.speed
            elif key == 'o':
                twist.linear.y = -1*self.speed
            elif key == 'p':
                twist.linear.x = self.speed
            elif key == 'l':
                twist.linear.y = self.speed
            else: 
                twist.linear.x=0;
                twist.linear.y=0
        self.pub.publish(twist)
        time.sleep(2)



 
 
def main(args):
    print "starting controller"
    ctr()
    rospy.init_node('ctr', anonymous=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass
 
if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    main(sys.argv)



