#! /usr/bin/python

import threading
import rospy
from geometry_msgs.msg import Twist

class ZTBridge():
  def __init__(self):
    t = Twist()
    t.linear.x = 0
    t.linear.y = 0
    t.linear.z = 0
    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = 0
    self.still = t 
    self.twist = self.still
    self.lock = threading.Condition()
    self.pub = rospy.Publisher("/odroid6/cmd_vel", Twist, queue_size = 1)
    rospy.init_node("zumy_teleop_bridge", anonymous=True)
    rospy.Subscriber("/turtle1/cmd_vel/", Twist, self.teleopCB)

  def init_twist(self):
    t = Twist()
    t.linear.x = 0
    t.linear.y = 0
    t.linear.z = 0
    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = 0
    self.still = t 

  def teleopCB(self, data):
    self.twist.linear.x = 0.1 * data.linear.x
    self.twist.angular.z = 1.0 * data.angular.z
  
  
  def run(self):
    rate = rospy.Rate(10) #"/turtle1/cmd_vel/" is published at 30Hz, so it will overwrite "/odroid6/cmd_vel"   
    while not rospy.is_shutdown(): 
      self.pub.publish(self.twist)
      self.init_twist()
      self.twist = self.still
      rate.sleep()
  
if __name__ == "__main__":
  ztb = ZTBridge()
  ztb.run()
