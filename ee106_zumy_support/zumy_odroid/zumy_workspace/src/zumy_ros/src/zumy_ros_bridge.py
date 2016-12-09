#!/usr/bin/python

import rospy
from geometry_msgs.msg import Vector3, Twist
from threading import Condition
from zumy import Zumy
from std_msgs.msg import String,Header,Int32,Float32,Bool
from sensor_msgs.msg import Imu
from zumy_ros.msg import ZumyStatus
import numpy as np
import socket,time
import sys

"""
Purpose: This code publishes commands (to topics that the zumy subscribes to), which drives the Zumy.

"""
def twist_to_speeds(msg):  #function to convert a geometry_message_twist to a robot action
  #done with math as done by doug
  r = 3.4/2.0 * 0.0254 #radius of the zumy is 3.4 inches/2, converted to metere.
  vx = msg.linear.x
  wz = msg.angular.z
  #zumy can only move in the x, and rotate in the z.  no other movement is possible

  #velociy of the left and right tracks
  vr = vx+(wz*r)
  vl = vx-(wz*r)
  return (vl,vr) #vleft, vright.

def speeds_to_twist(vel_data):
  #vel data is a tuple (vl,vr)
  r = 3.4/2 * 0.024 #radius of the zumy is 3.4 inches/2, converted to metere.
  vx = (vel_data[0] + vel_data[1]) / 2
  wz = r*(vel_data[1] - vel_data[0])/2
  return (vx,wz)


class ZumyROS:
  def __init__(self):
    self.zumy = Zumy()
    rospy.init_node('zumy_ros')
    self.cmd = (0,0)
    rospy.Subscriber('cmd_vel', Twist, self.cmd_callback,queue_size=1)
    rospy.Subscriber('enable', Bool, self.enable_callback,queue_size=1)
    rospy.Subscriber('/base_computer',String,self.watchdog_callback,queue_size=1) #/base_computer topic, the global watchdog.  May want to investigate what happens when there moer than one computer and more than one zumy

    self.lock = Condition()
    self.rate = rospy.Rate(30.0)
    self.name = socket.gethostname()

    if rospy.has_param("~timeout"):
      self.timeout = rospy.get_param('~timeout') #private value
    else:
      self.timeout = 2 #Length of watchdog timer in seconds, defaults to 2 sec.
    
    #publishers
    self.status_pub = rospy.Publisher("Status",ZumyStatus, queue_size=5)
    self.imu_pub = rospy.Publisher('imu', Imu, queue_size = 1)
    self.r_enc_pub = rospy.Publisher('r_enc',Float32, queue_size = 5)
    self.l_enc_pub = rospy.Publisher('l_enc',Float32, queue_size = 5)
    self.r_vel_pub = rospy.Publisher('r_vel',Float32,queue_size = 5)
    self.l_vel_pub = rospy.Publisher('l_vel',Float32,queue_size = 5)
    
    self.imu_count = 0

    self.batt_pub = rospy.Publisher('Batt',Float32,queue_size = 5)

    self.last_message_at = time.time()
    self.watchdog = True 

    #an array of times, which will be updated, and used to figure out how fast the while loop is going.
    self.laptimes = np.ones(6)*time.time()
    
    # NEW!
    self.IR_ai_side_pub = rospy.Publisher('/' + self.name + '/IR_ai_side', Float32, queue_size=1000)
    self.IR_ai_front_pub = rospy.Publisher("/" + self.name + "/IR_ai_front", Float32, queue_size=1000)
    self.IR_ai_top_pub = rospy.Publisher('/' + self.name + '/IR_ai_top', Float32, queue_size=1000)
    self.IR_ai_bottom_pub = rospy.Publisher("/" + self.name + "/IR_ai_bottom", Float32, queue_size=1000)
    # Publish twists to /cmd_vel in order to drive the Zumy.

  def cmd_callback(self, msg):
    self.lock.acquire()
    self.cmd = twist_to_speeds(msg) #update the commanded speed, the next time the main loop in run comes through, it'll be update on the zumy.
    self.lock.release()

  def enable_callback(self,msg):
    #enable or disable myself based on the content of the message.   
    if msg.data and self.watchdog:
      self.zumy.enable()
    else:
      self.zumy.disable()
      self.cmd = (0,0) #turn the motors off
    #do NOT update the watchdog, since the watchdog should do it itself.
    #If i'm told to go but the host's watchdog is down, something's very wrong, and i won't be doing much

  def watchdog_callback(self,msg):
    #update the last time i got a messag!
    self.last_message_at = time.time()
    self.watchdog = True

  def run(self):

    while not rospy.is_shutdown():
      self.lock.acquire()
      self.zumy.cmd(*self.cmd)

      try:
        imu_data = self.zumy.read_imu()
        imu_msg = Imu()
        imu_msg.header = Header(self.imu_count,rospy.Time.now(),self.name)
        imu_msg.linear_acceleration.x = 9.81 * imu_data[0]
        imu_msg.linear_acceleration.y = 9.81 * imu_data[1]
        imu_msg.linear_acceleration.z = 9.81 * imu_data[2]
        imu_msg.angular_velocity.x = 3.14 / 180.0 * imu_data[3]
        imu_msg.angular_velocity.y = 3.14 / 180.0 * imu_data[4]
        imu_msg.angular_velocity.z = 3.14 / 180.0 * imu_data[5]
        self.imu_pub.publish(imu_msg)
      except ValueError:
        pass

      try:
        enc_data = self.zumy.enc_pos()
        enc_msg = Float32()
        enc_msg.data = enc_data[0]
        self.r_enc_pub.publish(enc_msg)
        enc_msg.data = enc_data[1]
        self.l_enc_pub.publish(enc_msg)
      except ValueError:
        pass

      try:
        vel_data = self.zumy.enc_vel()
        vel_msg = Float32()
        vel_msg.data = vel_data[0]
        self.l_vel_pub.publish(vel_msg)
        vel_msg.data = vel_data[1]
        self.r_vel_pub.publish(vel_msg)
      except ValueError:
        pass
        
      try:
        v_bat = self.zumy.read_voltage()
        self.batt_pub.publish(v_bat)
      except ValueError:
        pass

      try:
        IR_ai_front_data = self.zumy.read_IR_ai_front()
        scaled_front_data = IR_ai_front_data * 3.3
        self.IR_ai_front_pub.publish(scaled_front_data)
      except ValueError:
        self.IR_ai_front_pub.publish(19)
        pass

      try:
        IR_ai_side_data = self.zumy.read_IR_ai_side()
        scaled_side_data = IR_ai_side_data * 3.3
        self.IR_ai_side_pub.publish(scaled_side_data)
      except ValueError:
        self.IR_ai_side_pub.publish(17)
        pass


      try:
        IR_ai_top_data = self.zumy.read_IR_ai_top()
        scaled_top_data = IR_ai_top_data * 3.3
        self.IR_ai_top_pub.publish(scaled_top_data)
      except ValueError:
        self.IR_ai_top_pub.publish(16)
        pass


      try:
        IR_ai_bottom_data = self.zumy.read_IR_ai_bottom()
        scaled_bottom_data = IR_ai_bottom_data * 3.3
        self.IR_ai_bottom_pub.publish(scaled_bottom_data)
      except ValueError:
        self.IR_ai_bottom_pub.publish(18)
        pass

      #END NEW
      
      if time.time() > (self.last_message_at + self.timeout): #i've gone too long without seeing the watchdog.
        self.watchdog = False
        self.zumy.disable()
      self.zumy.battery_protection() # a function that needs to be called with some regularity.


      #handle the robot status message
      status_msg = ZumyStatus()
      status_msg.enabled_status = self.zumy.enabled
      status_msg.battery_unsafe = self.zumy.battery_unsafe()
      #update the looptimes, which will get published in status_msg
      self.laptimes = np.delete(self.laptimes,0)
      self.laptimes = np.append(self.laptimes,time.time())
      #take average over the difference of the times... and then invert.  That will give you average freq.
      status_msg.loop_freq = 1/float(np.mean(np.diff(self.laptimes)))

      self.status_pub.publish(status_msg)
      self.lock.release() #must be last command involving the zumy.
    
      self.rate.sleep()
      self.first_move = False

    # If shutdown, turn off motors & disable anything else.
    self.zumy.disable()

if __name__ == '__main__':
  zr = ZumyROS()
  zr.run()
