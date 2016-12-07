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

"""
Purpose: This code publishes commands (to topics that the zumy subscribes to), which drives the Zumy.

"""

# NEW!
def skew_3d(omega):
  """
  Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.
  
  Args:
  omega - (3,) ndarray: the rotation vector
 
  Returns:
  omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix
  """
  omega = np.array(omega)
  if not omega.shape == (3,):
    raise TypeError('omega must be a 3-vector')
  
  #YOUR CODE HERE
  o1,o2,o3 = omega[0],omega[1],omega[2]
  omega_hat = np.array([[0,-o3, o2],[o3, 0, -o1],[-o2, o1, 0]])
  return omega_hat

# NEW!
def find_v(omega, theta, trans):
  """
  Finds the linear velocity term of the twist (v,omega) given omega, theta and translation
  
  Args:
  omega - (3,) ndarray : the axis you want to rotate about
  theta - scalar value
  trans - (3,) ndarray of 3x1 list : the translation component of the rigid body transform
  
  Returns:
  v - (3,1) ndarray : the linear velocity term of the twist (v,omega)
  """    
  r = rotation_3d(omega, theta)
  A = (np.eye(3)  - r).dot(skew_3d(omega)) + np.outer(omega, omega)*theta
  v = np.linalg.inv(A).dot(trans)
  return np.array([[v[0]],[v[1]],[v[2]]])

# NEW!
def create_rbt(omega, theta, trans):
  """
  Creates a rigid body transform using omega, theta, and the translation component.
  g = [R,p; 0,1], where R = exp(omega * theta), p = trans
  
  Args:
  omega - (3,) ndarray : the axis you want to rotate about
  theta - scalar value
  trans - (3,) ndarray or 3x1 array: the translation component of the rigid body motion
  
  Returns:
  g - (4,4) ndarray : the rigid body transform
  """
  rot = rotation_3d(omega, theta)
  g = np.zeros((4,4))
  g[:3,:3] = rot
  g[3,3] = 1
  g[:3,3] = trans
  return g

# NEW!
def rotation_3d(omega, theta):
  """
  Computes a 3D rotation matrix given a rotation axis and angle of rotation.
  
  Args:
  omega - (3,) ndarray: the axis of rotation
  theta: the angle of rotation
  
  Returns:
  rot - (3,3) ndarray: the resulting rotation matrix
  """
  if not omega.shape == (3,):
      raise TypeError('omega must be a 3-vector')
  
  #YOUR CODE HERE

  identity = np.eye(3)
  norm = np.linalg.norm(omega)
  omega_hat = skew_3d(omega)
  rot = identity + omega_hat*np.sin(norm*theta)/norm + np.dot(omega_hat, omega_hat) * (1-np.cos(norm*theta))/(norm*norm)
  return rot

# NEW!
def deg2rad(deg):
  # Zumy likes radians
  return deg * np.pi/180.

# NEW!
def turn_zumy(angle):
  # angle in degrees
  # returns a Twist that is published and turns zumy
  omega = np.array([0, 0, 1]) # rotate about z
  translation = np.array([0, 0, 0])
  theta = deg2rad(angle)
  omega = omega * theta
  rbt = create_rbt(omega, theta, translation)
  v = find_v(omega, theta, translation)
  linear = Vector3(v[0]/5, v[1]/5, v[2]/5)
  angular = Vector3(omega[0], omega[1], omega[2] / 10)
  twist = Twist(linear, angular)
  return twist

# NEW!
def move_zumy_forward():
  # returns a Twist that is published and moves Zumy forward 0.1 units
  linear = Vector3(.1, 0, 0)
  angular = Vector3(0, 0, 0)
  twist = Twist(linear, angular)
  return twist

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

# NEW!
def is_rotating(twist):
  angular_velocity = twist.angular
  if angular_velocity.z > .01:
    return True
  return False

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
    self.is_turning = False
    self.IR_ai_side_pub = rospy.Publisher('/' + self.name + '/IR_ai_side', Float32, queue_size=1)
    self.IR_ai_front_pub = rospy.Publisher("/" + self.name + "/IR_ai_front", Float32, queue_size=1)
    self.directions = {"F" : 0., "L" : 90., "R" : -90., "B" : 180.} # (CCW, 0 is N) the direction that the the zumy will face
    # Publish twists to /cmd_vel in order to drive the Zumy.
    self.zumy_vel_pub = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=2)

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
  
  # NEW!
  def stop(self):
    linear = Vector3(0, 0, 0)
    angular = Vector3(0, 0, 0)
    stop_twist = Twist(linear, angular)
    self.zumy_vel_pub.publish(stop_twist)
    self.is_turning = False

  def run(self):

    while not rospy.is_shutdown():
      self.lock.acquire()
      self.zumy.cmd(*self.cmd)

      self.is_turning = False

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

      # NEW!
      # Note: zumy code MUST be between acquire/release calls!
      # Note: read must be in a try-catch block!
      try:
        # self.zumy_vel_pub.publish(move_zumy_forward())
        # side port is p16
        IR_ai_side_data = self.zumy.read_IR_ai_side()
		scaled_side_data = IR_ai_side_data * 3.3
        self.IR_ai_side_pub.publish(scaled_side_data)
        
        '''
        if IR_ai_data < 0.5:
          twist = turn_zumy(self.directions['L'])
          self.zumy_vel_pub.publish(twist)
        else:
          twist = turn_zumy(self.directions['R'])
          self.zumy_vel_pub.publish(twist)
        '''
      except ValueError:
        self.IR_ai_side_pub.publish(33)
        pass
      try:

	# NEW! 
	# checks if acceleration about z axis is below a certain level.
        if imu_msg.angular_velocity.z < .05:

          IR_ai_front_data = self.zumy.read_IR_ai_front()
	  	  scaled_front_data = IR_ai_front_data * 3.3
          self.IR_ai_front_pub.publish(scaled_front_data)

          if scaled_front_data > 2.0:
            self.IR_ai_front_pub.publish(9999)
            twist = turn_zumy(self.directions['L'])
            self.zumy_vel_pub.publish(twist)
            start = time.time()
            while True:
              if time.time() > start + 10:
				self.stop()
                break
            
            self.is_turning = True
          
      except ValueError:
        self.IR_ai_front_pub.publish(36)
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

    # If shutdown, turn off motors & disable anything else.
    self.zumy.disable()

if __name__ == '__main__':
  zr = ZumyROS()
  zr.run()
