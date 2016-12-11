#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3

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

def rad2deg(rad):
  return rad * 180./np.pi

# NEW!
def is_rotating(twist):
  angular_velocity = twist.angular
  if angular_velocity.z > .01:
    return True
  return False