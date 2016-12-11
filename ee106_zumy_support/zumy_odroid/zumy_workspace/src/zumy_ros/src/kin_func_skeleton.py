#!/usr/bin/env python
"""Kinematic function skeleton code for Prelab 3.
Course: EE 106A, Fall 2015
Written by: Aaron Bestick, 9/10/14
Used by: EE106A, 9/11/15

This Python file is a code skeleton for Pre lab 3. You should fill in 
the body of the eight empty methods below so that they implement the kinematic 
functions described in the homework assignment.

When you think you have the methods implemented correctly, you can test your 
code by running "python kin_func_skeleton.py at the command line.

This code requires the NumPy and SciPy libraries. If you don't already have 
these installed on your personal computer, you can use the lab machines or 
the Ubuntu+ROS VM on the course page to complete this portion of the homework.
"""

import numpy as np
import scipy as sp
from scipy import linalg

np.set_printoptions(precision=4,suppress=True)

def transformation(theta):
    # h is the 4x4 homogeneous matrix
    # theta is a numpy vector of 7 joint angles
    initial_q = np.array([.7957, .9965, .3058])
    q = np.zeros((4,4))
    q[:3, :3] = np.identity(3)
    q[:3, 3] = initial_q
    q[3,3] = 1
    q[3, :3] = 0
    q1 = np.array([.0635, .2598, .1188])
    w1 = np.array([-.0059, .0113, .9999])
    q2 = np.array([.1106, .3116, .3885])
    w2 = np.array([-.7077, .7065, -.0122])
    q3 = np.array([.1827, .3838, .3881])
    w3 = np.array([.7065, .7077, -.0038])
    q4 = np.array([.3682, .5684, .3181])
    w4 = np.array([-.7077, .7065, -.0122])
    q5 = np.array([.4417, .6420, .3177])
    w5 = np.array([.7065, .7077, -.0038])
    q6 = np.array([.6332, .8337, .3067])
    w6 = np.array([-.7077, .7065, -.0122])
    q7 = np.array([.7152, .9158, .3063])
    w7 = np.array([.7065, .7077, -.0038])
    
    v1 = np.cross(-w1, q1)
    v2 = np.cross(-w2, q2)
    v3 = np.cross(-w3, q3)
    v4 = np.cross(-w4, q4)
    v5 = np.cross(-w5, q5)
    v6 = np.cross(-w6, q6)
    v7 = np.cross(-w7, q7)

    t1 = np.array([v1, w1]).reshape(6)
    t2 = np.array([v2, w2]).reshape(6)
    t3 = np.array([v3, w3]).reshape(6)
    t4 = np.array([v4, w4]).reshape(6)
    t5 = np.array([v5, w5]).reshape(6)
    t6 = np.array([v6, w6]).reshape(6)
    t7 = np.array([v7, w7]).reshape(6)
    
    xi = np.array([t1, t2, t3, t4, t5, t6, t7]).T
    h = prod_exp(xi, theta)
    
    return np.dot(h, q)
    

def skew_3d(omega):
    """
    Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.
    
    Args:
    omega - (3,) ndarray: the rotation vector
    
    Returns:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    #YOUR CODE HERE

    return np.array([[0, -omega[2], omega[1]], [omega[2], 0, -omega[0]], [-omega[1], omega[0], 0]])


def rotation_2d(theta):
    """
    Computes a 2D rotation matrix given the angle of rotation.
    
    Args:
    theta: the angle of rotation
    
    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """
    
    #YOUR CODE HERE
    rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    return rot

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

def hat_2d(xi):
    """
    Converts a 2D twist to its corresponding 3x3 matrix representation
    
    Args:
    xi - (3,) ndarray: the 2D twist
    
    Returns:
    xi_hat - (3,3) ndarray: the resulting 3x3 matrix
    """
    if not xi.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    

    #YOUR CODE HERE
    x_vel = xi[0]
    y_vel = xi[1]
    w = xi[2]
    xi_hat = np.array([[0,-w,x_vel],[w,0,y_vel],[0,0,0]])

    return xi_hat

def hat_3d(xi):
    """
    Converts a 3D twist to its corresponding 4x4 matrix representation
    
    Args:
    xi - (6,) ndarray: the 3D twist
    
    Returns:
    xi_hat - (4,4) ndarray: the corresponding 4x4 matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')
    

    #YOUR CODE HERE
    x_vel = xi[0]
    y_vel = xi[1]
    z_vel = xi[2]
    x_w = xi[3]
    y_w = xi[4]
    z_w = xi[5]
    omega_hat = skew_3d(np.transpose([x_w,y_w,z_w]))
    xi_hat = np.zeros((4,4))
    xi_hat[:3,:3] = omega_hat
    xi_hat[0,3] = x_vel
    xi_hat[1,3] = y_vel
    xi_hat[2,3] = z_vel
    xi_hat[3,3] = 0

    return xi_hat

def homog_2d(xi, theta):
    """
    Computes a 3x3 homogeneous transformation matrix given a 2D twist and a 
    joint displacement
    
    Args:
    xi - (3,) ndarray: the 2D twist
    theta: the joint displacement
    
    Returns:
    g - (3,3) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (3,):
        raise TypeError('xi must be a 3-vector')

    g = np.zeros((3,3))
    g[2,2] = 1
    vels = xi[:2]
    omega = xi[2]
    rot = rotation_2d(omega*theta)
    
    g[:2,:2] = rot
    a = omega*theta
    first = np.array([[1-np.cos(a), np.sin(a)], [-np.sin(a), 1 - np.cos(a)]])
    second = np.array([[0, -1], [1, 0]])
    third = np.array([[vels[0]/omega], [vels[1]/omega]])
    upper_right = np.dot(np.dot(first, second),  third)
    g[0,2] = upper_right[0]
    g[1,2] = upper_right[1]

    return g
    

    #YOUR CODE HERE
    

    return g

def homog_3d(xi, theta):
    """
    Computes a 4x4 homogeneous transformation matrix given a 3D twist and a 
    joint displacement.
    
    Args:
    xi - (6,) ndarray: the 3D twist
    theta: the joint displacement

    Returns:
    g - (4,4) ndarary: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    #YOUR CODE HERE
    g = np.zeros((4,4))
    g[3,3] = 1
    omega = xi[3:]
    omega2 = np.zeros((3,1))
    omega2[0] = xi[3]
    omega2[1] = xi[4]
    omega2[2] = xi[5]
    rot = rotation_3d(omega, theta)
    vels = xi[:3]
    g[:3,:3] = rot
    first = np.eye(3)-rot
    second = np.dot(skew_3d(omega), vels)
    third = theta*np.dot(np.dot(omega2, np.transpose(omega2)),vels)
    x = np.dot(omega2, np.transpose(omega2))
    trans = np.transpose(omega)

    fourth = np.dot(first, second)
    upper_right = (third + fourth) / (np.linalg.norm(omega)**2)
    
    g[0,3] = upper_right[0]
    g[1,3] = upper_right[1]
    g[2,3] = upper_right[2]

    return g

def prod_exp(xi, theta):
    """
    Computes the product of exponentials for a kinematic chain, given 
    the twists and displacements for each joint.
    
    Args:
    xi - (6,N) ndarray: the twists for each joint
    theta - (N,) ndarray: the displacement of each joint
    
    Returns:
    g - (4,4) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape[0] == 6:
        raise TypeError('xi must be a 6xN')

    #YOUR CODE HERE
    g = np.eye(4)
    xi = np.transpose(xi)
    for i in range(xi.shape[0]):
        g = np.dot(g, homog_3d(xi[i], theta[i]))

    return g

#-----------------------------Testing code--------------------------------------
#-------------(you shouldn't need to modify anything below here)----------------

def array_func_test(func_name, args, ret_desired):
    ret_value = func_name(*args)
    if not isinstance(ret_value, np.ndarray):
        print('[FAIL] ' + func_name.__name__ + '() returned something other than a NumPy ndarray')
    elif ret_value.shape != ret_desired.shape:
        print('[FAIL] ' + func_name.__name__ + '() returned an ndarray with incorrect dimensions')
    elif not np.allclose(ret_value, ret_desired, rtol=1e-3):
        print('[FAIL] ' + func_name.__name__ + '() returned an incorrect value')
    else:
        print('[PASS] ' + func_name.__name__ + '() returned the correct value!')

if __name__ == "__main__":
    print('Testing...')

    #Test skew_3d()
    arg1 = np.array([1.0, 2, 3])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -3.,  2.],
                            [ 3., -0., -1.],
                            [-2.,  1.,  0.]])
    array_func_test(skew_3d, func_args, ret_desired)

    #Test rotation_2d()
    arg1 = 2.658
    func_args = (arg1,)
    ret_desired = np.array([[-0.8853, -0.465 ],
                            [ 0.465 , -0.8853]])
    array_func_test(rotation_2d, func_args, ret_desired)

    #Test rotation_3d()
    arg1 = np.array([2.0, 1, 3])
    arg2 = 0.587
    func_args = (arg1,arg2)
    ret_desired = np.array([[-0.1325, -0.4234,  0.8962],
                            [ 0.8765, -0.4723, -0.0935],
                            [ 0.4629,  0.7731,  0.4337]])
    array_func_test(rotation_3d, func_args, ret_desired)

    #Test hat_2d()
    arg1 = np.array([2.0, 1, 3])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -3.,  2.],
                            [ 3.,  0.,  1.],
                            [ 0.,  0.,  0.]])
    array_func_test(hat_2d, func_args, ret_desired)

    #Test hat_3d()
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -2.,  4.,  2.],
                            [ 2., -0., -5.,  1.],
                            [-4.,  5.,  0.,  3.],
                            [ 0.,  0.,  0.,  0.]])
    array_func_test(hat_3d, func_args, ret_desired)

    #Test homog_2d()
    arg1 = np.array([2.0, 1, 3])
    arg2 = 0.658
    func_args = (arg1,arg2)
    ret_desired = np.array([[-0.3924, -0.9198,  0.1491],
                            [ 0.9198, -0.3924,  1.2348],
                            [ 0.    ,  0.    ,  1.    ]])
    array_func_test(homog_2d, func_args, ret_desired)

    #Test homog_3d()
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    arg2 = 0.658
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4249,  0.8601, -0.2824,  1.7814],
                            [ 0.2901,  0.1661,  0.9425,  0.9643],
                            [ 0.8575, -0.4824, -0.179 ,  0.1978],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(homog_3d, func_args, ret_desired)

    #Test prod_exp()
    arg1 = np.array([[2.0, 1, 3, 5, 4, 6], [5, 3, 1, 1, 3, 2], [1, 3, 4, 5, 2, 4]]).T
    arg2 = np.array([0.658, 0.234, 1.345])
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4392,  0.4998,  0.7466,  7.6936],
                            [ 0.6599, -0.7434,  0.1095,  2.8849],
                            [ 0.6097,  0.4446, -0.6562,  3.3598],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(prod_exp, func_args, ret_desired)

    #hw3
    arg1 = np.array([[0, 0, 0, 0, 0, 1], [0, -5, 0, -1, 0, 0], [0, -5, 4, -1, 0, 0], [7, 0, 0, 0, 0, 1], [0, -5, 7, -1, 0, 0], [-5, 0, 0, 0, 1, 0]]).T
    theta = np.array([0.785, -1.181, 2.301, -1.16, -.69, .97])
    res = prod_exp(arg1, theta)
    print res
    #start = np.array([[1, 0, 0, 0],
                            #[0, 1, 0, 7],
                            
                            #[0, 0, 1, 5],
                            #[0, 0, 0, 1]])


    #print np.dot(res, start)

    print('Done!')
