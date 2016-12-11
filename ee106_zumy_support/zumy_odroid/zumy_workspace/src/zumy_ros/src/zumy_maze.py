#!/usr/bin/env python
import rospy
import sys
import math
import numpy as np
import time
from std_msgs.msg import String,Header,Int32,Float32,Bool
from geometry_msgs.msg import Transform, Vector3, Twist
from sensor_msgs.msg import Imu
import utils

TURN_DELAY = 3.0
UNBLOCK_DELAY = 1.5
SPEED_CONST = 4.0
FWD_SPEED = 0.1
STOP_AND_THINK_DELAY = 2.0

class ZumyMaze:

    def __init__(self):
        rospy.init_node('zumy_maze')
        if len(sys.argv) < 2:
            print('Use: zumy_maze [ zumy name ]')
            sys.exit()
        self.name = sys.argv[1]
        self.delay_type = None
        self.side_data = None
        self.zumy_vel = rospy.Publisher('/'+self.name+'/cmd_vel', Twist, queue_size=2)
        self.calibrate_pub = rospy.Publisher('/'+self.name+'/calibrate', Float32, queue_size=1)
        self.zumy_direction = None
        self.is_turning = False
        self.turn_start_direction = None
        self.right_blocked = True
        self.front_blocked = False
        self.just_turned_right = False
        self.right_just_unblocked = False
        self.received_right_data = False
        self.done_turning_time = time.time()
        self.unblocked_time = time.time()

        self.stop_and_think_time = time.time()
        self
        # self.max_turn_degree = 76.0
        self.max_turn_degree = float(sys.argv[2])
        self.BLOCKED_THRESHOLD_FRONT = float(sys.argv[3])
        self.BLOCKED_THRESHOLD_SIDE = float(sys.argv[4])

        # self.BLOCKED_THRESHOLD_FRONT = 1.9
        # self.BLOCKED_THRESHOLD_SIDE = 1.5

        # self.done_calibration = False
        self.directions = {"F" : 0., "L" : 90., "R" : -90., "B" : 180.} # (CCW, 0 is N) the direction that the the zumy will face
        self.stop_zumy()

    def get_zumy_direction(self, message):
        # print('psi', message.data)
        self.zumy_direction = message.data

    def make_decision(self, is_right_blocked, is_front_blocked, just_unblocked):
        if self.zumy_direction is None:
            return
        if is_front_blocked and self.delay_type != 'stop_and_think' and self.delay_type != "done thinking":
            if self.delay_type is not None:
                print 'front blocked, not going to delay'
            self.delay_type = None
        if self.delay_type is not None and self.delay_type != "done thinking":
            if self.delay_type == 'turn':
                print 'turning...'
                if time.time() - self.done_turning_time <= TURN_DELAY:
                    self.move_zumy_forward()
                else:
                    print 'set turning to none bc finished naturally'
                    self.delay_type = None
            if self.delay_type == 'unblock':
                print 'unblock...'
                if time.time() - self.unblocked_time <= UNBLOCK_DELAY:
                    self.move_zumy_forward()
                else:
                    print 'set unblock to none bc finished naturally'
                    self.delay_type = None
            if self.delay_type == 'stop_and_think':
                if time.time() - self.stop_and_think_time <= STOP_AND_THINK_DELAY:
                    self.stop_zumy()
                else:
                    # make decision now -- make_decision is now called again with this stop and think flag false
                    print 'stopped and turn complete'
                    self.delay_type = 'done thinking'

        elif self.is_turning:
            # print "checking turn completion"
            self.check_turn_completion()
        else:
            if is_front_blocked and is_right_blocked:
                print 'fbrb'
                self.turn_zumy(self.directions["L"])
                print 1, self.side_data
                self.just_turned_right = False
            elif is_front_blocked and not is_right_blocked:
                print 'fbru'
                self.turn_zumy(self.directions["R"])
                print 2, self.side_data
                self.just_turned_right = True
            elif not is_front_blocked and is_right_blocked:
                print 'furb'
                self.move_zumy_forward()
                self.just_turned_right = False
            elif not is_front_blocked and not is_right_blocked:
                print 'furu'
                if self.just_turned_right:
                    #if we just turned, move forward before making a decision to turn right
                    self.just_turned_right = False
                    self.delay_type = 'turn'
                    print 'starting to delay: ' + self.delay_type  
                elif just_unblocked:
                    self.delay_type = 'unblock'
                    print 'starting to delay: ' + self.delay_type  
                    #if we just got unblocked, then move forward before turning right
                else:
                    print 'turning right, both unblocked'
                    self.turn_zumy(self.directions["R"])
                    print 3, self.side_data
                    self.just_turned_right = True
                    self.last_turned_right = time.time()
            else:
                print "BIG ERROR"
        self.received_right_data = False

    def front_data_received(self, message):
        data = message.data
        if self.received_right_data:
            if data > self.BLOCKED_THRESHOLD_FRONT:
                self.front_blocked = True
            else:
                self.front_blocked = False
            self.make_decision(self.right_blocked, self.front_blocked, self.right_just_unblocked)

    def side_data_received(self, message):
        data = message.data
        self.side_data = data
        if data > self.BLOCKED_THRESHOLD_SIDE:
            self.right_blocked = True
            self.right_just_unblocked = False
        else:
            if self.right_blocked:
                self.right_just_unblocked = True
                self.unblocked_time = time.time()
            else:
                self.right_just_unblocked = False
            self.right_blocked = False
        self.received_right_data = True

    def check_turn_completion(self):
        if abs(self.zumy_direction - self.turn_start_direction) >= utils.deg2rad(self.max_turn_degree):
            # print 'zumy dirn', self.zumy_direction
            difference = self.zumy_direction - self.turn_start_direction
            if difference >= 0:
                # print('pos dir')
                linear = Vector3(0, 0, 0)
                angular = Vector3(0, 0, -1.35 * SPEED_CONST)
                opp_twist = Twist(linear, angular)
                self.zumy_vel.publish(opp_twist)
            elif difference < 0:
                # print('neg dir')
                linear = Vector3(0, 0, 0)
                angular = Vector3(0, 0, 1.35 * SPEED_CONST)
                opp_twist = Twist(linear, angular)
                self.zumy_vel.publish(opp_twist)
            self.stop_zumy()
            self.done_turning_time = time.time()

    def stop_zumy(self):
	   if self.is_turning or self.delay_type == "stop_and_think":
            linear = Vector3(0, 0, 0)
            angular = Vector3(0, 0, 0)
            stop_twist = Twist(linear, angular)
            self.zumy_vel.publish(stop_twist)
            self.is_turning = False
	    self.zumy_direction = None
	   #self.calibrate_pub.publish(1.0) # tells kalman to recalibrate

    def move_zumy_forward(self):
        # returns a Twist that is published and moves Zumy forward 0.1 units
        # print "forward dir: ", self.zumy_direction
        linear = Vector3(FWD_SPEED, 0, 0)
        angular = Vector3(0, 0, 0)
        twist = Twist(linear, angular)
        self.zumy_vel.publish(twist)

    def turn_zumy(self, angle):
        # angle in degrees
        # returns a Twist that is published and turns zumy
        if self.delay_type is None and not self.is_turning:
            print 'delay type none in turn_zumy is none'
            self.stop_and_think_time = time.time()
            self.delay_type = "stop_and_think"
        elif self.delay_type == 'done thinking':
            self.delay_type = None
            self.is_turning = True
            omega = np.array([0, 0, 1]) # rotate about z
            translation = np.array([0, 0, 0])
            theta = utils.deg2rad(angle)
            omega = omega * theta
            rbt = utils.create_rbt(omega, theta, translation)
            v = utils.find_v(omega, theta, translation)
            linear = Vector3(0.003 * SPEED_CONST, v[1]/3, SPEED_CONST * v[2]/3)
            angular = Vector3(omega[0]/3, omega[1]/3, SPEED_CONST * omega[2]/12)

            twist = Twist(linear, angular)
            print 'STARTED TURNING'
            self.turn_start_direction = self.zumy_direction
            self.zumy_vel.publish(twist)

    def run(self):
        rospy.Subscriber("/"+self.name+"/IR_ai_side", Float32, self.side_data_received)
        rospy.Subscriber("/"+self.name+"/IR_ai_front", Float32, self.front_data_received)
        rospy.Subscriber("/"+self.name+"/psi", Float32, self.get_zumy_direction)
        # rospy.Subscriber("/"+self.name+"/done_calib", Bool, self.done_calibration)
        rospy.spin()
  
if __name__=='__main__':
    node = ZumyMaze()
    node.run()
