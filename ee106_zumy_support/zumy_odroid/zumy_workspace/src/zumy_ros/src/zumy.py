# -*- coding: utf-8 -*-
"""
Created on Wed Oct 29 06:39:30 2014

@author: ajc

Purpose: This code safely interfaces with zumy's ports to get sensor readings.

"""

from mbedrpc import *
import threading
import time
from serial import SerialException
import numpy as np #just so I can use the np.mean function.  Nothing strange going on here.

class Motor:
    def __init__(self, a1, a2):
        self.a1=a1
        self.a2=a2

    def cmd(self, speed):
        if speed >=0:
            self.a1.write(speed)
            self.a2.write(0)
        else:
            self.a1.write(0)
            self.a2.write(-speed)

imu_names = ['accel_x','accel_y','accel_z','gyro_x','gyro_y','gyro_z']
enc_pos_names = ['l_enc','r_enc']
enc_vel_names = ['l_spd','r_spd']

class Zumy:
    def __init__(self, dev='/dev/ttyACM0'):
        self.mbed=SerialRPC(dev, 115200)

        self.tracks = RPCFunction(self.mbed, "sm")
        self.mbed_enable = RPCFunction(self.mbed,"enable")

        self.an = AnalogIn(self.mbed, p15)
        # NEW!
        self.IR_ai_top = AnalogIn(self.mbed, p16)
        self.IR_ai_side = AnalogIn(self.mbed, p17)
        self.IR_ai_bottom = AnalogIn(self.mbed, p18)
        self.IR_ai_front = AnalogIn(self.mbed, p19)

        #END NEW
        self.imu_vars = [RPCVariable(self.mbed,name,delete = False) for name in imu_names]
        self.enc_pos_vars = [RPCVariable(self.mbed,name,delete = False) for name in enc_pos_names]
        self.enc_vel_vars = [RPCVariable(self.mbed,name,delete = False) for name in enc_vel_names]
        self.rlock = threading.Lock()

        self.enabled = True #note it's enableD, to avoid namespace collision with the function 'enable'

        self.volts = [8.0 for i in range(0,5)] #5 element long moving average filter, initialized to 8 volts  (it'll quickly change)

        #translate left and right (which are in meters/sec) to encoder_ticks / sec
        #12 ticks/encoder_back_shaft_rotation * (100.37 encoder_back_shaft_rotation / 1 output_shaft_rotation) * 1 output_shaft_rotation / (circumfrence = pi*d = 3.15 * 1.5 inches * .0254 meters/inch) = 12*100.37 / (pi*1.5*0.0254) = 5012.16 ticks/meter        
        self.translation_factor = 5012.16  #ticks/meter

        self.battery_lock = False #a boolean to tell me if my battery ever dipped below the battery threshold.
        self.enable() #tell the zumy that it's enabled.

    def __del__(self):
      self.disable()

    # NEW!
    def read_IR_ai_side(self):
      return self.IR_ai_side.read()

    # NEW!
    def read_IR_ai_front(self):
      return self.IR_ai_front.read()

    # NEW!
    def read_IR_ai_top(self):
      return self.IR_ai_top.read()

    # NEW!
    def read_IR_ai_bottom(self):
      return self.IR_ai_bottom.read()

    def cmd(self, left, right):
        self.rlock.acquire()
        # As of Rev. F, positive command is sent to both left and right
        try:
          if self.enabled: #don't do anything if i'm disabled
            self.tracks.run(str(self.translation_factor*left) + " " + str(self.translation_factor*right))
            pass
        except SerialException:
          pass
        self.rlock.release()

    def read_voltage(self):
        self.rlock.acquire()
        try:
          ain=self.an.read()*3.3
        except SerialException:
          pass
        self.rlock.release()
        volt=ain*(100+200) / (100)
        self.volts.insert(0,volt)  #add this data to the moving average filter
        self.volts.pop() #remove the last element
        return volt

    def enc_pos(self): #return in units of meters.
      self.rlock.acquire()
      try:
        rval = [float(int(var.read())/self.translation_factor) for var in self.enc_pos_vars]
      except SerialException:
        pass
      self.rlock.release()
      return rval

    def enc_vel(self): #return speed in meters/sec
      self.rlock.acquire()
      try:
        rval = [float(float(var.read())/self.translation_factor) for var in self.enc_vel_vars]
      except SerialException:
        pass
      self.rlock.release()
      return rval

    def read_imu(self):
      self.rlock.acquire()
      try:
        rval = [float(var.read()) for var in self.imu_vars]
      except SerialException:
        pass
      self.rlock.release()
      return rval

    def enable(self):
      #enable the zumy, but only if my battery hasn't been unhappy.
      self.rlock.acquire()
      if not self.battery_lock:
        self.enabled = True
        self.mbed_enable.run(str(1))
      self.rlock.release()

    def disable(self):
      self.rlock.acquire()

      #first, disable the zumy.
      self.mbed_enable.run(str(0))
      #second, disable the zumy flag.
      self.enabled = False
      self.rlock.release()

    def battery_protection(self):
      #a function that needs to be called with some regularity
      #disables the robot if the battery voltage is very low
      if np.mean(self.volts) < 6.6: #6.6 volts--> 3.3V per cell.
        #averages over the last 5 battery measurements, because loading effects are :(
        self.disable()
        self.battery_lock = True
    
    def battery_unsafe(self):
      #return true if i think my battery is unsafe.
      return self.battery_lock

if __name__ == '__main__':
    z=Zumy()
    z.cmd(0.05,0.05)
    time.sleep(3)
    z.cmd(0,0) 
