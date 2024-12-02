#142 Hz


import os
import sys
import time
import smbus
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from imusensor.MPU9250 import MPU9250
from imusensor.MPU9250 import MPU9250_2


address = 0x68
#address2 = 0x69

bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)


 
imu.begin()


imu.loadCalibDataFromFile("/home/catbot/imusensor/acc_calib.json")
print ("calib loaded properly")
 
print('<Begin>')

imu.readSensor()
imu.computeOrientation()


data_pitch1 = imu.pitch

dt = time.time()

while True:
    imu.readSensor()
    imu.computeOrientation()
	
    data_pitch1_2 = imu.pitch
    data2 = imu.yaw
    data3 = imu.roll
    
    mag_acc = math.sqrt((imu.AccelVals[0])**2 + (imu.AccelVals[1])**2+(imu.AccelVals[2])**2)

    dt = time.time()-dt
    
    w1_roll = (data_pitch1_2 - data_pitch1)/dt

    #print(f"1 w_roll = {w1_roll:.2f}, 2 w_roll = {w2_roll:.2f}, dt = {dt}")
    #print(f"{dt:.5f}")
    print(f"1 pitch = {data_pitch1_2:.2f}, mag_acc = {mag_acc:.2f}, w_roll = {w1_roll:.2f} dt = {dt:.2f}, freq = {1/dt:.2f}")
    
    data_pitch1 = data_pitch1_2
    dt = time.time()
    time.sleep(0.001)
    

    
    
    
    
    
