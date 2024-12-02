#142 Hz


import os
import sys
import time
import smbus
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from imusensor.MPU9250 import MPU9250


address = 0x68


bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)

imu.begin()

print('<Begin>')

imu.readSensor()
imu.computeOrientation()


data_pitch1 = imu.pitch

dt = time.time()

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import * 

# Control table address
ADDR_AX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_AX_GOAL_POSITION      = 30
ADDR_AX_PRESENT_POSITION   = 36

ADDR_XM_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_XM_GOAL_POSITION      = 116
ADDR_XM_PRESENT_POSITION   = 132
ADDR_XM_MOVING   = 122
# ADDR_MOVING_SPEED       = 32
# ADDR_TORQUE_LIMIT       = 34

# Data Byte Length
LEN_GOAL_POSITION       = 4
LEN_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 1.0     

FB = 0
RB = 1
MB = 2

XM_ID = [FB, RB]
AX_ID = [MB]

BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20 

portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_XM_GOAL_POSITION, LEN_GOAL_POSITION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

for id in XM_ID:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_XM_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

for id in AX_ID:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

packetHandler.write4ByteTxRx(portHandler, FB, ADDR_XM_GOAL_POSITION, 0)
packetHandler.write4ByteTxRx(portHandler, RB, ADDR_XM_GOAL_POSITION, 4095)

theta = -0
theta = int(theta*1024/300+512)

packetHandler.write4ByteTxRx(portHandler, MB, ADDR_AX_GOAL_POSITION, theta)

angle = 0
front = int(angle*4096/360)
rear = 4095-front
print('angle = ', angle)
param_front_position = [DXL_LOBYTE(DXL_LOWORD(front)), 
                        DXL_HIBYTE(DXL_LOWORD(front)), 
                        DXL_LOBYTE(DXL_HIWORD(front)), 
                        DXL_HIBYTE(DXL_HIWORD(front))]
    
param_rear_position = [DXL_LOBYTE(DXL_LOWORD(rear)), 
                       DXL_HIBYTE(DXL_LOWORD(rear)), 
                       DXL_LOBYTE(DXL_HIWORD(rear)), 
                       DXL_HIBYTE(DXL_HIWORD(rear))]

groupSyncWrite.addParam(FB, param_front_position)
groupSyncWrite.addParam(RB, param_rear_position)
def check_postioin():
    position = packetHandler.read4ByteTxRx(portHandler, FB, ADDR_XM_PRESENT_POSITION)
    print(position[0]*360/4096)
sw = 0
while True:
    imu.readSensor()
    imu.computeOrientation()
	
    data_pitch2 = imu.pitch


    dt = time.time()-dt
    
    w1_roll = (data_pitch2 - data_pitch1)/dt
    mag_acc = math.sqrt((imu.AccelVals[0])**2 + (imu.AccelVals[1])**2+(imu.AccelVals[2])**2)
    # print(mag_acc)
    mov = packetHandler.read1ByteTxRx(portHandler, FB, ADDR_XM_MOVING)
    #print(f"1 w_roll = {w1_roll:.2f}, 2 w_roll = {w2_roll:.2f}, dt = {dt}")
    #print(f"{dt:.5f}")
    if (mag_acc < 6 and data_pitch2 > -10 and data_pitch2 < 10):
      #falling
      #print(f"pitch = {data_pitch2:.2f}, Z_acc = {mag_acc:.2f}, w_roll = {w1_roll}, dt = {dt}, freq = {1/dt}")
      print('\033[91m' + 'Falling!' + '\033[0m')
      groupSyncWrite.txPacket()
      sw = 1
    if sw: 
        check_postioin()
    
    if (mag_acc > 20):
      sw = 0
      #landing, body return to 0
      print('\033[92m' + 'Landing!' + '\033[0m')
      packetHandler.write4ByteTxRx(portHandler, FB, ADDR_XM_GOAL_POSITION, 0)
      packetHandler.write4ByteTxRx(portHandler, RB, ADDR_XM_GOAL_POSITION, 4095)
      time.sleep(1)
    
    data_pitch1 = data_pitch2
    dt = time.time()
    time.sleep(0.001)


groupSyncWrite.clearParam()

for id in XM_ID:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_XM_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
for id in AX_ID:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

portHandler.closePort()

    
    
    
    
    
