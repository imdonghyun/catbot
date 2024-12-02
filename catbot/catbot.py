#142 Hz


import os
import sys
import time
import smbus
import math
import numpy as np
import utils

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

# motor ID setting
FB = 0
RB = 1
MB = 2

DXL10                       = 10                 
DXL11                       = 11
DXL12                       = 12

DXL20                       = 20                 
DXL21                       = 21
DXL22                       = 22

DXL30                       = 30                 
DXL31                       = 31
DXL32                       = 32

DXL40                       = 40                 
DXL41                       = 41
DXL42                       = 42

XM_ID = [FB, RB]
AX_ID = [MB, DXL10, DXL11, DXL12, DXL20, DXL21, DXL22, DXL30, DXL31, DXL32, DXL40, DXL41, DXL42]

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
groupSyncWrite_leg = GroupSyncWrite(portHandler, packetHandler, ADDR_AX_GOAL_POSITION, LEN_GOAL_POSITION)

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
class Leg():
    def __init__(self, id0, id1, id2):
        self.id0 = id0 # shoulder
        self.id1 = id1 # hip
        self.id2 = id2 # knee

    def add_param_move_height(self, height):
        hip, knee = utils.calc(height)

        if(self.id0 == DXL10 or self.id0 == DXL40):
            hip  = int(512 - (90-hip)/300*1024) # 1, 4에 1
            knee = int(512 + (90+knee)/300*1024)
        elif(self.id0 == DXL20 or self.id0 == DXL30):
            hip  = int(512 + (90-hip)/300*1024)  # 2, 3은 hip2
            knee = int(512 - (90+knee)/300*1024)

        param_hip_position = [DXL_LOBYTE(DXL_LOWORD(hip)), 
                            DXL_HIBYTE(DXL_LOWORD(hip)), 
                            DXL_LOBYTE(DXL_HIWORD(hip)), 
                            DXL_HIBYTE(DXL_HIWORD(hip))]
    
        param_knee_position = [DXL_LOBYTE(DXL_LOWORD(knee)), 
                                DXL_HIBYTE(DXL_LOWORD(knee)), 
                                DXL_LOBYTE(DXL_HIWORD(knee)), 
                                DXL_HIBYTE(DXL_HIWORD(knee))]

        dxl_addparam_result = groupSyncWrite_leg.addParam(self.id1, param_hip_position)
        dxl_addparam_result = groupSyncWrite_leg.addParam(self.id2, param_knee_position)
    
    def add_param_defalt(self, theta):
        hip, knee = utils.calc(18)
        theta /= 2
        theta += 20

        if(self.id0 == DXL10 or self.id0 == DXL40):
            hip  = int(512 - (90-hip+theta)/300*1024) # 1, 4에 1
            knee = int(512 + (90+knee)/300*1024)

        elif(self.id0 == DXL20 or self.id0 == DXL30):
            hip  = int(512 + (90-hip+theta)/300*1024)  # 2, 3은 hip2
            knee = int(512 - (90+knee)/300*1024)

        param_hip_position = [DXL_LOBYTE(DXL_LOWORD(hip)), 
                            DXL_HIBYTE(DXL_LOWORD(hip)), 
                            DXL_LOBYTE(DXL_HIWORD(hip)), 
                            DXL_HIBYTE(DXL_HIWORD(hip))]
    
        param_knee_position = [DXL_LOBYTE(DXL_LOWORD(knee)), 
                                DXL_HIBYTE(DXL_LOWORD(knee)), 
                                DXL_LOBYTE(DXL_HIWORD(knee)), 
                                DXL_HIBYTE(DXL_HIWORD(knee))]

        dxl_addparam_result = groupSyncWrite_leg.addParam(self.id1, param_hip_position)
        dxl_addparam_result = groupSyncWrite_leg.addParam(self.id2, param_knee_position)

        
        param_default_position = [DXL_LOBYTE(DXL_LOWORD(512)), 
                                  DXL_HIBYTE(DXL_LOWORD(512)), 
                                  DXL_LOBYTE(DXL_HIWORD(512)), 
                                  DXL_HIBYTE(DXL_HIWORD(512))]
    
        dxl_addparam_result = groupSyncWrite_leg.addParam(self.id0, param_default_position)
    
    def add_param_defalt_inner(self, theta):
        hip, knee = utils.calc(18)
        theta /= 2

        if(self.id0 == DXL10 or self.id0 == DXL40):
            hip  = int(0) # 1, 4에 1
            knee = int(512 + (90+knee)/300*1024)
            go = int(256)
        elif(self.id0 == DXL20 or self.id0 == DXL30):
            hip  = int(1023)  # 2, 3은 hip2
            knee = int(512 - (90+knee)/300*1024)
            go = int(768)

        param_hip_position = [DXL_LOBYTE(DXL_LOWORD(hip)), 
                            DXL_HIBYTE(DXL_LOWORD(hip)), 
                            DXL_LOBYTE(DXL_HIWORD(hip)), 
                            DXL_HIBYTE(DXL_HIWORD(hip))]
    
        param_knee_position = [DXL_LOBYTE(DXL_LOWORD(knee)), 
                                DXL_HIBYTE(DXL_LOWORD(knee)), 
                                DXL_LOBYTE(DXL_HIWORD(knee)), 
                                DXL_HIBYTE(DXL_HIWORD(knee))]

        dxl_addparam_result = groupSyncWrite_leg.addParam(self.id1, param_hip_position)
        dxl_addparam_result = groupSyncWrite_leg.addParam(self.id2, param_knee_position)

        
        param_default_position = [DXL_LOBYTE(DXL_LOWORD(go)), 
                                  DXL_HIBYTE(DXL_LOWORD(go)), 
                                  DXL_LOBYTE(DXL_HIWORD(go)), 
                                  DXL_HIBYTE(DXL_HIWORD(go))]
    
        dxl_addparam_result = groupSyncWrite_leg.addParam(self.id0, param_default_position)


def check_postioin():
    position = packetHandler.read4ByteTxRx(portHandler, FB, ADDR_XM_PRESENT_POSITION)
    a = position[0]*360/4096
    print(f"FMotor : {a:.2f}\t", end = "")
    return a

def torque_on():
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

def torque_off():
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


##### Main #####
# torque_off()
# time.sleep(1)
torque_on()
Left_Front_Leg = Leg(DXL10, DXL11, DXL12)
Left_Back_Leg = Leg(DXL30, DXL31, DXL32)
Right_Front_Leg = Leg(DXL20, DXL21, DXL22)
Right_Back_Leg = Leg(DXL40, DXL41, DXL42)
# Initial position of FB, RB
packetHandler.write4ByteTxRx(portHandler, FB, ADDR_XM_GOAL_POSITION, 0)
packetHandler.write4ByteTxRx(portHandler, RB, ADDR_XM_GOAL_POSITION, 4095)

# Back bone angle (lower back)
theta = -90
bbbv = int(theta*1024/300+512)

packetHandler.write4ByteTxRx(portHandler, MB, ADDR_AX_GOAL_POSITION, bbbv)
default_height = 15
# Left_Front_Leg.add_param_defalt(-theta)
# Left_Back_Leg.add_param_defalt(-theta)
# Right_Front_Leg.add_param_defalt(-theta)
# Right_Back_Leg.add_param_defalt(-theta)

Left_Front_Leg.add_param_defalt_inner(-theta)
Left_Back_Leg.add_param_defalt_inner(-theta)
Right_Front_Leg.add_param_defalt_inner(-theta)
Right_Back_Leg.add_param_defalt_inner(-theta)

groupSyncWrite_leg.txPacket()
groupSyncWrite_leg.clearParam()


# Rotation motor; 
angle = 330                     # When cat falls, FB, RB rotates this angle
front = int(angle*4096/360)
rear = 4095-front
print(f"back = {-theta}, angle = {angle}")
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


sw = 0
back_bone = 0
    
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
        a = check_postioin()
        print(f"Pitch = {data_pitch2:.2f}\tAnglur valocity = {w1_roll:.2f}")
        
        # if a > 260 and back_bone == 0:
        #     back_bone = 1
        #     packetHandler.write4ByteTxRx(portHandler, MB, ADDR_AX_GOAL_POSITION, 512)
        #     print('back bone 180')
    
    if (mag_acc > 20):
      sw = 0
      back_bone = 0
      #landing, body return to 0
      print('\033[92m' + 'Landing!' + '\033[0m')
      packetHandler.write4ByteTxRx(portHandler, FB, ADDR_XM_GOAL_POSITION, 0)
      packetHandler.write4ByteTxRx(portHandler, RB, ADDR_XM_GOAL_POSITION, 4095)
      packetHandler.write4ByteTxRx(portHandler, MB, ADDR_AX_GOAL_POSITION, bbbv)
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

    
    
    
    
    
