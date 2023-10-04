import os
import utils
import keyboard
from time import time

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

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MOVING_SPEED          = 32
ADDR_TORQUE_LIMIT          = 34

# Data Byte Length
LEN_MX_GOAL_POSITION       = 4
LEN_MX_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
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

BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM3'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

MOTOR_SPEED = 300

portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

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


# Enable Dynamixels Torque & speed limit
for i in [1,2,3,4]:
    for j in [0,1,2]:
        id = 10*i+j

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL10, ADDR_MX_GOAL_POSITION, 512)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL20, ADDR_MX_GOAL_POSITION, 512)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL30, ADDR_MX_GOAL_POSITION, 512)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL40, ADDR_MX_GOAL_POSITION, 512)
h=15

def moving(h):
    hip, knee = utils.calc(h)

    hip1  = int(512 - (90-hip)/300*1024)
    knee1 = int(512 + (90+knee)/300*1024)

    hip2  = int(512 + (90-hip)/300*1024)
    knee2 = int(512 - (90+knee)/300*1024)

    param_hip_position1 = [DXL_LOBYTE(DXL_LOWORD(hip1)), 
                           DXL_HIBYTE(DXL_LOWORD(hip1)), 
                           DXL_LOBYTE(DXL_HIWORD(hip1)), 
                           DXL_HIBYTE(DXL_HIWORD(hip1))]
    
    param_knee_position1 = [DXL_LOBYTE(DXL_LOWORD(knee1)), 
                            DXL_HIBYTE(DXL_LOWORD(knee1)), 
                            DXL_LOBYTE(DXL_HIWORD(knee1)), 
                            DXL_HIBYTE(DXL_HIWORD(knee1))]
    
    param_hip_position2 = [DXL_LOBYTE(DXL_LOWORD(hip2)), 
                           DXL_HIBYTE(DXL_LOWORD(hip2)), 
                           DXL_LOBYTE(DXL_HIWORD(hip2)), 
                           DXL_HIBYTE(DXL_HIWORD(hip2))]
    
    param_knee_position2 = [DXL_LOBYTE(DXL_LOWORD(knee2)), 
                            DXL_HIBYTE(DXL_LOWORD(knee2)), 
                            DXL_LOBYTE(DXL_HIWORD(knee2)), 
                            DXL_HIBYTE(DXL_HIWORD(knee2))]

    dxl_addparam_result = groupSyncWrite.addParam(DXL11, param_hip_position1)
    dxl_addparam_result = groupSyncWrite.addParam(DXL12, param_knee_position1)
    dxl_addparam_result = groupSyncWrite.addParam(DXL21, param_hip_position2)
    dxl_addparam_result = groupSyncWrite.addParam(DXL22, param_knee_position2)
    dxl_addparam_result = groupSyncWrite.addParam(DXL31, param_hip_position2)
    dxl_addparam_result = groupSyncWrite.addParam(DXL32, param_knee_position2)
    dxl_addparam_result = groupSyncWrite.addParam(DXL41, param_hip_position1)
    dxl_addparam_result = groupSyncWrite.addParam(DXL42, param_knee_position1)

    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    groupSyncWrite.clearParam()

rate = 0.3
while 1:
    if keyboard.is_pressed('up'):    
        if h>20:
            print("limited")
        else:
            h=h+rate
            moving(h)
    elif keyboard.is_pressed('down'):
        if h<6:
            print("limited")
        else:
            h=h-rate
            moving(h)
    
    time.sleep(0.05)

    

    

# Disable Dynamixels Torque
for i in [1,2,3,4]:
    for j in [0,1,2]:
        id = 10*i+j
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        

# Close port
portHandler.closePort()