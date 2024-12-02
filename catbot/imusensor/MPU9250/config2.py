from easydict import EasyDict as edict
import numpy as np



def getConfigVals():

	cfg2 = edict()

	# declaring register values. Don't change them as long as you are damn sure.

	cfg2.Address = 0x68
	cfg2.AccelOut = 0x3B # getting the accelerometer data.
	cfg2.ExtSensData00 = 0x49
	cfg2.AccelConfig = 0x1C
	cfg2.AccelRangeSelect2G = 0x00
	cfg2.AccelRangeSelect4G = 0x08
	cfg2.AccelRangeSelect8G = 0x10
	cfg2.AccelRangeSelect16G = 0x18

	cfg2.GyroConfig = 0x1B
	cfg2.GyroRangeSelect250DPS = 0x00
	cfg2.GyroRangeSelect500DPS = 0x08
	cfg2.GyroRangeSelect1000DPS = 0x10
	cfg2.GyroRangeSelect2000DPS = 0x18

	cfg2.AccelConfig2 = 0x1D
	cfg2.AccelLowPassFilter184 = 0x01
	cfg2.AccelLowPassFilter92 = 0x02
	cfg2.AccelLowPassFilter41 = 0x03
	cfg2.AccelLowPassFilter20 = 0x04
	cfg2.AccelLowPassFilter10 = 0x05
	cfg2.AccelLowPassFilter5 = 0x06

	cfg2.GyroConfig2 = 0x1A
	cfg2.GyroLowPassFilter184 = 0x01
	cfg2.GyroLowPassFilter92 = 0x02
	cfg2.GyroLowPassFilter41 = 0x03
	cfg2.GyroLowPassFilter20 = 0x04
	cfg2.GyroLowPassFilter10 = 0x05
	cfg2.GyroLowPassFilter5 = 0x06

	cfg2.SMPDivider = 0x19
	cfg2.InitPinConfig = 0x37
	cfg2.InitEnable = 0x38
	cfg2.InitDisable = 0x00
	cfg2.InitPulse50us = 0x00
	cfg2.InitWakeOnMotionEnable = 0x40
	cfg2.InitRawReadyEnable = 0x01

	cfg2.PowerManagement1 = 0x6B
	cfg2.PowerCycle = 0x20
	cfg2.PowerReset = 0x80
	cfg2.ClockPLL = 0x01

	cfg2.PowerManagement2 = 0x6C
	cfg2.SensorEnable = 0x00
	cfg2.DisableGyro = 0x07

	cfg2.UserControl = 0x6A
	cfg2.I2CMasterEnable = 0x20
	cfg2.I2CMasterClock = 0x0D

	cfg2.I2CMasterControl = 0x24
	cfg2.I2CSlave0Address = 0x25
	cfg2.I2CSlave0Register = 0x26
	cfg2.I2CSlave0Do = 0x63
	cfg2.I2CSlave0Control = 0x27
	cfg2.I2CSlave0Enable = 0x80
	cfg2.I2CReadFlad = 0x80

	cfg2.MotionDetectControl = 0x69
	cfg2.AccelIntelEnable = 0x80
	cfg2.AccelIntelMode = 0x40

	cfg2.LowPassAccelODR = 0x1E
	cfg2.LP_ACCEL_ODR_0_24HZ = 0
	cfg2.LP_ACCEL_ODR_0_49HZ = 1
	cfg2.LP_ACCEL_ODR_0_98HZ = 2
	cfg2.LP_ACCEL_ODR_1_95HZ = 3
	cfg2.LP_ACCEL_ODR_3_91HZ = 4
	cfg2.LP_ACCEL_ODR_7_81HZ = 5
	cfg2.LP_ACCEL_ODR_15_63HZ = 6
	cfg2.LP_ACCEL_ODR_31_25HZ = 7
	cfg2.LP_ACCEL_ODR_62_50HZ = 8
	cfg2.LP_ACCEL_ODR_125HZ = 9
	cfg2.LP_ACCEL_ODR_250HZ = 10
	cfg2.LP_ACCEL_ODR_500HZ = 11

	cfg2.WakeOnMotionThreshold = 0x1F
	cfg2.WhoAmI = 0x75

	# Ak8963 registers
	cfg2.Ak8963I2CAddress = 0x0C
	cfg2.Ak8963HXL = 0x03
	cfg2.Ak8963CNTL1 = 0x0A
	cfg2.Ak8963PowerDown = 0x00
	cfg2.Ak8963ContinuosMeasurment1 = 0x12
	cfg2.Ak8963ContinuosMeasurment2 = 0x16
	cfg2.Ak8963FuseROM = 0x0F
	cfg2.Ak8963CNTL2 = 0x0B
	cfg2.Ak8963Reset = 0x01
	cfg2.Ak8963ASA = 0x10
	cfg2.Ak8963WhoAmI = 0x00

	# end of register declaration

	cfg2.transformationMatrix = np.array([[0.0,1.0,0.0],[1.0,0.0,0.0],[0.0,0.0,-1.0]]).astype(np.int16)
	cfg2.I2CRate = 400000
	cfg2.TempScale = 333.87
	cfg2.TempOffset = 21.0
	cfg2.Gravity = 9.807
	cfg2.Degree2Radian = np.pi/180.0

	cfg2.acc_t_matrix_ned = np.array([[0.0,-1.0,0.0],[-1.0,0.0,0.0],[0.0,0.0,1.0]]) #np.array([-1.0,1.0,1.0])
	cfg2.gyro_t_matrix_ned = np.array([[0.0,1.0,0.0],[1.0,0.0,0.0],[0.0,0.0,-1.0]]) #np.array([1.0,-1.0,-1.0])
	cfg2.mag_t_matrix_ned = np.array([[0.0,-1.0,0.0],[1.0,0.0,0.0],[0.0,0.0,1.0]])

	return cfg2



