---
sidebar_position: 12
title: ROS读取GY-85数据
---

我们公司目前所用的陀螺仪是GY-85，与普通的mpu6050存在很大的差别，用市面上读取mpu6050的代码，是没有办法获取到正确的数据的，因此需要自己进行程序的改写，目前我已经写好了一个能用的c++节点，安装方法如下
## 安装
```shell
git clone https://gitee.com/ceoifung/mpu_gy85.git
catkin_make --pkg mpu_gy85
```

## 运行程序
- 直接运行节点
```shell
rosrun mpu_gy85 mpu_gy85_node
```

:::tip
程序默认打开的是i2c-1的设备
:::

- launch文件运行
```xml
<launch>
	<node pkg="mpu_gy85" name="mpu_gy85_node" type="mpu_gy85_node" output="screen">
  <!-- 设置i2c设备, bus代表总线的ID，默认是1，可以通过修改这里设置总线 -->
		<param name="bus" value="1"/>
	</node>
</launch>

```

## 关于寄存器的问题
- 加速度计的寄存器地址
```cpp
#define ACCEL_ADDR 0x53   // i2c设备地址
#define ACCL_PWR 0x2d
#define ACCL_DATA_FORMAT 0x31
#define ACCEL_XOUT_H 0x32 // High byte of X-axis acceleration data
#define ACCEL_XOUT_L 0x33 // Low byte of X-axis acceleration data
#define ACCEL_YOUT_H 0x34 // High byte of Y-axis acceleration data
#define ACCEL_YOUT_L 0x35 // Low byte of Y-axis acceleration data
#define ACCEL_ZOUT_H 0x36 // High byte of Z-axis acceleration data
#define ACCEL_ZOUT_L 0x37 // Low byte of Z-axis acceleration data

int Gy85::initAccel(int bus)
{
    adxlBus = ceoifung::I2cPort(bus, ACCEL_ADDR);
    adxlBus.setBusAddress(bus);
    adxlBus.setDeviceAddress(ACCEL_ADDR);
    int ret = adxlBus.openConnection();
    if (ret != 0)
        return -1;
    adxlBus.writeByte(ACCL_PWR, 0);
    adxlBus.writeByte(ACCL_PWR, 8);
    adxlBus.writeByte(ACCL_DATA_FORMAT, 0);
    adxlBus.writeByte(ACCL_DATA_FORMAT, 11);
    return 0;
}

```

:::tip
数据处理
```cpp
ret = adxlBus.readByteBuffer(ACCEL_XOUT_H, read_buf, 6);
data[0] = (short)((read_buf[1] << 8) | read_buf[0]);
data[1] = (short)((read_buf[3] << 8) | read_buf[2]);
data[2] = (short)((read_buf[5] << 8) | read_buf[4]);
```
:::

- 陀螺仪的寄存器地址
```cpp
// 定义陀螺仪
#define GY85_ADDR 0x68   // i2c地址
#define GY85_REG_XL 0x1d // 陀螺仪x轴低字节寄存器
#define GY85_REG_XH 0x1e // 陀螺仪x轴高字节寄存器
#define GY85_REG_YL 0x1f // 陀螺仪y轴低字节寄存器
#define GY85_REG_YH 0x20 // 陀螺仪y轴高字节寄存器
#define GY85_REG_ZL 0x21 // 陀螺仪z轴低字节寄存器
#define GY85_REG_ZH 0x22 // 陀螺仪z轴高字节寄存器
#define PWR_MGMT_1 0x3E  // 电源管理

// 以下是伪代码
// 初始化陀螺仪
int Gy85::initItg3205(int bus)
{
    itgBus = ceoifung::I2cPort(bus, ITG_3205_ADDR);
    itgBus.setBusAddress(bus);
    itgBus.setDeviceAddress(ITG_3205_ADDR);
    int ret = itgBus.openConnection();
    if (ret != 0)
    {
        return -1;
    }
    itgBus.writeByte(PWR_MGMT_1, 0);
    // 0x15
    itgBus.writeByte(ITG_3205_SMPLRT_DIV, 0x07 | 0x00);
    itgBus.writeByte(ITG_3205_DLPF_FS, ITG_3205_FS_SEL_2000_DEG_SEC | 0x01);
    itgBus.writeByte(ITG_3205_INT_CFG, 0x00);
    itgBus.writeByte(0x17, 0x20 | 0x04 | 0x01);
    return 0;
}
```

:::tip
处理获取到的陀螺仪数据的时候，需要注意数据的高位和低位的问题
```cpp
// 陀螺仪数据获取
ret = itgBus.readByteBuffer(ITG_3205_REG_XL, read_buf, 6);
data[0] = (short)((read_buf[0] << 8) | read_buf[1]);
data[1] = (short)((read_buf[2] << 8) | read_buf[3]);
data[2] = (short)((read_buf[4] << 8) | read_buf[5]);
if (data[0] & (1 << 16 - 1))
	data[0] = data[0] - (1 << 16);
if (data[1] & (1 << 16 - 1))
	data[1] = data[1] - (1 << 16);
if (data[2] & (1 << 16 - 1))
	data[2] = data[2] - (1 << 16);
```
:::

## python测试代码
以下代码可能存在问题，因此在实际运用的时候，需要酌情修改一下
```python
'''
    Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
	http://www.electronicwings.com
'''
import smbus			#import SMBus module of I2C
from time import sleep          #import
import math

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x32
ACCEL_YOUT_H = 0x34
ACCEL_ZOUT_H = 0x36
GYRO_XOUT_H  = 0x1d
GYRO_YOUT_H  = 0x1f
GYRO_ZOUT_H  = 0x21

ACCEL_PWR_CTRL = 0x27
DataFormat = 0x31
DF_Range_16g = 0x03
TapDuration = 0x21
TapLatency = 0x22
TapWindow = 0x23
ActivityThreshold = 0x24
InactivityThreshold = 0x25
InactivityTime = 0x26
AxesEnable = 0x27 # Axis enable control fro activity and inactivity detection
FreeFallThreshold = 0x28
FreeFallTime = 0x29
TapAxes = 0x2A # Axis control for single tap/double tap
TapAxesStatus = 0x2B # Source of tap
BandwidthRate = 0x2C # Data rate and power mode control
PowerControl = 0x2D
InterruptEnable = 0x2E
InterruptMapping = 0x2F
InterruptSource = 0x30
TapThreshold = 0x1D
# Settings for Activity/Inactivity Control
AE_ActivityAC = 0x80
AE_ActivityX = 0x40
AE_ActivityY = 0x20
AE_ActivityZ = 0x10
AE_InactivityAC = 0x08
AE_InactivityX = 0x04
AE_InactivityY = 0x02
AE_InactivityZ = 0x01
	# Options for data format
DF_Range_2g = 0x00
DF_Range_4g = 0x01
DF_Range_8g = 0x02
DF_Range_16g = 0x03
PC_Measure = 0x08

axesScale = 16

ACCEL_ADDR = 0x53

def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)
 	# accel 电源管理
	# bus.write_byte_data(ACCEL_ADDR, ACCEL_PWR_CTRL, 0x00)
	# bus.write_byte_data(ACCEL_ADDR, ACCEL_PWR_CTRL, 8)
	# setOption(DataFormat, DF_Range_16g)
	wakeUp()
	setScale()
	setTapThreshold()
	setTapDuration()
	setTapLatency()
	setTapWindow()
	setActivityThreshold()
	setInactivityThreshold()
	setInactivityTime()
	setFreeFallThreshold()
	setFreeFallTime()
	# intervals = math.floor(g / 0.0625)
	# if intervals < 256:
	# 	setOption(TapThreshold, intervals)
	# bus.write_byte_data(ACCEL_ADDR, DataFormat, DF_Range_16g|0x00)

def wakeUp():
	bus.write_byte_data(ACCEL_ADDR, PowerControl, 0x00)
	bus.write_byte_data(ACCEL_ADDR, PowerControl, PC_Measure)

def getAxes():
	scaleFactor = (axesScale*2)/1024
	acc_x = read_raw_data(ACCEL_ADDR, ACCEL_XOUT_H)
	acc_y = read_raw_data(ACCEL_ADDR, ACCEL_YOUT_H)
	acc_z = read_raw_data(ACCEL_ADDR, ACCEL_ZOUT_H)
	# (accel_x, accel_y, accel_z) = read_raw_data(ACCEL_ADDR, )
	return (acc_x * scaleFactor, acc_y * scaleFactor, acc_z * scaleFactor)

def setScale(scale=16):
	global axesScale
	if scale == 2:
		axesScale = 2
		setOption(DataFormat, DF_Range_2g)
	elif scale == 4:
		axesScale = 4
		setOption(DataFormat, DF_Range_4g)
	elif scale == 8:
		axesScale = 8
		setOption(DataFormat, DF_Range_8g)
	elif scale == 16:
		axesScale = 16
		setOption(DataFormat, DF_Range_16g)

def setActivityThreshold( g=-1, axis='z', change=.5):
		# If g is left unset, assume currently inactive and set to current state
		if g == -1:	
			(x, y, z) = getAxes()
			if axis == 'x':
				addActivity(AE_ActivityX)
				g = math.fabs(x) + change
			elif axis == 'y':
				addActivity(AE_ActivityY)
				g = math.fabs(y) + change
			elif axis == 'z':
				addActivity(AE_ActivityZ)
				g = math.fabs(z) + change
	
		# Figure out g's and then intervals based on 62.5 mg
		# Range 0-16g
		intervals = math.floor(math.fabs(g) / 0.0625 )
		print( intervals)
		if intervals < 256:
			setOption(ActivityThreshold, intervals)
			
def setInactivityThreshold( g=-1, axis='z', change=.1):
	# If g is left unset, assume currently inactive and set to current state
	if g == -1:	
		(x, y, z) = getAxes()
		if axis == 'x':
			addActivity(AE_InactivityX)
			g = math.fabs(x) + change
		elif axis == 'y':
			addActivity(AE_InactivityY)
			g = math.fabs(y) + change
		elif axis == 'z':
			addActivity(AE_InactivityZ)
			g = math.fabs(z) + change
			
	# Figure out g's and then intervals based on 62.5 mg
	# Range 0-16g
	intervals = math.floor(math.fabs(g) / 0.0625 )
	if intervals < 256:
		bus.write_byte_data(ACCEL_ADDR, InactivityThreshold, intervals)

def setInactivityTime( sec=1):
	# Figure out microseconds and then intervals based on 1.25 ms 
	# Range 0-255
	if sec < 256:
		setOption(InactivityTime, sec)
		
def setFreeFallThreshold( g = .4):
	# Figure out g's and then intervals based on 62.5 mg
	# Range 0-8 g
	intervals = math.floor(g / 0.0625)
	if intervals < 256:
		setOption(FreeFallThreshold, intervals)
		
def setFreeFallTime( sec=.0500):
	# Figure out microseconds and then intervals based on 5 ms 
	# Range 0-101
	intervals = math.floor(sec * 1000 / 5)
	if intervals < 256:
		setOption(FreeFallTime, intervals)

def setTapThreshold( g=3):
	# Figure out g's and then intervals based on 62.5 mg
	# Range 0-8 g
	intervals = math.floor(g / 0.0625)
	if intervals < 256:
		setOption(TapThreshold, intervals)
	
def setTapDuration( millisec=10):
	# Figure out microseconds and then intervals based on 625 us
	# Range 0-159 millisec
	intervals = math.floor(millisec * 1000/625)
	if intervals < 256:
		setOption(TapDuration, intervals)
		
def setTapLatency( millisec=150):
	# Figure out microseconds and then intervals based on 1.25 ms
	# Range 0-318
	intervals = math.floor(millisec / 1.25)
	if intervals < 256:
		setOption(TapLatency, intervals)
		
def setTapWindow( millisec=100):
	# Figure out microseconds and then intervals based on 1.25 ms 
	# Range 
	intervals = math.floor(millisec / 1.25)
	if intervals < 256:
		setOption(TapWindow, intervals)

def addActivity( *function_set):
	addOption(AxesEnable, *function_set)

def addOption(register, *function_set):
	options = bus.read_byte_data(ACCEL_ADDR,register)
	for function in function_set:
		options = options | function
	bus.write_byte_data(ACCEL_ADDR, register, options)

def setOption(register, *function_set):
	options = 0x00
	for function in function_set:
		options = options | function
	bus.write_byte_data(ACCEL_ADDR, register, options)	

def read_raw_data(addr, reg):
	#Accelero and Gyro value are 16-bit
	high = bus.read_byte_data(addr, reg)
	low = bus.read_byte_data(addr, reg+1)
    # if addr == ACCEL_ADDR:
    #     high = bus.read_byte_data(addr, reg+1)
    #     low = bus.read_byte_data(addr, reg)
    # else:
	# 	high = bus.read_byte_data(addr, reg)
	# 	low = bus.read_byte_data(addr, reg+1)

	#concatenate higher and lower value
	value = ((high << 8) | low)

	#to get signed value from mpu6050
	if(value > 32768):
		value = value - 65536
	return value


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68  # MPU6050 device address

MPU_Init()

print (" Reading Data of Gyroscope and Accelerometer")

while True:
	
	#Read Accelerometer raw value
	# acc_x = read_raw_data(ACCEL_ADDR, ACCEL_XOUT_H)
	# acc_y = read_raw_data(ACCEL_ADDR, ACCEL_YOUT_H)
	# acc_z = read_raw_data(ACCEL_ADDR, ACCEL_ZOUT_H)
	
	#Read Gyroscope raw value
	gyro_x = read_raw_data(Device_Address,GYRO_XOUT_H)
	gyro_y = read_raw_data(Device_Address,GYRO_YOUT_H)
	gyro_z = read_raw_data(Device_Address,GYRO_ZOUT_H)
	
	acc_x, acc_y, acc_z = getAxes()
	#Full scale range +/- 250 degree/C as per sensitivity scale factor
	Ax = acc_x/16384.0
	Ay = acc_y/16384.0
	Az = acc_z/16384.0
	
	Gx = gyro_x/14.375
	Gy = gyro_y/14.375
	Gz = gyro_z/14.375
	

	print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
	sleep(1)

```