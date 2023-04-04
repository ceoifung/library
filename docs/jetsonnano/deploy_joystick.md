---
sidebar_position: 7
title: 2.4G无线手柄的使用
---

## 手柄Python键值对应关系

**相关名词定义：PUSH：代表推动，表示模拟量；PRESSED：表示按键被按下**

| 键值         | 对应关系                       | 备注                                     |
| ------------ | ------------------------------ | ---------------------------------------- |
| L1           | 手柄左侧L1按键                 | 0代表释放，1代表按下                     |
| L2           | 手柄左侧L2按键                 | 0代表释放，1代表按下                     |
| L2_PUSH      | 手柄左侧L2按键按下的力度模拟量 | 范围0-1                                  |
| R1           | 手柄右侧R1按键                 | 0代表释放，1代表按下                     |
| R2           | 手柄右侧R2按键                 | 0代表释放，1代表按下                     |
| R2_PUSH      | 手柄左侧R2按键按下的力度模拟量 | 范围0-1                                  |
| UP           | 手柄左侧向上                   | 0代表释放，1代表按下                     |
| DOWN         | 手柄左侧向下                   | 0代表释放，1代表按下                     |
| LEFT         | 手柄左侧向左                   | 0代表释放，1代表按下                     |
| RIGHT        | 手柄左侧向右                   | 0代表释放，1代表按下                     |
| **LX_UP**    | `左侧摇杆`向上                 | 0代表释放，1代表按下                     |
| **LX_DOWN**  | `左侧摇杆`向下                 | 0代表释放，1代表按下                     |
| **LX_LEFT**  | `左侧摇杆`向左                 | 0代表释放，1代表按下                     |
| **LX_RIGHT** | `左侧摇杆`向右                 | 0代表释放，1代表按下                     |
| **RX_UP**    | `右侧摇杆`向上                 | 0代表释放，1代表按下                     |
| **RX_DOWN**  | `右侧摇杆`向下                 | 0代表释放，1代表按下                     |
| **RX_LEFT**  | `右侧摇杆`向左                 | 0代表释放，1代表按下                     |
| **RX_RIGHT** | `右侧摇杆`向右                 | 0代表释放，1代表按下                     |
| A            | 右侧A按键                      | 0代表释放，1代表按下                     |
| B            | 右侧B按键                      | 0代表释放，1代表按下                     |
| X            | 右侧X按键                      | 0代表释放，1代表按下                     |
| Y            | 右侧Y按键                      | 0代表释放，1代表按下                     |
| SELECT       | SELECT按键                     | 0代表释放，1代表按下                     |
| START        | START按键                      | 0代表释放，1代表按下                     |
| LEFT_RIGHT   | 左侧左右按键                   | 0代表释放，1代表向右，-1代表向左         |
| UP_DOWN      | 左侧上下按键                   | 0代表释放，1代表向下，-1代表向上         |
| LX_PUSH      | 左侧摇杆左右推动模拟值         | 0代表释放，范围0-1代表向右，0~-1代表向左 |
| LY_PUSH      | 左侧摇杆上下推动模拟值         | 0代表释放，范围0-1代表向下，0~-1代表向上 |
| RX_PUSH      | 右摇杆左右推动模拟值           | 0代表释放，范围0-1代表向右，0~-1代表向左 |
| RY_PUSH      | 右摇杆上下推动模拟值           | 0代表释放，范围0-1代表向下，0~-1代表向上 |
| LX_PRESSED   | 左侧摇杆被按下                 | 0代表释放，1代表按下                     |
| RX_PRESSED   | 右侧摇杆被按下                 | 0代表释放，1代表按下                     |


## 适用于树莓派的手柄工具类
```python

class XRJoyUtils:
    """手柄操作数据读取类
    """
    def __init__(self, js='/dev/xrjoy', db=0):
        # 定义xrrobot按键
        # button
        self.joy_state = {'FLAG': 0,  # 是否有数据变更
                          'L1': 0, 'L2': 0, 'L2_PUSH': 0.0, 'R1': 0, 'R2': 0, 'R2_PUSH': 0.0,  # 侧面按键
                          'A': 0, 'B': 0, 'X': 0, 'Y': 0,  # 右侧按钮
                          'SELECT': 0, 'START': 0,  # 功能按钮
                          'LEFT': 0,'RIGHT': 0, 'UP': 0, 'DOWN': 0,
                          'LX_PUSH': 0, 'LY_PUSH': 0.0,
                          'RX_PUSH': 0, 'RY_PUSH': 0.0,
                          'LX_UP': 0, 'LX_DOWN': 0, 'LX_LEFT': 0, 'LX_RIGHT': 0,
                          'RX_UP': 0, 'RX_DOWN': 0, 'RX_LEFT': 0, 'RX_RIGHT': 0,
                          "LX_PRESSED": 0, "RX_PRESSED": 0}
        self.axis_names = {
            0x00: 'LX_PUSH',  # 左侧摇杆左右
            0x01: 'LY_PUSH',  # 左侧摇杆上下
            0x02: 'RX_PUSH',  # 右侧摇杆左右
            0x05: 'RY_PUSH',  # 右侧摇杆上下
            0x09: 'R2_PUSH',  # R2按下力度
            0x0a: 'L2_PUSH',  # L2按下力度
            0x10: 'LEFT_RIGHT',  # 左侧按键左右值
            0x11: 'UP_DOWN',    # 左侧按键上下值
        }
        # These constants were borrowed from linux/input.h
        self.button_names = {
            0x130: 'A',
            0x131: 'B',
            0x133: 'X',
            0x134: 'Y',
            0x136: 'L1',
            0x137: 'R1',
            0x138: 'L2',
            0x139: 'R2',
            0x13a: 'SELECT',
            0x13b: 'START',
            0x13d: 'LX_PRESSED',
            0x13e: 'RX_PRESSED'
        }
        self.axis_map = []
        self.button_map = []
        self.db = db  # 调试信息打印，1：打印，0：不打印

        if sys.version_info > (3, 0):  # 兼容python3.6
            self.py_v3 = True
        else:
            self.py_v3 = False
        # 1、搜寻打印设备信息
        self.find_js()
        # 2、Open the joystick device.
        self.js = js
        print('Opening %s...' % self.js)
        self.jsdev = open(self.js, 'rb')
        # 3、打印设备名称
        self.print_dev()
        time.sleep(0.1)
        self.setAsynRecv()

    def find_js(self):
        # Iterate over the joystick devices.
        print('Available devices:')
        for dev in os.listdir('/dev/input'):
            if dev.startswith('js'):
                print('  /dev/input/%s' % (dev))

    def print_dev(self):
        # Get the device name.
        # buf = bytearray(63)
        buf = array.array('B', [0] * 64)
        # JSIOCGNAME(len)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf)
        for i in range(buf.count(0)):
            buf.remove(0)
        js_name = ''.join('%s' % chr(i) for i in buf)
        print('Device name: %s' % js_name)

        # Get number of axes and buttons.
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a11, buf)  # JSIOCGAXES
        num_axes = buf[0]

        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, buf)  # JSIOCGBUTTONS
        num_buttons = buf[0]

        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(self.jsdev, 0x80406a32, buf)  # JSIOCGAXMAP

        for axis in buf[:num_axes]:
            axis_name = self.axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)

        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(self.jsdev, 0x80406a34, buf)  # JSIOCGBTNMAP

        for btn in buf[:num_buttons]:
            # if btn in self.button_names.keys():
            btn_name = self.button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)

        if self.db:  # 调试信息
            print('%d axes found: %s' % (num_axes, ', '.join(self.axis_map)))
            print('%d buttons found: %s' %
                  (num_buttons, ', '.join(self.button_map)))

    def setAsynRecv(self):
        if self.py_v3:
            self.analyThd = threading.Thread(
                target=self.joy_analy, daemon=True)
            print('py3 start joy_analy')
            self.analyThd.start()
        else:
            self.analyThd = threading.Thread(target=self.joy_analy)
            self.analyThd.setDaemon(True)
            print('py2 start joy_analy')
            self.analyThd.start()

    def joy_analy(self):
        # Main event loop
        while True:
            evbuf = self.jsdev.read(8)
            if evbuf:
                tim, value, type, number = struct.unpack(
                    'IhBB', evbuf)  # 图中标出的数字是指此处的 number，用来判断此数据是哪个按键的变化
                if self.db:  # 调试信息
                    print('value = 0x%02x, type= 0x%02x, number = 0x%02x ' %
                          (value, type, number))
                    if type & 0x80:
                        if self.db:  # 调试信息
                            print("(initial)")
                        else:
                            pass
                if type & 0x01:
                    self.joy_state['FLAG'] = 1
                    button = self.button_map[number]
                    # print(button, value)
                    if "unknown" not in button:
                        # fvalue = value
                        self.joy_state[button] = value
                        # print(button)
                    if self.db:  # 调试信息
                        button = self.button_map[number]
                        if button:
                            # button_states[button] = value
                            if value:
                                print("%s pressed" % (button))
                            elif value == 0:
                                print("%s released" % (button))

                if type & 0x02:
                    self.joy_state['FLAG'] = 1

                    axis = self.axis_map[number]

                    if axis and "unknown" not in axis:
                        
                        fvalue = 0
                        if value > 10000:
                            fvalue = value / 32767.0
                        elif value < -10000:
                            fvalue = value / 32767.0
                        if axis == "UP_DOWN":
                            self.joy_state["UP"] = 1 if fvalue < 0 else 0
                            self.joy_state["DOWN"] = 1 if fvalue > 0 else 0
                        if axis == "LEFT_RIGHT":
                            self.joy_state["LEFT"] = 1 if fvalue < 0 else 0
                            self.joy_state["RIGHT"] = 1 if fvalue > 0 else 0
                        if axis == "LX_PUSH":
                            self.joy_state["LX_LEFT"] = 1 if fvalue < 0 else 0
                            self.joy_state["LX_RIGHT"] = 1 if fvalue > 0 else 0
                        if axis == "LY_PUSH":
                            self.joy_state["LX_UP"] = 1 if fvalue < 0 else 0
                            self.joy_state["LX_DOWN"] = 1 if fvalue > 0 else 0
                        if axis == "RX_PUSH":
                            self.joy_state["RX_LEFT"] = 1 if fvalue < 0 else 0
                            self.joy_state["RX_RIGHT"] = 1 if fvalue > 0 else 0
                        if axis == "RY_PUSH":
                            self.joy_state["RX_UP"] = 1 if fvalue < 0 else 0
                            self.joy_state["RX_DOWN"] = 1 if fvalue > 0 else 0
                        self.joy_state[axis] = fvalue

                    if self.db:  # 调试信息
                        axis = self.axis_map[number]
                        if axis:
                            if value > 10000:
                                fvalue = value / 32767.0
                                # axis_states[axis] = fvalue
                                print("%s: %.3f" % (axis, fvalue))
                            elif value < -10000:
                                fvalue = value / 32767.0
                                # axis_states[axis] = fvalue
                                print("%s: %.3f" % (axis, fvalue))
                            elif value == 0:
                                print("%s released" % (axis))

```
引用方式
```python
if __name__ == "__main__":
    joy = XRJoyUtils()
    while True:
        if joy.joy_state['FLAG']:
            joy.joy_state["FLAG"] = 0
            buf = joy.joy_state
            print(buf)
```
## JetsonNano 六足手柄的驱动
```python
import os
import struct
import array
from fcntl import ioctl
import threading
import sys
import time


class XRJoyUtils:
    """手柄操作数据读取类
    """
    def __init__(self, js='/dev/xrjoy', db=0):
        # 定义xrrobot按键
        # button
        self.joy_state = {'FLAG': 0,  # 是否有数据变更
                          'L1': 0, 'L2': 0, 'L2_PUSH': 0.0, 'R1': 0, 'R2': 0, 'R2_PUSH': 0.0,  # 侧面按键
                          'A': 0, 'B': 0, 'X': 0, 'Y': 0,  # 右侧按钮
                          'SELECT': 0, 'START': 0,  # 功能按钮
                          'LEFT': 0,'RIGHT': 0, 'UP': 0, 'DOWN': 0,
                          'LX_PUSH': 0, 'LY_PUSH': 0.0,
                          'RX_PUSH': 0, 'RY_PUSH': 0.0,
                          'LX_UP': 0, 'LX_DOWN': 0, 'LX_LEFT': 0, 'LX_RIGHT': 0,
                          'RX_UP': 0, 'RX_DOWN': 0, 'RX_LEFT': 0, 'RX_RIGHT': 0,
                          "LX_PRESSED": 0, "RX_PRESSED": 0}
        # 这里的键值有改动，主要是两侧摇杆的键值对应不上
        self.axis_names = {
            0x00: 'LX_PUSH',  # 左侧摇杆左右
            0x01: 'LY_PUSH',  # 左侧摇杆上下
            0x02: 'RX_PUSH',  # 右侧摇杆左右
            0x03: 'RY_PUSH',  # 右侧摇杆上下
            0x04: 'R2_PUSH',  # R2按下力度
            0x05: 'L2_PUSH',  # L2按下力度
            0x10: 'LEFT_RIGHT',  # 左侧按键左右值
            0x11: 'UP_DOWN',    # 左侧按键上下值
        }
        # These constants were borrowed from linux/input.h
        self.button_names = {
            0x130: 'A',
            0x131: 'B',
            0x133: 'X',
            0x134: 'Y',
            0x136: 'L1',
            0x137: 'R1',
            0x138: 'L2',
            0x139: 'R2',
            0x13a: 'SELECT',
            0x13b: 'START',
            0x13d: 'LX_PRESSED',
            0x13e: 'RX_PRESSED'
        }
        self.axis_map = []
        self.button_map = []
        self.db = db  # 调试信息打印，1：打印，0：不打印

        if sys.version_info > (3, 0):  # 兼容python3.6
            self.py_v3 = True
        else:
            self.py_v3 = False
        # 1、搜寻打印设备信息
        self.find_js()
        # 2、Open the joystick device.
        self.js = js
        print('Opening %s...' % self.js)
        try:
            self.jsdev = open(self.js, 'rb')
        except FileNotFoundError:
            self.jsdev = None
            print("File is not found.")
        except PermissionError:
            self.jsdev = None
            print("You don't have permission to access this file.")
        # print(self.jsdev)
        # 3、打印设备名称
        self.print_dev()
        time.sleep(0.1)
        self.setAsynRecv()
        time.sleep(2)
        # 防止数据错误
        self.joy_state["FLAG"] = 0


    def find_js(self):
        # Iterate over the joystick devices.
        print('Available devices:')
        for dev in os.listdir('/dev/input'):
            if dev.startswith('js'):
                print('  /dev/input/%s' % (dev))

    def print_dev(self):
        # Get the device name.
        # buf = bytearray(63)
        buf = array.array('B', [0] * 64)
        # JSIOCGNAME(len)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf)
        for i in range(buf.count(0)):
            buf.remove(0)
        js_name = ''.join('%s' % chr(i) for i in buf)
        print('Device name: %s' % js_name)

        # Get number of axes and buttons.
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a11, buf)  # JSIOCGAXES
        num_axes = buf[0]

        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, buf)  # JSIOCGBUTTONS
        num_buttons = buf[0]

        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(self.jsdev, 0x80406a32, buf)  # JSIOCGAXMAP

        for axis in buf[:num_axes]:
            axis_name = self.axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)

        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(self.jsdev, 0x80406a34, buf)  # JSIOCGBTNMAP

        for btn in buf[:num_buttons]:
            # if btn in self.button_names.keys():
            btn_name = self.button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)

        if self.db:  # 调试信息
            print('%d axes found: %s' % (num_axes, ', '.join(self.axis_map)))
            print('%d buttons found: %s' %
                  (num_buttons, ', '.join(self.button_map)))

    def setAsynRecv(self):
        if self.py_v3:
            self.analyThd = threading.Thread(
                target=self.joy_analy, daemon=True)
            print('py3 start joy_analy')
            self.analyThd.start()
        else:
            self.analyThd = threading.Thread(target=self.joy_analy)
            self.analyThd.setDaemon(True)
            print('py2 start joy_analy')
            self.analyThd.start()

    def joy_analy(self):
        # Main event loop
        while True:
            if self.jsdev is None:
                try:
                    print("初始化手柄无效，重新初始化..")
                    self.jsdev = open(self.js, 'rb')
                    time.sleep(1)
                except IOError:
                    print("手柄无法获取...")
                    self.jsdev = None
                    time.sleep(1)
            else:
                evbuf = self.jsdev.read(8)
                if evbuf:
                    tim, value, type, number = struct.unpack(
                        'IhBB', evbuf)  # 图中标出的数字是指此处的 number，用来判断此数据是哪个按键的变化
                    if self.db:  # 调试信息
                        print('value = 0x%02x, type= 0x%02x, number = 0x%02x ' %
                            (value, type, number))
                        if type & 0x80:
                            if self.db:  # 调试信息
                                print("(initial)")
                            else:
                                pass
                    # print("type==", type)
                    if type & 0x80:
                        print("测试模式, 将左右两侧摇杆的数据复位，防止干扰")
                        self.joy_state['LX_PUSH'] = 0
                        self.joy_state['LY_PUSH'] = 0
                        self.joy_state['RX_PUSH'] = 0
                        self.joy_state['RY_PUSH'] = 0
                        self.joy_state["LX_LEFT"] = 0
                        self.joy_state["LX_RIGHT"] = 0
                        self.joy_state["RX_LEFT"] = 0
                        self.joy_state["RX_LEFT"] = 0
                        self.joy_state["LX_UP"] = 0
                        self.joy_state["LX_DOWN"] = 0
                        self.joy_state["RX_UP"] = 0
                        self.joy_state["RX_DOWN"] = 0
                    if type & 0x01:
                        self.joy_state['FLAG'] = 1
                        # print(number)
                        button = self.button_map[number]
                        # print(button, value)
                        if "unknown" not in button:
                            # fvalue = value
                            self.joy_state[button] = value
                            # print(button)
                        if self.db:  # 调试信息
                            button = self.button_map[number]
                            if button:
                                # button_states[button] = value
                                if value:
                                    print("%s pressed" % (button))
                                elif value == 0:
                                    print("%s released" % (button))

                    if type & 0x02:
                        self.joy_state['FLAG'] = 1
                        # print(number)
                        axis = self.axis_map[number]
                        if axis and "unknown" not in axis:
                            print("value=====", value)
                            fvalue = 0
                            if value > 10000:
                                fvalue = value / 32767.0
                            elif value < -10000:
                                fvalue = value / 32767.0
                            
                            if axis == "UP_DOWN":
                                self.joy_state["UP"] = 1 if fvalue < 0 else 0
                                self.joy_state["DOWN"] = 1 if fvalue > 0 else 0
                            if axis == "LEFT_RIGHT":
                                self.joy_state["LEFT"] = 1 if fvalue < 0 else 0
                                self.joy_state["RIGHT"] = 1 if fvalue > 0 else 0
                            if axis == "LX_PUSH":
                                print("LX_PUSH", fvalue)
                                if fvalue == 0:
                                    self.joy_state["LX_LEFT"] = 0
                                    self.joy_state["LX_RIGHT"] = 0
                                else:
                                    self.joy_state["LX_LEFT"] = 1 if fvalue < 0 else 0
                                    self.joy_state["LX_RIGHT"] = 1 if fvalue > 0 else 0
                            if axis == "LY_PUSH":
                                print("LY_PUSH", fvalue)
                                if fvalue == 0:
                                    self.joy_state["LX_UP"] = 0
                                    self.joy_state["LX_DOWN"] = 0
                                else:
                                    self.joy_state["LX_UP"] = 1 if fvalue < 0 else 0
                                    self.joy_state["LX_DOWN"] = 1 if fvalue > 0 else 0
                            if axis == "RX_PUSH":
                                print("RX_PUSH", fvalue)
                                if fvalue == 0:
                                    self.joy_state["RX_LEFT"] = 0
                                    self.joy_state["RX_LEFT"] = 0
                                else:
                                    self.joy_state["RX_LEFT"] = 1 if fvalue < 0 else 0
                                    self.joy_state["RX_RIGHT"] = 1 if fvalue > 0 else 0
                            if axis == "RY_PUSH":
                                print("RY_PUSH", fvalue)
                                if fvalue == 0:
                                    self.joy_state["RX_UP"] = 0
                                    self.joy_state["RX_DOWN"] = 0
                                else:
                                    self.joy_state["RX_UP"] = 1 if fvalue < 0 else 0
                                    self.joy_state["RX_DOWN"] = 1 if fvalue > 0 else 0
                            self.joy_state[axis] = fvalue
                            # 最后再置位，防止检测不到按键的抬起事件
                            self.joy_state['FLAG'] = 1

                        if self.db:  # 调试信息
                            axis = self.axis_map[number]
                            if axis:
                                if value > 10000:
                                    fvalue = value / 32767.0
                                    # axis_states[axis] = fvalue
                                    print("%s: %.3f" % (axis, fvalue))
                                elif value < -10000:
                                    fvalue = value / 32767.0
                                    # axis_states[axis] = fvalue
                                    print("%s: %.3f" % (axis, fvalue))
                                elif value == 0:
                                    print("%s released" % (axis))
```
引用方式
```python
if __name__ == "__main__":
    joy = XRJoyUtils("/dev/input/js0")
    while True:
        if joy.joy_state['FLAG']:
            joy.joy_state["FLAG"] = 0
            buf = joy.joy_state
            print(buf)
```

## JetsonNano与树莓派六足的手柄驱动异同
:::tip
不同的平台，可能测试的值不一样，像jetson nano，手柄连上去测试的时候，`type=0x80`，而在香蕉派平台上，却是`type=0x82`，所以在应用的时候，可以实际测试一下，然后进行不同的判断
:::
经过测试，JetsonNano的手柄驱动，在连接的时候，会初始化摇杆，导致摇杆的的`LX_PUSH、RX_PUSH、LY_PUSH、RY_PUSH`中的值变成映射后的-1，导致一直产生摇杆事件，从而导致机器人无法停止下来，一直左相同的运动，所以需要在原先的驱动下修改：

- 在jetsonNano的驱动代码中的`joy_analy`函数中添加如下内容
```python
# print("type==", type)
if type & 0x80: # 0x80表示在手柄连接上去之后，进行键值测试的模式，在此模式下，将影响摇杆的键值复位即可
	print("测试模式, 将左右两侧摇杆的数据复位，防止干扰")
	self.joy_state['LX_PUSH'] = 0
	self.joy_state['LY_PUSH'] = 0
	self.joy_state['RX_PUSH'] = 0
	self.joy_state['RY_PUSH'] = 0
	self.joy_state["LX_LEFT"] = 0
	self.joy_state["LX_RIGHT"] = 0
	self.joy_state["RX_LEFT"] = 0
	self.joy_state["RX_LEFT"] = 0
	self.joy_state["LX_UP"] = 0
	self.joy_state["LX_DOWN"] = 0
	self.joy_state["RX_UP"] = 0
	self.joy_state["RX_DOWN"] = 0
```
- 修改摇杆的映射键值，在`__init__`函数中修改
```python
 # 这里的键值有改动，主要是两侧摇杆的键值对应不上
 self.axis_names = {
	 0x00: 'LX_PUSH',  # 左侧摇杆左右
	 0x01: 'LY_PUSH',  # 左侧摇杆上下
	 0x02: 'RX_PUSH',  # 右侧摇杆左右
	 0x03: 'RY_PUSH',  # 右侧摇杆上下
	 0x04: 'R2_PUSH',  # R2按下力度
	 0x05: 'L2_PUSH',  # L2按下力度
	 0x10: 'LEFT_RIGHT',  # 左侧按键左右值
	 0x11: 'UP_DOWN',    # 左侧按键上下值
 }
```
- 修改摇杆抬起的响应灵敏度
```python
if type & 0x02:
	....
	# 在摇杆处理逻辑的最后将FLAG置位，防止检测不到摇杆的抬起事件
	self.joy_state['FLAG'] = 1
```