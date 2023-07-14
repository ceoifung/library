---
sidebar_position: 1
title: Udev规则映射
---

## 树莓派摄像头设备映射
由于树莓派在插入摄像头的时候，会出现两个设备，分别是`/dev/video0`和`/dev/video1`，因此在实际用的时候，用户可能由于插拔，导致在代码中调用的摄像头可能是无效的，比如

```python
import cv2

video = cv2.VideoCapture(0)
```

以上代码由于用户直接使用video设备，然而经过插拔之后，摄像头的真实设备号已经改变了，因此会报错，所以最佳的方法是将用户的设备映射成指定的设备

```shell
cd /etc/udev/rules.d
vim 58-xrudev.rules
```

输入如下内容

```shell
KERNEL=="video*", ATTRS{idVendor}=="038f", ATTRS{idProduct}=="6001", ATTR{index}=="0", MODE:="0777", SYMLINK+="xrcamera"
KERNEL=="js*", ATTRS{idVendor}=="0079", ATTRS{idProduct}=="181c", MODE:="0777", SYMLINK+="xrjoy"
KERNEL=="video*", ATTRS{idVendor}=="0c45", ATTRS{idProduct}=="6367", ATTR{index}=="0", MODE:="0777", SYMLINK+="xrcamera"
KERNEL=="video*", ATTRS{idVendor}=="cc01", ATTRS{idProduct}=="cc01", ATTR{index}=="0", MODE:="0777", SYMLINK+="xrcamera"
KERNEL=="video*", ATTRS{idVendor}=="cc02", ATTRS{idProduct}=="cc02", ATTR{index}=="0", MODE:="0777", SYMLINK+="xrcamera"
KERNEL=="video*", ATTRS{idVendor}=="cc03", ATTRS{idProduct}=="cc03", ATTR{index}=="0", MODE:="0777", SYMLINK+="xrcamera"
KERNEL=="video*", ATTRS{idVendor}=="cc04", ATTRS{idProduct}=="cc04", ATTR{index}=="0", MODE:="0777", SYMLINK+="xrcamera"
KERNEL=="video*", ATTRS{idVendor}=="cc05", ATTRS{idProduct}=="cc05", ATTR{index}=="0", MODE:="0777", SYMLINK+="xrcamera"
KERNEL=="video*", ATTRS{idVendor}=="0c45", ATTRS{idProduct}=="64ab", ATTR{index}=="0", MODE:="0777", SYMLINK+="xrcamera"
KERNEL=="video*", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="525a", ATTR{index}=="0", MODE:="0777", SYMLINK+="xrcamera"

```

:::tip
注意：其中的`ATTR{index}=="0"`千万不能省略，不然的话，依然会导致出现映射设备不是正常设备的境况，经过实际的测试，这个映射方式是最有效的
:::