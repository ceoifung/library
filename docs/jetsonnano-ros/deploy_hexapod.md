---
sidebar_position: 4
title: hexapod六足程序部署
---

## 环境准备
- 硬件：Jetson Nano
- ros版本：ros1, medolic
- 摄像头：astra camera

## 安装ROS依赖项
```shell
sudo apt-get install git 
sudo apt-get install ros-melodic-sound-play 
sudo apt-get install ros-melodic-diagnostic-updater
sudo apt-get install ros-melodic-xacro
sudo apt-get install ros-melodic-openni2-launch
sudo apt-get install ros-melodic-depthimage-to-laserscan
sudo apt-get install ros-melodic-joystick-drivers
sudo apt-get install ros-melodic-imu-filter-madgwick
sudo apt-get install ros-melodic-robot-localization
sudo apt-get install ros-melodic-rtabmap
sudo apt-get install ros-melodic-rtabmap-ros
sudo apt-get install ros-melodic-robot-state-publisher
sudo apt-get install ros-melodic-gazebo-ros-control
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-move_base
sudo apt-get install ros-melodic-navfn
sudo apt-get install ros-melodic-amcl
sudo apt-get install libusb-1.0-0-dev
```

## 安装编译Astra Camera 
```shell
# 安装必须的依赖
sudo apt install ros-melodic-rgbd-launch ros-melodic-libuvc ros-melodic-libuvc-camera ros-melodic-libuvc-ros
# 进入catkin工作空间
cd ~/catkin_ws/src
# 下载源码
git clone https://github.com/orbbec/ros_astra_camera
roscd astra_camera
# 创立摄像头规则
./scripts/create_udev_rules
cd ~/catkin_ws
# 编译摄像头源码
catkin_make --pkg astra_camera
```

### 编译过程中遇到的问题
#### libuvc is not found
```shell
CMake Error at /usr/share/cmake-3.10/Modules/FindPkgConfig.cmake:649 (message):
  None of the required 'libuvc' found
Call Stack (most recent call first):
  ros_astra_camera/CMakeLists.txt:33 (pkg_search_module)


CMake Error at ros_astra_camera/CMakeLists.txt:35 (message):
  libuvc is not found
```
- 解决方法
```shell
sudo apt install ros-melodic-rgbd-launch ros-melodic-libuvc ros-melodic-libuvc-camera ros-melodic-libuvc-ros
sudo ldconfig

# 如果还是不行，那么安装如下方法，重新编译libuvc
git clone https://github.com/ktossell/libuvc
cd libuvc
mkdir build
cd build
cmake ..
make && sudo make install
sudo ldconfig
```

#### None of the required 'libglog' found

```shell
CMake Error at /usr/share/cmake-3.10/Modules/FindPkgConfig.cmake:649 (message):
  None of the required 'libglog' found
Call Stack (most recent call first):
  ros_astra_camera/CMakeLists.txt:37 (pkg_search_module)
```
- 解决方法
```shell
sudo apt-get install libgoogle-glog-dev 
```

#### 没有压缩流推出来
默认的astra camera在推送视频流的时候，取消了CompressedImage推流，因此可以推流的launch下面增加这一行
```shell
<node name="image_transport" pkg="image_transport" type="republish" args="raw in:=/camera/color/image_raw compressed out:=/image_raw" />

# 最后推送出来的压缩流就是 /image_raw/compressed
```