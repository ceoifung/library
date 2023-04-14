---
sidebar_position: 6
title: IJKPlayer编译so
---
ijkplayer是bilibili开源的一款轻量播放器插件，[github地址](https://github.com/bilibili/ijkplayer)，默认情况下不支持mjpg视频流播放，因此需要下载下来重新编译

## 环境准备
- window系统一台
- ubuntu18.04的虚拟机或者window子系统
- android sdk 和android ndk r14版本

## 编译方法
### 下载android sdk和android ndk
```shell
wget 
```
### 新建脚本
```shell
mkdir ~/work && cd ~/work
vim build.sh
```
在build.sh添加如下内容
```shell
git clone https://github.com/Bilibili/ijkplayer.git ijkplayer-android
cd ijkplayer-android
git checkout -B latest k0.8.8


```

```shell
./init-android.sh
./init-android-openssl.sh
./init-android-soundtouch.sh
cd android/contrib
./compile-ffmpeg.sh clean
./compile-ffmpeg.sh all

cd ..
./compile-ijk.sh all
```