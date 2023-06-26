---
sidebar_position: 1
title: Cuda版本的OpenCV部署
---
:::tip
在使用media-pipe相关框架的时候，如果直接跑的opencv，jetson nano会默认使用cpu来跑，这样会导致画面卡顿，效果不佳，因此需要编译opencv cuda版本以及media-pipe的cuda版本，也就是GPU版本。由于这方面的工作之前已经在国产nano的ROS系统中编译好了，推荐直接从国产的nano系统中，复制media-pipe的cuda版本到python3的安装目录下，而opencv的cuda版本则需要重新编译
:::


## 安装opencv-contrib-python cuda版本

- 复制安装脚本

```shell
vim install.sh
```

将下面的脚本复制到install.sh 中

```shell
#!/usr/bin/env bash
# 2019 Michael de Gans

set -e

# change default constants here:
readonly PREFIX=/usr/local  # install prefix, (can be ~/.local for a user install)
readonly DEFAULT_VERSION=4.4.1  # controls the default version (gets reset by the first argument)
readonly CPUS=$(nproc)  # controls the number of jobs

# better board detection. if it has 6 or more cpus, it probably has a ton of ram too
# 加快cpu编译进度
if [[ $CPUS -gt 4 ]]; then
    # something with a ton of ram
    JOBS=$CPUS
else
    JOBS=1  # you can set this to 4 if you have a swap file
    # otherwise a Nano will choke towards the end of the build
fi

cleanup () {
# https://stackoverflow.com/questions/226703/how-do-i-prompt-for-yes-no-cancel-input-in-a-linux-shell-script
    while true ; do
        echo "Do you wish to remove temporary build files in /tmp/build_opencv ? "
        if ! [[ "$1" -eq "--test-warning" ]] ; then
            echo "(Doing so may make running tests on the build later impossible)"
        fi
        read -p "Y/N " yn
        case ${yn} in
            [Yy]* ) rm -rf ~/work/tmp/build_opencv ; break;;
            [Nn]* ) exit ;;
            * ) echo "Please answer yes or no." ;;
        esac
    done
}

setup () {
    cd ~/work/tmp/
    if [[ -d "build_opencv" ]] ; then
        echo "It appears an existing build exists in /tmp/build_opencv"
        cleanup
    fi
    mkdir build_opencv
    cd build_opencv
}

git_source () {
    echo "Getting version '$1' of OpenCV"
    git clone --depth 1 --branch "$1" https://gitee.com/mirrors/opencv.git
    git clone --depth 1 --branch "$1" https://gitee.com/cubone/opencv_contrib.git
}

install_dependencies () {
    # open-cv has a lot of dependencies, but most can be found in the default
    # package repository or should already be installed (eg. CUDA).
    echo "Installing build dependencies."
    sudo apt-get update
    #sudo apt-get dist-upgrade -y --autoremove
    sudo apt-get install -y \
        build-essential \
        cmake \
        git \
        gfortran \
        libatlas-base-dev \
        libavcodec-dev \
        libavformat-dev \
        libavresample-dev \
        libcanberra-gtk3-module \
        libdc1394-22-dev \
        libeigen3-dev \
        libglew-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer-plugins-good1.0-dev \
        libgstreamer1.0-dev \
        libgtk-3-dev \
        libjpeg-dev \
        libjpeg8-dev \
        libjpeg-turbo8-dev \
        liblapack-dev \
        liblapacke-dev \
        libopenblas-dev \
        libpng-dev \
        libpostproc-dev \
        libswscale-dev \
        libtbb-dev \
        libtbb2 \
        libtesseract-dev \
        libtiff-dev \
        libv4l-dev \
        libxine2-dev \
        libxvidcore-dev \
        libx264-dev \
        pkg-config \
        python-dev \
        python-numpy \
        python3-dev \
        python3-numpy \
        python3-matplotlib \
        qv4l2 \
        v4l-utils \
        v4l2ucp \
        zlib1g-dev
}

configure () {
    local CMAKEFLAGS="
        -D BUILD_EXAMPLES=OFF
        -D BUILD_opencv_python2=ON
        -D BUILD_opencv_python3=ON
        -D CMAKE_BUILD_TYPE=RELEASE
        -D CMAKE_INSTALL_PREFIX=${PREFIX}
        -D CUDA_ARCH_BIN=5.3,6.2,7.2
        -D CUDA_ARCH_PTX=
        -D CUDA_FAST_MATH=ON
        -D CUDNN_VERSION='8.0'
        -D EIGEN_INCLUDE_PATH=/usr/include/eigen3 
        -D ENABLE_NEON=ON
        -D OPENCV_DNN_CUDA=ON
        -D OPENCV_ENABLE_NONFREE=ON
        -D OPENCV_EXTRA_MODULES_PATH=~/work/tmp/build_opencv/opencv_contrib/modules
        -D OPENCV_GENERATE_PKGCONFIG=ON
        -D WITH_CUBLAS=ON
        -D WITH_CUDA=ON
        -D WITH_CUDNN=ON
        -D WITH_GSTREAMER=ON
        -D WITH_LIBV4L=ON
        -D WITH_OPENGL=ON"

    if [[ "$1" != "test" ]] ; then
        CMAKEFLAGS="
        ${CMAKEFLAGS}
        -D BUILD_PERF_TESTS=OFF
        -D BUILD_TESTS=OFF"
    fi

    echo "cmake flags: ${CMAKEFLAGS}"

    cd opencv
    mkdir build
    cd build
    cmake ${CMAKEFLAGS} .. 2>&1 | tee -a configure.log
}

main () {

    local VER=${DEFAULT_VERSION}

    # parse arguments
    if [[ "$#" -gt 0 ]] ; then
        VER="$1"  # override the version
    fi

    if [[ "$#" -gt 1 ]] && [[ "$2" == "test" ]] ; then
        DO_TEST=1
    fi

    # prepare for the build:
    setup
    install_dependencies
    git_source ${VER}

    if [[ ${DO_TEST} ]] ; then
        configure test
    else
        configure
    fi

    # start the build
    make -j${JOBS} 2>&1 | tee -a build.log

    if [[ ${DO_TEST} ]] ; then
        make test 2>&1 | tee -a test.log
    fi

    # avoid a sudo make install (and root owned files in ~) if $PREFIX is writable
    if [[ -w ${PREFIX} ]] ; then
        make install 2>&1 | tee -a install.log
    else
        sudo make install 2>&1 | tee -a install.log
    fi

    cleanup --test-warning

}

main "$@"
```

:::tip
只编译opencv+v4ls，不编译cuda的脚本如下
```shell
#!/usr/bin/env bash
# 2023 powered by ceoifung

set -e

# change default constants here:
readonly PREFIX=/usr/local  # install prefix, (can be ~/.local for a user install)
readonly DEFAULT_VERSION=4.6.0  # controls the default version (gets reset by the first argument)
readonly CPUS=$(nproc)  # controls the number of jobs

# better board detection. if it has 6 or more cpus, it probably has a ton of ram too
# 加快cpu编译进度
if [[ $CPUS -gt 4 ]]; then
    # something with a ton of ram
    JOBS=$CPUS
else
    JOBS=1  # you can set this to 4 if you have a swap file
    # otherwise a Nano will choke towards the end of the build
fi

cleanup () {
# https://stackoverflow.com/questions/226703/how-do-i-prompt-for-yes-no-cancel-input-in-a-linux-shell-script
    while true ; do
        echo "Do you wish to remove temporary build files in /tmp/build_opencv ? "
        if ! [[ "$1" -eq "--test-warning" ]] ; then
            echo "(Doing so may make running tests on the build later impossible)"
        fi
        read -p "Y/N " yn
        case ${yn} in
            [Yy]* ) rm -rf ~/work/tmp/build_opencv ; break;;
            [Nn]* ) exit ;;
            * ) echo "Please answer yes or no." ;;
        esac
    done
}

setup () {
    if [[ ! -d "~/work/tmp/build_opencv/" ]] ; then
        echo "~/work/tmp/build_opencv not exits,now start mkdir"
        mkdir -p ~/work/tmp/build_opencv/
    fi
    cd ~/work/tmp/build_opencv
    # if [[ ! -d "build_opencv" ]] ; then
    #     echo "It appears an existing build exists in /tmp/build_opencv"
    #     cleanup
    # fi
    # mkdir build_opencv
    # cd build_opencv
}

git_source () {
    echo "Getting version '$1' of OpenCV"
    git clone --depth 1 --branch "$1" https://gitee.com/mirrors/opencv.git
    git clone --depth 1 --branch "$1" https://gitee.com/cubone/opencv_contrib.git
}

install_dependencies () {
    # open-cv has a lot of dependencies, but most can be found in the default
    # package repository or should already be installed (eg. CUDA).
    echo "Installing build dependencies."
    sudo apt-get update
    #sudo apt-get dist-upgrade -y --autoremove
    sudo apt-get install -y \
        build-essential \
        cmake \
        git \
        gfortran \
        libatlas-base-dev \
        libavcodec-dev \
        libavformat-dev \
        libavresample-dev \
        libcanberra-gtk3-module \
        libdc1394-22-dev \
        libeigen3-dev \
        libglew-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer-plugins-good1.0-dev \
        libgstreamer1.0-dev \
        libgtk-3-dev \
        libjpeg-dev \
        libjpeg8-dev \
        libjpeg-turbo8-dev \
        liblapack-dev \
        liblapacke-dev \
        libopenblas-dev \
        libpng-dev \
        libpostproc-dev \
        libswscale-dev \
        libtbb-dev \
        libtbb2 \
        libtesseract-dev \
        libtiff-dev \
        libv4l-dev \
        libxine2-dev \
        libxvidcore-dev \
        libx264-dev \
        pkg-config \
        python-dev \
        python-numpy \
        python3-dev \
        python3-numpy \
        python3-matplotlib \
        qv4l2 \
        v4l-utils \
        v4l2ucp \
        zlib1g-dev
}

configure () {
    local CMAKEFLAGS="
        -D BUILD_EXAMPLES=OFF
        -D PYTHON3_EXECUTABLE=$(python3 -c 'import sys; print(sys.executable)')
        -D PYTHON3_NUMPY_INCLUDE_DIRS=$(python3 -c 'import numpy; print (numpy.get_include())')
        -D PYTHON3_PACKAGES_PATH=$(python3 -c 'from distutils.sysconfig import get_python_lib; print(get_python_lib())') 
        -D BUILD_opencv_python2=ON
        -D BUILD_opencv_python3=ON
        -D CMAKE_BUILD_TYPE=RELEASE
        -D CMAKE_INSTALL_PREFIX=${PREFIX}
        -D EIGEN_INCLUDE_PATH=/usr/include/eigen3 
        -D OPENCV_EXTRA_MODULES_PATH=~/work/tmp/build_opencv/opencv_contrib/modules
        -D WITH_LIBV4L=ON"

    if [[ "$1" != "test" ]] ; then
        CMAKEFLAGS="
        ${CMAKEFLAGS}
        -D BUILD_PERF_TESTS=OFF
        -D BUILD_TESTS=OFF"
    fi

    echo "cmake flags: ${CMAKEFLAGS}"

    cd opencv
    #mkdir build
    cd build
    cmake ${CMAKEFLAGS} .. 2>&1 | tee -a configure.log
}

main () {

    local VER=${DEFAULT_VERSION}

    # parse arguments
    if [[ "$#" -gt 0 ]] ; then
        VER="$1"  # override the version
    fi

    if [[ "$#" -gt 1 ]] && [[ "$2" == "test" ]] ; then
        DO_TEST=1
    fi

    # prepare for the build:
    setup
    install_dependencies
    git_source ${VER}
    # cd ~/work/tmp/build_opencv
    if [[ ${DO_TEST} ]] ; then
        configure test
    else
        configure
    fi

    # start the build
    make -j${JOBS} 2>&1 | tee -a build.log

    if [[ ${DO_TEST} ]] ; then
        make test 2>&1 | tee -a test.log
    fi

    # avoid a sudo make install (and root owned files in ~) if $PREFIX is writable
    if [[ -w ${PREFIX} ]] ; then
        make install 2>&1 | tee -a install.log
    else
        sudo make install 2>&1 | tee -a install.log
    fi

    cleanup --test-warning

}

main "$@"

```
:::

- 运行安装命令

```shell
chmod 777 install.sh
./install.sh
```

### 可能遇到的错误

- 报错

```shell
E: The repository 'http://ppa.launchpad.net/hzwhuang/ss-qt5/ubuntu bionic Release' does not have a Release file.
N: Updating from such a repository can't be done securely, and is therefore disabled by default.
N: See apt-secure(8) manpage for repository creation and user configuration details.
W: An error occurred during the signature verification. The repository is not updated and the previous index files will be used. GPG error: http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu bionic Release: The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>

```

- 解决方法：

[参考连接](https://blog.csdn.net/heart_hang/article/details/95043542?spm=1001.2101.3001.6650.4&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-4-95043542-blog-119899612.pc_relevant_default&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-4-95043542-blog-119899612.pc_relevant_default&utm_relevant_index=5)

```shell
sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

- 报错

```shell
E: The repository 'http://ppa.launchpad.net/hzwhuang/ss-qt5/ubuntu bionic Release' does not have a Release file.
N: Updating from such a repository can't be done securely, and is therefore disabled by default.
N: See apt-secure(8) manpage for repository creation and user configuration details.

```

- 解决方法：https://blog.csdn.net/lun55423/article/details/108916991

```shell
sudo vim /etc/apt/sources.list.d/hzwhuang-ubuntu-ss-qt5-bionic.list
#看这个文件   
#将里面的bionic 改成xenial ,保存再运行 
sudo apt-get update
```

- 报错
```shell
fatal error: boostdesc_bgm.i:No such file or directory
```
- 解决方法
https://blog.csdn.net/ben_xiao_hai_123/article/details/126262891 将相关的文件下载下来，放到指定的目录下，然后重新编译即可

## 安装media-pipe cuda版本

将提供的jetson nano版本的media-pipe cuda版本的wheel上传到JetsonNano上面

- 安装pip3

```shell
sudo apt-get install python3-pip
# 安装mediapipe
pip3 install mediapipe-0.8.5_cuda102-cp36-cp36m-linux_aarch64.whl
```

可能会报如下错误：

```shell
protobuf requires Python '>=3.7' but the running Python is 3.6.9
```

解决方法

```shell
# 升级pip3
pip3 install --upgrade pip
# 然后在重新安装
pip3 install mediapipe-0.8.5_cuda102-cp36-cp36m-linux_aarch64.whl
```

- 可能报错

```shell
  Complete output from command python setup.py egg_info:
    Traceback (most recent call last):
      File "<string>", line 1, in <module>
      File "/tmp/pip-build-asenf7xw/opencv-contrib-python/setup.py", line 10, in <module>
        import skbuild
    ModuleNotFoundError: No module named 'skbuild'
```

- 解决方法：安装skbuild
```shell
# pip2
pip install scikit-build -i http://pypi.douban.com/simple --trusted-host pypi.douban.com
# pip3
pip3 install scikit-build -i http://pypi.douban.com/simple --trusted-host pypi.douban.com
```

## 安装python3 cv_bridge

```shell
pip3 install rosdep rosinstall catkin_pkg rospkg numpy pyyaml opencv-python
# 不要用github的连接，github的版本适配了ros2，导致软件无法更新了
git clone https://gitee.com/ceoifung/vision_opencv.git
cd ../
catkin_make install -DPYTHON_EXECUTABLE=/usr/bin/python3
```

- 可能遇到错误
```shell
..
CMake Error at CMakeLists.txt:1 (cmake_minimum_required):
CMake 3.14 or higher is required.  You are running version 3.10.2


-- Configuring incomplete, errors occurred!

```

- 将cmakelist.xml的cmake版本手动降下来


## catkin_ws编译

### fatal error: riki_msgs/Velocities.h: 没有那个文件或目录

```
# 首先编译riki_msgs，不然程序将会报错
catkin_make -DCATKIN_WHITELIST_PACKAGES=riki_msgs
# 然后在编译所有的程序
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```


### uvc_camera推流只有裸流没有压缩的视频流

这种情况下，查看uvc_camera的源码，发现有如下代码：

```c
 pnode.getParam("format", format);

/* advertise image streams and info streams */
if (format != "jpeg")
    pub = it.advertise("image_raw", 1);
else
    pubjpeg = node.advertise<CompressedImage>("image_raw/compressed", 1);

```

这里判断了用户是否设置了jpeg图片输出格式，如果用户在launch文件没有设置format，那么默认会将视频流推送到image_raw中，所以在launch文件中添加format字段

```shell
<launch>
  <node name="uvc_camera_node" pkg="uvc_camera" type="uvc_camera_node" output="screen">
        <remap from="/image_raw" to="/camera/rgb/image_raw" />
        <remap from="/camera_info" to="/camera/rgb/camera_info" />
        <param name="width" type="int" value="640" />
        <param name="height" type="int" value="480" />
        <param name="fps" type="int" value="10" />
        <param name="frame" type="string" value="wide_stereo" />
         <!-- 在此添加字段，有可能是默认，设置参数为jpeg -->

        <param name="format" type="string" value="jpeg"/>
        <param name="auto_focus" type="bool" value="False" />
        <param name="focus_absolute" type="int" value="0" />
        <!-- other supported params: auto_exposure, exposure_absolute, brightness, power_line_frequency -->

        <param name="device" type="string" value="/dev/video0" />
        <param name="camera_info_url" type="string" value="file://$(find uvc_camera)/example.yaml" />
   </node>
  <node pkg="topic_tools" type="transform" name="compressed_image" args="/camera/rgb/image_raw/compressed /compressed_image sensor_msgs/CompressedImage 'm'" required="true" output="screen">

```

## 安装MediaPipe后运行程序报错

```shell
ons/__init__.py", line 17, in <module>
    import mediapipe.python.solutions.drawing_utils
  File "/home/xrrobot/.local/lib/python3.6/site-packages/mediapipe/python/solutions/drawing_utils.py", line 21, in <module>
    import dataclasses
ModuleNotFoundError: No module named 'dataclasses'

```
- 解决方法
```shell
pip3 install dataclasses
```

- 没有drawing_style

```shell
Can't find file: mediapipe/modules/face_detection/face_detection_front.tflite
Traceback (most recent call last):
  File "/home/xrrobot/cv_bridge_ws/src/xrros_opencv/ceoifung/scripts/RosChatter.py", line 30, in <module>
    xrDetector = XRHandDetector()
  File "/home/xrrobot/cv_bridge_ws/src/xrros_opencv/ceoifung/scripts/XRHandDetector.py", line 38, in __init__
    self.mpDrawingStyle = mp.solutions.drawing_styles
AttributeError: module 'mediapipe.python.solutions' has no attribute 'drawing_styles'
```

- 解决方法：可以将以前国产的六足机器人的python3的site-packages复制出来，直接放到python3的site-packages目录，这样的话，就可以避免很多错误了，因为这里面的mediapipe都是我之前手动改过的，因此在实际用的时候，需要将改动的地方同步上去