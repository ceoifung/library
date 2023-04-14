---
sidebar_position: 9
title: 视觉SLAM开发记录
---

视觉slam建图，所用的摄像头是astra_pro

## 安装摄像头标定文件
```shell
sudo apt install ros-$ROS_DISTRO-camera-calibration
```
[标定教程](https://blog.csdn.net/qq_43269411/article/details/107266922?spm=1001.2101.3001.6650.3&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-3-107266922-blog-102480647.235%5Ev28%5Epc_relevant_t0_download&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-3-107266922-blog-102480647.235%5Ev28%5Epc_relevant_t0_download&utm_relevant_index=6)

## 启动六足的视觉slam
### astra_pro推送的摄像头主题数据
```shell
$rostopic list
/camera/color/camera_info
/camera/color/image_raw
/camera/color/image_raw/compressed
/camera/color/image_raw/compressed/parameter_descriptions
/camera/color/image_raw/compressed/parameter_updates
/camera/depth/camera_info
/camera/depth/image_raw
/camera/depth/points
/camera/ir/camera_info
/camera/ir/image_raw
```
### rtabmap.launch配置文件说明
主要是修改摄像头的深度主题以及普通的视频流主题
```xml
<!-- 原先在这里面设置了odom的摄像头主题 -->
<remap from="rgb/image_in"       to="/camera/rgb/image_rect_color"/>
<remap from="depth/image_in"     to="/camera/depth_registered/image_raw"/>
<remap from="rgb/camera_info_in" to="/camera/depth_registered/camera_info"/>

<!-- 将上面的参数改为如下主题 -->
<remap from="rgb/image_in"       to="/camera/color/image_raw"/>
<remap from="depth/image_in"     to="/camera/depth/image_raw"/>
<remap from="rgb/camera_info_in" to="/camera/depth/camera_info"/>
```

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<!-- rtabmap launch file -->

<launch>
   <!-- 发布从base_link到camera_link的静态变换；将z值调整到正确的摄像机高度 -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_camera_link" args="0 0 0.098 0 0 0  /base_link /camera_link"/>
    
  <group ns="rtabmap">
        <!-- 与里程计同步深度 -->
        <node pkg="nodelet" type="nodelet" name="standalone_nodelet" args="manager"/>
        <node pkg="nodelet" type="nodelet" name="data_odom_sync" args="load rtabmap_ros/data_odom_sync standalone_nodelet">
          <remap from="odom_in"            to="/odometry/calculated"/>
          <!-- 在这里修改摄像头的主题 -->
          <remap from="rgb/image_in"       to="/camera/color/image_raw"/>
          <remap from="depth/image_in"     to="/camera/depth/image_raw"/>
          <remap from="rgb/camera_info_in" to="/camera/depth/camera_info"/>

          <remap from="rgb/image_out"       to="/rtabmap/camera/rgb/image_rect_color"/>
          <remap from="depth/image_out"     to="/rtabmap/camera/depth_registered/image_raw"/>
          <remap from="rgb/camera_info_out" to="/rtabmap/camera/depth_registered/camera_info"/>
          <remap from="odom_out"            to="/rtabmap/odometry/synchronized"/>
        </node>
        <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" args="--delete_db_on_start">
          <param name="frame_id" type="string" value="base_link"/>
          <remap from="odom" to="/rtabmap/odometry/synchronized"/>

          <param name="subscribe_depth" type="bool" value="true"/>
          <remap from="rgb/image" to="/rtabmap/camera/rgb/image_rect_color"/>
          <remap from="depth/image" to="/rtabmap/camera/depth_registered/image_raw"/>
          <remap from="rgb/camera_info" to="/rtabmap/camera/depth_registered/camera_info"/>

          <param name="subscribe_scan" type="bool" value="true"/>
          <remap from="scan" to="/scan"/>
          <param name="queue_size" type="int" value="30"/>
          <param name="publish_tf" type="bool" value="true"/>
          <param name="map_filter_radius" type="double" value="0"/>
          <remap from="goal_out" to="current_goal"/>

          <!-- RTAB-Map的参数 -->
          <param name="RGBD/OptimizeFromGraphEnd" type="string" value="true"/>
          <param name="Kp/MaxDepth" type="string" value="4.0"/>
          <param name="Reg/Strategy" type="string" value="2"/>
          <param name="lcp/CorrespondenceRatio" type="string" value="0.2"/>
          <param name="Vis/MinInliers" type="string" value="5"/>
          <param name="Vis/InlierDistance" type="string" value="0.1"/>
          <param name="RGBD/AngularUpdate" type="string" value="0.01"/>
          <param name="RGBD/LinearUpdate" type="string" value="0.01"/>
          <param name="Rtabmap/TimeThr" type="string" value="0"/>
          <param name="Mem/RehearsalSimilarity" type="string" value="0.30"/>
          <param name="RGBD/NeighborLinkRefining" type="string" value="true"/>
          <param name="Rtabmap/DetectionRate" type="string" value="1"/>
          <param name="RGBD/ProximityBySpace" type="string" value="true"/>
          <param name="Grid/Sensor" type="string" value="0"/>
          <param name="RGBD/ProximityPathMaxNeighbors" type="string" value="10"/>
        </node>
    </group>
</launch>

```
```shell
roslaunch hexapod_bringup hexapod_pi_full.launch
```