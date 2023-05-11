---
sidebar_position: 10
title: hexapod slam开发记录
---
## launch文件选择
```shell
roslaunch hexapod_bringup hexapod_pi_full.launch
```

```xml
<!-- 默认采用大白摄像头 -->
<arg name="camera_name" default="dabai_dcw" />
```

## rviz调试
打开rviz软件，可以加载默认的视图，然后选择相应的主题，可以分别加载地图，激光雷达点以及摄像头画面