---
sidebar_position: 3
title: 添加rc.local服务
---

## jetson nano的rc.local服务
jetson nano 默认是没有rc.local服务的，但是systemd 默认会读取 /etc/systemd/system 下的配置文件，该目录下的文件会链接 /lib/systemd/system/ 下的文件。一般系统安装完 /lib/systemd/system/ 下会有 rc-local.service 文件，即我们需要的配置文件。

## 实现方法
- 将 /lib/systemd/system/rc-local.service 链接到 /etc/systemd/system/ 目录下面来

```shell
ln -fs /lib/systemd/system/rc-local.service /etc/systemd/system/rc-local.service
```

- 修改文件内容

```shell
sudo vim /etc/systemd/system/rc-local.service
```
- 在文件末尾增加

```shell
[Install]
WantedBy=multi-user.target
Alias=rc-local.service
```

- 创建rc.local文件

```shell
sudo vim /etc/rc.local
```
在里面添加如下内容
```shell
#!/bin/bash -e
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script does nothing.
 
# 在这里添加你要添加的脚本
exit 0
```

- 加入系统程序中

```shell
sudo systemctl enable rc-local   #这条语句就是创建一个超链接，在系统启动服务程序中.
```

- 给文件赋予可执行权限

```shell
sudo chmod +x /etc/rc.local
```