---
sidebar_position: 2
title: ttyTHS1串口的使用
---

## ttyTHS1串口的使用

/dev/ttyTHS1 permission denied　解决办法：永久有可操作权限
先尝试：

```shell
#修改权限为可读可写可执行，但是这种设置电脑重启后，又会出现这种问题，还要重新设置
sudo chmod 777 /dev/ttyTHS1
#添加用户组
sudo usermod -a -G　dialout xrrobot  #注意这里的wsh需换成自己系统的用户名
```

如果上述方式重启后串口权限依旧打不开，则尝试下面的方法：

- 先查看串口是否被其他服务占用

```shell
sudo lsof | grep ttyTHS1 # 发现一直占用该串口的服务进程是： nvgetty
**#上面的lsof命令找不到的话安装lsof即可**
systemctl stop nvgetty	 # 停止服务
systemctl disable nvgetty # 取消服务
```

- 新建串口规则

```shell
sudo vim /etc/udev/rules.d/55-myserial.rules
```
- 填入一下内容

```shell
KERNEL=="ttyTHS1", SUBSYSTEM=="tty", GROUP="users", MODE="0666"
#其中GROUP改为自己系统用户名
```

- 应用规则

```shell
# 重启规则
sudo service udev reload 
sudo service udev restart
```