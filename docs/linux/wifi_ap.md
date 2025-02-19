---
sidebar_position: 2
title: WIFI热点配置
---

## Linux系统WIFI热点配置

```shell
#!/bin/sh
#sleep 10
sleep 2
var=lpage-
wkname=WIFI_AP
password=123456

mac=$(ifconfig eth0 | awk  '/ether/{print $2 ;exit}' |sed 's/\://g')
mac=${var}${mac}

if nmcli connection | grep $wkname;then
echo $password | sudo -S nmcli d disconnect $wkname
echo $password | sudo -S nmcli c delete $wkname
fi

echo $password | sudo -S nmcli con add type wifi ifname wlan0 mode ap con-name WIFI_AP ssid $mac
echo $password | sudo -S nmcli con modify WIFI_AP 802-11-wireless.band bg
echo $password | sudo -S nmcli con modify WIFI_AP 802-11-wireless.channel 11
echo $password | sudo -S nmcli con modify WIFI_AP 802-11-wireless.security.key-mgmt wpa-psk
echo $password | sudo -S nmcli con modify WIFI_AP 802-11-wireless.security.proto rsn
echo $password | sudo -S nmcli con modify WIFI_AP 802-11-wireless.security.group ccmp
echo $password | sudo -S nmcli con modify WIFI_AP 802-11-wireless.security.pairwise ccmp
echo $password | sudo -S nmcli con modify WIFI_AP 802-11-wireless.security.psk 12345678
echo $password | sudo -S nmcli con modify WIFI_AP ipv4.addr 192.168.1.1/24
echo $password | sudo -S nmcli con modify WIFI_AP ipv4.method shared
echo $password | sudo -S nmcli con up WIFI_AP
```


:::tip
其中的密码需要根据实际的来修改
:::