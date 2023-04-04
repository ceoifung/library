---
sidebar_position: 6
title: create_ap启动热点
---
create_ap是第三方开发的一个linux系统启动热点的工具，具有很强的适用性

## 依赖安装
```shell
sudo apt-get install -y util-linux procps hostapd iproute2 iw haveged dnsmasq
git clone https://github.com/oblique/create_ap
cd create_ap
vim create_ap 
# 修改默认IP192.168.12.1为192.168.1.1
GATEWAY=192.168.88.100
sudo make install 
```

## 修改createap.conf
```shell
vim /etc/create_ap.conf
```
修改如下
```shell
CHANNEL=default
GATEWAY=192.168.88.100
WPA_VERSION=2
ETC_HOSTS=0
DHCP_DNS=gateway
NO_DNS=0
NO_DNSMASQ=0
HIDDEN=0
MAC_FILTER=0
MAC_FILTER_ACCEPT=/etc/hostapd/hostapd.accept
ISOLATE_CLIENTS=0
SHARE_METHOD=nat
IEEE80211N=0
IEEE80211AC=0
HT_CAPAB=[HT40+]
VHT_CAPAB=
DRIVER=nl80211
NO_VIRT=0
COUNTRY=
FREQ_BAND=2.4
NEW_MACADDR=
DAEMONIZE=0
NO_HAVEGED=0
WIFI_IFACE=wlan0
INTERNET_IFACE=eth0
PASSPHRASE=
USE_PSK=0
SSID=XiaoRGEEK_PiX_id:c27960
```

## 修改rc.local
```shell
sudo vim /etc/rc.local

sed -i '$d' /etc/create_ap.conf
MAC=$(ifconfig wlan0| awk  '/ether/{print $2 ;exit}' |sed 's/\://g' | sed 's/^......//')
ID="SSID=XiaoRGEEK_PiX_id:"
SSID=${ID}${MAC}
echo $SSID >> /etc/create_ap.conf
```

## 一些常用的命令
```shell
sudo systemctl daemon-reload
sudo systemctl start create_ap.service
sudo systemctl enable create_ap   #开机启动服务
sudo systemctl status create_ap.service

##调试执行代码
sudo systemctl stop create_ap
sudo systemctl disable create_ap        #关闭开机启动服务
sudo ifconfig wlan0 up
vim /var/log/syslog
```

## 视频流卡顿问题
经过测试，create_ap启动热点，进行摄像头推流的时候，摄像头画面异常卡顿，然而使用有线连接的时候，画面则不卡顿，经过测试后，发现是create_ap启动的热点的时候，使用了默认的信道，导致推流卡顿，因此可以修改createap.conf文件，将channel修改为其他的
```shell
CHANNEL=12
```
## 信道查询
想查看网卡都支持哪一个信道，可以使用如下命令进行查看
```shell
iw list
```
可以在控制台的输出看到
```shell
Frequencies:
    * 2412 MHz [1] (22.0 dBm)
    * 2417 MHz [2] (22.0 dBm)
    * 2422 MHz [3] (22.0 dBm)
    * 2427 MHz [4] (22.0 dBm)
    * 2432 MHz [5] (22.0 dBm)
    * 2437 MHz [6] (22.0 dBm)
    * 2442 MHz [7] (22.0 dBm)
    * 2447 MHz [8] (22.0 dBm)
    * 2452 MHz [9] (22.0 dBm)
    * 2457 MHz [10] (22.0 dBm)
    * 2462 MHz [11] (22.0 dBm)
    * 2467 MHz [12] (22.0 dBm)
    * 2472 MHz [13] (22.0 dBm)
    * 2484 MHz [14] (disabled)
```
上面支持的就是2.4G的信道
## 网卡5G支持
在终端输入`iw list`命令可以查看是否支持5G
```shell
 Frequencies:
    * 5180 MHz [36] (22.0 dBm) (no IR)
    * 5200 MHz [40] (22.0 dBm) (no IR)
    * 5220 MHz [44] (22.0 dBm) (no IR)
    * 5240 MHz [48] (22.0 dBm) (no IR)
    * 5260 MHz [52] (22.0 dBm) (no IR, radar detection)
    * 5280 MHz [56] (22.0 dBm) (no IR, radar detection)
    * 5300 MHz [60] (22.0 dBm) (no IR, radar detection)
    * 5320 MHz [64] (22.0 dBm) (no IR, radar detection)
    * 5340 MHz [68] (disabled)
    * 5360 MHz [72] (disabled)
    * 5380 MHz [76] (disabled)
    * 5400 MHz [80] (disabled)
    * 5420 MHz [84] (disabled)
    * 5440 MHz [88] (disabled)
    * 5460 MHz [92] (disabled)
    * 5480 MHz [96] (disabled)
    * 5500 MHz [100] (22.0 dBm) (no IR, radar detection)
    * 5520 MHz [104] (22.0 dBm) (no IR, radar detection)
    * 5540 MHz [108] (22.0 dBm) (no IR, radar detection)
    * 5560 MHz [112] (22.0 dBm) (no IR, radar detection)
    * 5580 MHz [116] (22.0 dBm) (no IR, radar detection)
    * 5600 MHz [120] (22.0 dBm) (no IR, radar detection)
    * 5620 MHz [124] (22.0 dBm) (no IR, radar detection)
    * 5640 MHz [128] (22.0 dBm) (no IR, radar detection)
    * 5660 MHz [132] (22.0 dBm) (no IR, radar detection)
    * 5680 MHz [136] (22.0 dBm) (no IR, radar detection)
    * 5700 MHz [140] (22.0 dBm) (no IR, radar detection)
    * 5720 MHz [144] (22.0 dBm) (no IR, radar detection)
    * 5745 MHz [149] (22.0 dBm) (no IR)
    * 5765 MHz [153] (22.0 dBm) (no IR)
    * 5785 MHz [157] (22.0 dBm) (no IR)
    * 5805 MHz [161] (22.0 dBm) (no IR)
    * 5825 MHz [165] (22.0 dBm) (no IR)
    * 5845 MHz [169] (disabled)
    * 5865 MHz [173] (disabled)
    * 5885 MHz [177] (disabled)
    * 5905 MHz [181] (disabled)
```
如上所示，网卡支持的5G信道。但是网卡支持5G，但不代表create_ap可以启动5G网络，可以看到上面的打印信息信道后面有语句`(no IR)`或者`no IR, radar detection`，这表明所在的地区不支持开启5G热点信号，根据newbing的介绍，可以改动当前所在的国家或者地区，但是尝试过后，没有效果
:::tip
如果个频道有`no lR`的标记，表示该频道不允许发起接收操作，只能作为响应者(Responder)使用。这通常是由于国家或地区法规限制，或者该地区需要进行雷达监测（`radar detection`）来避免干扰。如果你想使用一个有noIR标记的频道作为热点(Hotspot)，你可能需要修改你的国家或地区代码，或者使用另一个没有no IR标记的频道
:::
## 参考信息来源
- [ERROR: Your adapter can not transmit to channel 149, frequency band 5GHz. ](https://github.com/oblique/create_ap/issues/107)
- [5G Wif 热点开启失败原因解决](https://www.jianshu.com/p/b0817fa392da)
- newbing解答：

:::info
你好，这是必应。我可以帮你查找一些关于jetson nano启用5G失败怎么修改国家或地区代码的信息根据我在网上搜索的结果
jetson nano的国家或地区代码 (reqdomain)会影响无线网卡支持的频道和信道，不同的国家或地区有不同的法规限制。如果你想启用5G，你可能需要修改你的国家或地区代码为一个支持5G的值，例如US或CA。修改国家或地区代码的方法有以下几种:
- 你可以使用`iw reg set`命令来临时修改国家或地区代码，例如: `sudo iw reg set US`。这个方法只在当前会话有效，重启后会失效。
- 你可以修改`/etc/default/crda`文件来永久修改国家或地区代码，例如: `sudo nano /etc/default/crda`然后将`REGDOMAIN=00改为REGDOMAIN=US`。这个方法需要重启才能生效。
- 你可以下载并安装NVIDIA的SDK Manager，然后使用它来刷入一个新的系统镜像，其中包含了你想要的国家或地区代码。这个方法比较复杂，但是可以保证系统的完整性和稳定性。你可以参考一些教程来了解更多的细节和步骤
:::

