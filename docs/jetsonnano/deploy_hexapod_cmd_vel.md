---
sidebar_position: 11
title: hexapod 步态调试记录
---
六足机器人通过发送cmd_vel控制机器人运动，目前发现：发送的cmd_vel符合一般的通信协议，但实际的运动姿态却是相反的

## git ssl错误
```shell
server certificate verification failed. CAfile: /etc/ssl/certs/ca-certificates.crt CRLfile: none
```
```shell
git config --global http.sslverify false

```