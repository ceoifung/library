---
sidebar_position: 2
title: 创建Maven仓库
---

部署完nexus之后，我们可以创建一个maven仓库，用来存放我们的库代码
## 登录nexus，创建Maven仓库
![创建Maven仓库](./images/create_maven_repositories.png)
![](./images/%E5%BE%AE%E4%BF%A1%E5%9B%BE%E7%89%87_20230410165837.png)
![](./images/%E5%BE%AE%E4%BF%A1%E5%9B%BE%E7%89%87_20230410170006.png)

执行完以上的步骤之后，仓库已经创建好了
## 查看仓库地址
回到仓库首页，可以看到创库已经创建好，点进去
![](./images/%E5%BE%AE%E4%BF%A1%E5%9B%BE%E7%89%87_20230410170159.png)

可以看到仓库的地址: `http://localhost:8081/repository/xiaor_maven`

![](./images/微信图片_20230410170254.png)

## 我们自己的maven仓库地址
```xml
http://192.168.3.109:9000/repository/xr_maven/
```
:::note
用户名：admin
密码：xiaorgeek001?
:::
支持匿名访问