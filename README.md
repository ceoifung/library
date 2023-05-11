<!--
 * @Author: Ceoifung
 * @Date: 2023-03-31 10:21:49
 * @LastEditors: Ceoifung
 * @LastEditTime: 2023-05-11 17:23:43
 * @Description: XiaoRGEEK All Rights Reserved. Copyright © 2023
-->
# 吾生也有涯

这是使用Docusaurus 2创建的个人知识网站

### 更新日志
#### 2023-5-13
- 添加六足开发文档
- 修复ProjectCard样式错误的问题
#### 2023-5-6
- 修改首页显示内容
#### 2023-5-5
- 增加控件`ProjectCard`
- 修改本地调试以及全局配置跟路由，不在展示首页，直接跳转到项目入口
#### 2023-5-4
- 修改编辑的地址为`http://192.168.3.249:8081/ceoifung/library/blob/master/`
- 测试内容发布正常

#### 2023-4-19
- 增加陀螺仪一章节
#### 2023-4-14
- 增加了python pi仓库上传的教程
- 更新修改了提交android仓库代码的文档
#### 2023-4-10
- 添加maven私有仓库搭建教程
- 添加六足ros功能搭建教程
#### 2023-4-4
- 增加本地搜索功能
#### 2023-4-3
- 增加hexapod ros一章节学习描述

#### 2023-3-31
- 初始搭建网站
- 增加openwrt和滴滴车窗开发记录文档
### 安装

```
yarn
```

### 本地调试

```
yarn start
```

### Build

```
yarn build
```

### Deployment

Using SSH:

```
$ USE_SSH=true yarn deploy
```

Not using SSH:

```
$ GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
