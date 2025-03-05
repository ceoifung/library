---
sidebar_position: 3
title: systemback iso镜像制作
---

## 安装systemback

参考如下连接

https://zhuanlan.zhihu.com/p/576420693

https://www.cnblogs.com/huaibin/p/15000585.html

## 转成ISO镜像

安装systemback之后，生成sblive还原文件

### 安装cdrtools

```shell
wget https://nchc.dl.sourceforge.net/project/cdrtools/alpha/cdrtools-3.02a07.tar.gz
# （此步骤可能下载失败，解决办法是直接浏览器打开此链接下载https://nchc.dl.sourceforge.net/project/cdrtools/alpha/cdrtools-3.02a07.tar.gz，再拷贝至Ubuntu即可）

tar -xzvf cdrtools-3.02a07.tar.gz
cd cdrtools-3.02
make
sudo make install

```

### 创建打包脚本

```shell
vim sblive2iso
```

填写如下内容

```shell
#!/bin/bash

# 检查是否传入了.sblive文件路径
if [ -z "$1" ]; then
    echo "请传入.sblive文件路径作为参数，例如：$0 /home/username/systemback_live_2016-04-27.sblive"
    exit 1
fi

# 获取.sblive文件路径
SBLIVE_FILE="$1"

# 检查文件是否存在
if [ ! -f "$SBLIVE_FILE" ]; then
    echo "文件不存在：$SBLIVE_FILE"
    exit 1
fi
# 获取文件名（去掉路径部分）
SBLIVE_FILENAME=$(basename "$SBLIVE_FILE")
# 获取用户目录
USER_HOME=$(getent passwd "$USER" | cut -d: -f6)

# 在用户目录下创建临时目录
TEMP_DIR="$USER_HOME/sblive_temp"
mkdir -p "$TEMP_DIR"
echo "临时目录已创建：$TEMP_DIR"

# 解压.sblive文件
echo "正在解压.sblive文件..."
tar -xf "$SBLIVE_FILE" -C "$TEMP_DIR"

# 重命名syslinux目录为isolinux
echo "正在重命名syslinux目录为isolinux..."
mv "$TEMP_DIR/syslinux" "$TEMP_DIR/isolinux"
mv "$TEMP_DIR/isolinux/syslinux.cfg" "$TEMP_DIR/isolinux/isolinux.cfg"

# 检查mkisofs工具是否已安装
if ! command -v mkisofs &>/dev/null; then
    echo "mkisofs工具未安装，请先安装mkisofs（例如：sudo apt-get install genisoimage）"
    exit 1
fi

# 生成ISO文件
ISO_FILE="$USER_HOME/$SBLIVE_FILENAME.iso"
echo "正在生成ISO文件..."
cd "$TEMP_DIR/../"
# mkisofs -iso-level 3 -r -V sblive -cache-inodes -J -l -b isolinux/isolinux.bin -no-emul-boot -boot-load-size 4 -boot-info-table -c isolinux/boot.cat -o "$ISO_FILE" "$TEMP_DIR"

/opt/schily/bin/mkisofs -iso-level 3 -r -V sblive -cache-inodes -J -l -b isolinux/isolinux.bin -no-emul-boot -boot-load-size 4 -boot-info-table -c isolinux/boot.cat -o "$ISO_FILE" sblive_temp

# 检查ISO文件是否生成成功
if [ -f "$ISO_FILE" ]; then
    echo "ISO文件生成成功：$ISO_FILE"
else
    echo "ISO文件生成失败，请检查错误信息"
fi

# 删除临时目录
echo "删除临时目录..."
sudo rm -rf "$TEMP_DIR"

echo "脚本执行完成！"

```

保存之后给权限，并且复制到系统目录中全局使用

```shell
chmod 777 sblive2iso
sudo mv sblive2iso /usr/local/bin/
```

使用方式

```shell
sblive2iso [加入sblive文件]
```