---
layout: post
title: Ubuntu20 Install Noetic
date: 2022-12-20 21:10:00
updated: 2022-12-20 21:10:00
tags: [ROS, Noetic]
categories: [ROS, Noetic]
comments: true
---

# Ubuntu20 Install Noetic

## 前提

网上有很多教程都是，一上来就更换软件源。对此，一定要慎重！换源后并不一定能更好的安装，甚至可能什么软件都下载不了。Anyway，在修改任何操作前养成**备份**的好习惯！在虚拟机中，可以先**拍摄快照**！

## 准备

### 1 更换 ROS 源

清华：

```shell
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main" > /etc/apt/sources.list.d/ros-latest.list'
```

### 2 密钥

```shell
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### 3 公钥

```shell
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
```

### 4 更新源

```shell
sudo apt update
```

## 安装

```shell
sudo apt install ros-noetic-desktop-full -y
```

再次输入命令，确认已经安装完成：

```shell
$ sudo apt install ros-noetic-desktop-full -y
Reading package lists... Done
Building dependency tree       
Reading state information... Done
ros-noetic-desktop-full is already the newest version (1.5.0-1focal.20221124.141646).
0 upgraded, 0 newly installed, 0 to remove and 0 not upgraded.
```

## 环境设置

```shell
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 软件包构建依赖项（可选）

```shell
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
```

## 小海龟测试

```shell
# master
roscore
# turtle simulator node
rosrun turtlesim turtle_node
# turtle keyboard control node
rosrun turtlesim turtle_teleop_key
```

## rosdep 初始化

```
sudo rosdep init
rosdep update
```

实际上，如果我们不会用到 rosdep （类似 apt、pip 的包管理工具），完全可以不用初始化。鱼香 ROS 在这篇文章中，给出了解释：[rosdep不初始化到底行不行](https://juejin.cn/post/7064453605901729829)。

### rosdep 初始化错误

参见 rosdep 初始化部分 ：[Kinetic 安装 ](https://merlotliu.gitbook.io/ros-learning/ros-env-config/ros-installation)

## Reference 

1. [https://wiki.ros.org/noetic/Installation/Ubuntu](https://wiki.ros.org/noetic/Installation/Ubuntu)
1. [【安装】Ubuntu20.04下安装ROS的完整过程（内含已装好ROS的虚拟机、虚拟机创建过程、ROS安装过程及全过程录屏）](https://blog.csdn.net/qq_46106285/article/details/120982412)
1. [rosdep不初始化到底行不行](https://juejin.cn/post/7064453605901729829)
